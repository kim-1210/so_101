"""카메라로 물체를 찾고, 물체 자세를 ROS 2 토픽으로 발행한다.

처리 순서
1. RealSense에서 컬러/깊이 영상을 받는다.
2. 마이크 명령을 STT와 GPT로 SAM3용 영어 프롬프트 2개로 바꾼다.
3. 실행 시작 때 SAM3로 물체와 놓을 곳을 한 번씩 찾는다.
4. 검출 결과를 화면으로 확인하고 q면 진행, r이면 다시 녹음한다.
5. 둘 다 맞으면 mask에 해당하는 깊이값을 3차원 점들로 바꾼다.
6. 3차원 점들에서 물체 자세를 계산한다.
7. camera 좌표를 base 좌표로 변환하여 ROS 2 토픽으로 발행한다.
"""

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

from object_pose_estimator import ObjectPoseEstimator
from pub_control_poses import ObjectPosePublishers
from realsense_data import RealSenseCamera
from show_visual import ObjectPoseVisualizer
from sam3 import Sam3_Node
from speak_to_sam.recording import AudioRecorder
from speak_to_sam.whisper_ai import STT
from llm_ai.gpt_ai import GetTarget

# 파일 및 모델 설정
ROOT_DIR = Path(__file__).resolve().parent
BASE_CAMERA_TRANSFORM_PATH = ROOT_DIR / "base_camera.json"
API_KEY_PATH = ROOT_DIR / "llm_ai" / "api_key.json"
RECORDING_DIR = ROOT_DIR / "speak_to_sam" / "recordings"
SAM3_DEBUG_DIR = ROOT_DIR / "debug_sam3"

# 영상과 물체 검출 설정
STATUS_PRINT_INTERVAL_SECONDS = 1.0
BASE_DOWN_AXIS = np.array([0.0, 0.0, -1.0])
MASK_ERODE_KERNEL_SIZE = 3
POSE_SMOOTHING_ALPHA = 0.2
POSE_MATCH_MAX_DISTANCE = 0.08
BASKET_MIN_POINT_COUNT = 20
BASKET_ROTATION_QUATERNION = np.array([0.0, 0.0, 0.0, 1.0])
BASKET_OBJECT_OVERLAP_RATIO_THRESHOLD = 0.7
BASKET_OBJECT_DEPTH_DIFF_THRESHOLD_METERS = 0.05
OBJECT_SEGMENT_PROMPT = "block"
WHISPER_WEIGHT = "large"
DETECTION_REVIEW_WINDOW_NAME = "SAM3 detection review"
OBJECT_REVIEW_COLOR = np.array([0, 255, 0], dtype=np.uint8)
PLACE_REVIEW_COLOR = np.array([255, 0, 255], dtype=np.uint8)
REVIEW_MASK_OPACITY = 0.35
GPT_TARGET_PROMPT = (
    "사용자의 한국어 명령에서 집을 물체와 놓을 장소를 찾아줘. "
    "무조건 SAM3에서 찾기 쉬운 영어 단어로만 답해줘. (한글로 답하면 안됨)"
)


def safe_debug_name(name: str) -> str:
    """파일 이름에 쓰기 어려운 문자를 _로 바꾼다."""
    return "".join(
        character if character.isalnum() or character in {"-", "_"} else "_"
        for character in name
    ).strip("_")


def save_debug_image(name: str, image: np.ndarray) -> None:
    """SAM3 중간 결과를 덮어써서 저장한다."""
    SAM3_DEBUG_DIR.mkdir(exist_ok=True)
    cv2.imwrite(str(SAM3_DEBUG_DIR / f"{safe_debug_name(name)}.jpg"), image)


def print_mask_debug(prompt: str, masks) -> None:
    """mask 개수와 크기를 출력해서 색칠 전 단계 문제를 찾는다."""
    mask_areas = [
        int(np.squeeze(mask).astype(bool).sum())
        for mask in as_mask_list(masks)
    ]
    print(
        f"[DEBUG]: SAM3 prompt='{prompt}' "
        f"mask_count={len(mask_areas)} areas={mask_areas}",
        flush=True,
    )


@dataclass
class ObjectMaskDetection:
    """SAM3가 찾은 물체 이름과 mask를 함께 보관한다."""

    prompt: str
    mask: np.ndarray


@dataclass
class ObjectSegmentationResult:
    """물체별 SAM3 성공/실패 결과를 함께 보관한다."""

    detections: list[ObjectMaskDetection]
    failed_prompts: list[str]


def parse_boolean(value: str) -> bool:
    """명령행의 true/false 문자열을 bool 값으로 바꾼다."""
    normalized_value = value.lower()
    if normalized_value in {"true", "1", "yes", "y", "on"}:
        return True
    if normalized_value in {"false", "0", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError("true 또는 false를 입력해주세요.")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="물체 자세를 검출하고 ROS 2 토픽으로 발행합니다."
    )
    parser.add_argument(
        "--visual",
        type=parse_boolean,
        default=False,
        metavar="true|false",
        help="검출 화면을 표시합니다. (기본값: false)",
    )
    arguments, _ = parser.parse_known_args()
    return arguments


class VoiceTargetSelector:
    """마이크 명령을 SAM3가 쓸 object/place 영어 프롬프트로 바꾼다."""

    def __init__(self):
        self.recorder = AudioRecorder(output_dir=RECORDING_DIR)
        self.stt = STT()
        self.gpt = GetTarget()
        self._load_models()

    def _load_models(self):
        print(f"[INFO]: Loading Whisper model: {WHISPER_WEIGHT}", flush=True)
        self.stt.load_model(WHISPER_WEIGHT)

        with API_KEY_PATH.open(encoding="utf-8") as file:
            keys = json.load(file)

        self.gpt.load_api(keys["gpt_key"])
        self.gpt.set_default_msg(GPT_TARGET_PROMPT)
        print("[INFO]: Voice target selector ready.", flush=True)

    def ask_targets(self) -> tuple[list[str], str]:
        audio_path = self._record_once()
        stt_text = self.stt.get_stt(audio_path)
        print(f"[INFO]: STT: {stt_text}", flush=True)

        target_obj, target_place = self.gpt.get_answer(stt_text)
        target_obj_prompts = normalize_target_object_prompts(target_obj)
        target_place = target_place.strip()

        print(
            f"[INFO]: SAM3 prompts | objects={target_obj_prompts}, "
            f"place='{target_place}'",
            flush=True,
        )
        return target_obj_prompts, target_place

    def _record_once(self) -> Path:
        input("[MIC]: Enter를 누르면 녹음을 시작합니다.")
        audio_path = self.recorder.start()
        input("[MIC]: 말한 뒤 Enter를 누르면 녹음을 끝냅니다.")
        audio_path = self.recorder.stop()
        print(f"[INFO]: Audio saved: {audio_path}", flush=True)
        return audio_path

    def close(self):
        self.recorder.close()


class ObjectPoseSmoother:
    """프레임마다 튀는 물체 위치를 이전 값과 섞어서 부드럽게 만든다."""

    def __init__(
        self,
        alpha: float = POSE_SMOOTHING_ALPHA,
        max_match_distance: float = POSE_MATCH_MAX_DISTANCE,
    ):
        self.alpha = alpha
        self.max_match_distance = max_match_distance
        self.previous_positions: list[np.ndarray] = []

    def smooth(self, transforms: list[np.ndarray]) -> list[np.ndarray]:
        if not transforms:
            return []

        smoothed_transforms = []
        used_previous_indices = set()
        new_previous_positions = []

        for transform in transforms:
            current_position = transform[:3, 3]
            previous_index = self._find_nearest_previous(
                current_position,
                used_previous_indices,
            )

            if previous_index is None:
                smoothed_position = current_position
            else:
                previous_position = self.previous_positions[previous_index]
                smoothed_position = (
                    (1.0 - self.alpha) * previous_position
                    + self.alpha * current_position
                )
                used_previous_indices.add(previous_index)

            smoothed_transform = transform.copy()
            smoothed_transform[:3, 3] = smoothed_position
            smoothed_transforms.append(smoothed_transform)
            new_previous_positions.append(smoothed_position)

        self.previous_positions = new_previous_positions
        return smoothed_transforms

    def _find_nearest_previous(
        self,
        current_position: np.ndarray,
        used_previous_indices: set[int],
    ) -> int | None:
        best_index = None
        best_distance = self.max_match_distance

        for index, previous_position in enumerate(self.previous_positions):
            if index in used_previous_indices:
                continue

            distance = np.linalg.norm(current_position - previous_position)
            if distance < best_distance:
                best_distance = distance
                best_index = index

        return best_index


def sort_masks_for_stable_order(masks: list[np.ndarray]) -> list[np.ndarray]:
    """connected component 순서가 흔들리지 않도록 mask 중심 기준으로 정렬한다."""
    return sorted(masks, key=_mask_sort_key)


def sort_object_detections_for_stable_order(
    detections: list[ObjectMaskDetection],
) -> list[ObjectMaskDetection]:
    """물체 이름별 SAM3 mask 순서가 흔들리지 않도록 정렬한다."""
    return sorted(detections, key=lambda detection: _mask_sort_key(detection.mask))


def _mask_sort_key(mask: np.ndarray) -> tuple[float, float]:
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return (float("inf"), float("inf"))
    return (float(xs.mean()), float(ys.mean()))


def erode_mask_edge(mask: np.ndarray) -> np.ndarray:
    """mask 가장자리의 깊이 노이즈를 줄이기 위해 가장자리를 조금 깎는다."""
    kernel = np.ones(
        (MASK_ERODE_KERNEL_SIZE, MASK_ERODE_KERNEL_SIZE),
        np.uint8,
    )
    eroded = cv2.erode(mask.astype(np.uint8), kernel, iterations=1)
    return eroded.astype(bool)


def load_base_from_camera_transform() -> np.ndarray:
    """JSON 보정값으로 camera 좌표를 base 좌표로 바꾸는 행렬을 만든다."""
    with BASE_CAMERA_TRANSFORM_PATH.open(encoding="utf-8") as file:
        calibration = json.load(file)

    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(
        calibration["orientation"]
    ).as_matrix()
    transform[:3, 3] = calibration["position"]
    return transform


def normalize_target_object_prompts(target_obj) -> list[str]:
    """GPT가 준 물체 결과를 SAM3가 돌릴 수 있는 문자열 리스트로 맞춘다."""
    if isinstance(target_obj, str):
        candidates = [target_obj]
    else:
        candidates = list(target_obj)

    prompts = []
    for candidate in candidates:
        prompt = str(candidate).strip()
        if prompt:
            prompts.append(prompt)

    if not prompts:
        raise ValueError("GPT가 물체 이름을 비워서 반환했습니다.")

    return prompts


def extract_masked_object_points(
    camera: RealSenseCamera,
    near_depth: np.ndarray,
    object_mask: np.ndarray,
) -> np.ndarray:
    """AI가 찾은 물체 영역의 깊이값만 3차원 점으로 변환한다."""
    object_depth = np.where(object_mask, near_depth, 0)
    return camera.depth_to_points(object_depth)


def calculate_mask_points_center(points: np.ndarray) -> np.ndarray:
    """3D 점들의 x, y, z 범위 중앙을 계산한다."""
    minimum_xyz = points.min(axis=0)
    maximum_xyz = points.max(axis=0)
    return (minimum_xyz + maximum_xyz) / 2.0


def as_mask_list(masks) -> list[np.ndarray]:
    """2D mask 1개와 3D mask 묶음을 모두 리스트로 바꾼다."""
    if masks is None:
        return []
    if isinstance(masks, list):
        return [mask for mask in masks if mask is not None]

    masks = np.asarray(masks)
    masks = np.squeeze(masks)

    if masks.ndim == 2:
        return [masks]
    if masks.ndim == 3:
        return [mask for mask in masks]

    return []


def combine_masks(masks, image_shape: tuple[int, int]) -> np.ndarray:
    """여러 mask를 하나의 True/False mask로 합친다."""
    combined_mask = np.zeros(image_shape, dtype=bool)

    for mask in as_mask_list(masks):
        mask = np.squeeze(mask).astype(bool)
        if mask.shape != image_shape:
            continue
        combined_mask |= mask

    return combined_mask


def split_masks_by_connected_components(masks) -> list[np.ndarray]:
    """SAM3 mask를 물체별 connected component mask 목록으로 정리한다."""
    split_masks = []

    for mask in as_mask_list(masks):
        binary_mask = np.squeeze(mask).astype(np.uint8)
        binary_mask = (binary_mask > 0).astype(np.uint8)

        num_labels, labels = cv2.connectedComponents(
            binary_mask,
            connectivity=8,
        )

        for label_id in range(1, num_labels):
            split_masks.append((labels == label_id).astype(np.uint8))

    return split_masks


def segment_objects_once(
    sam3: Sam3_Node,
    color_image: np.ndarray,
    object_prompts: list[str],
) -> ObjectSegmentationResult | None:
    """실행 시작 때 SAM3로 물체 이름마다 mask를 한 번씩 만든다."""
    detections = []
    failed_prompts = []

    for object_prompt in object_prompts:
        sam3.change_obj_name(object_prompt)
        segmentation_result = sam3.obj_segmentation(color_image)
        if segmentation_result is None:
            failed_prompts.append(object_prompt)
            print(
                f"[WARN]: SAM3 object prompt failed: '{object_prompt}'",
                flush=True,
            )
            continue

        masks, show_img_bgr = segmentation_result
        print_mask_debug(object_prompt, masks)
        save_debug_image(f"sam3_object_{object_prompt}", show_img_bgr)

        object_masks = split_masks_by_connected_components(masks)
        if not object_masks:
            failed_prompts.append(object_prompt)
            print(
                "[WARN]: SAM3 object prompt produced no usable mask: "
                f"'{object_prompt}'",
                flush=True,
            )
            continue

        for object_mask in sort_masks_for_stable_order(object_masks):
            detections.append(
                ObjectMaskDetection(
                    prompt=object_prompt,
                    mask=object_mask,
                )
            )

    if failed_prompts:
        fallback_result = segment_objects_with_combined_prompt(
            sam3,
            color_image,
            object_prompts,
        )
        if fallback_result is not None:
            return fallback_result

    if not detections:
        return None

    return ObjectSegmentationResult(
        detections=sort_object_detections_for_stable_order(detections),
        failed_prompts=failed_prompts,
    )


def segment_objects_with_combined_prompt(
    sam3: Sam3_Node,
    color_image: np.ndarray,
    object_prompts: list[str],
) -> ObjectSegmentationResult | None:
    """notebook과 같이 물체 list 전체를 SAM3 prompt로 넣어 다시 시도한다."""
    prompt_label = ", ".join(object_prompts)
    print(
        "[INFO]: Retrying SAM3 with combined object prompts: "
        f"{object_prompts}",
        flush=True,
    )

    sam3.change_obj_name(object_prompts)
    segmentation_result = sam3.obj_segmentation(color_image)
    if segmentation_result is None:
        return None

    masks, show_img_bgr = segmentation_result
    print_mask_debug(prompt_label, masks)
    save_debug_image("sam3_object_combined", show_img_bgr)

    object_masks = split_masks_by_connected_components(masks)
    if not object_masks:
        return None

    detections = [
        ObjectMaskDetection(prompt=prompt_label, mask=object_mask)
        for object_mask in sort_masks_for_stable_order(object_masks)
    ]
    return ObjectSegmentationResult(
        detections=sort_object_detections_for_stable_order(detections),
        failed_prompts=[],
    )


def segment_place_once(
    sam3: Sam3_Node,
    color_image: np.ndarray,
    place_prompt: str,
) -> np.ndarray | None:
    """실행 시작 때 SAM3로 놓을 곳 mask를 한 번만 만든다."""
    sam3.change_obj_name(place_prompt)
    segmentation_result = sam3.obj_segmentation(color_image)
    if segmentation_result is None:
        return None

    masks, show_img_bgr = segmentation_result
    print_mask_debug(place_prompt, masks)
    save_debug_image(f"sam3_place_{place_prompt}", show_img_bgr)

    place_masks = as_mask_list(masks)
    if not place_masks:
        return None

    return np.asarray(place_masks)


def draw_review_masks(
    image: np.ndarray,
    masks,
    color: np.ndarray,
) -> None:
    """검출 확인용 이미지 위에 mask를 반투명하게 칠한다."""
    for mask in as_mask_list(masks):
        mask = np.squeeze(mask).astype(bool)
        if mask.shape != image.shape[:2]:
            continue
        image[mask] = (
            image[mask] * (1.0 - REVIEW_MASK_OPACITY)
            + color * REVIEW_MASK_OPACITY
        ).astype(np.uint8)


def draw_review_label(
    image: np.ndarray,
    mask: np.ndarray,
    label: str,
) -> None:
    """mask 중심 근처에 물체 이름을 적는다."""
    ys, xs = np.nonzero(np.squeeze(mask).astype(bool))
    if len(xs) == 0:
        return

    x = int(xs.mean())
    y = int(ys.mean())
    cv2.putText(
        image,
        label,
        (x + 8, max(y - 8, 24)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 0),
        4,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        label,
        (x + 8, max(y - 8, 24)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )


def make_detection_review_image(
    color_image: np.ndarray,
    object_masks,
    place_masks,
    object_prompts: list[str],
    place_prompt: str,
) -> np.ndarray:
    """사용자가 SAM3 검출 결과를 Enter/r로 판단할 수 있는 이미지를 만든다."""
    review_image = color_image.copy()

    for detection in object_masks:
        draw_review_masks(review_image, [detection.mask], OBJECT_REVIEW_COLOR)
        draw_review_label(review_image, detection.mask, detection.prompt)

    draw_review_masks(review_image, place_masks, PLACE_REVIEW_COLOR)

    lines = [
        f"objects: {', '.join(object_prompts)} (green)",
        f"place: {place_prompt} (magenta)",
        "Enter: continue    r: record again",
    ]
    y = 28
    for line in lines:
        cv2.putText(
            review_image,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            review_image,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        y += 28

    return review_image


def wait_for_detection_review(
    color_image: np.ndarray,
    object_masks,
    place_masks,
    object_prompts: list[str],
    place_prompt: str,
) -> bool:
    """Enter면 True, r이나 창 닫기면 False를 반환한다."""
    review_image = make_detection_review_image(
        color_image,
        object_masks,
        place_masks,
        object_prompts,
        place_prompt,
    )
    save_debug_image(
        f"review_{'_'.join(object_prompts)}_to_{place_prompt}",
        review_image,
    )

    window_name = (
        f"{DETECTION_REVIEW_WINDOW_NAME}: "
        f"{', '.join(object_prompts)} -> {place_prompt}"
    )
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.imshow(window_name, review_image)
    print("[CHECK]: SAM3 결과 확인: Enter=진행, r=다시 녹음", flush=True)

    while True:
        key = cv2.waitKey(50) & 0xFF
        try:
            window_is_open = cv2.getWindowProperty(
                window_name,
                cv2.WND_PROP_VISIBLE,
            ) >= 1
        except cv2.error:
            window_is_open = False
        if not window_is_open:
            return False
        if key in (10, 13):
            cv2.destroyWindow(window_name)
            return True
        if key in (ord("r"), ord("R"), 27):
            cv2.destroyWindow(window_name)
            return False


def wait_for_each_object_review(
    color_image: np.ndarray,
    object_detections: list[ObjectMaskDetection],
    place_masks,
    object_prompts: list[str],
    place_prompt: str,
) -> bool:
    """검출된 모든 물체와 장소를 한 화면에서 확인한다."""
    return wait_for_detection_review(
        color_image,
        object_detections,
        place_masks,
        object_prompts,
        place_prompt,
    )


def calculate_mask_median_depth_meters(
    near_depth: np.ndarray,
    mask: np.ndarray,
    depth_scale: float,
) -> float | None:
    """mask 영역의 대표 깊이를 미터 단위 중앙값으로 계산한다."""
    mask = np.squeeze(mask).astype(bool)
    if mask.shape != near_depth.shape:
        return None

    valid_depth = near_depth[mask & (near_depth > 0)]
    if len(valid_depth) == 0:
        return None

    return float(np.median(valid_depth) * depth_scale)


def remove_basket_like_object_masks(
    object_detections: list[ObjectMaskDetection],
    basket_masks,
    near_depth: np.ndarray,
    depth_scale: float,
    basket_depth_meters: float | None,
) -> list[ObjectMaskDetection]:
    """basket과 2D로 겹치고 깊이도 비슷한 object 후보를 제거한다."""
    if basket_depth_meters is None:
        return object_detections

    basket_mask = combine_masks(basket_masks, near_depth.shape)
    if not basket_mask.any():
        return object_detections

    filtered_detections = []

    for detection in object_detections:
        object_mask = np.squeeze(detection.mask).astype(bool)
        if object_mask.shape != near_depth.shape:
            filtered_detections.append(detection)
            continue

        object_area = int(object_mask.sum())
        if object_area == 0:
            continue

        overlap_area = int((object_mask & basket_mask).sum())
        overlap_ratio = overlap_area / object_area
        if overlap_ratio < BASKET_OBJECT_OVERLAP_RATIO_THRESHOLD:
            filtered_detections.append(detection)
            continue

        object_depth_meters = calculate_mask_median_depth_meters(
            near_depth,
            object_mask,
            depth_scale,
        )
        if object_depth_meters is None:
            filtered_detections.append(detection)
            continue

        depth_difference = abs(object_depth_meters - basket_depth_meters)
        if depth_difference > BASKET_OBJECT_DEPTH_DIFF_THRESHOLD_METERS:
            filtered_detections.append(detection)

    return filtered_detections


def find_basket_position_in_base(
    camera: RealSenseCamera,
    near_depth: np.ndarray,
    basket_masks: np.ndarray,
    base_from_camera: np.ndarray,
) -> np.ndarray | None:
    """basket mask에서 base 기준 중앙 위치를 한 번 계산한다."""
    best_position = None
    best_point_count = 0

    for basket_mask in basket_masks:
        cut_mask = camera.get_depth_cut_mask(basket_mask, near_depth)
        cut_mask = erode_mask_edge(cut_mask)
        basket_points = extract_masked_object_points(
            camera,
            near_depth,
            cut_mask,
        )

        if len(basket_points) < BASKET_MIN_POINT_COUNT:
            continue

        if len(basket_points) <= best_point_count:
            continue

        basket_center_in_camera = calculate_mask_points_center(basket_points)
        basket_center_homogeneous = np.append(basket_center_in_camera, 1.0)
        basket_center_in_base = base_from_camera @ basket_center_homogeneous
        best_position = basket_center_in_base[:3]
        best_point_count = len(basket_points)

    return best_position


def print_status_at_interval(message: str, last_print_time: float) -> float:
    """같은 상태 문구는 1초마다 한 번만 출력한다."""
    now = time.monotonic()
    if now - last_print_time >= STATUS_PRINT_INTERVAL_SECONDS:
        print(message, flush=True)
        return now
    return last_print_time


def point_object_z_axis_downward(
    base_from_object: np.ndarray,
) -> np.ndarray:
    """물체 z축을 base의 정확한 아래 방향(0, 0, -1)으로 고정한다."""
    corrected_transform = base_from_object.copy()
    detected_rotation = base_from_object[:3, :3]

    # 검출된 x축의 수평 방향은 유지하되, 위·아래 성분은 제거한다.
    x_axis = detected_rotation[:, 0].copy()
    x_axis[2] = 0.0
    if np.linalg.norm(x_axis) < 1e-6:
        x_axis = detected_rotation[:, 1].copy()
        # x_axis[2] = 0.0
    x_axis /= np.linalg.norm(x_axis)

    z_axis = BASE_DOWN_AXIS
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    corrected_transform[:3, :3] = np.column_stack(
        (x_axis, y_axis, z_axis)
    )
    return corrected_transform


def run_detection_loop(
    ros_node: ObjectPosePublishers,
    camera: RealSenseCamera,
    sam3: Sam3_Node,
    voice_selector: VoiceTargetSelector,
    visualizer: ObjectPoseVisualizer | None = None,
) -> None:
    """카메라 프레임을 계속 처리하고 계산된 자세를 발행한다."""
    ros_node.get_logger().info(f"Using SAM3 device: {sam3.device}")

    base_from_camera = load_base_from_camera_transform()
    camera_from_base = np.linalg.inv(base_from_camera)
    last_status_print_time = 0.0
    pose_estimators: list[ObjectPoseEstimator] = []
    pose_smoother = ObjectPoseSmoother()
    object_detections = None
    target_obj_prompts = None
    target_place_prompt = None

    basket_check = False
    basket_position = None
    basket_masks_for_visual = None
    basket_depth_meters = None

    while rclpy.ok():
        if target_obj_prompts is None or target_place_prompt is None:
            target_obj_prompts, target_place_prompt = voice_selector.ask_targets()
            object_detections = None
            pose_estimators = []
            pose_smoother = ObjectPoseSmoother()
            basket_check = False
            basket_position = None
            basket_masks_for_visual = None
            basket_depth_meters = None

        frames = camera.get_frames()
        if not frames:
            last_status_print_time = print_status_at_interval(
                "Waiting for camera frame...",
                last_status_print_time,
            )
            continue

        color_image = camera.get_color_image(frames)
        near_depth = camera.get_near_depth(frames)

        if object_detections is None:
            object_segmentation = segment_objects_once(
                sam3,
                color_image,
                target_obj_prompts,
            )
            if object_segmentation is None:
                ros_node.get_logger().warn(
                    f"SAM3 object detection failed: {target_obj_prompts}. "
                    "Recording again."
                )
                target_obj_prompts = None
                target_place_prompt = None
                continue

            if object_segmentation.failed_prompts:
                ros_node.get_logger().warn(
                    "SAM3 object detection failed for "
                    f"{object_segmentation.failed_prompts}. "
                    "Recording again."
                )
                target_obj_prompts = None
                target_place_prompt = None
                object_detections = None
                continue

            object_detections = object_segmentation.detections
            ros_node.get_logger().info(
                "SAM3 object masks saved once: "
                f"count={len(object_detections)}"
            )

        if basket_check == False:
            basket_masks = segment_place_once(
                sam3,
                color_image,
                target_place_prompt,
            )
            if basket_masks is not None:
                basket_position = find_basket_position_in_base(
                    camera,
                    near_depth,
                    basket_masks,
                    base_from_camera,
                )

                if basket_position is not None:
                    basket_check = True
                    basket_masks_for_visual = basket_masks
                    basket_depth_meters = calculate_mask_median_depth_meters(
                        near_depth,
                        combine_masks(basket_masks, near_depth.shape),
                        camera.depth_scale,
                    )
                    ros_node.get_logger().info(
                        "Basket position saved: "
                        f"x={basket_position[0]:.3f}, "
                        f"y={basket_position[1]:.3f}, "
                        f"z={basket_position[2]:.3f}"
                    )
                    is_review_approved = wait_for_each_object_review(
                        color_image,
                        object_detections,
                        basket_masks_for_visual,
                        target_obj_prompts,
                        target_place_prompt,
                    )
                    if not is_review_approved:
                        ros_node.get_logger().info(
                            "Detection rejected by user. Recording again."
                        )
                        target_obj_prompts = None
                        target_place_prompt = None
                        object_detections = None
                        pose_estimators = []
                        pose_smoother = ObjectPoseSmoother()
                        basket_check = False
                        basket_position = None
                        basket_masks_for_visual = None
                        basket_depth_meters = None
                        continue
                else:
                    ros_node.get_logger().warn(
                        f"SAM3 place depth detection failed: "
                        f"'{target_place_prompt}'. Recording again."
                    )
                    target_obj_prompts = None
                    target_place_prompt = None
                    object_detections = None
                    continue
            else:
                ros_node.get_logger().warn(
                    f"SAM3 place detection failed: '{target_place_prompt}'. "
                    "Recording again."
                )
                target_obj_prompts = None
                target_place_prompt = None
                object_detections = None
                continue

        current_object_detections = remove_basket_like_object_masks(
            object_detections,
            basket_masks_for_visual,
            near_depth,
            camera.depth_scale,
            basket_depth_meters,
        )

        cut_masks = []
        camera_from_objects = []
        base_from_objects = []

        while len(pose_estimators) < len(current_object_detections):
            pose_estimators.append(ObjectPoseEstimator())

        # target obj 개수에 맞게 표시
        for mask_index, detection in enumerate(current_object_detections):
            one_object_mask = detection.mask
            cut_mask = camera.get_depth_cut_mask(one_object_mask, near_depth)
            cut_mask = erode_mask_edge(cut_mask)

            object_points = extract_masked_object_points(
                camera,
                near_depth,
                cut_mask,
            )

            camera_from_object = pose_estimators[mask_index].estimate(
                object_points
            )

            if camera_from_object is None:
                continue
            
            base_from_object = base_from_camera @ camera_from_object
            base_from_object = point_object_z_axis_downward(base_from_object)

            camera_from_object = camera_from_base @ base_from_object

            cut_masks.append(cut_mask)
            camera_from_objects.append(camera_from_object)
            base_from_objects.append(base_from_object)

        # obj와 base 간극 변환
        base_from_objects = pose_smoother.smooth(base_from_objects)
        camera_from_objects = [
            camera_from_base @ base_from_object
            for base_from_object in base_from_objects
        ]
        # ros topic 전송
        ros_node.publish_object_and_gripper_poses(np.array(base_from_objects))
        if basket_position is not None:
            ros_node.publisher_basket(
                basket_position,
                BASKET_ROTATION_QUATERNION,
            )

        # 시각화
        if visualizer is not None:
            should_continue = visualizer.show(
                color_image,
                cut_masks,
                camera_from_objects,
                basket_masks_for_visual,
            )
            if not should_continue:
                ros_node.get_logger().info("Visual window closed by user.")
                break

        rclpy.spin_once(ros_node, timeout_sec=0.0)


def main() -> None:
    """ROS 노드와 카메라를 준비하고 종료할 때 정리한다."""
    arguments = parse_arguments()
    rclpy.init()
    ros_node = ObjectPosePublishers()
    camera = None
    visualizer = None
    voice_selector = None

    try:
        # 깊이를 1m로
        camera = RealSenseCamera(maximum_distance=2.3)
        sam3_node = Sam3_Node(obj_name=OBJECT_SEGMENT_PROMPT)
        voice_selector = VoiceTargetSelector()
        if arguments.visual:
            # 시각 부착 여부
            visualizer = ObjectPoseVisualizer(
                camera.camera_matrix,
                camera.distortion_coefficients,
            )
        run_detection_loop(
            ros_node, 
            camera, 
            sam3_node, 
            voice_selector,
            visualizer
        )
    except KeyboardInterrupt:
        ros_node.get_logger().info("Stopped by user.")
    finally:
        if voice_selector is not None:
            voice_selector.close()
        if camera is not None:
            camera.stop()
        if visualizer is not None:
            visualizer.close()
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
