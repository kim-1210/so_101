"""카메라로 물체를 찾고, 물체 자세를 ROS 2 토픽으로 발행한다.

처리 순서
1. RealSense에서 컬러/깊이 영상을 받는다.
2. AI 모델로 컬러 영상 속 물체 영역(mask)을 찾는다.
3. mask에 해당하는 깊이값을 3차원 점들로 바꾼다.
4. 3차원 점들에서 물체 자세를 계산한다.
5. camera 좌표를 base 좌표로 변환하여 ROS 2 토픽으로 발행한다.
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import torch
from PIL import Image
from scipy.spatial.transform import Rotation
from transformers import AutoImageProcessor

from object_pose_estimator import ObjectPoseEstimator
from pub_control_pose import ObjectPosePublisher
from pub_control_poses import ObjectPosePublishers
from realsense_data import RealSenseCamera
from rf_detr_model import CubeRFDetr
from show_visual import ObjectPoseVisualizer
from sam3 import Sam3_Node

# 파일 및 모델 설정
ROOT_DIR = Path(__file__).resolve().parent
MODEL_WEIGHTS_PATH = ROOT_DIR / "best_model.pt"
BASE_CAMERA_TRANSFORM_PATH = ROOT_DIR / "base_camera.json"
PRETRAINED_MODEL_NAME = "stevenbucaille/rf-detr-seg-medium"

# 영상과 물체 검출 설정
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 800
MASK_THRESHOLD = 0.82
STATUS_PRINT_INTERVAL_SECONDS = 1.0
BASE_DOWN_AXIS = np.array([0.0, 0.0, -1.0])
MASK_ERODE_KERNEL_SIZE = 3
POSE_SMOOTHING_ALPHA = 0.2
POSE_MATCH_MAX_DISTANCE = 0.08

BASKET_MIN_POINT_COUNT = 20
BASKET_ROTATION_QUATERNION = np.array([0.0, 0.0, 0.0, 1.0])
BASKET_OBJECT_OVERLAP_RATIO_THRESHOLD = 0.7
BASKET_OBJECT_DEPTH_DIFF_THRESHOLD_METERS = 0.05


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


class ObjectSegmenter:
    """컬러 영상에서 물체에 해당하는 픽셀을 찾는다."""

    def __init__(self, device: torch.device):
        self.device = device
        self.processor = AutoImageProcessor.from_pretrained(
            PRETRAINED_MODEL_NAME
        )
        self.model = self._load_model()

    def _load_model(self) -> CubeRFDetr:
        model = CubeRFDetr(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
        model.load_state_dict(
            torch.load(MODEL_WEIGHTS_PATH, map_location=self.device)
        )
        model.to(self.device)
        model.eval()
        return model

    # min_area # 100픽셀보다 작은점은 제거
    def predict_mask(self, color_image_bgr: np.ndarray, min_area: int = 7) -> np.ndarray:
        """BGR 영상을 받아 물체 영역을 True/False 배열로 반환한다."""
        color_image_rgb = cv2.cvtColor(color_image_bgr, cv2.COLOR_BGR2RGB)
        model_inputs = self.processor(
            images=Image.fromarray(color_image_rgb),
            return_tensors="pt",
        )
        model_inputs = {
            name: value.to(self.device)
            for name, value in model_inputs.items()
        }

        with torch.no_grad():
            logits = self.model(model_inputs)[0, 0]
            probabilities = torch.sigmoid(logits)

        mask = probabilities.cpu().numpy() > MASK_THRESHOLD

        reverse_binary_mask = mask.astype(np.uint8)

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            reverse_binary_mask,
            connectivity=8,
        )

        clean_binary = np.zeros_like(reverse_binary_mask)

        for label_id in range(1, num_labels):
            area = stats[label_id, cv2.CC_STAT_AREA]

            if area >= min_area:
                clean_binary[labels == label_id] = 1

        # clean_mask = np.where(clean_binary == 1, 0, 255).astype(np.uint8)
        clean_mask = (clean_binary * 255).astype(np.uint8)

        return clean_mask
    
    def div_mask(self, mask):
        binary_mask = (mask > 0).astype(np.uint8)

        num_labels, labels, _, _ = cv2.connectedComponentsWithStats(
            binary_mask,
            connectivity=8,
        )

        masks = []
        for label_id in range(1, num_labels):
            masks.append((labels == label_id).astype(np.uint8))

        return masks


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
    object_masks: list[np.ndarray],
    basket_masks,
    near_depth: np.ndarray,
    depth_scale: float,
    basket_depth_meters: float | None,
) -> list[np.ndarray]:
    """basket과 2D로 겹치고 깊이도 비슷한 object 후보를 제거한다."""
    if basket_depth_meters is None:
        return object_masks

    basket_mask = combine_masks(basket_masks, near_depth.shape)
    if not basket_mask.any():
        return object_masks

    filtered_masks = []

    for object_mask in object_masks:
        object_mask = np.squeeze(object_mask).astype(bool)
        if object_mask.shape != near_depth.shape:
            filtered_masks.append(object_mask)
            continue

        object_area = int(object_mask.sum())
        if object_area == 0:
            continue

        overlap_area = int((object_mask & basket_mask).sum())
        overlap_ratio = overlap_area / object_area
        if overlap_ratio < BASKET_OBJECT_OVERLAP_RATIO_THRESHOLD:
            filtered_masks.append(object_mask)
            continue

        object_depth_meters = calculate_mask_median_depth_meters(
            near_depth,
            object_mask,
            depth_scale,
        )
        if object_depth_meters is None:
            filtered_masks.append(object_mask)
            continue

        depth_difference = abs(object_depth_meters - basket_depth_meters)
        if depth_difference > BASKET_OBJECT_DEPTH_DIFF_THRESHOLD_METERS:
            filtered_masks.append(object_mask)

    return filtered_masks


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
    visualizer: ObjectPoseVisualizer | None = None,
) -> None:
    """카메라 프레임을 계속 처리하고 계산된 자세를 발행한다."""
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    ros_node.get_logger().info(f"Using device: {device}")

    segmenter = ObjectSegmenter(device)
    base_from_camera = load_base_from_camera_transform()
    camera_from_base = np.linalg.inv(base_from_camera)
    last_status_print_time = 0.0
    pose_estimators: list[ObjectPoseEstimator] = []
    pose_smoother = ObjectPoseSmoother()

    basket_check = False
    basket_position = None
    basket_masks_for_visual = None
    basket_depth_meters = None

    while rclpy.ok():
        frames = camera.get_frames()
        if not frames:
            last_status_print_time = print_status_at_interval(
                "Waiting for camera frame...",
                last_status_print_time,
            )
            continue

        color_image = camera.get_color_image(frames)
        near_depth = camera.get_near_depth(frames)
        object_mask = segmenter.predict_mask(color_image)
        object_masks = sort_masks_for_stable_order(
            segmenter.div_mask(object_mask)
        )

        if basket_check == False:
            basket_segmentation_result = sam3.obj_segmentation(color_image)
            if basket_segmentation_result is not None:
                basket_masks, _show_img_bgr = basket_segmentation_result
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

        object_masks = remove_basket_like_object_masks(
            object_masks,
            basket_masks_for_visual,
            near_depth,
            camera.depth_scale,
            basket_depth_meters,
        )

        cut_masks = []
        camera_from_objects = []
        base_from_objects = []

        while len(pose_estimators) < len(object_masks):
            pose_estimators.append(ObjectPoseEstimator())

        # target obj 개수에 맞게 표시
        for mask_index, one_object_mask in enumerate(object_masks):
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

    try:
        # 깊이를 1m로
        camera = RealSenseCamera(maximum_distance=3.0)
        sam3_node = Sam3_Node(obj_name="small container")
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
            visualizer
        )
    except KeyboardInterrupt:
        ros_node.get_logger().info("Stopped by user.")
    finally:
        if camera is not None:
            camera.stop()
        if visualizer is not None:
            visualizer.close()
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
