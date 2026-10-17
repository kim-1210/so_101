"""Intel RealSense의 컬러/깊이 영상을 읽고 3차원 점으로 변환한다.

카메라 내부 파라미터와 왜곡계수는 코드에 하드코딩하지 않고,
현재 선택된 RealSense 컬러 스트림 프로파일에서 읽는다.
"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np
import pyrealsense2 as rs2


DEFAULT_COLOR_WIDTH = 1280
DEFAULT_COLOR_HEIGHT = 800

# 작은 물체의 3차원 위치를 구할 때는 가능한 경우 1280 x 720을 유지한다.
# 처리량을 줄여야 한다면 생성자에서 depth_width=848, depth_height=480으로 바꾼다.
DEFAULT_DEPTH_WIDTH = 1280
DEFAULT_DEPTH_HEIGHT = 720

DEFAULT_FRAMES_PER_SECOND = 30
DEFAULT_FRAME_TIMEOUT_MS = 3000

# 이전 코드에서 이 이름들을 import한 경우를 위한 호환용 상수이다.
IMAGE_WIDTH = DEFAULT_COLOR_WIDTH
IMAGE_HEIGHT = DEFAULT_COLOR_HEIGHT
FRAMES_PER_SECOND = DEFAULT_FRAMES_PER_SECOND
FRAME_TIMEOUT_MS = DEFAULT_FRAME_TIMEOUT_MS


class RealSenseCamera:
    """Color 기준으로 정렬된 RealSense 컬러/깊이 영상을 제공한다.

    모든 내부 파라미터는 ``pipeline.start()``가 선택한 실제 스트림
    프로파일에서 가져온다. 기본 좌표계는 왜곡을 별도로 제거하지 않은
    RealSense color optical frame이다.
    """

    def __init__(
        self,
        maximum_distance: float = 3.0,
        *,
        color_width: int = DEFAULT_COLOR_WIDTH,
        color_height: int = DEFAULT_COLOR_HEIGHT,
        depth_width: int = DEFAULT_DEPTH_WIDTH,
        depth_height: int = DEFAULT_DEPTH_HEIGHT,
        frames_per_second: int = DEFAULT_FRAMES_PER_SECOND,
        frame_timeout_ms: int = DEFAULT_FRAME_TIMEOUT_MS,
    ) -> None:
        if maximum_distance <= 0:
            raise ValueError("maximum_distance는 0보다 커야 합니다.")

        self.maximum_distance = float(maximum_distance)
        self.color_width = int(color_width)
        self.color_height = int(color_height)
        self.depth_width = int(depth_width)
        self.depth_height = int(depth_height)
        self.frames_per_second = int(frames_per_second)
        self.frame_timeout_ms = int(frame_timeout_ms)

        self.pipeline = rs2.pipeline()
        self.config = rs2.config()
        self._started = False

        self._configure_streams()

        try:
            self.pipeline_profile = self.pipeline.start(self.config)
            self._started = True
            self._initialize_from_active_profile()
        except Exception:
            if self._started:
                self.pipeline.stop()
                self._started = False
            raise

    def _configure_streams(self) -> None:
        self.config.enable_stream(
            rs2.stream.color,
            self.color_width,
            self.color_height,
            rs2.format.bgr8,
            self.frames_per_second,
        )
        self.config.enable_stream(
            rs2.stream.depth,
            self.depth_width,
            self.depth_height,
            rs2.format.z16,
            self.frames_per_second,
        )

    def _initialize_from_active_profile(self) -> None:
        depth_sensor = self.pipeline_profile.get_device().first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())

        self.align_to_color = rs2.align(rs2.stream.color)

        color_profile = (
            self.pipeline_profile.get_stream(rs2.stream.color)
            .as_video_stream_profile()
        )
        self.color_intrinsics = color_profile.get_intrinsics()

        # align(rs2.stream.color)의 출력 Depth는 Color 좌표계를 사용한다.
        self.aligned_depth_intrinsics = self.color_intrinsics

        # 기존 코드와의 호환을 위해 camera_matrix 이름을 유지한다.
        self.camera_matrix = self._intrinsics_to_camera_matrix(
            self.color_intrinsics
        )
        self.distortion_coefficients = (
            self._intrinsics_to_distortion_coefficients(
                self.color_intrinsics
            )
        )

        # 실제로 활성화된 컬러 프로파일 크기를 사용한다.
        self.aligned_width = int(self.color_intrinsics.width)
        self.aligned_height = int(self.color_intrinsics.height)

        self._create_undistortion_maps()

    def _create_undistortion_maps(self) -> None:
        """RealSense가 제공한 Color 보정값으로 OpenCV remap을 준비한다."""
        model = self.color_intrinsics.model
        compatible_models = {
            rs2.distortion.none,
            rs2.distortion.brown_conrady,
            rs2.distortion.modified_brown_conrady,
        }

        self.undistortion_available = model in compatible_models
        self.undistort_map_x: np.ndarray | None = None
        self.undistort_map_y: np.ndarray | None = None
        self.undistortion_roi: tuple[int, int, int, int] | None = None

        if not self.undistortion_available:
            # inverse_brown_conrady 등의 계수는 OpenCV의 일반 Brown-Conrady
            # 계수와 같은 방향으로 바로 사용할 수 없다.
            self.undistorted_camera_matrix = self.camera_matrix.copy()
            return

        image_size = (self.aligned_width, self.aligned_height)
        (
            self.undistorted_camera_matrix,
            self.undistortion_roi,
        ) = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix,
            self.distortion_coefficients,
            image_size,
            alpha=0,
            newImgSize=image_size,
        )

        self.undistort_map_x, self.undistort_map_y = (
            cv2.initUndistortRectifyMap(
                self.camera_matrix,
                self.distortion_coefficients,
                None,
                self.undistorted_camera_matrix,
                image_size,
                cv2.CV_32FC1,
            )
        )

    def get_frames(self):
        """동일 frameset의 Color와 Color에 정렬된 Depth를 반환한다."""
        frames = self.pipeline.wait_for_frames(
            timeout_ms=self.frame_timeout_ms
        )
        return self.align_to_color.process(frames)

    def get_color_image(self, frames) -> np.ndarray:
        """왜곡 제거 전 Color 좌표계의 BGR 영상을 반환한다."""
        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError("Color 프레임을 받지 못했습니다.")
        return np.asanyarray(color_frame.get_data())

    def get_depth_meters(self, frames) -> np.ndarray:
        """Color에 정렬된 Depth를 미터 단위 float32 배열로 반환한다."""
        raw_depth = self._get_raw_depth(frames)
        return raw_depth.astype(np.float32) * self.depth_scale

    def get_near_depth(self, frames) -> np.ndarray:
        """가까운 값만 남긴 raw Z16 Depth를 반환한다.

        기존 코드와의 호환을 위해 반환 단위를 바꾸지 않는다.
        ``depth_to_points()``는 정수 배열을 raw Z16으로 자동 판별한다.
        """
        raw_depth = self._get_raw_depth(frames)
        depth_meters = raw_depth.astype(np.float32) * self.depth_scale
        is_near = (
            (depth_meters > 0.0)
            & (depth_meters < self.maximum_distance)
        )
        return np.where(is_near, raw_depth, 0).astype(raw_depth.dtype)

    def _get_raw_depth(self, frames) -> np.ndarray:
        """Color에 정렬된 Z16 Depth와 해당 출력 intrinsics를 읽는다."""
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            raise RuntimeError("Depth 프레임을 받지 못했습니다.")

        self.aligned_depth_intrinsics = (
            depth_frame.profile
            .as_video_stream_profile()
            .get_intrinsics()
        )
        return np.asanyarray(depth_frame.get_data())

    def depth_to_points(
        self,
        depth_image: np.ndarray,
        *,
        undistorted: bool = False,
    ) -> np.ndarray:
        """Depth를 color optical frame의 Nx3 점으로 변환한다.

        Args:
            depth_image: ``get_depth_meters()``의 float 미터 배열 또는
                ``get_near_depth()``의 정수 raw Z16 배열. dtype에 따라
                단위를 자동으로 판별한다.
            undistorted: ``get_undistorted_pair()``에서 얻은 Depth라면 True.
        """
        depth_array = np.asarray(depth_image)
        if depth_array.ndim != 2:
            raise ValueError("depth_image는 HxW 형태의 2차원 배열이어야 합니다.")

        if np.issubdtype(depth_array.dtype, np.integer):
            depth_meters = depth_array.astype(np.float32) * self.depth_scale
        else:
            depth_meters = depth_array.astype(np.float32)

        expected_shape = (self.aligned_height, self.aligned_width)
        if depth_meters.shape != expected_shape:
            raise ValueError(
                "Depth 해상도가 Color 정렬 해상도와 다릅니다: "
                f"입력={depth_meters.shape}, 예상={expected_shape}"
            )

        is_valid = (
            (depth_meters > 0.0)
            & (depth_meters <= self.maximum_distance)
        )
        pixel_y, pixel_x = np.nonzero(is_valid)

        if pixel_x.size == 0:
            return np.empty((0, 3), dtype=np.float64)

        z = depth_meters[pixel_y, pixel_x].astype(np.float64)

        if undistorted:
            if not self.undistortion_available:
                raise RuntimeError(
                    "현재 Color distortion model은 OpenCV 방식의 왜곡 제거를 "
                    "지원하지 않습니다. 원본 Color/Depth를 사용하세요."
                )

            fx = float(self.undistorted_camera_matrix[0, 0])
            fy = float(self.undistorted_camera_matrix[1, 1])
            cx = float(self.undistorted_camera_matrix[0, 2])
            cy = float(self.undistorted_camera_matrix[1, 2])

            x = (pixel_x.astype(np.float64) - cx) * z / fx
            y = (pixel_y.astype(np.float64) - cy) * z / fy
            return np.column_stack((x, y, z))

        intrinsics = self.aligned_depth_intrinsics
        if intrinsics.model == rs2.distortion.none:
            x = (
                pixel_x.astype(np.float64) - float(intrinsics.ppx)
            ) * z / float(intrinsics.fx)
            y = (
                pixel_y.astype(np.float64) - float(intrinsics.ppy)
            ) * z / float(intrinsics.fy)
            return np.column_stack((x, y, z))

        points = [
            rs2.rs2_deproject_pixel_to_point(
                intrinsics,
                [float(x), float(y)],
                float(depth),
            )
            for x, y, depth in zip(pixel_x, pixel_y, z)
        ]
        return np.asarray(points, dtype=np.float64)

    def get_undistorted_image(self, color_image: np.ndarray) -> np.ndarray:
        """RealSense factory intrinsics로 Color 영상의 왜곡을 제거한다.

        SAM 마스크로 Depth를 조회할 때는 이 함수만 단독으로 사용하지 말고,
        ``get_undistorted_pair()``로 Color와 Depth를 함께 변환해야 한다.
        """
        return self._undistort_color(color_image)

    def _undistort_color(self, color_image: np.ndarray) -> np.ndarray:
        """이전 코드와 동일한 이름으로 Color 영상의 왜곡을 제거한다."""
        self._require_undistortion_maps()
        return cv2.remap(
            color_image,
            self.undistort_map_x,
            self.undistort_map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

    def get_undistorted_depth(
        self,
        depth_meters: np.ndarray,
    ) -> np.ndarray:
        """Aligned Depth를 Color와 동일한 map으로 왜곡 제거한다."""
        self._require_undistortion_maps()
        return cv2.remap(
            np.asarray(depth_meters, dtype=np.float32),
            self.undistort_map_x,
            self.undistort_map_y,
            interpolation=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )

    def get_undistorted_pair(
        self,
        color_image: np.ndarray,
        depth_meters: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Color와 aligned Depth를 동일한 undistorted 좌표계로 변환한다."""
        return (
            self.get_undistorted_image(color_image),
            self.get_undistorted_depth(depth_meters),
        )

    def _require_undistortion_maps(self) -> None:
        if (
            not self.undistortion_available
            or self.undistort_map_x is None
            or self.undistort_map_y is None
        ):
            raise RuntimeError(
                "이 RealSense Color distortion model은 cv2.remap 방식으로 "
                "바로 처리할 수 없습니다. 원본 Color와 aligned Depth를 "
                "그대로 사용하세요. "
                f"model={self.color_intrinsics.model}"
            )

    def get_depth_cut_mask(
        self,
        mask: np.ndarray,
        depth: np.ndarray,
    ) -> np.ndarray:
        """유효한 Depth가 있는 영역만 남긴 2차원 bool mask를 반환한다."""
        mask_2d = np.squeeze(mask).astype(bool)
        depth_2d = np.asarray(depth)

        if mask_2d.shape != depth_2d.shape:
            raise ValueError(
                "mask와 depth의 크기가 다릅니다: "
                f"mask={mask_2d.shape}, depth={depth_2d.shape}"
            )

        valid_mask = (depth_2d > 0.0).astype(np.uint8)
        kernel = np.ones((5, 5), np.uint8)
        valid_mask = cv2.morphologyEx(
            valid_mask,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=2,
        )
        return mask_2d & (valid_mask > 0)

    def get_calibration(self) -> dict[str, Any]:
        """현재 활성 스트림에서 읽은 보정값을 복사해 반환한다."""
        intrinsics = self.color_intrinsics
        return {
            "width": int(intrinsics.width),
            "height": int(intrinsics.height),
            "fx": float(intrinsics.fx),
            "fy": float(intrinsics.fy),
            "cx": float(intrinsics.ppx),
            "cy": float(intrinsics.ppy),
            "distortion_model": str(intrinsics.model),
            "distortion_coefficients": np.asarray(
                intrinsics.coeffs,
                dtype=np.float64,
            ).copy(),
            "camera_matrix": self.camera_matrix.copy(),
            "depth_scale": self.depth_scale,
        }

    @staticmethod
    def _intrinsics_to_camera_matrix(intrinsics) -> np.ndarray:
        return np.array(
            [
                [intrinsics.fx, 0.0, intrinsics.ppx],
                [0.0, intrinsics.fy, intrinsics.ppy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _intrinsics_to_distortion_coefficients(intrinsics) -> np.ndarray:
        return np.asarray(
            intrinsics.coeffs,
            dtype=np.float64,
        ).reshape(1, -1)

    def get_depth_scale(self) -> float:
        return self.depth_scale

    def stop(self) -> None:
        if self._started:
            self.pipeline.stop()
            self._started = False

    def __enter__(self) -> "RealSenseCamera":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()

    # 기존 클래스 사용부를 바로 깨뜨리지 않기 위한 호환용 이름이다.
    get_frame = get_frames
    get_image = get_color_image
    get_depth = get_depth_meters
    get_depth_none_far = get_near_depth
    calc_intrinsic = depth_to_points
    deactivate = stop


# 기존 클래스 이름을 import하는 다른 파일을 위한 별칭이다.
Realsense_Data = RealSenseCamera


if __name__ == "__main__":
    with RealSenseCamera() as camera:
        calibration = camera.get_calibration()
        print("활성 RealSense 보정값")
        print("해상도:", calibration["width"], "x", calibration["height"])
        print("fx, fy:", calibration["fx"], calibration["fy"])
        print("cx, cy:", calibration["cx"], calibration["cy"])
        print("왜곡 모델:", calibration["distortion_model"])
        print("왜곡계수:", calibration["distortion_coefficients"])
        print("Depth scale:", calibration["depth_scale"])

        frames = camera.get_frames()
        color_image = camera.get_color_image(frames)
        depth_meters = camera.get_depth_meters(frames)

        print("Color shape:", color_image.shape)
        print("Aligned depth shape:", depth_meters.shape)
