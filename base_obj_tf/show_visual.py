"""물체 검출 결과를 OpenCV 창으로 보여준다."""

import cv2
import numpy as np


WINDOW_NAME = "Object pose detection"
MASK_COLOR = np.array([0, 255, 0], dtype=np.uint8)
BASKET_MASK_COLOR = np.array([255, 0, 255], dtype=np.uint8)
MASK_OPACITY = 0.35
AXIS_LENGTH_METERS = 0.05


class ObjectPoseVisualizer:
    """컬러 영상 위에 검출 mask와 물체 좌표축을 그린다."""

    def __init__(
        self,
        camera_matrix: np.ndarray,
        distortion_coefficients: np.ndarray | None = None,
    ) -> None:
        self.camera_matrix = camera_matrix
        if distortion_coefficients is None:
            distortion_coefficients = np.zeros((1, 5), dtype=np.float64)
        self.distortion_coefficients = distortion_coefficients
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    def show(
        self,
        color_image: np.ndarray,
        object_masks,
        camera_from_objects,
        basket_masks=None,
    ) -> bool:
        """결과 창을 갱신한다. q 또는 ESC를 누르면 False를 반환한다."""
        visual = color_image.copy()
        object_masks = self._as_mask_list(object_masks)
        camera_from_objects = self._as_list(camera_from_objects)
        basket_masks = self._as_mask_list(basket_masks)

        for object_mask in object_masks:
            self._draw_mask(visual, object_mask, MASK_COLOR)

        if not camera_from_objects:
            cv2.putText(
                visual,
                "Object pose not detected",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            for camera_from_object in camera_from_objects:
                self._draw_object_axes(visual, camera_from_object)

        for basket_mask in basket_masks:
            self._draw_mask(visual, basket_mask, BASKET_MASK_COLOR)

        cv2.imshow(WINDOW_NAME, visual)
        key = cv2.waitKey(1) & 0xFF
        window_is_open = cv2.getWindowProperty(
            WINDOW_NAME,
            cv2.WND_PROP_VISIBLE,
        ) >= 1
        return window_is_open and key not in (ord("q"), 27)

    @staticmethod
    def _as_list(value):
        """단일 값과 리스트 입력을 같은 방식으로 처리한다."""
        if value is None:
            return []
        if isinstance(value, list):
            return [item for item in value if item is not None]
        return [value]

    @staticmethod
    def _as_mask_list(value):
        """2D mask 1개와 3D mask 묶음을 모두 리스트로 바꾼다."""
        if value is None:
            return []
        if isinstance(value, list):
            return [item for item in value if item is not None]

        masks = np.asarray(value)
        masks = np.squeeze(masks)

        if masks.ndim == 2:
            return [masks]
        if masks.ndim == 3:
            return [mask for mask in masks]

        return []

    @staticmethod
    def _draw_mask(
        image: np.ndarray,
        object_mask: np.ndarray,
        color: np.ndarray,
    ) -> None:
        """검출된 물체 영역을 반투명 초록색으로 표시한다."""
        mask = np.squeeze(object_mask).astype(bool)
        if mask.shape != image.shape[:2]:
            return

        image[mask] = (
            image[mask] * (1.0 - MASK_OPACITY)
            + color * MASK_OPACITY
        ).astype(np.uint8)

    def _draw_object_axes(
        self,
        image: np.ndarray,
        camera_from_object: np.ndarray,
    ) -> None:
        """계산된 물체 위치에 x, y, z 좌표축을 그린다."""
        rotation_vector, _ = cv2.Rodrigues(camera_from_object[:3, :3])
        translation_vector = camera_from_object[:3, 3]
        cv2.drawFrameAxes(
            image,
            self.camera_matrix,
            self.distortion_coefficients,
            rotation_vector,
            translation_vector,
            AXIS_LENGTH_METERS,
            2,
        )

    def close(self) -> None:
        """시각화 창을 닫는다."""
        try:
            cv2.destroyWindow(WINDOW_NAME)
        except cv2.error:
            # 사용자가 창의 X 버튼으로 먼저 닫은 경우다.
            pass
