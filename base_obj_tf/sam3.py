import torch
import cv2
import numpy as np
from transformers import Sam3VideoModel, Sam3VideoProcessor

class Sam3_Node():
    def __init__(self, obj_name="block", score_threshold = 0.82, mask_threshold = 0.75):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.dtype = torch.bfloat16 if self.device.startswith("cuda") else torch.float32

        self.model = Sam3VideoModel.from_pretrained("facebook/sam3").to(self.device, self.dtype) 
        self.processor = Sam3VideoProcessor.from_pretrained("facebook/sam3")
        self.obj_name = obj_name

        self.mask_colors = [
            (255, 99, 71),
            (60, 179, 113),
            (30, 144, 255),
            (255, 175, 0),
            (186, 85, 211),
            (0, 206, 209),
        ]
        self.score_threshold = score_threshold
        self.mask_threshold = mask_threshold

    def change_obj_name(self, obj_name):
        self.obj_name = obj_name

    def _create_inference_session(self):
        """
        중요:
        이미지 한 장마다 새로운 session을 만든다.
        그래야 video memory가 계속 쌓이지 않는다.
        """
        inference_session = self.processor.init_video_session(
            inference_device=self.device,
            processing_device="cpu",
            video_storage_device="cpu",
            dtype=self.dtype,
        )

        inference_session = self.processor.add_text_prompt(
            inference_session=inference_session,
            text=self.obj_name,
        )

        return inference_session

    def _transfer_img_tensor(self, image_rgb):
        inputs = self.processor(
            images=image_rgb,
            device=self.device,
            return_tensors="pt",
        )
        return inputs

    def obj_segmentation(self, image_bgr):
        """
        입력:
            image_bgr: cv2.imread로 읽은 이미지

        출력:
            성공: masks, show_img_bgr
            실패: None
        """

        # OpenCV는 BGR, SAM3는 RGB 기준으로 보는 게 안전함
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

        inference_session = self._create_inference_session()
        inputs = self._transfer_img_tensor(image_rgb)

        try:
            with torch.inference_mode():
                model_outputs = self.model(
                    inference_session=inference_session,
                    frame=inputs.pixel_values[0],
                    reverse=False,
                )

            frame_outputs = self.processor.postprocess_outputs(
                inference_session,
                model_outputs,
                original_sizes=inputs.original_sizes,
            )

            if len(frame_outputs["object_ids"]) == 0:
                return None

            masks = frame_outputs["masks"]
            scores = frame_outputs.get("scores", None)

            # 1. score_threshold 적용
            if scores is not None:
                if not torch.is_tensor(scores):
                    scores = torch.tensor(scores, device=masks.device)

                keep = scores >= self.score_threshold

                if keep.sum().item() == 0:
                    return None

                masks = masks[keep]

            # 2. GPU -> CPU
            masks = masks.detach().float().cpu()

            # 3. shape 정리: (N, 1, H, W) -> (N, H, W)
            if masks.ndim == 4:
                masks = masks.squeeze(1)

            # 4. mask 값이 logit이면 sigmoid 적용
            if masks.min() < 0 or masks.max() > 1:
                masks = torch.sigmoid(masks)

            # 5. mask_threshold 적용
            masks = (masks > self.mask_threshold).numpy().astype(np.uint8)

            # 6. 빈 mask 제거
            valid_masks = []

            for mask in masks:
                if mask.sum() > 0:
                    valid_masks.append(mask)

            if len(valid_masks) == 0:
                return None

            masks = np.stack(valid_masks, axis=0)

            show_img_rgb = self._draw_segmentation(image_rgb, masks)

            # cv2.imshow용으로 다시 BGR 변환
            show_img_bgr = cv2.cvtColor(show_img_rgb, cv2.COLOR_RGB2BGR)

            return masks, show_img_bgr

        finally:
            # 중요: 매 이미지 추론 후 메모리 정리
            del inputs
            if "model_outputs" in locals():
                del model_outputs
            if "frame_outputs" in locals():
                del frame_outputs
            del inference_session

            if self.device == "cuda":
                torch.cuda.empty_cache()

    def _draw_segmentation(self, image_rgb, masks, alpha=0.4):
        if masks is None or len(masks) == 0:
            return image_rgb.copy()

        draw_image = image_rgb.copy()
        masks = np.asarray(masks)

        if masks.ndim == 2:
            masks = masks[np.newaxis, :, :]

        for idx, mask in enumerate(masks):
            mask = mask > 0

            if mask.shape[:2] != draw_image.shape[:2]:
                mask = cv2.resize(
                    mask.astype(np.uint8),
                    (draw_image.shape[1], draw_image.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                ).astype(bool)

            color = np.array(
                self.mask_colors[idx % len(self.mask_colors)],
                dtype=np.uint8,
            )

            draw_image[mask] = (
                (1 - alpha) * draw_image[mask] + alpha * color
            ).astype(np.uint8)

        return draw_image
