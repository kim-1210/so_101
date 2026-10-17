"""RF-DETR의 특징을 이용해 물체 mask 한 장을 만드는 모델이다."""

from collections.abc import Mapping

import torch
import torch.nn as nn
import torch.nn.functional as F
from transformers import RfDetrForInstanceSegmentation


PRETRAINED_MODEL_NAME = "stevenbucaille/rf-detr-seg-medium"
BACKBONE_CHANNELS = 256


class CubeRFDetr(nn.Module):

    def __init__(self, backbone=None, width: int = 640, height: int = 480):
        super().__init__()
        self.output_size = (height, width)
        if backbone is None:
            backbone = self._load_pretrained_backbone()
        self.backbone = backbone

        self.head = self._build_segmentation_head()

    @staticmethod
    def _load_pretrained_backbone():
        pretrained_model = RfDetrForInstanceSegmentation.from_pretrained(
            PRETRAINED_MODEL_NAME
        )
        return pretrained_model.model.model.backbone

    @staticmethod
    def _build_segmentation_head() -> nn.Sequential:
        return nn.Sequential(
            nn.Conv2d(
                BACKBONE_CHANNELS,
                128,
                kernel_size=3,
                padding=1,
                bias=False,
            ),
            nn.BatchNorm2d(128),
            nn.SiLU(inplace=True),
            nn.Conv2d(128, 64, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(64),
            nn.SiLU(inplace=True),
            nn.Conv2d(64, 1, kernel_size=1),
        )

    def forward(
        self, 
        pixel_values=None, 
        pixel_mask=None, 
        **_unused_kwargs
    ):
        if isinstance(pixel_values, Mapping):
            model_inputs = pixel_values
            pixel_values = model_inputs["pixel_values"]
            pixel_mask = model_inputs.get("pixel_mask")

        if pixel_mask is None:
            pixel_mask = self._make_full_pixel_mask(pixel_values)

        feature_map, _ = self.backbone(
            pixel_values=pixel_values,
            pixel_mask=pixel_mask,
        )
        low_resolution_logits = self.head(feature_map)

        return F.interpolate(
            low_resolution_logits,
            size=self.output_size,
            mode="bilinear",
            align_corners=False,
        )

    @staticmethod
    def _make_full_pixel_mask(pixel_values: torch.Tensor) -> torch.Tensor:
        batch_size, _, height, width = pixel_values.shape
        return torch.ones(
            batch_size,
            height,
            width,
            dtype=torch.long,
            device=pixel_values.device,
        )
