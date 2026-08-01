"""Configurable contrast enhancement and binarization for the ground target."""

from dataclasses import dataclass
from typing import Dict

import cv2
import numpy as np


@dataclass
class ImageEnhancementConfig:
    enhancement_mode: str = "clahe_adaptive"
    clahe_clip_limit: float = 2.5
    clahe_tile_grid_size: int = 8
    # The larger blur/block and one light opening are deliberate.  The first
    # live D435 trial with 5/31/C=5 produced thousands of tiny floor-texture
    # components and reduced the node below 1 FPS.
    enhancement_blur_kernel: int = 9
    adaptive_block_size: int = 51
    adaptive_c: float = 9.0
    binary_morph_kernel: int = 3
    binary_close_iterations: int = 0
    binary_open_iterations: int = 1
    fallback_to_clahe: bool = True

    @classmethod
    def from_dict(cls, values: Dict) -> "ImageEnhancementConfig":
        known = {field_info.name for field_info in cls.__dataclass_fields__.values()}
        return cls(**{key: value for key, value in values.items() if key in known})

    def validate(self):
        self.enhancement_mode = str(self.enhancement_mode).strip().lower()
        if self.enhancement_mode not in ("clahe", "otsu", "clahe_adaptive"):
            raise ValueError(
                "enhancement_mode must be clahe, otsu, or clahe_adaptive"
            )
        if self.clahe_clip_limit <= 0.0:
            raise ValueError("clahe_clip_limit must be positive")
        if self.clahe_tile_grid_size < 2:
            raise ValueError("clahe_tile_grid_size must be at least two")
        if self.enhancement_blur_kernel < 1:
            raise ValueError("enhancement_blur_kernel must be positive")
        if self.enhancement_blur_kernel % 2 == 0:
            self.enhancement_blur_kernel += 1
        if self.adaptive_block_size < 3:
            raise ValueError("adaptive_block_size must be at least three")
        if self.adaptive_block_size % 2 == 0:
            self.adaptive_block_size += 1
        if self.binary_morph_kernel < 1:
            raise ValueError("binary_morph_kernel must be positive")
        if self.binary_morph_kernel % 2 == 0:
            self.binary_morph_kernel += 1
        if self.binary_close_iterations < 0 or self.binary_open_iterations < 0:
            raise ValueError("binary morphology iterations cannot be negative")


class ImageEnhancer:
    """Produce an enhanced BGR frame usable by the existing CV backend and UI."""

    def __init__(self, config=None):
        self.config = config or ImageEnhancementConfig()
        self.config.validate()
        grid = (self.config.clahe_tile_grid_size,) * 2
        self.clahe = cv2.createCLAHE(
            clipLimit=self.config.clahe_clip_limit,
            tileGridSize=grid,
        )

    @staticmethod
    def _check_image(image_bgr):
        if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("ImageEnhancer expects one BGR image")

    def _clahe_gray(self, image_bgr):
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        return self.clahe.apply(gray)

    def _clean_binary(self, binary):
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.config.binary_morph_kernel, self.config.binary_morph_kernel),
        )
        if self.config.binary_close_iterations:
            binary = cv2.morphologyEx(
                binary,
                cv2.MORPH_CLOSE,
                kernel,
                iterations=self.config.binary_close_iterations,
            )
        if self.config.binary_open_iterations:
            binary = cv2.morphologyEx(
                binary,
                cv2.MORPH_OPEN,
                kernel,
                iterations=self.config.binary_open_iterations,
            )
        return binary

    def enhance(self, image_bgr, mode=None):
        self._check_image(image_bgr)
        selected_mode = self.config.enhancement_mode if mode is None else str(mode).lower()
        if selected_mode not in ("clahe", "otsu", "clahe_adaptive"):
            raise ValueError("unsupported enhancement mode {}".format(selected_mode))

        enhanced_gray = self._clahe_gray(image_bgr)
        if selected_mode == "clahe":
            output_gray = enhanced_gray
        else:
            blurred = cv2.GaussianBlur(
                enhanced_gray,
                (self.config.enhancement_blur_kernel,) * 2,
                0,
            )
            if selected_mode == "otsu":
                _, output_gray = cv2.threshold(
                    blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
                )
            else:
                output_gray = cv2.adaptiveThreshold(
                    blurred,
                    255,
                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    cv2.THRESH_BINARY,
                    self.config.adaptive_block_size,
                    self.config.adaptive_c,
                )
            output_gray = self._clean_binary(output_gray)
        return cv2.cvtColor(output_gray, cv2.COLOR_GRAY2BGR)
