from __future__ import annotations

import numpy as np

from .config import WorkspaceMaskConfig


def center_rect_bounds(width: int, height: int, width_fraction: float, height_fraction: float) -> tuple[int, int, int, int]:
    if not (0.0 < width_fraction <= 1.0):
        raise ValueError(f"width_fraction 必须在 (0,1]，实际 {width_fraction}")
    if not (0.0 < height_fraction <= 1.0):
        raise ValueError(f"height_fraction 必须在 (0,1]，实际 {height_fraction}")
    rect_w = max(1, int(round(width * width_fraction)))
    rect_h = max(1, int(round(height * height_fraction)))
    x0 = (width - rect_w) // 2
    y0 = (height - rect_h) // 2
    return x0, y0, x0 + rect_w, y0 + rect_h


def make_center_mask(width: int, height: int, config: WorkspaceMaskConfig) -> np.ndarray:
    if config.type != "center_fraction":
        raise ValueError(f"当前只支持 center_fraction 掩码，实际 type={config.type!r}")
    mask = np.zeros((height, width), dtype=np.uint8)
    x0, y0, x1, y1 = center_rect_bounds(width, height, config.width_fraction, config.height_fraction)
    mask[y0:y1, x0:x1] = 255
    return mask


def apply_depth_mask(depth: np.ndarray, mask: np.ndarray, masked_depth_value: int = 0) -> np.ndarray:
    if depth.shape[:2] != mask.shape:
        raise ValueError(f"depth 和 mask 尺寸不一致: depth={depth.shape}, mask={mask.shape}")
    masked = depth.copy()
    masked[mask == 0] = masked_depth_value
    return masked


def apply_color_mask(color_bgr: np.ndarray, mask: np.ndarray, masked_color: tuple[int, int, int] = (0, 0, 0)) -> np.ndarray:
    if color_bgr.shape[:2] != mask.shape:
        raise ValueError(f"color 和 mask 尺寸不一致: color={color_bgr.shape}, mask={mask.shape}")
    masked = color_bgr.copy()
    masked[mask == 0] = np.asarray(masked_color, dtype=np.uint8)
    return masked


def apply_color_mask_for_preview(color_bgr: np.ndarray, mask: np.ndarray, dim_outside: bool = True) -> np.ndarray:
    if color_bgr.shape[:2] != mask.shape:
        raise ValueError(f"color 和 mask 尺寸不一致: color={color_bgr.shape}, mask={mask.shape}")
    preview = color_bgr.copy()
    if dim_outside:
        preview[mask == 0] = (preview[mask == 0] * 0.25).astype(np.uint8)
    else:
        preview[mask == 0] = 0
    return preview
