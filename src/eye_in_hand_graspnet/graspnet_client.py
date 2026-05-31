from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

import numpy as np

from .config import GraspNetConfig
from .transforms import BestGrasp, parse_best_grasp


@dataclass(frozen=True)
class GraspNetResponse:
    best_grasp: BestGrasp
    raw: dict[str, Any]


class GraspNetHttpClient:
    """通过 HTTP 调用 GraspNet-baseline 服务，避免本工程依赖旧复现仓库代码。"""

    def __init__(self, config: GraspNetConfig):
        self.config = config

    def infer(
        self,
        *,
        color_bgr: np.ndarray,
        depth_raw: np.ndarray,
        workspace_mask: np.ndarray,
        intrinsics: dict[str, Any],
        depth_scale_m: float,
        seed: int | None,
    ) -> GraspNetResponse:
        if self.config.request_format != "multipart":
            raise ValueError(f"当前只实现 multipart 请求，实际 request_format={self.config.request_format!r}")

        try:
            import cv2
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError("未找到 cv2，请安装 opencv-python。") from exc
        try:
            import requests
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError("未找到 requests，请安装 requests。") from exc

        color_ok, color_png = cv2.imencode(".png", color_bgr)
        depth_ok, depth_png = cv2.imencode(".png", depth_raw)
        mask_ok, mask_png = cv2.imencode(".png", workspace_mask)
        if not color_ok or not depth_ok or not mask_ok:
            raise RuntimeError("编码 GraspNet 请求图像失败")

        files = {
            self.config.color_field: ("color.png", color_png.tobytes(), "image/png"),
            self.config.depth_field: ("depth.png", depth_png.tobytes(), "image/png"),
            self.config.mask_field: ("workspace_mask.png", mask_png.tobytes(), "image/png"),
        }
        data: dict[str, str] = {
            self.config.intrinsics_field: json.dumps(intrinsics, ensure_ascii=False),
            "depth_scale_m": repr(float(depth_scale_m)),
            self.config.factor_depth_field: repr(1.0 / float(depth_scale_m)),
        }
        if seed is not None:
            data[self.config.seed_field] = str(int(seed))
        for key, value in self.config.extra_fields.items():
            data[str(key)] = json.dumps(value, ensure_ascii=False) if isinstance(value, (dict, list)) else str(value)

        response = requests.post(
            self.config.url,
            data=data,
            files=files,
            timeout=float(self.config.timeout_s),
        )
        response.raise_for_status()
        payload = response.json()
        best_grasp = parse_best_grasp(payload)
        return GraspNetResponse(best_grasp=best_grasp, raw=payload)
