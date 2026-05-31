from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

import numpy as np

from .array_codec import encode_npy
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
        if self.config.request_format == "json_npy":
            payload = self._build_json_npy_payload(
                color_bgr=color_bgr,
                depth_raw=depth_raw,
                workspace_mask=workspace_mask,
                intrinsics=intrinsics,
                depth_scale_m=depth_scale_m,
                seed=seed,
            )
            response_payload = self._post_json(payload)
            best_grasp = parse_best_grasp(response_payload)
            return GraspNetResponse(best_grasp=best_grasp, raw=response_payload)
        if self.config.request_format == "multipart":
            response_payload = self._post_multipart(
                color_bgr=color_bgr,
                depth_raw=depth_raw,
                workspace_mask=workspace_mask,
                intrinsics=intrinsics,
                depth_scale_m=depth_scale_m,
                seed=seed,
            )
            best_grasp = parse_best_grasp(response_payload)
            return GraspNetResponse(best_grasp=best_grasp, raw=response_payload)
        raise ValueError(f"未知 request_format={self.config.request_format!r}，可选 json_npy/multipart")

    def _build_json_npy_payload(
        self,
        *,
        color_bgr: np.ndarray,
        depth_raw: np.ndarray,
        workspace_mask: np.ndarray,
        intrinsics: dict[str, Any],
        depth_scale_m: float,
        seed: int | None,
    ) -> dict[str, Any]:
        factor_depth = 1.0 / float(depth_scale_m)
        color_rgb = np.asarray(color_bgr, dtype=np.uint8)[..., ::-1]
        payload: dict[str, Any] = {
            "color_rgb": encode_npy(color_rgb),
            "depth": encode_npy(np.asarray(depth_raw)),
            "intrinsic_matrix": encode_npy(_intrinsic_matrix(intrinsics)),
            self.config.factor_depth_field: float(factor_depth),
            "top_k": int(self.config.top_k),
        }
        if workspace_mask is not None:
            payload[self.config.mask_field] = encode_npy(np.asarray(workspace_mask, dtype=np.uint8))
        if seed is not None:
            payload[self.config.seed_field] = int(seed)
        for key, value in self.config.extra_fields.items():
            payload[str(key)] = value
        return payload

    def _post_json(self, payload: dict[str, Any]) -> dict[str, Any]:
        try:
            import requests
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError("未找到 requests，请安装 requests。") from exc

        response = requests.post(
            self.config.url,
            json=payload,
            timeout=float(self.config.timeout_s),
        )
        return _json_response_or_error(response)

    def _post_multipart(
        self,
        *,
        color_bgr: np.ndarray,
        depth_raw: np.ndarray,
        workspace_mask: np.ndarray,
        intrinsics: dict[str, Any],
        depth_scale_m: float,
        seed: int | None,
    ) -> dict[str, Any]:
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
        return _json_response_or_error(response)


def _intrinsic_matrix(intrinsics: dict[str, Any]) -> np.ndarray:
    if "K" in intrinsics:
        matrix = np.asarray(intrinsics["K"], dtype=np.float32)
    else:
        matrix = np.asarray(
            [
                [float(intrinsics["fx"]), 0.0, float(intrinsics["ppx"])],
                [0.0, float(intrinsics["fy"]), float(intrinsics["ppy"])],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
    if matrix.shape != (3, 3):
        raise ValueError(f"相机内参矩阵应为 3x3，实际 shape={matrix.shape}")
    return matrix


def _json_response_or_error(response: Any) -> dict[str, Any]:
    text = response.text
    try:
        payload = response.json()
    except ValueError:
        payload = None
    if response.status_code >= 400:
        detail = payload if payload is not None else text
        raise RuntimeError(f"GraspNet 服务端 HTTP {response.status_code}: {detail}")
    if payload is None:
        raise RuntimeError(f"GraspNet 服务端返回的不是 JSON: {text[:500]}")
    if isinstance(payload, dict) and "error" in payload:
        raise RuntimeError(f"GraspNet 服务端错误: {payload['error']}")
    if not isinstance(payload, dict):
        raise RuntimeError(f"GraspNet 服务端 JSON 顶层应为 dict，实际 {type(payload).__name__}")
    return payload
