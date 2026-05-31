from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from .config import CameraConfig


@dataclass(frozen=True)
class WristFrame:
    color_bgr: np.ndarray
    depth_raw: np.ndarray
    depth_scale_m: float
    color_intrinsics: dict[str, Any]
    timestamp_ms: float
    camera_serial: str


class WristRealSenseCamera:
    """只启动腕部相机，避免固定相机误入 GraspNet 输入链路。"""

    def __init__(self, config: CameraConfig):
        self.config = config
        self._rs = None
        self._pipeline = None
        self._align = None
        self._depth_scale_m: float | None = None
        self._color_intrinsics: dict[str, Any] | None = None

    def start(self) -> None:
        try:
            import pyrealsense2 as rs
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError("未找到 pyrealsense2，请在 hand_eye 环境安装 pyrealsense2。") from exc

        self._rs = rs
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.config.serial)
        config.enable_stream(
            rs.stream.color,
            self.config.color_width,
            self.config.color_height,
            rs.format.bgr8,
            self.config.fps,
        )
        config.enable_stream(
            rs.stream.depth,
            self.config.depth_width,
            self.config.depth_height,
            rs.format.z16,
            self.config.fps,
        )
        profile = pipeline.start(config)
        self._pipeline = pipeline
        self._align = rs.align(rs.stream.color) if self.config.align_depth_to_color else None

        for _ in range(max(0, self.config.warmup_frames)):
            try:
                pipeline.wait_for_frames(1000)
            except RuntimeError:
                pass

        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()
        self._color_intrinsics = _intrinsics_to_dict(intr, rs.rs2_fov(intr))
        depth_sensor = profile.get_device().first_depth_sensor()
        self._depth_scale_m = float(depth_sensor.get_depth_scale())
        print(
            f"[camera] 腕部相机已启动 serial={self.config.serial}, "
            f"color={self.config.color_width}x{self.config.color_height}, "
            f"depth_scale_m={self._depth_scale_m:.9f}"
        )

    def capture(self) -> WristFrame:
        if self._pipeline is None or self._color_intrinsics is None or self._depth_scale_m is None:
            raise RuntimeError("腕部相机尚未启动")
        frames = self._pipeline.wait_for_frames(int(self.config.timeout_ms))
        if self._align is not None:
            frames = self._align.process(frames)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame:
            raise RuntimeError("腕部相机未获取到 color frame")
        if not depth_frame:
            raise RuntimeError("腕部相机未获取到 depth frame")
        return WristFrame(
            color_bgr=np.asanyarray(color_frame.get_data()).copy(),
            depth_raw=np.asanyarray(depth_frame.get_data()).copy(),
            depth_scale_m=float(self._depth_scale_m),
            color_intrinsics=self._color_intrinsics,
            timestamp_ms=float(getattr(color_frame, "get_timestamp", lambda: time.time() * 1000.0)()),
            camera_serial=self.config.serial,
        )

    def stop(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        self._pipeline = None

    def __enter__(self) -> "WristRealSenseCamera":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()


def _intrinsics_to_dict(intr: Any, fov: Any) -> dict[str, Any]:
    return {
        "width": int(intr.width),
        "height": int(intr.height),
        "ppx": float(intr.ppx),
        "ppy": float(intr.ppy),
        "fx": float(intr.fx),
        "fy": float(intr.fy),
        "model": str(intr.model),
        "coeffs": [float(v) for v in intr.coeffs],
        "fov": [float(v) for v in fov],
        "K": [
            [float(intr.fx), 0.0, float(intr.ppx)],
            [0.0, float(intr.fy), float(intr.ppy)],
            [0.0, 0.0, 1.0],
        ],
    }
