#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from queue import Empty, Full, Queue
import sys
from threading import Event, Thread
import time
from pathlib import Path
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from eye_in_hand_graspnet.camera import WristRealSenseCamera
from eye_in_hand_graspnet.config import (
    CameraConfig,
    GraspNetConfig,
    PreviewConfig,
    WorkspaceMaskConfig,
    load_json,
    resolve_path,
)
from eye_in_hand_graspnet.graspnet_client import GraspNetHttpClient
from eye_in_hand_graspnet.mask import (
    apply_color_mask,
    apply_color_mask_for_preview,
    apply_depth_mask,
    make_center_mask,
)
from eye_in_hand_graspnet.preview import build_wrist_preview_canvas
from eye_in_hand_graspnet.transforms import BestGrasp


@dataclass(frozen=True)
class RealtimeConfig:
    infer_every: int = 1
    max_frames: int = 0
    save_every: int = 0
    window_name: str | None = None


@dataclass(frozen=True)
class RealtimePreviewConfig:
    random_seed: int | None
    random_seed_fixed: bool
    camera: CameraConfig
    workspace_mask: WorkspaceMaskConfig
    graspnet: GraspNetConfig
    preview: PreviewConfig
    realtime: RealtimeConfig

    @property
    def effective_random_seed(self) -> int | None:
        if not self.random_seed_fixed or self.random_seed is None:
            return None
        return int(self.random_seed)


@dataclass(frozen=True)
class InferenceRequest:
    frame_index: int
    color_bgr: np.ndarray
    depth_raw: np.ndarray
    workspace_mask: np.ndarray
    intrinsics: dict[str, Any]
    depth_scale_m: float
    seed: int | None


@dataclass(frozen=True)
class InferenceResult:
    frame_index: int
    best_grasp: BestGrasp | None
    error: str | None
    infer_ms: float


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="实时预览腕部相机 GraspNet 抓取结果。")
    parser.add_argument(
        "--config",
        default="configs/wrist_graspnet_realtime_preview.jsonc",
        help="独立实时预览配置文件路径，默认 configs/wrist_graspnet_realtime_preview.jsonc",
    )
    parser.add_argument(
        "--infer-every",
        type=int,
        default=None,
        help="覆盖配置 realtime.infer_every；每隔多少帧提交一次 GraspNet 推理。",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help="覆盖配置 realtime.max_frames；0 表示一直运行到按 q/Esc。",
    )
    parser.add_argument(
        "--save-every",
        type=int,
        default=None,
        help="覆盖配置 realtime.save_every；0 表示只显示不周期保存。",
    )
    parser.add_argument(
        "--window-name",
        default=None,
        help="覆盖配置 realtime.window_name 或 preview.window_name。",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    config = load_realtime_preview_config(args.config)
    infer_every = int(args.infer_every if args.infer_every is not None else config.realtime.infer_every)
    max_frames = int(args.max_frames if args.max_frames is not None else config.realtime.max_frames)
    save_every = int(args.save_every if args.save_every is not None else config.realtime.save_every)
    if infer_every <= 0:
        raise ValueError("--infer-every 必须为正整数")
    if max_frames < 0:
        raise ValueError("--max-frames 必须 >= 0")
    if save_every < 0:
        raise ValueError("--save-every 必须 >= 0")

    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError("实时预览需要 opencv-python，请先安装 cv2。") from exc

    effective_seed = config.effective_random_seed
    if effective_seed is not None:
        np.random.seed(int(effective_seed))

    client = GraspNetHttpClient(config.graspnet)
    window_name = args.window_name or config.realtime.window_name or f"{config.preview.window_name} realtime"
    last_grasp: BestGrasp | None = None
    last_error: str | None = None
    last_infer_frame: int | None = None
    last_score: float | None = None
    frame_index = 0
    infer_count = 0
    inference_busy = False
    request_queue: Queue[InferenceRequest | None] = Queue(maxsize=1)
    result_queue: Queue[InferenceResult] = Queue(maxsize=1)
    stop_event = Event()
    worker = Thread(
        target=_infer_worker,
        args=(client, request_queue, result_queue, stop_event),
        name="wrist-graspnet-preview-infer",
        daemon=True,
    )
    worker.start()

    print("[preview] 实时腕部 GraspNet 预览启动。按 q 或 Esc 退出。")
    print(
        f"[preview] config={resolve_path(args.config)}, camera.serial={config.camera.serial}, infer_every={infer_every}, "
        f"seed_effective={effective_seed}"
    )

    try:
        with WristRealSenseCamera(config.camera) as camera:
            while True:
                loop_t0 = time.perf_counter()
                frame = camera.capture()
                frame_index += 1

                mask = make_center_mask(
                    frame.color_bgr.shape[1],
                    frame.color_bgr.shape[0],
                    config.workspace_mask,
                )
                masked_depth = apply_depth_mask(
                    frame.depth_raw,
                    mask,
                    config.workspace_mask.masked_depth_value,
                )
                masked_color = apply_color_mask(frame.color_bgr, mask)
                preview_color = apply_color_mask_for_preview(
                    frame.color_bgr,
                    mask,
                    config.workspace_mask.preview_dim_outside,
                )

                result = _get_latest_result(result_queue)
                if result is not None:
                    inference_busy = False
                    infer_count += 1
                    last_infer_frame = result.frame_index
                    if result.error is None and result.best_grasp is not None:
                        last_grasp = result.best_grasp
                        last_error = None
                        last_score = result.best_grasp.score
                        print(
                            f"[preview] frame={result.frame_index}, infer={infer_count}, "
                            f"score={_format_optional(result.best_grasp.score)}, "
                            f"xyz={_format_xyz(result.best_grasp.translation)}, infer_ms={result.infer_ms:.1f}"
                        )
                    else:
                        last_error = result.error or "未知 GraspNet 推理错误"
                        print(f"[preview] GraspNet 推理失败 frame={result.frame_index}: {last_error}")

                if not inference_busy and (frame_index - 1) % infer_every == 0:
                    request = InferenceRequest(
                        frame_index=frame_index,
                        color_bgr=masked_color,
                        depth_raw=masked_depth,
                        workspace_mask=mask,
                        intrinsics=frame.color_intrinsics,
                        depth_scale_m=frame.depth_scale_m,
                        seed=effective_seed,
                    )
                    if _put_latest(request_queue, request):
                        inference_busy = True

                canvas = build_wrist_preview_canvas(
                    cv2,
                    color_bgr=preview_color,
                    depth_raw=masked_depth,
                    depth_scale_m=frame.depth_scale_m,
                    mask=mask,
                    best_grasp=last_grasp,
                    intrinsics=frame.color_intrinsics,
                    show_depth=config.preview.show_depth,
                    depth_min_m=config.preview.depth_min_m,
                    depth_max_m=config.preview.depth_max_m,
                )
                _draw_status(
                    cv2,
                    canvas,
                    frame_index=frame_index,
                    infer_count=infer_count,
                    infer_every=infer_every,
                    inference_busy=inference_busy,
                    last_error=last_error,
                    last_infer_frame=last_infer_frame,
                    last_score=last_score,
                    elapsed_ms=(time.perf_counter() - loop_t0) * 1000.0,
                )

                if save_every > 0 and frame_index % save_every == 0 and config.preview.save_path is not None:
                    config.preview.save_path.parent.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(config.preview.save_path), canvas)

                display = canvas
                if config.preview.scale != 1.0:
                    display = cv2.resize(
                        display,
                        None,
                        fx=float(config.preview.scale),
                        fy=float(config.preview.scale),
                        interpolation=cv2.INTER_AREA,
                    )
                cv2.imshow(window_name, display)
                key = cv2.waitKey(max(1, int(config.preview.wait_ms))) & 0xFF
                if key in (ord("q"), 27):
                    print("[preview] 用户退出。")
                    break
                if max_frames > 0 and frame_index >= max_frames:
                    print(f"[preview] 达到 max_frames={max_frames}，退出。")
                    break
    finally:
        stop_event.set()
        _put_latest(request_queue, None)
        worker.join(timeout=0.5)

    try:
        cv2.destroyWindow(window_name)
    except Exception:
        pass
    return 0


def load_realtime_preview_config(path: str | Path) -> RealtimePreviewConfig:
    data = load_json(resolve_path(path))
    preview_data = data.get("preview", {})
    realtime_data = data.get("realtime", {})
    return RealtimePreviewConfig(
        random_seed=data.get("random_seed"),
        random_seed_fixed=bool(data.get("random_seed_fixed", data.get("random_seed") is not None)),
        camera=CameraConfig(**data["camera"]),
        workspace_mask=WorkspaceMaskConfig(**data.get("workspace_mask", {})),
        graspnet=GraspNetConfig(**data["graspnet"]),
        preview=PreviewConfig(
            enabled=bool(preview_data.get("enabled", True)),
            window_name=str(preview_data.get("window_name", "wrist GraspNet realtime preview")),
            scale=float(preview_data.get("scale", 0.75)),
            wait_ms=int(preview_data.get("wait_ms", 1)),
            save_path=resolve_path(preview_data["save_path"]) if preview_data.get("save_path") else None,
            show_depth=bool(preview_data.get("show_depth", True)),
            depth_min_m=float(preview_data.get("depth_min_m", 0.0)),
            depth_max_m=float(preview_data.get("depth_max_m", 1.5)),
        ),
        realtime=RealtimeConfig(
            infer_every=int(realtime_data.get("infer_every", 1)),
            max_frames=int(realtime_data.get("max_frames", 0)),
            save_every=int(realtime_data.get("save_every", 0)),
            window_name=realtime_data.get("window_name"),
        ),
    )


def _infer_worker(
    client: GraspNetHttpClient,
    request_queue: Queue[InferenceRequest | None],
    result_queue: Queue[InferenceResult],
    stop_event: Event,
) -> None:
    while not stop_event.is_set():
        try:
            request = request_queue.get(timeout=0.05)
        except Empty:
            continue
        if request is None:
            request_queue.task_done()
            break

        infer_t0 = time.perf_counter()
        try:
            response = client.infer(
                color_bgr=request.color_bgr,
                depth_raw=request.depth_raw,
                workspace_mask=request.workspace_mask,
                intrinsics=request.intrinsics,
                depth_scale_m=request.depth_scale_m,
                seed=request.seed,
            )
            result = InferenceResult(
                frame_index=request.frame_index,
                best_grasp=response.best_grasp,
                error=None,
                infer_ms=(time.perf_counter() - infer_t0) * 1000.0,
            )
        except Exception as exc:
            result = InferenceResult(
                frame_index=request.frame_index,
                best_grasp=None,
                error=f"{type(exc).__name__}: {exc}",
                infer_ms=(time.perf_counter() - infer_t0) * 1000.0,
            )
        _put_latest(result_queue, result)
        request_queue.task_done()


def _get_latest_result(result_queue: Queue[InferenceResult]) -> InferenceResult | None:
    latest: InferenceResult | None = None
    while True:
        try:
            latest = result_queue.get_nowait()
            result_queue.task_done()
        except Empty:
            return latest


def _put_latest(queue: Queue[Any], item: Any) -> bool:
    try:
        queue.put_nowait(item)
        return True
    except Full:
        try:
            queue.get_nowait()
            queue.task_done()
        except Empty:
            pass
        try:
            queue.put_nowait(item)
            return True
        except Full:
            return False


def _draw_status(
    cv2,
    canvas: np.ndarray,
    *,
    frame_index: int,
    infer_count: int,
    infer_every: int,
    inference_busy: bool,
    last_error: str | None,
    last_infer_frame: int | None,
    last_score: float | None,
    elapsed_ms: float,
) -> None:
    if last_error:
        text = f"frame {frame_index} infer {infer_count} error: {last_error[:90]}"
        color = (0, 0, 255)
    else:
        busy = "busy" if inference_busy else "idle"
        score = _format_optional(last_score)
        last_frame = "n/a" if last_infer_frame is None else str(last_infer_frame)
        text = (
            f"frame {frame_index} infer {infer_count} every {infer_every} {busy} "
            f"last_frame {last_frame} score {score} loop {elapsed_ms:.1f}ms"
        )
        color = (255, 255, 255)
    cv2.putText(canvas, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(canvas, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, color, 1, cv2.LINE_AA)


def _format_optional(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{float(value):.3f}"


def _format_xyz(values: np.ndarray) -> str:
    vec = np.asarray(values, dtype=float).reshape(3)
    return "[" + ", ".join(f"{v:.3f}" for v in vec) + "]"


if __name__ == "__main__":
    raise SystemExit(main())
