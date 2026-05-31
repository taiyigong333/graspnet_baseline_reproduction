from __future__ import annotations

import time
from typing import Any

import numpy as np

from .camera import WristRealSenseCamera
from .config import AppConfig, load_T_tcp_cam, write_json
from .graspnet_client import GraspNetHttpClient
from .mask import apply_color_mask, apply_color_mask_for_preview, apply_depth_mask, make_center_mask
from .preview import draw_wrist_preview
from .robot import RtdeTcpReader, XmlRpcMotionClient, format_pose, maybe_move_start
from .transforms import compute_tcp_target


def run_wrist_grasp_cycle(config: AppConfig, *, execute: bool) -> dict[str, Any]:
    if config.random_seed is not None:
        np.random.seed(int(config.random_seed))

    _validate_execute_config(config, execute=execute)
    T_tcp_cam = load_T_tcp_cam(config.calibration)
    motion_client = XmlRpcMotionClient(config.motion) if execute else None

    if execute and motion_client is not None and "start" in config.motion.enabled_steps:
        maybe_move_start(motion_client, config.motion)

    with WristRealSenseCamera(config.camera) as camera:
        frame = camera.capture()

    mask = make_center_mask(
        frame.color_bgr.shape[1],
        frame.color_bgr.shape[0],
        config.workspace_mask,
    )
    masked_depth = apply_depth_mask(frame.depth_raw, mask, config.workspace_mask.masked_depth_value)
    masked_color = apply_color_mask(frame.color_bgr, mask)
    preview_color = apply_color_mask_for_preview(
        frame.color_bgr,
        mask,
        config.workspace_mask.preview_dim_outside,
    )

    client = GraspNetHttpClient(config.graspnet)
    grasp_response = client.infer(
        color_bgr=masked_color,
        depth_raw=masked_depth,
        workspace_mask=mask,
        intrinsics=frame.color_intrinsics,
        depth_scale_m=frame.depth_scale_m,
        seed=config.random_seed,
    )

    tcp_now = RtdeTcpReader(config.robot).read_current_tcp()
    target = compute_tcp_target(
        T_tcp_cam=T_tcp_cam,
        current_tcp_pose=tcp_now,
        best_grasp=grasp_response.best_grasp,
        tcp_rotation_mode=config.grasp.tcp_rotation_mode,
        tcp_rotation_offset_matrix=config.grasp.tcp_rotation_offset_matrix,
        fixed_tcp_rotvec=config.grasp.fixed_tcp_rotvec,
        tcp_translation_offset_m=config.grasp.tcp_translation_offset_m,
        target_base_offset_m=config.grasp.target_base_offset_m,
        pregrasp_offset_m=config.grasp.pregrasp_offset_m,
        approach_axis_index=config.grasp.approach_axis_index,
        approach_sign=config.grasp.approach_sign,
    )

    if config.preview.enabled:
        draw_wrist_preview(
            color_bgr=preview_color,
            mask=mask,
            best_grasp=grasp_response.best_grasp,
            intrinsics=frame.color_intrinsics,
            window_name=config.preview.window_name,
            scale=config.preview.scale,
            wait_ms=config.preview.wait_ms,
            save_path=config.preview.save_path,
        )

    result = {
        "timestamp_unix_s": time.time(),
        "execute": bool(execute),
        "camera": {
            "serial": frame.camera_serial,
            "timestamp_ms": frame.timestamp_ms,
            "depth_scale_m": frame.depth_scale_m,
        },
        "mask": {
            "type": config.workspace_mask.type,
            "width_fraction": config.workspace_mask.width_fraction,
            "height_fraction": config.workspace_mask.height_fraction,
            "kept_pixels": int(np.count_nonzero(mask)),
            "total_pixels": int(mask.size),
        },
        "current_tcp": tcp_now,
        "best_grasp": {
            "score": grasp_response.best_grasp.score,
            "width": grasp_response.best_grasp.width,
            "height": grasp_response.best_grasp.height,
            "depth": grasp_response.best_grasp.depth,
            "translation": grasp_response.best_grasp.translation.tolist(),
            "rotation_matrix": grasp_response.best_grasp.rotation_matrix.tolist(),
            "object_id": grasp_response.best_grasp.object_id,
        },
        "target": target.to_json_dict(),
        "graspnet_raw": grasp_response.raw,
    }
    write_json(config.outputs.last_result_path, result)

    print("[result] current_tcp      = " + format_pose(tcp_now))
    print("[result] grasp_center_base= " + _format_xyz(target.grasp_center_base))
    print("[result] tcp_pregrasp     = " + format_pose(target.tcp_pregrasp))
    print("[result] tcp_goal         = " + format_pose(target.tcp_goal))
    print(f"[result] saved={config.outputs.last_result_path}")

    if execute and motion_client is not None:
        if "pregrasp" in config.motion.enabled_steps:
            motion_client.move_tcp(target.tcp_pregrasp, label="pregrasp")
        if "grasp" in config.motion.enabled_steps:
            motion_client.move_tcp(target.tcp_goal, label="grasp")
    else:
        print("[motion] dry-run：已完成拍照、GraspNet 推理、RTDE 读取和目标点计算，未发送 XML-RPC 移动命令。")

    return result


def _format_xyz(values: np.ndarray) -> str:
    x, y, z = [float(v) for v in values]
    return f"[{x:.9f}, {y:.9f}, {z:.9f}]"


def _validate_execute_config(config: AppConfig, *, execute: bool) -> None:
    if not execute:
        return
    if "start" in config.motion.enabled_steps and config.motion.fixed_start_tcp is None:
        raise ValueError(
            "motion.enabled_steps 包含 start，但 motion.fixed_start_tcp 仍为 null。"
            "请先填写现场确认过的固定观察 TCP，再使用 --execute。"
        )
