from __future__ import annotations

import time
from typing import Any

import numpy as np

from .camera import WristRealSenseCamera
from .config import AppConfig, load_T_tcp_cam, write_json
from .graspnet_client import GraspNetHttpClient
from .mask import apply_color_mask, apply_color_mask_for_preview, apply_depth_mask, make_center_mask
from .preview import draw_wrist_preview
from .robot import (
    RtdeTcpReader,
    XMLRPCTargetBridge,
    format_pose,
    maybe_set_start_target,
    resolve_motion_waits_s,
    run_urp_program,
    set_target_and_wait,
    wait_for_xmlrpc_polling,
)
from .transforms import compute_tcp_target


def run_wrist_grasp_cycle(config: AppConfig, *, execute: bool) -> dict[str, Any]:
    if config.random_seed is not None:
        np.random.seed(int(config.random_seed))

    _validate_execute_config(config, execute=execute)
    T_tcp_cam = load_T_tcp_cam(config.calibration)
    tcp_reader = RtdeTcpReader(config.robot)
    bridge = (
        XMLRPCTargetBridge(config.motion.xmlrpc_host, int(config.motion.xmlrpc_port))
        if execute
        else None
    )

    try:
        if execute and bridge is not None:
            print("[motion] 启动 PC 侧 XML-RPC get_target 服务，并把初始目标钉在当前 TCP。")
            tcp_initial = tcp_reader.read_current_tcp()
            bridge.start()
            bridge.set_target(tcp_initial, config.motion.open_gripper)
            run_urp_program(config.robot, config.motion, label="GRASP-URP")
            wait_for_xmlrpc_polling(bridge, config.motion)
            if "start" in config.motion.enabled_steps:
                maybe_set_start_target(bridge, config.motion)

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

        tcp_now = tcp_reader.read_current_tcp()
        target = compute_tcp_target(
            T_tcp_cam=T_tcp_cam,
            current_tcp_pose=tcp_now,
            best_grasp=grasp_response.best_grasp,
            tcp_rotation_mode=config.grasp.tcp_rotation_mode,
            grasp_to_tcp_rotation_matrix=config.grasp.grasp_to_tcp_rotation_matrix,
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
        print("[result] tcp_offset_base  = " + _format_xyz(target.tcp_translation_offset_base))
        print("[result] target_base_offset= " + _format_xyz(target.target_base_offset))
        print("[result] pregrasp_offset  = " + _format_xyz(target.pregrasp_offset_base))
        print("[result] tcp_pregrasp     = " + format_pose(target.tcp_pregrasp))
        print("[result] tcp_goal         = " + format_pose(target.tcp_goal))
        print(f"[result] saved={config.outputs.last_result_path}")

        if execute and bridge is not None:
            _execute_motion_sequence(bridge, config, tcp_now, target.tcp_pregrasp, target.tcp_goal)
        else:
            print("[motion] dry-run：已完成拍照、GraspNet 推理、RTDE 读取和目标点计算，未更新 XML-RPC 目标。")

        return result
    finally:
        if bridge is not None:
            bridge.stop()


def _format_xyz(values: np.ndarray) -> str:
    x, y, z = [float(v) for v in values]
    return f"[{x:.9f}, {y:.9f}, {z:.9f}]"


def _validate_execute_config(config: AppConfig, *, execute: bool) -> None:
    allowed_steps = {"start", "open-current", "pregrasp", "grasp", "close"}
    unknown_steps = sorted(set(config.motion.enabled_steps) - allowed_steps)
    if unknown_steps:
        raise ValueError(f"motion.enabled_steps 包含未知步骤: {unknown_steps}，可选 {sorted(allowed_steps)}。")
    if not execute:
        return
    base_offset_norm = float(np.linalg.norm(np.asarray(config.grasp.target_base_offset_m, dtype=float)))
    if base_offset_norm > 0.03:
        raise ValueError(
            "grasp.target_base_offset_m 的模长超过 3 cm。"
            "该字段是 base 坐标系下的全局小补偿，不能用来修正 GraspNet 姿态、TCP 轴定义或标定错误。"
            "请先改回毫米级补偿并通过 --dry-run 检查输出分量。"
        )
    if (
        str(config.grasp.tcp_rotation_mode).lower() == "graspnet"
        and "grasp" in config.motion.enabled_steps
        and "pregrasp" not in config.motion.enabled_steps
    ):
        raise ValueError(
            "当前启用了 tcp_rotation_mode=graspnet 且直接执行 grasp，但 motion.enabled_steps 缺少 pregrasp。"
            "姿态跟随 GraspNet 时必须先低速到预抓取点确认方向，再执行最终抓取点。"
        )
    if "start" in config.motion.enabled_steps and config.motion.fixed_start_tcp is None:
        raise ValueError(
            "motion.enabled_steps 包含 start，但 motion.fixed_start_tcp 仍为 null。"
            "请先填写现场确认过的固定观察 TCP，再使用 --execute。"
        )
    if not config.motion.grasp_program:
        raise ValueError("实机执行需要配置 motion.grasp_program，且示教器程序必须轮询 PC XML-RPC get_target()。")


def _execute_motion_sequence(
    bridge: XMLRPCTargetBridge,
    config: AppConfig,
    current_tcp: list[float],
    tcp_pregrasp: list[float],
    tcp_goal: list[float],
) -> None:
    waits = resolve_motion_waits_s(config.motion)
    steps = set(config.motion.enabled_steps)
    if "open-current" in steps:
        set_target_and_wait(
            bridge,
            current_tcp,
            config.motion.open_gripper,
            waits["open-current"],
            label="open-current",
        )
    if "pregrasp" in steps:
        set_target_and_wait(
            bridge,
            tcp_pregrasp,
            config.motion.open_gripper,
            waits["move-pregrasp"],
            label="pregrasp",
        )
    if "grasp" in steps:
        set_target_and_wait(
            bridge,
            tcp_goal,
            config.motion.open_gripper,
            waits["move-grasp"],
            label="grasp",
        )
    if "close" in steps:
        set_target_and_wait(
            bridge,
            tcp_goal,
            config.motion.close_gripper,
            waits["close-gripper"],
            label="close",
        )
