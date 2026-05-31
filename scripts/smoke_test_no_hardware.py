#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import replace
import json
import re
import sys
import urllib.request
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from eye_in_hand_graspnet.config import _strip_jsonc_comments, load_T_tcp_cam, load_config
from eye_in_hand_graspnet.graspnet_client import GraspNetHttpClient
from eye_in_hand_graspnet.mask import apply_color_mask, center_rect_bounds, make_center_mask
from eye_in_hand_graspnet.pipeline import _validate_execute_config
from eye_in_hand_graspnet.preview import build_projected_gripper_segments, project_camera_point_to_pixel
from eye_in_hand_graspnet.robot import XMLRPCTargetBridge
from eye_in_hand_graspnet.transforms import BestGrasp, compute_tcp_target, parse_best_grasp, pose_to_matrix
from eye_in_hand_graspnet.array_codec import decode_npy


def call_get_target(url: str) -> list[float]:
    request_body = (
        "<?xml version='1.0'?>"
        "<methodCall><methodName>get_target</methodName><params/></methodCall>"
    ).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=request_body,
        headers={"Content-Type": "text/xml"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=5) as response:
        body = response.read().decode("utf-8")
    return [float(value) for value in re.findall(r"<double>(.*?)</double>", body)]


def main() -> int:
    jsonc_text = '{"url": "http://127.0.0.1:18080/infer", "value": 1} // comment'
    assert json.loads(_strip_jsonc_comments(jsonc_text))["url"] == "http://127.0.0.1:18080/infer"

    config = load_config("configs/eye_in_hand_ur7e_wrist.jsonc")
    T_tcp_cam = load_T_tcp_cam(config.calibration)

    mask = make_center_mask(1280, 720, config.workspace_mask)
    assert mask.shape == (720, 1280)
    assert int(mask.sum() // 255) == 640 * 360
    assert center_rect_bounds(1280, 720, 0.5, 0.5) == (320, 180, 960, 540)
    color = np.full((720, 1280, 3), 200, dtype=np.uint8)
    masked_color = apply_color_mask(color, mask)
    assert masked_color[0, 0].tolist() == [0, 0, 0]
    assert masked_color[360, 640].tolist() == [200, 200, 200]
    client = GraspNetHttpClient(config.graspnet)
    payload = client._build_json_npy_payload(
        color_bgr=masked_color,
        depth_raw=np.ones((720, 1280), dtype=np.uint16) * 1000,
        workspace_mask=mask,
        intrinsics={"K": [[600.0, 0.0, 640.0], [0.0, 600.0, 360.0], [0.0, 0.0, 1.0]]},
        depth_scale_m=0.001,
        seed=config.random_seed,
    )
    assert payload["factor_depth"] == 1000.0
    assert payload["top_k"] == config.graspnet.top_k
    assert decode_npy(payload["color_rgb"])[360, 640].tolist() == [200, 200, 200]
    assert decode_npy(payload["workspace_mask"]).shape == (720, 1280)

    best_grasp = BestGrasp(
        score=0.9,
        width=0.05,
        height=0.02,
        depth=0.03,
        rotation_matrix=np.eye(3),
        translation=np.asarray([0.10, -0.04, 0.42], dtype=float),
        object_id=1,
    )
    current_tcp = [0.30, -0.20, 0.35, 0.0, 0.0, 0.0]
    result = compute_tcp_target(
        T_tcp_cam=T_tcp_cam,
        current_tcp_pose=current_tcp,
        best_grasp=best_grasp,
        tcp_rotation_mode="current",
        grasp_to_tcp_rotation_matrix=config.grasp.grasp_to_tcp_rotation_matrix,
        tcp_rotation_offset_matrix=np.eye(3),
        fixed_tcp_rotvec=[0.0, 0.0, 0.0],
        tcp_translation_offset_m=[0.0, 0.0, 0.0],
        target_base_offset_m=[0.0, 0.0, 0.0],
        pregrasp_offset_m=0.08,
        approach_axis_index=0,
        approach_sign=-1.0,
    )
    expected_center = (pose_to_matrix(current_tcp) @ np.asarray(T_tcp_cam) @ np.r_[best_grasp.translation, 1.0])[:3]
    np.testing.assert_allclose(result.grasp_center_base, expected_center, atol=1e-9)
    grasp_to_tcp = np.asarray(config.grasp.grasp_to_tcp_rotation_matrix, dtype=float)
    np.testing.assert_allclose(
        grasp_to_tcp,
        np.asarray([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
        atol=1e-12,
    )
    np.testing.assert_allclose(grasp_to_tcp.T @ grasp_to_tcp, np.eye(3), atol=1e-12)
    assert abs(float(np.linalg.det(grasp_to_tcp)) - 1.0) < 1e-12
    np.testing.assert_allclose(result.R_base_grasp_as_tcp, result.R_base_grasp @ grasp_to_tcp, atol=1e-12)
    np.testing.assert_allclose(result.R_base_tcp_goal, result.T_base_tcp_now[:3, :3], atol=1e-12)
    np.testing.assert_allclose(result.tcp_translation_offset_base, np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(result.target_base_offset, np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(result.approach_axis_base, result.R_base_grasp[:, 0], atol=1e-12)
    np.testing.assert_allclose(result.pregrasp_offset_base, -0.08 * result.approach_axis_base, atol=1e-12)
    assert len(result.tcp_goal) == 6
    assert len(result.tcp_pregrasp) == 6

    graspnet_rotation_result = compute_tcp_target(
        T_tcp_cam=np.eye(4),
        current_tcp_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        best_grasp=best_grasp,
        tcp_rotation_mode="graspnet",
        grasp_to_tcp_rotation_matrix=config.grasp.grasp_to_tcp_rotation_matrix,
        tcp_rotation_offset_matrix=np.eye(3),
        fixed_tcp_rotvec=[0.0, 0.0, 0.0],
        tcp_translation_offset_m=[0.0, 0.0, 0.0],
        target_base_offset_m=[0.0, 0.0, 0.0],
        pregrasp_offset_m=0.08,
        approach_axis_index=0,
        approach_sign=-1.0,
    )
    np.testing.assert_allclose(graspnet_rotation_result.R_base_tcp_goal[:, 0], [0.0, 1.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(graspnet_rotation_result.R_base_tcp_goal[:, 1], [0.0, 0.0, 1.0], atol=1e-12)
    np.testing.assert_allclose(graspnet_rotation_result.R_base_tcp_goal[:, 2], [1.0, 0.0, 0.0], atol=1e-12)
    # GraspNet 局部轴到 UR TCP 轴的修正是右乘局部旋转，只能改变姿态，不能移动抓取中心。
    np.testing.assert_allclose(graspnet_rotation_result.grasp_center_base, best_grasp.translation, atol=1e-12)
    np.testing.assert_allclose(graspnet_rotation_result.tcp_goal[:3], best_grasp.translation, atol=1e-12)

    local_tcp_offset_result = compute_tcp_target(
        T_tcp_cam=np.eye(4),
        current_tcp_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        best_grasp=best_grasp,
        tcp_rotation_mode="graspnet",
        grasp_to_tcp_rotation_matrix=config.grasp.grasp_to_tcp_rotation_matrix,
        tcp_rotation_offset_matrix=np.eye(3),
        fixed_tcp_rotvec=[0.0, 0.0, 0.0],
        tcp_translation_offset_m=[0.0, 0.0, 0.02],
        target_base_offset_m=[0.0, 0.0, 0.0],
        pregrasp_offset_m=0.08,
        approach_axis_index=0,
        approach_sign=-1.0,
    )
    np.testing.assert_allclose(local_tcp_offset_result.tcp_translation_offset_base, [0.02, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(local_tcp_offset_result.tcp_goal[:3], best_grasp.translation + [0.02, 0.0, 0.0], atol=1e-12)

    intrinsic = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
    assert project_camera_point_to_pixel(np.array([0.0, 0.0, 1.0]), intrinsic) == (320, 240)
    segments = build_projected_gripper_segments(best_grasp, intrinsic)
    assert {label for _start, _end, label in segments} == {"finger", "palm", "tail", "approach"}

    parsed = parse_best_grasp(
        {
            "best_grasp": [
                0.9,
                0.05,
                0.02,
                0.03,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.1,
                -0.04,
                0.42,
                1,
            ]
        }
    )
    np.testing.assert_allclose(parsed.rotation_matrix, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(parsed.translation, best_grasp.translation, atol=1e-12)

    expected_target = [0.31, -0.02, 0.18, 0.1, 0.2, -0.3, config.motion.open_gripper]
    bridge = XMLRPCTargetBridge("127.0.0.1", 0)
    bridge.set_target(expected_target[:6], expected_target[6])
    bridge.start()
    try:
        actual_target = call_get_target(f"http://127.0.0.1:{bridge.port}/RPC2")
    finally:
        bridge.stop()
    np.testing.assert_allclose(actual_target, expected_target, atol=1e-9)
    assert bridge.request_count() == 1

    _validate_execute_config(config, execute=True)
    missing_start_config = replace(config, motion=replace(config.motion, fixed_start_tcp=None))
    try:
        _validate_execute_config(missing_start_config, execute=True)
    except ValueError as exc:
        assert "motion.fixed_start_tcp" in str(exc)
    else:
        raise AssertionError("--execute 必须在 start 步骤缺少 fixed_start_tcp 时拒绝运行")

    big_base_offset_config = replace(config, grasp=replace(config.grasp, target_base_offset_m=[0.0, 0.0, -0.1]))
    try:
        _validate_execute_config(big_base_offset_config, execute=True)
    except ValueError as exc:
        assert "target_base_offset_m" in str(exc)
    else:
        raise AssertionError("--execute 必须拒绝大于 3 cm 的 base 全局补偿")

    direct_graspnet_config = replace(
        config,
        grasp=replace(config.grasp, tcp_rotation_mode="graspnet"),
        motion=replace(config.motion, enabled_steps=["start", "grasp"]),
    )
    try:
        _validate_execute_config(direct_graspnet_config, execute=True)
    except ValueError as exc:
        assert "缺少 pregrasp" in str(exc)
    else:
        raise AssertionError("--execute 必须拒绝 graspnet 姿态下跳过 pregrasp 直接抓取")

    print("[smoke] 配置、中心掩码、GraspNet 输出解析、eye-in-hand TCP 目标计算和 XML-RPC get_target 通过。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
