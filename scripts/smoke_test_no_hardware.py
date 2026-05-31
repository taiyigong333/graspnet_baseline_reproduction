#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from eye_in_hand_graspnet.config import load_T_tcp_cam, load_config
from eye_in_hand_graspnet.graspnet_client import GraspNetHttpClient
from eye_in_hand_graspnet.mask import apply_color_mask, center_rect_bounds, make_center_mask
from eye_in_hand_graspnet.pipeline import _validate_execute_config
from eye_in_hand_graspnet.transforms import BestGrasp, compute_tcp_target, parse_best_grasp, pose_to_matrix
from eye_in_hand_graspnet.array_codec import decode_npy


def main() -> int:
    config = load_config("configs/eye_in_hand_ur7e_wrist.json")
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
    assert len(result.tcp_goal) == 6
    assert len(result.tcp_pregrasp) == 6

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

    try:
        _validate_execute_config(config, execute=True)
    except ValueError as exc:
        assert "motion.fixed_start_tcp" in str(exc)
    else:
        raise AssertionError("--execute 必须在 fixed_start_tcp 为空时拒绝运行")

    print("[smoke] 配置、中心掩码、GraspNet 输出解析和 eye-in-hand TCP 目标计算通过。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
