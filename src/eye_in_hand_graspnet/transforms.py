from __future__ import annotations

from dataclasses import dataclass
from math import acos, cos, sin, sqrt
from typing import Any

import numpy as np


@dataclass(frozen=True)
class BestGrasp:
    score: float | None
    width: float | None
    height: float | None
    depth: float | None
    rotation_matrix: np.ndarray
    translation: np.ndarray
    object_id: int | None = None
    raw: dict[str, Any] | None = None


@dataclass(frozen=True)
class TcpTargetResult:
    T_base_tcp_now: np.ndarray
    T_base_cam_now: np.ndarray
    T_base_grasp: np.ndarray
    grasp_center_base: np.ndarray
    tcp_goal: list[float]
    tcp_pregrasp: list[float]
    R_base_grasp: np.ndarray
    R_base_tcp_goal: np.ndarray

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "T_base_tcp_now": self.T_base_tcp_now.tolist(),
            "T_base_cam_now": self.T_base_cam_now.tolist(),
            "T_base_grasp": self.T_base_grasp.tolist(),
            "grasp_center_base": self.grasp_center_base.tolist(),
            "tcp_pregrasp": self.tcp_pregrasp,
            "tcp_goal": self.tcp_goal,
        }


def as_matrix4(values: Any, name: str = "matrix") -> np.ndarray:
    matrix = np.asarray(values, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"{name} 应为 4x4 矩阵，实际 shape={matrix.shape}")
    return matrix


def as_vector3(values: Any, name: str = "vector") -> np.ndarray:
    vector = np.asarray(values, dtype=float)
    if vector.shape != (3,):
        raise ValueError(f"{name} 应为长度 3 的向量，实际 shape={vector.shape}")
    return vector


def as_pose6(values: Any, name: str = "pose") -> list[float]:
    pose = [float(v) for v in values]
    if len(pose) != 6:
        raise ValueError(f"{name} 应为长度 6 的 [x,y,z,rx,ry,rz]，实际长度={len(pose)}")
    return pose


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.asarray(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )


def rotvec_to_matrix(rotvec: Any) -> np.ndarray:
    rv = as_vector3(rotvec, "rotvec")
    theta = float(np.linalg.norm(rv))
    if theta < 1e-12:
        return np.eye(3)
    axis = rv / theta
    axis_skew = skew(axis)
    return np.eye(3) + sin(theta) * axis_skew + (1.0 - cos(theta)) * (axis_skew @ axis_skew)


def matrix_to_rotvec(matrix: Any) -> np.ndarray:
    R = np.asarray(matrix, dtype=float)
    if R.shape != (3, 3):
        raise ValueError(f"rotation matrix 应为 3x3，实际 shape={R.shape}")
    # 数值误差会让 trace 略微越界，夹紧后再求反三角。
    cos_theta = float((np.trace(R) - 1.0) * 0.5)
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta = acos(cos_theta)
    if theta < 1e-12:
        return np.zeros(3, dtype=float)
    if abs(theta - np.pi) < 1e-6:
        axis = np.empty(3, dtype=float)
        axis[0] = sqrt(max(0.0, (R[0, 0] + 1.0) * 0.5))
        axis[1] = sqrt(max(0.0, (R[1, 1] + 1.0) * 0.5))
        axis[2] = sqrt(max(0.0, (R[2, 2] + 1.0) * 0.5))
        if R[0, 1] < 0.0:
            axis[1] = -axis[1]
        if R[0, 2] < 0.0:
            axis[2] = -axis[2]
        norm = float(np.linalg.norm(axis))
        if norm < 1e-12:
            return np.zeros(3, dtype=float)
        return axis / norm * theta
    axis = np.asarray(
        [
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ],
        dtype=float,
    ) / (2.0 * sin(theta))
    return axis * theta


def pose_to_matrix(pose6: Any) -> np.ndarray:
    x, y, z, rx, ry, rz = as_pose6(pose6)
    T = np.eye(4, dtype=float)
    T[:3, :3] = rotvec_to_matrix([rx, ry, rz])
    T[:3, 3] = [x, y, z]
    return T


def matrix_to_pose(T: Any) -> list[float]:
    matrix = as_matrix4(T, "T")
    rotvec = matrix_to_rotvec(matrix[:3, :3])
    return [float(v) for v in np.r_[matrix[:3, 3], rotvec]]


def make_transform(rotation: Any, translation: Any) -> np.ndarray:
    R = np.asarray(rotation, dtype=float)
    if R.shape != (3, 3):
        raise ValueError(f"rotation 应为 3x3，实际 shape={R.shape}")
    t = as_vector3(translation, "translation")
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def normalize(vector: np.ndarray, name: str) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError(f"{name} 不能是零向量")
    return vector / norm


def parse_best_grasp(payload: dict[str, Any]) -> BestGrasp:
    """兼容常见 GraspNet JSON：best_grasp 字典或 17 维 grasp 数组。"""
    data = payload.get("best_grasp", payload)
    if isinstance(data, list):
        if len(data) < 17:
            raise ValueError(f"17 维 GraspNet 数组长度不足: {len(data)}")
        rotation = np.asarray(data[4:13], dtype=float).reshape(3, 3)
        translation = np.asarray(data[13:16], dtype=float)
        return BestGrasp(
            score=float(data[0]),
            width=float(data[1]),
            height=float(data[2]),
            depth=float(data[3]),
            rotation_matrix=rotation,
            translation=translation,
            object_id=int(data[16]),
            raw={"best_grasp": data},
        )

    if not isinstance(data, dict):
        raise ValueError("GraspNet 响应中 best_grasp 应为 dict 或 17 维 list")

    rotation_value = _first_present(data, ("rotation_matrix", "R_cam_grasp", "rotation", "rotation_3x3"))
    translation_value = _first_present(data, ("translation", "t_cam_grasp", "center"))
    if rotation_value is None or translation_value is None:
        raise KeyError("best_grasp 缺少 rotation_matrix/translation 字段")

    rotation = np.asarray(rotation_value, dtype=float)
    if rotation.shape == (9,):
        rotation = rotation.reshape(3, 3)
    if rotation.shape != (3, 3):
        raise ValueError(f"best_grasp rotation_matrix 应为 3x3 或长度 9，实际 shape={rotation.shape}")

    translation = as_vector3(translation_value, "best_grasp.translation")
    return BestGrasp(
        score=_optional_float(data.get("score")),
        width=_optional_float(data.get("width")),
        height=_optional_float(data.get("height")),
        depth=_optional_float(data.get("depth")),
        rotation_matrix=rotation,
        translation=translation,
        object_id=_optional_int(data.get("object_id")),
        raw=data,
    )


def compute_tcp_target(
    *,
    T_tcp_cam: Any,
    current_tcp_pose: Any,
    best_grasp: BestGrasp,
    tcp_rotation_mode: str,
    tcp_rotation_offset_matrix: Any,
    fixed_tcp_rotvec: Any,
    tcp_translation_offset_m: Any,
    target_base_offset_m: Any,
    pregrasp_offset_m: float,
    approach_axis_index: int,
    approach_sign: float,
) -> TcpTargetResult:
    T_tcp_cam_m = as_matrix4(T_tcp_cam, "T_tcp_cam")
    T_base_tcp_now = pose_to_matrix(current_tcp_pose)
    T_base_cam_now = T_base_tcp_now @ T_tcp_cam_m

    T_cam_grasp = make_transform(best_grasp.rotation_matrix, best_grasp.translation)
    T_base_grasp = T_base_cam_now @ T_cam_grasp
    grasp_center_base = T_base_grasp[:3, 3].copy()
    R_base_grasp = T_base_grasp[:3, :3]

    R_base_tcp_goal = resolve_tcp_rotation(
        mode=tcp_rotation_mode,
        R_base_grasp=R_base_grasp,
        R_base_tcp_now=T_base_tcp_now[:3, :3],
        tcp_rotation_offset_matrix=tcp_rotation_offset_matrix,
        fixed_tcp_rotvec=fixed_tcp_rotvec,
    )
    tcp_offset = as_vector3(tcp_translation_offset_m, "tcp_translation_offset_m")
    base_offset = as_vector3(target_base_offset_m, "target_base_offset_m")
    tcp_goal_translation = grasp_center_base + R_base_tcp_goal @ tcp_offset + base_offset

    T_base_tcp_goal = make_transform(R_base_tcp_goal, tcp_goal_translation)
    tcp_goal = matrix_to_pose(T_base_tcp_goal)

    if approach_axis_index not in (0, 1, 2):
        raise ValueError("approach_axis_index 只能是 0, 1, 2")
    approach_axis = normalize(R_base_grasp[:, approach_axis_index], "approach_axis")
    pregrasp_translation = tcp_goal_translation + float(approach_sign) * float(pregrasp_offset_m) * approach_axis
    T_base_tcp_pregrasp = make_transform(R_base_tcp_goal, pregrasp_translation)
    tcp_pregrasp = matrix_to_pose(T_base_tcp_pregrasp)

    return TcpTargetResult(
        T_base_tcp_now=T_base_tcp_now,
        T_base_cam_now=T_base_cam_now,
        T_base_grasp=T_base_grasp,
        grasp_center_base=grasp_center_base,
        tcp_goal=tcp_goal,
        tcp_pregrasp=tcp_pregrasp,
        R_base_grasp=R_base_grasp,
        R_base_tcp_goal=R_base_tcp_goal,
    )


def resolve_tcp_rotation(
    *,
    mode: str,
    R_base_grasp: np.ndarray,
    R_base_tcp_now: np.ndarray,
    tcp_rotation_offset_matrix: Any,
    fixed_tcp_rotvec: Any,
) -> np.ndarray:
    mode_l = str(mode).lower()
    if mode_l == "graspnet":
        return R_base_grasp @ np.asarray(tcp_rotation_offset_matrix, dtype=float)
    if mode_l == "fixed":
        return rotvec_to_matrix(fixed_tcp_rotvec)
    if mode_l == "current":
        return np.asarray(R_base_tcp_now, dtype=float)
    raise ValueError(f"未知 tcp_rotation_mode={mode!r}，可选 graspnet/fixed/current")


def _optional_float(value: Any) -> float | None:
    return None if value is None else float(value)


def _optional_int(value: Any) -> int | None:
    return None if value is None else int(value)


def _first_present(data: dict[str, Any], keys: tuple[str, ...]) -> Any:
    for key in keys:
        if key in data and data[key] is not None:
            return data[key]
    return None
