from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class CameraConfig:
    serial: str
    color_width: int = 1280
    color_height: int = 720
    depth_width: int = 1280
    depth_height: int = 720
    fps: int = 30
    warmup_frames: int = 10
    timeout_ms: int = 3000
    align_depth_to_color: bool = True


@dataclass(frozen=True)
class WorkspaceMaskConfig:
    type: str = "center_fraction"
    width_fraction: float = 0.5
    height_fraction: float = 0.5
    masked_depth_value: int = 0
    preview_dim_outside: bool = True


@dataclass(frozen=True)
class CalibrationConfig:
    path: Path
    matrix_key: str = "T_tcp_cam"


@dataclass(frozen=True)
class RobotConfig:
    host: str
    rtde_enabled: bool = True


@dataclass(frozen=True)
class GraspNetConfig:
    url: str
    timeout_s: float = 30.0
    request_format: str = "json_npy"
    seed_field: str = "seed"
    color_field: str = "color"
    depth_field: str = "depth"
    mask_field: str = "workspace_mask"
    intrinsics_field: str = "intrinsics"
    factor_depth_field: str = "factor_depth"
    top_k: int = 50
    extra_fields: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class GraspConfig:
    tcp_rotation_mode: str = "current"
    grasp_to_tcp_rotation_matrix: list[list[float]] = field(
        default_factory=lambda: [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ]
    )
    tcp_rotation_offset_matrix: list[list[float]] = field(
        default_factory=lambda: [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    fixed_tcp_rotvec: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    tcp_translation_offset_m: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    target_base_offset_m: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pregrasp_offset_m: float = 0.08
    approach_axis_index: int = 0
    approach_sign: float = -1.0


@dataclass(frozen=True)
class MotionConfig:
    xmlrpc_host: str = "0.0.0.0"
    xmlrpc_port: int = 50000
    dashboard_port: int = 29999
    grasp_program: str | None = "aaaaaa.urp"
    stop_before_load: bool = True
    play_after_load: bool = True
    program_settle_s: float = 2.0
    grasp_program_start_wait_s: float = 0.5
    require_grasp_program_running: bool = True
    require_xmlrpc_polling: bool = True
    xmlrpc_poll_check_s: float = 2.0
    fixed_start_tcp: list[float] | None = None
    enabled_steps: list[str] = field(default_factory=lambda: ["start", "pregrasp", "grasp"])
    settle_s_after_start: float = 0.5
    motion_wait_s: float = 3.0
    motion_waits_s: dict[str, float] = field(default_factory=dict)
    open_gripper: float = 100.0
    close_gripper: float = 0.0


@dataclass(frozen=True)
class PreviewConfig:
    enabled: bool = True
    window_name: str = "wrist GraspNet preview"
    scale: float = 0.75
    wait_ms: int = 1
    save_path: Path | None = None
    show_depth: bool = True
    depth_min_m: float = 0.0
    depth_max_m: float = 1.5


@dataclass(frozen=True)
class OutputsConfig:
    last_result_path: Path = PROJECT_ROOT / "outputs" / "last_wrist_grasp_result.json"


@dataclass(frozen=True)
class AppConfig:
    random_seed: int | None
    random_seed_fixed: bool
    camera: CameraConfig
    workspace_mask: WorkspaceMaskConfig
    calibration: CalibrationConfig
    robot: RobotConfig
    graspnet: GraspNetConfig
    grasp: GraspConfig
    motion: MotionConfig
    preview: PreviewConfig
    outputs: OutputsConfig

    @property
    def effective_random_seed(self) -> int | None:
        if not self.random_seed_fixed or self.random_seed is None:
            return None
        return int(self.random_seed)


def resolve_path(path_like: str | Path) -> Path:
    path = Path(path_like)
    if path.is_absolute():
        return path
    return PROJECT_ROOT / path


def load_json(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    return json.loads(_strip_jsonc_comments(text))


def _strip_jsonc_comments(text: str) -> str:
    """去掉 JSONC 注释，同时保留字符串里的 // 和 /*...*/。"""
    result: list[str] = []
    in_string = False
    escaped = False
    i = 0
    while i < len(text):
        char = text[i]
        next_char = text[i + 1] if i + 1 < len(text) else ""

        if in_string:
            result.append(char)
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            i += 1
            continue

        if char == '"':
            in_string = True
            result.append(char)
            i += 1
            continue
        if char == "/" and next_char == "/":
            i += 2
            while i < len(text) and text[i] not in "\r\n":
                i += 1
            continue
        if char == "/" and next_char == "*":
            i += 2
            while i + 1 < len(text) and not (text[i] == "*" and text[i + 1] == "/"):
                if text[i] in "\r\n":
                    result.append(text[i])
                i += 1
            i += 2
            continue

        result.append(char)
        i += 1

    return "".join(result)


def load_config(path: str | Path) -> AppConfig:
    config_path = resolve_path(path)
    data = load_json(config_path)

    preview_data = data.get("preview", {})
    outputs_data = data.get("outputs", {})
    return AppConfig(
        random_seed=data.get("random_seed"),
        random_seed_fixed=bool(data.get("random_seed_fixed", data.get("random_seed") is not None)),
        camera=CameraConfig(**data["camera"]),
        workspace_mask=WorkspaceMaskConfig(**data.get("workspace_mask", {})),
        calibration=CalibrationConfig(
            path=resolve_path(data["calibration"]["path"]),
            matrix_key=str(data["calibration"].get("matrix_key", "T_tcp_cam")),
        ),
        robot=RobotConfig(**data["robot"]),
        graspnet=GraspNetConfig(**data["graspnet"]),
        grasp=GraspConfig(**data.get("grasp", {})),
        motion=_load_motion_config(data["motion"]),
        preview=PreviewConfig(
            enabled=bool(preview_data.get("enabled", True)),
            window_name=str(preview_data.get("window_name", "wrist GraspNet preview")),
            scale=float(preview_data.get("scale", 0.75)),
            wait_ms=int(preview_data.get("wait_ms", 1)),
            save_path=resolve_path(preview_data["save_path"]) if preview_data.get("save_path") else None,
            show_depth=bool(preview_data.get("show_depth", True)),
            depth_min_m=float(preview_data.get("depth_min_m", 0.0)),
            depth_max_m=float(preview_data.get("depth_max_m", 1.5)),
        ),
        outputs=OutputsConfig(
            last_result_path=resolve_path(
                outputs_data.get("last_result_path", "outputs/last_wrist_grasp_result.json")
            )
        ),
    )


def load_T_tcp_cam(config: CalibrationConfig) -> list[list[float]]:
    data = load_json(config.path)
    matrix = data.get(config.matrix_key)
    if matrix is None:
        raise KeyError(f"标定文件 {config.path} 缺少矩阵字段: {config.matrix_key}")
    return matrix


def _load_motion_config(data: dict[str, Any]) -> MotionConfig:
    legacy_keys = {"xmlrpc_url", "move_tcp_method", "move_tcp_kwargs"} & set(data)
    if legacy_keys:
        joined = ", ".join(sorted(legacy_keys))
        raise ValueError(
            f"motion 中仍包含旧的远端 move_tcp XML-RPC 字段: {joined}。"
            "本工程现在与 graspnet-baseline-in-ur7e 一致：Windows 侧启动 get_target() 服务，"
            "示教器 URP 轮询该服务，并由 Dashboard 负责 load/play。"
        )
    return MotionConfig(**data)


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")
