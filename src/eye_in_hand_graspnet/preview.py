from __future__ import annotations

from pathlib import Path

import numpy as np

from .transforms import BestGrasp


def draw_wrist_preview(
    *,
    color_bgr: np.ndarray,
    depth_raw: np.ndarray,
    depth_scale_m: float,
    mask: np.ndarray,
    best_grasp: BestGrasp | None,
    intrinsics: dict,
    window_name: str,
    scale: float,
    wait_ms: int,
    save_path: Path | None,
    show_depth: bool = True,
    depth_min_m: float = 0.0,
    depth_max_m: float = 1.5,
) -> None:
    try:
        import cv2
    except ModuleNotFoundError:
        print("[preview] 未安装 cv2，跳过预览。")
        return

    image = color_bgr.copy()
    x0, y0, x1, y1 = _mask_bbox(mask)
    cv2.rectangle(image, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 255), 2)

    if best_grasp is not None:
        try:
            intrinsic_matrix = _intrinsic_matrix(intrinsics)
        except (KeyError, TypeError, ValueError):
            intrinsic_matrix = None
        if intrinsic_matrix is not None:
            point_uv = project_camera_point_to_pixel(best_grasp.translation, intrinsic_matrix)
            if point_uv is not None:
                u, v = point_uv
                if 0 <= u < image.shape[1] and 0 <= v < image.shape[0]:
                    draw_grasp_marker(cv2, image, u, v, best_grasp, intrinsic_matrix)

    preview_canvas = _build_preview_canvas(
        cv2,
        image,
        depth_raw,
        depth_scale_m,
        mask,
        show_depth,
        depth_min_m,
        depth_max_m,
    )
    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(save_path), preview_canvas)

    display = preview_canvas
    if scale != 1.0:
        display = cv2.resize(display, None, fx=float(scale), fy=float(scale), interpolation=cv2.INTER_AREA)
    cv2.imshow(window_name, display)
    cv2.waitKey(int(wait_ms))


def _build_preview_canvas(
    cv2,
    color_overlay_bgr: np.ndarray,
    depth_raw: np.ndarray,
    depth_scale_m: float,
    mask: np.ndarray,
    show_depth: bool,
    depth_min_m: float,
    depth_max_m: float,
) -> np.ndarray:
    if not show_depth:
        return color_overlay_bgr

    depth_bgr = colorize_depth_for_preview(
        cv2,
        depth_raw=depth_raw,
        depth_scale_m=depth_scale_m,
        mask=mask,
        depth_min_m=depth_min_m,
        depth_max_m=depth_max_m,
    )
    if depth_bgr.shape[:2] != color_overlay_bgr.shape[:2]:
        depth_bgr = cv2.resize(
            depth_bgr,
            (color_overlay_bgr.shape[1], color_overlay_bgr.shape[0]),
            interpolation=cv2.INTER_NEAREST,
        )
    return np.hstack([color_overlay_bgr, depth_bgr])


def colorize_depth_for_preview(
    cv2,
    *,
    depth_raw: np.ndarray,
    depth_scale_m: float,
    mask: np.ndarray,
    depth_min_m: float,
    depth_max_m: float,
) -> np.ndarray:
    if depth_max_m <= depth_min_m:
        raise ValueError(f"preview.depth_max_m 必须大于 depth_min_m，实际 {depth_min_m}..{depth_max_m}")

    depth = np.asarray(depth_raw, dtype=np.float32) * float(depth_scale_m)
    clipped = np.clip(depth, float(depth_min_m), float(depth_max_m))
    normalized = ((clipped - float(depth_min_m)) / (float(depth_max_m) - float(depth_min_m)) * 255.0).astype(np.uint8)
    colormap = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)
    depth_bgr = cv2.applyColorMap(normalized, colormap)

    # 0 深度和掩码外区域都不参与抓取判断，预览中置黑能避免伪彩误导。
    invalid = np.asarray(depth_raw) <= 0
    if mask.shape == depth_bgr.shape[:2]:
        invalid = invalid | (mask == 0)
        x0, y0, x1, y1 = _mask_bbox(mask)
        cv2.rectangle(depth_bgr, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 255), 2)
    depth_bgr[invalid] = 0
    return depth_bgr


def _mask_bbox(mask: np.ndarray) -> tuple[int, int, int, int]:
    ys, xs = np.where(mask > 0)
    if len(xs) == 0 or len(ys) == 0:
        return 0, 0, mask.shape[1], mask.shape[0]
    return int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1


def project_camera_point_to_pixel(
    translation: np.ndarray,
    intrinsic_matrix: np.ndarray,
) -> tuple[int, int] | None:
    xyz = np.asarray(translation, dtype=np.float64).reshape(3)
    z = float(xyz[2])
    if z <= 1e-8:
        return None
    intr = np.asarray(intrinsic_matrix, dtype=np.float64)
    if intr.shape != (3, 3):
        return None
    u = intr[0, 0] * float(xyz[0]) / z + intr[0, 2]
    v = intr[1, 1] * float(xyz[1]) / z + intr[1, 2]
    return int(round(u)), int(round(v))


def build_projected_gripper_segments(
    grasp: BestGrasp,
    intrinsic_matrix: np.ndarray,
) -> list[tuple[tuple[int, int], tuple[int, int], str]]:
    """把 GraspNet 小夹爪三维线框投影到当前腕部相机图像。"""
    center = np.asarray(grasp.translation, dtype=np.float64).reshape(3)
    rotation = np.asarray(grasp.rotation_matrix, dtype=np.float64).reshape(3, 3)
    approach_axis = _unit_vector(rotation[:, 0], "grasp approach axis")
    jaw_axis = _unit_vector(rotation[:, 1], "grasp jaw axis")

    width = _coerce_positive_length(grasp.width, default=0.08, lower=0.015, upper=0.16)
    depth = _coerce_positive_length(grasp.depth, default=0.04, lower=0.025, upper=0.10)
    finger_width_m = 0.004
    depth_base_m = 0.020
    tail_length_m = 0.035
    half_width = width * 0.5

    def gripper_point(x_m: float, y_m: float) -> np.ndarray:
        return center + approach_axis * x_m + jaw_axis * y_m

    palm_x = -(depth_base_m + finger_width_m * 0.5)
    tail_end_x = -(depth_base_m + finger_width_m + tail_length_m)
    left_y = -(half_width + finger_width_m * 0.5)
    right_y = half_width + finger_width_m * 0.5
    points = {
        "left_palm": gripper_point(palm_x, left_y),
        "left_tip": gripper_point(depth, left_y),
        "right_palm": gripper_point(palm_x, right_y),
        "right_tip": gripper_point(depth, right_y),
        "tail_start": gripper_point(palm_x, 0.0),
        "tail_end": gripper_point(tail_end_x, 0.0),
        "target": center,
    }
    lines = [
        ("left_palm", "left_tip", "finger"),
        ("right_palm", "right_tip", "finger"),
        ("left_palm", "right_palm", "palm"),
        ("tail_end", "tail_start", "tail"),
        ("tail_end", "target", "approach"),
    ]

    projected: list[tuple[tuple[int, int], tuple[int, int], str]] = []
    for start_name, end_name, label in lines:
        start = project_camera_point_to_pixel(points[start_name], intrinsic_matrix)
        end = project_camera_point_to_pixel(points[end_name], intrinsic_matrix)
        if start is not None and end is not None:
            projected.append((start, end, label))
    return projected


def draw_grasp_marker(
    cv2,
    image_bgr: np.ndarray,
    u: int,
    v: int,
    grasp: BestGrasp,
    intrinsic_matrix: np.ndarray,
) -> None:
    draw_projected_gripper(cv2, image_bgr, grasp, intrinsic_matrix)

    marker_color = (0, 0, 255)
    text_color = (255, 255, 255)
    shadow_color = (0, 0, 0)
    cv2.circle(image_bgr, (u, v), 10, marker_color, 2)
    cv2.drawMarker(image_bgr, (u, v), marker_color, markerType=cv2.MARKER_CROSS, markerSize=32, thickness=2)

    label_lines = [
        f"target ({u}, {v})",
        f"score {_format_optional(grasp.score)}, width {_format_optional(grasp.width)}m",
        f"xyz {_format_vector(grasp.translation)}",
    ]
    x0 = min(max(8, u + 16), max(8, image_bgr.shape[1] - 360))
    y0 = max(24, v - 48)
    for i, text in enumerate(label_lines):
        y = y0 + i * 24
        cv2.putText(image_bgr, text, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, shadow_color, 3, cv2.LINE_AA)
        cv2.putText(image_bgr, text, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, text_color, 1, cv2.LINE_AA)


def draw_projected_gripper(cv2, image_bgr: np.ndarray, grasp: BestGrasp, intrinsic_matrix: np.ndarray) -> None:
    segment_colors = {
        "finger": (0, 255, 255),
        "palm": (0, 180, 255),
        "tail": (255, 180, 0),
        "approach": (80, 255, 80),
    }
    try:
        segments = build_projected_gripper_segments(grasp, intrinsic_matrix)
    except (TypeError, ValueError):
        return
    for start, end, label in segments:
        color = segment_colors.get(label, (0, 255, 255))
        if label == "approach" and hasattr(cv2, "arrowedLine"):
            cv2.arrowedLine(image_bgr, start, end, color, 2, cv2.LINE_AA, tipLength=0.25)
        else:
            cv2.line(image_bgr, start, end, color, 3, cv2.LINE_AA)


def _intrinsic_matrix(intrinsics: dict) -> np.ndarray:
    if "K" in intrinsics:
        matrix = np.asarray(intrinsics["K"], dtype=np.float64)
    else:
        matrix = np.asarray(
            [
                [float(intrinsics["fx"]), 0.0, float(intrinsics["ppx"])],
                [0.0, float(intrinsics["fy"]), float(intrinsics["ppy"])],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
    if matrix.shape != (3, 3):
        raise ValueError(f"相机内参矩阵应为 3x3，实际 shape={matrix.shape}")
    return matrix


def _unit_vector(vector: np.ndarray, name: str) -> np.ndarray:
    values = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(values))
    if norm <= 1e-8:
        raise ValueError(f"{name} 为零向量，无法绘制小夹爪。")
    return values / norm


def _coerce_positive_length(value: object, default: float, lower: float, upper: float) -> float:
    try:
        length = abs(float(value))
    except (TypeError, ValueError):
        length = float(default)
    if not np.isfinite(length) or length <= 1e-8:
        length = float(default)
    return float(np.clip(length, lower, upper))


def _format_optional(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{float(value):.3f}"


def _format_vector(values: np.ndarray) -> str:
    vec = np.asarray(values, dtype=np.float64).reshape(-1)
    return "[" + ", ".join(f"{v:.3f}" for v in vec[:3]) + "]"
