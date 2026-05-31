from __future__ import annotations

from pathlib import Path

import numpy as np

from .transforms import BestGrasp


def draw_wrist_preview(
    *,
    color_bgr: np.ndarray,
    mask: np.ndarray,
    best_grasp: BestGrasp | None,
    intrinsics: dict,
    window_name: str,
    scale: float,
    wait_ms: int,
    save_path: Path | None,
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
        _draw_grasp_axes(image, best_grasp, intrinsics)

    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(save_path), image)

    if scale != 1.0:
        image = cv2.resize(image, None, fx=float(scale), fy=float(scale), interpolation=cv2.INTER_AREA)
    cv2.imshow(window_name, image)
    cv2.waitKey(int(wait_ms))


def _mask_bbox(mask: np.ndarray) -> tuple[int, int, int, int]:
    ys, xs = np.where(mask > 0)
    if len(xs) == 0 or len(ys) == 0:
        return 0, 0, mask.shape[1], mask.shape[0]
    return int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1


def _draw_grasp_axes(image: np.ndarray, best_grasp: BestGrasp, intrinsics: dict) -> None:
    try:
        import cv2
    except ModuleNotFoundError:
        return
    t = best_grasp.translation
    if float(t[2]) <= 1e-6:
        return
    K = np.asarray(intrinsics.get("K"), dtype=float)
    if K.shape != (3, 3):
        return

    origin = _project(K, t)
    if origin is None:
        return
    cv2.circle(image, origin, 5, (0, 0, 255), -1)
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    for axis_idx, color in enumerate(colors):
        point = t + best_grasp.rotation_matrix[:, axis_idx] * 0.04
        projected = _project(K, point)
        if projected is not None:
            cv2.line(image, origin, projected, color, 2)


def _project(K: np.ndarray, xyz: np.ndarray) -> tuple[int, int] | None:
    z = float(xyz[2])
    if z <= 1e-6:
        return None
    uvw = K @ xyz
    return int(round(float(uvw[0] / z))), int(round(float(uvw[1] / z)))
