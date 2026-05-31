from __future__ import annotations

import base64
import io
from typing import Any

import numpy as np


def encode_npy(array: np.ndarray) -> dict[str, str]:
    """把 numpy 数组编码为 JSON 可携带的 .npy + base64。"""
    buffer = io.BytesIO()
    np.save(buffer, np.asarray(array), allow_pickle=False)
    return {
        "encoding": "npy_base64",
        "data": base64.b64encode(buffer.getvalue()).decode("ascii"),
    }


def decode_npy(payload: dict[str, Any]) -> np.ndarray:
    """仅供本工程烟测使用，服务端会自己解码请求字段。"""
    if payload.get("encoding") != "npy_base64":
        raise ValueError(f"不支持的数组编码: {payload.get('encoding')!r}")
    raw = base64.b64decode(str(payload["data"]).encode("ascii"))
    return np.load(io.BytesIO(raw), allow_pickle=False)
