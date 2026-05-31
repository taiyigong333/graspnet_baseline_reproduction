#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from eye_in_hand_graspnet.config import load_config
from eye_in_hand_graspnet.pipeline import run_wrist_grasp_cycle


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="腕部相机 eye-in-hand GraspNet UR7e 抓取循环。")
    parser.add_argument(
        "--config",
        default="configs/eye_in_hand_ur7e_wrist.jsonc",
        help="配置文件路径，默认 configs/eye_in_hand_ur7e_wrist.jsonc",
    )
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--dry-run", action="store_true", help="只计算和打印目标点，不启动示教器 URP。")
    action.add_argument(
        "--execute",
        action="store_true",
        help="启动 PC 侧 get_target 服务，并通过 Dashboard 启动示教器 URP 轮询目标。",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    config = load_config(args.config)
    execute = bool(args.execute)
    run_wrist_grasp_cycle(config, execute=execute)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
