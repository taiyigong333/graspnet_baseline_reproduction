from __future__ import annotations

import time
from typing import Any

from .config import MotionConfig, RobotConfig
from .transforms import as_pose6


class RtdeTcpReader:
    def __init__(self, config: RobotConfig):
        self.config = config

    def read_current_tcp(self) -> list[float]:
        if not self.config.rtde_enabled:
            raise RuntimeError("robot.rtde_enabled=false，不能读取当前 TCP")
        try:
            from rtde_receive import RTDEReceiveInterface
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError("未找到 rtde_receive，请在 hand_eye 环境安装 ur-rtde。") from exc

        rtde = RTDEReceiveInterface(self.config.host)
        try:
            tcp = rtde.getActualTCPPose()
            return as_pose6(tcp, "RTDE getActualTCPPose")
        finally:
            # ur-rtde 版本不一致时清理接口名称不同，逐个尝试即可。
            for method_name in ("disconnect", "stopScript"):
                method = getattr(rtde, method_name, None)
                if callable(method):
                    try:
                        method()
                    except Exception:
                        pass


class XmlRpcMotionClient:
    def __init__(self, config: MotionConfig):
        try:
            import xmlrpc.client
        except ImportError as exc:
            raise ImportError("当前 Python 无法导入 xmlrpc.client，请检查 Python 安装中的 pyexpat/XML 组件。") from exc
        self.config = config
        self._server = xmlrpc.client.ServerProxy(config.xmlrpc_url, allow_none=True)

    def move_tcp(self, pose6: list[float], *, label: str) -> Any:
        pose = as_pose6(pose6, label)
        method = getattr(self._server, self.config.move_tcp_method)
        kwargs = dict(self.config.move_tcp_kwargs)
        print(f"[motion] {label}: 调用 {self.config.move_tcp_method} pose={format_pose(pose)} kwargs={kwargs}")
        if kwargs:
            # 常见 XML-RPC 服务不支持关键字参数，因此默认把 kwargs 作为第二个 dict 参数传入。
            return method(pose, kwargs)
        return method(pose)


def maybe_move_start(client: XmlRpcMotionClient, config: MotionConfig) -> None:
    if config.fixed_start_tcp is None:
        return
    client.move_tcp(as_pose6(config.fixed_start_tcp, "motion.fixed_start_tcp"), label="start")
    if config.settle_s_after_start > 0:
        time.sleep(float(config.settle_s_after_start))


def format_pose(pose6: list[float]) -> str:
    pose = as_pose6(pose6)
    return (
        "["
        f"{pose[0]:.9f}, {pose[1]:.9f}, {pose[2]:.9f}, "
        f"{pose[3]:.9f}, {pose[4]:.9f}, {pose[5]:.9f}"
        "]"
    )
