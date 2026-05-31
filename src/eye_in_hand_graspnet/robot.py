from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import socket
import threading
import time
from typing import Any

from .config import MotionConfig, RobotConfig
from .transforms import as_pose6


_DASHBOARD_ERROR_MARKERS = (
    "file not found",
    "not found",
    "failed",
    "failure",
    "error",
)


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


class XMLRPCTargetBridge:
    """给示教器 URP 轮询的目标位姿服务。

    示教器侧通过 rpc.get_target() 获取 [x,y,z,rx,ry,rz,gripper]。
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 50000):
        self.host = str(host)
        self.port = int(port)
        self._target_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._target_gripper = 100.0
        self._request_count = 0
        self._lock = threading.Lock()
        self._server: Any | None = None
        self._thread: threading.Thread | None = None

    def set_target(self, tcp_pose: list[float], gripper: float) -> None:
        pose = as_pose6(tcp_pose, "xmlrpc target tcp_pose")
        with self._lock:
            self._target_pose = pose
            self._target_gripper = float(gripper)
        print(f"[xmlrpc] target={format_pose(pose)}, gripper={self._target_gripper:.3f}")

    def get_target(self) -> list[float]:
        with self._lock:
            self._request_count += 1
            return list(self._target_pose) + [float(self._target_gripper)]

    def request_count(self) -> int:
        with self._lock:
            return int(self._request_count)

    def start(self) -> None:
        if self._thread is not None:
            return
        # 与旧工程一致：不用 xmlrpc.server，避开部分 Windows Python 的 pyexpat/XML 组件问题。
        self._server = ThreadingHTTPServer((self.host, self.port), _TargetXMLRPCHandler)
        self._server.target_bridge = self
        self.port = int(self._server.server_address[1])
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        print(f"[xmlrpc] 已启动 get_target 服务: {self.host}:{self.port}")

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
        self._server = None
        self._thread = None

    def __enter__(self) -> "XMLRPCTargetBridge":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()


class _TargetXMLRPCHandler(BaseHTTPRequestHandler):
    server: Any

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length).decode("utf-8", errors="replace")
        if "<methodName>get_target</methodName>" not in body:
            self._send_xml(_fault_response(404, "unknown method"), status=200)
            return
        values = self.server.target_bridge.get_target()
        self._send_xml(_array_response(values), status=200)

    def do_GET(self) -> None:
        self._send_xml(_fault_response(405, "POST only"), status=405)

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _send_xml(self, text: str, status: int = 200) -> None:
        body = text.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/xml; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class URDashboardClient:
    """UR Dashboard 端口客户端，用于加载和启动示教器 URP。"""

    def __init__(self, host: str, port: int = 29999, timeout_s: float = 3.0):
        self.host = str(host)
        self.port = int(port)
        self.timeout_s = float(timeout_s)

    def command(self, command: str) -> str:
        message = command if command.endswith("\n") else command + "\n"
        with socket.create_connection((self.host, self.port), timeout=self.timeout_s) as sock:
            sock.settimeout(self.timeout_s)
            try:
                sock.recv(1024)
            except socket.timeout:
                pass
            sock.sendall(message.encode("utf-8"))
            response = sock.recv(4096).decode("utf-8", errors="replace").strip()
        print(f"[dashboard] {command} -> {response}")
        return response

    def stop(self) -> str:
        return self.command("stop")

    def load(self, program: str) -> str:
        return self.command(f"load {program}")

    def play(self) -> str:
        return self.command("play")

    def running(self) -> str:
        return self.command("running")


def run_urp_program(robot: RobotConfig, motion: MotionConfig, *, label: str = "URP") -> None:
    program = motion.grasp_program
    if not program:
        print(f"[{label}] 未配置 motion.grasp_program，跳过 Dashboard load/play。")
        return

    dashboard = URDashboardClient(robot.host, int(motion.dashboard_port))
    if motion.stop_before_load:
        dashboard.stop()
        time.sleep(0.5)
    load_response = dashboard.load(program)
    _raise_if_dashboard_failed("load", program, load_response)
    time.sleep(max(0.0, float(motion.program_settle_s)))
    if motion.play_after_load:
        play_response = dashboard.play()
        _raise_if_dashboard_failed("play", program, play_response)
    if motion.grasp_program_start_wait_s > 0:
        print(f"[{label}] 等待示教器程序启动 {motion.grasp_program_start_wait_s:.1f}s")
        time.sleep(float(motion.grasp_program_start_wait_s))
    if motion.require_grasp_program_running:
        running = dashboard.running()
        if "true" not in running.lower():
            raise RuntimeError(
                f"{label} 未保持运行，Dashboard running={running!r}。"
                "请检查示教器程序是否正在轮询 Windows PC 的 XML-RPC get_target()。"
            )


def wait_for_xmlrpc_polling(bridge: XMLRPCTargetBridge, config: MotionConfig) -> None:
    if not config.require_xmlrpc_polling:
        return
    wait_s = float(config.xmlrpc_poll_check_s)
    before = bridge.request_count()
    time.sleep(max(0.0, wait_s))
    after = bridge.request_count()
    print(f"[xmlrpc] 示教器 get_target 请求数 +{after - before}, total={after}")
    if after <= before:
        raise RuntimeError(
            "Dashboard 显示 URP 已启动，但示教器没有访问 PC XML-RPC get_target()。"
            "请检查示教器 RPC URL、Windows 防火墙、PC IP 和 motion.xmlrpc_port。"
        )


def set_target_and_wait(
    bridge: XMLRPCTargetBridge,
    tcp_pose: list[float],
    gripper: float,
    wait_s: float,
    *,
    label: str,
) -> None:
    pose = as_pose6(tcp_pose, label)
    print(f"[motion-state] {label}")
    print(f"[motion] {label}: tcp={format_pose(pose)}, gripper={float(gripper):.3f}, wait_s={wait_s:.1f}")
    bridge.set_target(pose, float(gripper))
    time.sleep(max(0.0, float(wait_s)))


def maybe_set_start_target(bridge: XMLRPCTargetBridge, config: MotionConfig) -> None:
    if config.fixed_start_tcp is None:
        return
    wait_s = float(config.motion_waits_s.get("start", config.settle_s_after_start))
    set_target_and_wait(
        bridge,
        as_pose6(config.fixed_start_tcp, "motion.fixed_start_tcp"),
        config.open_gripper,
        wait_s,
        label="start",
    )


def resolve_motion_waits_s(config: MotionConfig) -> dict[str, float]:
    labels = ("start", "open-current", "move-pregrasp", "move-grasp", "close-gripper")
    default_wait_s = float(config.motion_wait_s)
    if default_wait_s < 0:
        raise ValueError("motion.motion_wait_s 不能为负数。")
    waits: dict[str, float] = {}
    for label in labels:
        wait_s = float(config.motion_waits_s.get(label, default_wait_s))
        if wait_s < 0:
            raise ValueError(f"motion.motion_waits_s.{label} 不能为负数。")
        waits[label] = wait_s
    return waits


def _array_response(values: list[float]) -> str:
    items = "\n".join(f"<value><double>{float(value):.17g}</double></value>" for value in values)
    return (
        "<?xml version='1.0'?>\n"
        "<methodResponse><params><param><value><array><data>\n"
        f"{items}\n"
        "</data></array></value></param></params></methodResponse>"
    )


def _fault_response(code: int, message: str) -> str:
    return (
        "<?xml version='1.0'?>\n"
        "<methodResponse><fault><value><struct>"
        f"<member><name>faultCode</name><value><int>{int(code)}</int></value></member>"
        f"<member><name>faultString</name><value><string>{message}</string></value></member>"
        "</struct></value></fault></methodResponse>"
    )


def _raise_if_dashboard_failed(command: str, program: str, response: str) -> None:
    response_text = str(response).strip()
    response_lower = response_text.lower()
    if any(marker in response_lower for marker in _DASHBOARD_ERROR_MARKERS):
        raise RuntimeError(f"Dashboard {command} {program!r} 失败：{response_text}")


def format_pose(pose6: list[float]) -> str:
    pose = as_pose6(pose6)
    return (
        "["
        f"{pose[0]:.9f}, {pose[1]:.9f}, {pose[2]:.9f}, "
        f"{pose[3]:.9f}, {pose[4]:.9f}, {pose[5]:.9f}"
        "]"
    )
