"""Sharpa 真机遥操、力反馈与实时接管 Web 控制服务（完全独立于外部仓库）。"""
from __future__ import annotations

from collections import deque
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
SHARPA_ROOT = ROOT / "sharpa_teleop"
WEB_CONFIG_PATH = SHARPA_ROOT / "config/web_config.json"
PIXI_PYTHON = SHARPA_ROOT / ".pixi/envs/default/bin/python"
FK_WORKER_SCRIPT = SHARPA_ROOT / "tools/scene_fk_worker.py"
DEFAULT_MAPPING = ROOT / "hardware/encos_g4_gateway_12dof/config/right_sim_mapping_12dof.json"
RECORDINGS_DIR = SHARPA_ROOT / "recordings"

from motorhand.hand_profiles import AVAILABLE_HAND_MODELS, get_hand_profile, get_hand_runtime

SHARPA_FINGER_PREFIXES = ("thumb", "index", "middle", "ring", "pinky")


class SharpaSceneBridge:
    """独立维护 MotorHand (12-DOF VE40 / 16-DOF) + Sharpa WaveSE 并排 3D GLB 与实时 FK 姿态。"""

    def __init__(self, hand: str = "right", glove_model: str = "12dof"):
        self._lock = threading.Lock()
        self._hand = hand
        self._glove_model = get_hand_profile(glove_model).model_id
        self._proc: subprocess.Popen | None = None
        self._glb_path = SHARPA_ROOT / f".cache/sharpa_scene_{hand}_{self._glove_model}.glb"
        self._glb_bytes: bytes | None = None
        self._object_pos: list[float] = [0.065, 0.0, 0.26]
        self._object_radius: float = 0.035
        self._bodies: dict[str, list[float]] = {}
        self._seq: int = 0
        self._ready: bool = False
        self._last_fk_ts: float = 0.0
        self._error: str | None = None
        self._start_worker(hand, self._glove_model)

    def _start_worker(self, hand: str, glove_model: str) -> None:
        self.close()
        if not PIXI_PYTHON.is_file() or not FK_WORKER_SCRIPT.is_file():
            self._error = "缺少 sharpa_teleop/.pixi 或 scene_fk_worker.py"
            return
        self._hand = hand
        self._glove_model = get_hand_profile(glove_model).model_id
        self._glb_path = SHARPA_ROOT / f".cache/sharpa_scene_{hand}_{self._glove_model}.glb"
        try:
            proc = subprocess.Popen(
                [
                    str(PIXI_PYTHON), str(FK_WORKER_SCRIPT),
                    "--hand", hand,
                    "--glove-model", self._glove_model,
                    "--glb-out", str(self._glb_path),
                ],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                text=True, bufsize=1,
            )
            line = proc.stdout.readline() if proc.stdout else ""
            if not line:
                proc.terminate()
                self._error = "3D FK 工作进程启动无响应"
                return
            info = json.loads(line)
            if not info.get("ready"):
                proc.terminate()
                self._error = "3D FK 工作进程初始化失败"
                return
            glb_bytes = self._glb_path.read_bytes()
            with self._lock:
                self._proc = proc
                self._glb_bytes = glb_bytes
                self._object_pos = info.get("object_pos", self._object_pos)
                self._object_radius = float(info.get("object_radius", 0.035))
                self._bodies = info.get("bodies", {})
                self._seq += 1
                self._ready = True
                self._error = None
        except (OSError, ValueError) as exc:
            self._error = str(exc)

    def ensure_hand(self, hand: str, glove_model: str | None = None) -> None:
        target_model = get_hand_profile(glove_model).model_id if glove_model else self._glove_model
        if hand in ("right", "left") and (hand != self._hand or target_model != self._glove_model):
            self._start_worker(hand, target_model)

    def update_from_frame(self, frame: dict) -> None:
        now = time.monotonic()
        with self._lock:
            if not self._ready or self._proc is None or self._proc.poll() is not None:
                return
            if now - self._last_fk_ts < 0.028:  # ~35 Hz 上限，避免阻塞 UDP 接收
                return
            self._last_fk_ts = now
            proc = self._proc
        payload = {
            "glove_tree_rad": frame.get("glove_tree_rad"),
            "joint_names": frame.get("joint_names"),
            "position_rad": frame.get("position_rad"),
            "qpos": frame.get("qpos"),
            "simulated": bool(frame.get("simulated", False)),
        }
        try:
            assert proc.stdin is not None and proc.stdout is not None
            proc.stdin.write(json.dumps(payload, separators=(",", ":")) + "\n")
            proc.stdin.flush()
            resp_line = proc.stdout.readline()
            if not resp_line:
                return
            resp = json.loads(resp_line)
            bodies = resp.get("bodies")
            if isinstance(bodies, dict):
                with self._lock:
                    self._bodies = bodies
                    self._seq += 1
        except (OSError, ValueError):
            pass

    def glb_bytes(self, hand: str | None = None, glove_model: str | None = None) -> bytes | None:
        target_hand = hand if hand in ("right", "left") else self._hand
        target_model = get_hand_profile(glove_model).model_id if glove_model else self._glove_model
        if target_hand != self._hand or target_model != self._glove_model:
            self._start_worker(target_hand, target_model)
        with self._lock:
            return self._glb_bytes

    def snapshot(self, *, running: bool, simulated: bool, show_object: bool,
                 contact_fingers: list[str]) -> dict:
        with self._lock:
            return {
                "ready": self._ready,
                "error": self._error,
                "hand": self._hand,
                "glove_model": self._glove_model,
                "dof": get_hand_profile(self._glove_model).dof,
                "seq": self._seq,
                "running": running,
                "simulated": simulated,
                "show_object": show_object,
                "object_pos": self._object_pos,
                "object_radius": self._object_radius,
                "contact_fingers": contact_fingers,
                "bodies": self._bodies,
            }

    def close(self) -> None:
        with self._lock:
            proc = self._proc
            self._proc = None
            self._ready = False
        if proc is not None:
            try:
                if proc.stdin:
                    proc.stdin.close()
                if proc.stdout:
                    proc.stdout.close()
                proc.terminate()
                proc.wait(timeout=2.0)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    proc.kill()
                except OSError:
                    pass


def load_web_config() -> dict:
    """读取独立的 Sharpa Web 默认配置（sharpa_teleop/config/web_config.json）。"""
    defaults = {
        "default_mode": "force-contact",
        "default_hand": "right",
        "approved_firmware": "3.0.10",
        "expected_pn_right": "WaveSE-R-01",
        "expected_pn_left": "WaveSE-L-01",
        "default_hardness": 0.3,
        "default_rate_hz": 50,
        "skip_glove_home_default": True,
        "sim_with_object_default": True,
        "glove_driver": 2,
    }
    if WEB_CONFIG_PATH.is_file():
        try:
            loaded = json.loads(WEB_CONFIG_PATH.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                defaults.update(loaded)
        except (OSError, ValueError):
            pass
    return defaults

VALID_MODES = (
    "force-contact",
    "force-linear",
    "teleop",
    "takeover",
    "replay",
    "record",
    "sim-teleop",
    "sim",
)
SIM_MODES = {"sim", "sim-teleop"}
HARDWARE_OUTPUT_MODES = {"force-contact", "force-linear", "teleop", "takeover", "replay"}


def _extract_json_from_stdout(stdout: str, expect_type: type):
    """Sharpa SDK 会向 stdout 输出 spdlog 行，倒序提取真正的 JSON 结果行。"""
    for line in reversed(stdout.splitlines()):
        text = line.strip()
        if not text:
            continue
        if expect_type is list and not text.startswith("["):
            continue
        if expect_type is dict and not text.startswith("{"):
            continue
        try:
            obj = json.loads(text)
        except json.JSONDecodeError:
            continue
        if isinstance(obj, expect_type):
            return obj
    raise RuntimeError(f"未能从 motorhand_sharpa 输出中解析有效 JSON：{stdout.strip()[-200:]}")


def _free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def resolve_sharpa_binary(simulation: bool = False) -> Path | None:
    """优先使用含真实 SDK 的构建产物；纯仿真模式可回退到无 SDK 构建。"""
    hw_bin = SHARPA_ROOT / ".cache/sharpa-control/motorhand_sharpa"
    sim_bin = SHARPA_ROOT / ".cache/sharpa-sim-control/motorhand_sharpa"
    if simulation:
        for cand in (hw_bin, sim_bin):
            if cand.is_file():
                return cand
        return None
    return hw_bin if hw_bin.is_file() else None


def build_runtime_mapping(zero_path: Path | None, base_mapping: Path | None = None,
                          hand_model: str = "12dof") -> Path:
    """将当前标零文件中的 12/16 轴零偏同步叠加到对应型号映射文件，使 Sharpa 链路复用同一套零位。"""
    spec = get_hand_profile(hand_model)
    source = base_mapping if base_mapping is not None else spec.mapping_path
    data = json.loads(source.read_text(encoding="utf-8"))
    if zero_path is not None and zero_path.is_file():
        try:
            saved = json.loads(zero_path.read_text(encoding="utf-8"))
            zero_deg = saved.get("zero_deg", {})
            motors = data.get("motors", {})
            for key, entry in motors.items():
                if key in zero_deg:
                    new_zero = float(zero_deg[key])
                    old_zero = float(entry["zero_deg"])
                    delta = new_zero - old_zero
                    entry["zero_deg"] = new_zero
                    entry["min_deg"] = float(entry["min_deg"]) + delta
                    entry["max_deg"] = float(entry["max_deg"]) + delta
        except (OSError, ValueError, KeyError, TypeError):
            pass
    out = SHARPA_ROOT / f".cache/runtime_glove_mapping_{spec.model_id}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    tmp.replace(out)
    return out


class SharpaManager:
    """管理 motorhand_sharpa 子进程、UDP 实时遥测与非阻塞接管/停止控制（并列支持 12-DOF 与 16-DOF）。"""

    def __init__(self, *, glove_port: str | None = None, zero_path: Path | None = None,
                 binary_override: Path | None = None, hand_model: str = "12dof"):
        self.glove_port = glove_port or ""
        self.zero_path = zero_path
        self.binary_override = binary_override
        self._lock = threading.RLock()
        self._hand_model: str = get_hand_profile(hand_model).model_id
        self._proc: subprocess.Popen | None = None
        self._mode: str = ""
        self._hand: str = "right"
        self._serial: str = ""
        self._telemetry_sock: socket.socket | None = None
        self._telemetry_thread: threading.Thread | None = None
        self._log_thread: threading.Thread | None = None
        self._control_port: int = 0
        self._stop_rx = threading.Event()
        self._latest_frame: dict | None = None
        self._latest_frame_ts: float = 0.0
        self._logs: deque[str] = deque(maxlen=80)
        self._discovered: list[dict] = []
        self._last_inspect: dict | None = None
        self._last_error: str | None = None
        self._exit_code: int | None = None
        self._active_output: str = ""
        self._with_object: bool = False
        self._demo_axes_active: bool = False
        self._demo_axes: list[float] = [0.0] * get_hand_profile(self._hand_model).dof
        self._scene_bridge: SharpaSceneBridge | None = None

    def set_hand_model(self, hand_model: str) -> dict:
        spec = get_hand_profile(hand_model)
        if self.is_running():
            self.stop()
        with self._lock:
            self._hand_model = spec.model_id
            self._demo_axes_active = False
            self._demo_axes = [0.0] * spec.dof
            self._latest_frame = None
            self._latest_frame_ts = 0.0
            self._last_error = None
        if self._scene_bridge is not None:
            self._scene_bridge.ensure_hand(self._hand, spec.model_id)
        return self.status()

    def attach_scene_bridge(self, bridge: SharpaSceneBridge) -> None:
        self._scene_bridge = bridge

    def _binary(self, simulation: bool = False) -> Path:
        if self.binary_override is not None:
            return self.binary_override
        path = resolve_sharpa_binary(simulation=simulation)
        if path is None:
            raise RuntimeError("未找到编译好的 motorhand_sharpa 可执行文件，请在 sharpa_teleop/ 执行 pixi run build")
        return path

    def _append_log(self, line: str) -> None:
        text = line.rstrip("\r\n")
        if not text:
            return
        stamp = datetime.now().strftime("%H:%M:%S")
        with self._lock:
            self._logs.append(f"[{stamp}] {text}")

    def list_recordings(self) -> list[dict]:
        RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
        items = []
        for p in sorted(RECORDINGS_DIR.glob("*.jsonl"), key=lambda x: x.stat().st_mtime, reverse=True):
            st = p.stat()
            items.append({
                "name": p.name,
                "path": str(p),
                "size_kb": round(st.st_size / 1024.0, 1),
                "mtime": datetime.fromtimestamp(st.st_mtime).strftime("%Y-%m-%d %H:%M:%S"),
            })
        return items

    def is_running(self) -> bool:
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    def discover(self) -> list[dict]:
        if self.is_running():
            raise ValueError("控制会话运行期间不能执行设备发现，请先安全停止")
        binary = self._binary(simulation=False)
        proc = subprocess.run(
            [str(binary), "discover", "--root", str(SHARPA_ROOT)],
            capture_output=True, text=True, timeout=10, check=False,
        )
        if proc.returncode != 0:
            err = (proc.stderr or proc.stdout or "设备发现失败").strip()
            with self._lock:
                self._last_error = err
            self._append_log(f"discover 失败: {err}")
            raise RuntimeError(err)
        payload = _extract_json_from_stdout(proc.stdout, list)
        with self._lock:
            self._discovered = payload
            self._last_error = None
        self._append_log(f"发现 {len(payload)} 台 Sharpa 设备")
        return payload

    def clear_link_latch(self, serial: str) -> dict:
        if self.is_running():
            raise ValueError("控制会话运行期间不能执行清除网线锁存，请先安全停止")
        if not serial or not serial.strip():
            raise ValueError("请先填写或发现 Sharpa 序列号 (SN)")
        binary = self._binary(simulation=False)
        proc = subprocess.run(
            [str(binary), "clear-link-latch", "--root", str(SHARPA_ROOT), "--serial", serial.strip()],
            capture_output=True, text=True, timeout=12, check=False,
        )
        if proc.returncode != 0:
            err = (proc.stderr or proc.stdout or "清除网线锁存失败").strip()
            with self._lock:
                self._last_error = err
            self._append_log(f"clear-link-latch 失败: {err}")
            raise RuntimeError(err)
        data = _extract_json_from_stdout(proc.stdout, dict)
        with self._lock:
            self._last_error = None
        self._append_log(f"已清除网线锁存 (0x0907): {data}")
        return data

    def inspect(self, serial: str, *, hand: str = "right", firmware: str = "3.0.10", pn: str = "") -> dict:
        if self.is_running():
            raise ValueError("控制会话运行期间不能执行只读体检，请先安全停止")
        if not serial or not serial.strip():
            raise ValueError("请先填写或发现 Sharpa 序列号 (SN)")
        if hand not in ("right", "left"):
            raise ValueError("手侧必须为 right 或 left")
        binary = self._binary(simulation=False)
        cmd = [str(binary), "inspect", "--root", str(SHARPA_ROOT),
               "--serial", serial.strip(), "--hand", hand]
        if firmware.strip():
            cmd += ["--approved-firmware", firmware.strip()]
        if pn.strip():
            cmd += ["--expected-pn", pn.strip()]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=12, check=False)
        if proc.returncode != 0:
            err = (proc.stderr or proc.stdout or "只读体检失败").strip()
            with self._lock:
                self._last_error = err
            self._append_log(f"inspect 失败: {err}")
            raise RuntimeError(err)
        data = _extract_json_from_stdout(proc.stdout, dict)
        with self._lock:
            self._last_inspect = data
            self._last_error = None
        self._append_log(f"体检通过: {data.get('serial')} ({data.get('firmware')}) fault={data.get('fault')}")
        return data

    def start(self, options: dict) -> dict:
        replace_running = bool(options.get("replace_running", False))
        with self._lock:
            is_running = self._proc is not None and self._proc.poll() is None
            running_sim = self._mode in SIM_MODES or (self._latest_frame or {}).get("simulated", False)
        if is_running:
            if replace_running and running_sim:
                self._append_log("自动停止离线仿真，准备切换至真机模式…")
                self.stop()
            else:
                raise ValueError("Sharpa 会话正在运行，请先安全停止")

        mode = str(options.get("mode", "force-contact")).strip()
        if mode not in VALID_MODES:
            raise ValueError(f"不支持的 Sharpa 模式: {mode}")
        simulation = mode in SIM_MODES or bool(options.get("simulation", False))
        hand = str(options.get("hand", "right")).strip()
        if hand not in ("right", "left"):
            raise ValueError("手侧必须为 right 或 left")
        serial = str(options.get("serial", "")).strip()
        if not simulation and not serial:
            raise ValueError("真机模式必须填写或先发现 Sharpa 序列号 (SN)")
        execute = bool(options.get("execute", False))
        feedback = bool(options.get("feedback", mode in ("force-contact", "force-linear", "takeover")))
        needs_execute = (not simulation and mode in HARDWARE_OUTPUT_MODES) or (mode == "sim-teleop" and feedback)
        if needs_execute and not execute:
            raise ValueError("真机出力前请先勾选「确认允许真机出力 (--execute)」")

        hardness = float(options.get("hardness", 0.3))
        if not 0.0 <= hardness <= 1.0:
            raise ValueError("力反馈硬度应在 0.0 ~ 1.0 之间")
        rate = int(options.get("rate", 50))
        if not 1 <= rate <= 200:
            raise ValueError("控制频率应在 1 ~ 200 Hz 之间")

        trajectory = str(options.get("trajectory", "")).strip()
        if mode in ("replay", "takeover"):
            if not trajectory:
                raise ValueError("回放/接管模式必须选择一条 22 轴轨迹文件 (.jsonl)")
            traj_path = Path(trajectory)
            if not traj_path.is_absolute():
                traj_path = (RECORDINGS_DIR / traj_path).resolve()
            if not traj_path.is_file():
                raise ValueError(f"轨迹文件不存在: {traj_path}")
            trajectory = str(traj_path)

        output_path = ""
        if mode == "record" or bool(options.get("record_output", False)):
            RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
            custom = str(options.get("output_name", "")).strip()
            if custom:
                safe_name = Path(custom).name
                if not safe_name.endswith(".jsonl"):
                    safe_name += ".jsonl"
            else:
                safe_name = f"sharpa-{mode}-{datetime.now().strftime('%Y%m%d-%H%M%S')}.jsonl"
            output_path = str(RECORDINGS_DIR / safe_name)

        glove_model = get_hand_profile(options.get("glove_model") or options.get("hand_model") or self._hand_model).model_id
        spec = get_hand_profile(glove_model)
        binary = self._binary(simulation=simulation)
        telemetry_port = _free_udp_port()
        control_port = _free_udp_port()
        zero_for_model = self.zero_path if (self.zero_path is not None and glove_model == self._hand_model) else spec.default_zero_path
        mapping_path = build_runtime_mapping(zero_for_model, hand_model=glove_model)

        cmd = [
            str(binary), mode,
            "--root", str(SHARPA_ROOT),
            "--hand", hand,
            "--glove-model", glove_model,
            "--mapping", str(mapping_path),
            "--rate", str(rate),
            "--curves-port", str(telemetry_port),
            "--control-port", str(control_port),
            "--feedback-hardness", f"{hardness:.3f}",
        ]
        if simulation and mode not in SIM_MODES:
            cmd.append("--simulation")
            if bool(options.get("fake_glove", False)):
                cmd.append("--fake-glove")
        if serial and not simulation:
            cmd += ["--serial", serial]
        firmware = str(options.get("firmware", "")).strip()
        if firmware and not simulation:
            cmd += ["--approved-firmware", firmware]
        pn = str(options.get("pn", "")).strip()
        if pn and not simulation:
            cmd += ["--expected-pn", pn]
        if execute:
            cmd.append("--execute")
        if mode == "sim-teleop" and feedback:
            cmd.append("--feedback")
        if simulation and bool(options.get("with_object", True)):
            cmd.append("--with-object")
        if bool(options.get("skip_glove_home", True)) and not simulation:
            cmd.append("--no-glove-home")
        if trajectory:
            cmd += ["--trajectory", trajectory]
        if output_path:
            cmd += ["--output", output_path]

        env = os.environ.copy()
        env["MOTORHAND_GLOVE_DRIVER"] = "2"
        if self.glove_port:
            env["MOTORHAND_GLOVE_PORT"] = self.glove_port

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", telemetry_port))
        sock.settimeout(0.1)

        with self._lock:
            self._stop_rx.clear()
            self._telemetry_sock = sock
            self._control_port = control_port
            self._hand_model = glove_model
            self._mode = mode
            self._hand = hand
            self._serial = serial or ("SIM-SHARPA-LEFT" if hand == "left" else "SIM-SHARPA-RIGHT")
            self._latest_frame = None
            self._latest_frame_ts = 0.0
            self._last_error = None
            self._exit_code = None
            self._active_output = output_path
            self._with_object = bool(simulation and options.get("with_object", True))
            self._demo_axes_active = False
            self._demo_axes = [0.0] * spec.dof
            self._logs.clear()
        if self._scene_bridge is not None:
            self._scene_bridge.ensure_hand(hand, glove_model)

        self._append_log("启动命令: " + " ".join(cmd))
        try:
            proc = subprocess.Popen(
                cmd,
                cwd=str(SHARPA_ROOT),
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            sock.close()
            with self._lock:
                self._telemetry_sock = None
                self._control_port = 0
                self._last_error = str(exc)
            raise

        with self._lock:
            self._proc = proc

        self._telemetry_thread = threading.Thread(target=self._rx_loop, args=(sock,), daemon=True)
        self._telemetry_thread.start()
        self._log_thread = threading.Thread(target=self._log_loop, args=(proc,), daemon=True)
        self._log_thread.start()
        return self.status()

    def _rx_loop(self, sock: socket.socket) -> None:
        while not self._stop_rx.is_set():
            try:
                data, _ = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                frame = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if isinstance(frame, dict):
                with self._lock:
                    self._latest_frame = frame
                    self._latest_frame_ts = time.monotonic()
                if self._scene_bridge is not None:
                    self._scene_bridge.update_from_frame(frame)

    def _log_loop(self, proc: subprocess.Popen) -> None:
        assert proc.stdout is not None
        try:
            for line in proc.stdout:
                self._append_log(line)
        except OSError:
            pass
        code = proc.wait()
        with self._lock:
            self._exit_code = code
            if code not in (0, -signal.SIGINT, -signal.SIGTERM) and not self._last_error:
                tail = list(self._logs)[-1] if self._logs else f"进程异常退出 ({code})"
                self._last_error = tail
        self._append_log(f"motorhand_sharpa 已退出 (code={code})")

    def send_control(self, command: str) -> dict:
        if command not in ("toggle", "takeover", "replay", "stop"):
            raise ValueError(f"无效控制命令: {command}")
        with self._lock:
            running = self._proc is not None and self._proc.poll() is None
            port = self._control_port
        if not running or not port:
            raise ValueError("当前没有运行中的 Sharpa 控制会话")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(command.encode("ascii"), ("127.0.0.1", port))
        self._append_log(f"已发送实时控制指令: {command}")
        return self.status()

    def send_demo_axes(self, values: list[float] | None, *, active: bool | None = None) -> dict:
        with self._lock:
            running = self._proc is not None and self._proc.poll() is None
            port = self._control_port
            spec = get_hand_profile(self._hand_model)
        if not running or not port:
            raise ValueError("请先启动 Sharpa 离线仿真 (sim) 再使用交互式全轴控制")
        use_manual = (values is not None) if active is None else bool(active)
        if not use_manual:
            payload = "axes auto"
            with self._lock:
                self._demo_axes_active = False
        else:
            if not isinstance(values, list) or len(values) != spec.dof:
                raise ValueError(f"演示关节数组长度应为 {spec.dof}")
            clean = [float(v) for v in values]
            payload = "axes " + " ".join(f"{v:.5f}" for v in clean)
            with self._lock:
                self._demo_axes_active = True
                self._demo_axes = clean
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(payload.encode("ascii"), ("127.0.0.1", port))
        return self.status()

    def stop(self) -> dict:
        with self._lock:
            proc = self._proc
            port = self._control_port
        if proc is not None and proc.poll() is None:
            if port:
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                        s.sendto(b"stop", ("127.0.0.1", port))
                except OSError:
                    pass
            try:
                proc.send_signal(signal.SIGINT)
            except OSError:
                pass
            try:
                proc.wait(timeout=6.0)
            except subprocess.TimeoutExpired:
                self._append_log("等待优雅停机超时，发送 SIGTERM")
                try:
                    proc.terminate()
                    proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=2.0)
        self._stop_rx.set()
        with self._lock:
            sock = self._telemetry_sock
            self._telemetry_sock = None
            self._control_port = 0
            if proc is not None:
                self._exit_code = proc.poll()
                if proc.stdout is not None:
                    try:
                        proc.stdout.close()
                    except OSError:
                        pass
            self._proc = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        return self.status()

    def scene_snapshot(self) -> dict:
        if self._scene_bridge is None:
            return {"ready": False, "seq": 0}
        with self._lock:
            running = self._proc is not None and self._proc.poll() is None
            frame = self._latest_frame
            simulated = bool(running and (self._mode in SIM_MODES or (frame and frame.get("simulated"))))
            show_object = bool(running and simulated and self._with_object)
            contact_fingers: list[str] = []
            if running and isinstance(frame, dict):
                contacts = frame.get("contact") or []
                names = frame.get("joint_names") or []
                seen = set()
                for name, hit in zip(names, contacts):
                    if hit:
                        for prefix in SHARPA_FINGER_PREFIXES:
                            if prefix in name:
                                seen.add(prefix)
                contact_fingers = sorted(seen)
        return self._scene_bridge.snapshot(
            running=running, simulated=simulated,
            show_object=show_object, contact_fingers=contact_fingers,
        )

    def status(self) -> dict:
        with self._lock:
            running = self._proc is not None and self._proc.poll() is None
            pid = self._proc.pid if running and self._proc is not None else None
            frame = dict(self._latest_frame) if self._latest_frame is not None else None
            age_ms = (time.monotonic() - self._latest_frame_ts) * 1000.0 if self._latest_frame_ts else None
            # 去掉 68 维大数组，避免状态轮询冗余体积
            if frame is not None and "glove_tree_rad" in frame:
                frame.pop("glove_tree_rad", None)
            if frame is not None and "qpos" in frame:
                frame.pop("qpos", None)
            hw_bin = resolve_sharpa_binary(simulation=False)
            sim_bin = resolve_sharpa_binary(simulation=True)
            rt = get_hand_runtime(self._hand_model)
            spec = rt.spec
            mp = rt.load_mapping()
            demo_axes_joints = [m.joint for m in mp.motors]
            demo_axes_limits = [list(m.model_rad_limits()) for m in mp.motors]
            return {
                "binary_ready": hw_bin is not None,
                "sim_binary_ready": sim_bin is not None,
                "binary_path": str(hw_bin or sim_bin or ""),
                "hand_model": spec.model_id,
                "hand_model_label": spec.label,
                "dof": spec.dof,
                "available_hand_models": list(AVAILABLE_HAND_MODELS),
                "demo_axes_active": bool(frame.get("demo_axes_active")) if (frame is not None and "demo_axes_active" in frame) else self._demo_axes_active,
                "demo_axes": list(self._demo_axes),
                "demo_axes_joints": demo_axes_joints,
                "demo_axes_limits": demo_axes_limits,
                "running": running,
                "running_simulated": bool(running and (self._mode in SIM_MODES or (frame and frame.get("simulated")))),
                "pid": pid,
                "mode": self._mode,
                "hand": self._hand,
                "serial": self._serial,
                "can_takeover": bool(running and (self._mode == "takeover" or (frame and frame.get("can_takeover")))),
                "takeover": bool(frame.get("takeover")) if frame is not None else False,
                "exit_code": self._exit_code,
                "last_error": self._last_error,
                "active_output": self._active_output,
                "glove_port": self.glove_port,
                "zero_path": str(self.zero_path) if self.zero_path else None,
                "discovered_devices": list(self._discovered),
                "last_inspect": self._last_inspect,
                "recordings": self.list_recordings(),
                "web_config": load_web_config(),
                "logs": list(self._logs)[-40:],
                "frame": frame,
                "frame_age_ms": round(age_ms, 1) if age_ms is not None else None,
            }


def run_sharpa_web_host(port: int, web_control, glove_port: str | None,
                        zero_path: Path | None, stop_event) -> None:
    """在切换到 sharpa_real 标签时承载同一端口的 Web 控制台与 Sharpa 独立控制进程。"""
    initial_model = web_control.current_hand_model() if hasattr(web_control, "current_hand_model") else "12dof"
    manager = SharpaManager(glove_port=glove_port, zero_path=zero_path, hand_model=initial_model)
    scene_bridge = SharpaSceneBridge(hand=load_web_config().get("default_hand", "right"), glove_model=initial_model)
    manager.attach_scene_bridge(scene_bridge)
    tools_dir = Path(__file__).resolve().parent

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def reply(self, status, body: bytes, content_type: str):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Pragma", "no-cache")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                self.close_connection = True

        def reply_json(self, status, obj):
            self.reply(status, json.dumps(obj, ensure_ascii=False).encode("utf-8"), "application/json")

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/":
                script = tools_dir / "ve33_sharpa.js"
                viewer = tools_dir / "quest_view/build/sharpa-viewer.js"
                ver = script.stat().st_mtime_ns if script.is_file() else 0
                vver = viewer.stat().st_mtime_ns if viewer.is_file() else 0
                html = (tools_dir / "ve33_sharpa.html").read_text(encoding="utf-8")
                html = html.replace('src="/sharpa-control.js"', f'src="/sharpa-control.js?v={ver}"')
                html = html.replace('src="/sharpa-viewer.js"', f'src="/sharpa-viewer.js?v={vver}"')
                self.reply(200, html.encode("utf-8"), "text/html; charset=utf-8")
            elif path == "/sharpa-control.js":
                script = tools_dir / "ve33_sharpa.js"
                if not script.is_file():
                    return self.send_error(404)
                self.reply(200, script.read_bytes(), "text/javascript; charset=utf-8")
            elif path == "/sharpa-viewer.js":
                viewer = tools_dir / "quest_view/build/sharpa-viewer.js"
                if not viewer.is_file():
                    return self.send_error(404)
                self.reply(200, viewer.read_bytes(), "text/javascript; charset=utf-8")
            elif path == "/api/v1/sharpa-model.glb":
                hand = "left" if "hand=left" in self.path else "right"
                gm = "16dof" if "glove_model=16dof" in self.path else (
                    "12dof" if "glove_model=12dof" in self.path else manager._hand_model
                )
                glb = scene_bridge.glb_bytes(hand, gm)
                if glb is None:
                    return self.send_error(503, "Sharpa 3D GLB not ready")
                self.reply(200, glb, "model/gltf-binary")
            elif path == "/api/v1/sharpa-scene":
                self.reply_json(200, manager.scene_snapshot())
            elif path == "/api/v1/hand-model":
                spec = get_hand_profile(manager._hand_model)
                self.reply_json(200, {
                    "hand_model": spec.model_id,
                    "label": spec.label,
                    "dof": spec.dof,
                    "available_hand_models": list(AVAILABLE_HAND_MODELS),
                })
            elif path == "/api/v1/control":
                state = web_control.read()
                spec = get_hand_profile(manager._hand_model)
                state["task"] = "sharpa_real"
                state["available_tasks"] = list(web_control.task_names)
                state["hand_model"] = spec.model_id
                state["hand_model_label"] = spec.label
                state["dof"] = spec.dof
                state["available_hand_models"] = list(AVAILABLE_HAND_MODELS)
                state["sharpa"] = manager.status()
                self.reply_json(200, state)
            elif path == "/api/v1/sharpa":
                self.reply_json(200, manager.status())
            else:
                self.send_error(404)

        def do_POST(self):
            origin = self.headers.get("Origin")
            allowed = {f"http://{host}:{self.server.server_port}" for host in ("localhost", "127.0.0.1")}
            if origin and origin not in allowed:
                return self.send_error(403)
            if self.headers.get_content_type() != "application/json":
                return self.send_error(415)
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 16384:
                    raise ValueError("包长无效")
                message = json.loads(self.rfile.read(size))
                if not isinstance(message, dict):
                    raise ValueError("请求必须是对象")
                if self.path == "/api/v1/hand-model":
                    target = str(message.get("hand_model") or message.get("model") or "").strip().lower()
                    spec = get_hand_profile(target)
                    if manager.is_running():
                        manager.stop()
                    web_control.command({"action": "switch_hand_model", "hand_model": spec.model_id})
                    self.reply_json(200, {"ok": True, "requested_hand_model": spec.model_id})
                elif self.path == "/api/v1/demo-axes":
                    st = manager.send_demo_axes(message.get("values"), active=message.get("active"))
                    self.reply_json(200, {"ok": True, "status": st})
                elif self.path == "/api/v1/control":
                    action = message.get("action")
                    if action == "switch_task":
                        if manager.is_running():
                            manager.stop()
                        web_control.command(message)
                        self.reply(HTTPStatus.NO_CONTENT, b"", "text/plain")
                    elif action == "switch_hand_model":
                        if manager.is_running():
                            manager.stop()
                        web_control.command(message)
                        self.reply(HTTPStatus.NO_CONTENT, b"", "text/plain")
                    else:
                        raise ValueError("Sharpa 模式下请使用 Sharpa 控制台操作")
                elif self.path == "/api/v1/sharpa":
                    action = message.get("action")
                    if action == "discover":
                        devices = manager.discover()
                        self.reply_json(200, {"ok": True, "devices": devices, "status": manager.status()})
                    elif action == "clear_link_latch":
                        res = manager.clear_link_latch(str(message.get("serial", "")))
                        self.reply_json(200, {"ok": True, "result": res, "status": manager.status()})
                    elif action == "inspect":
                        info = manager.inspect(
                            str(message.get("serial", "")),
                            hand=str(message.get("hand", "right")),
                            firmware=str(message.get("firmware", "3.0.10")),
                            pn=str(message.get("pn", "")),
                        )
                        self.reply_json(200, {"ok": True, "inspect": info, "status": manager.status()})
                    elif action == "start":
                        st = manager.start(message)
                        self.reply_json(200, {"ok": True, "status": st})
                    elif action == "stop":
                        st = manager.stop()
                        self.reply_json(200, {"ok": True, "status": st})
                    elif action == "toggle_takeover":
                        st = manager.send_control("toggle")
                        self.reply_json(200, {"ok": True, "status": st})
                    elif action == "set_takeover":
                        cmd = "takeover" if bool(message.get("takeover")) else "replay"
                        st = manager.send_control(cmd)
                        self.reply_json(200, {"ok": True, "status": st})
                    elif action == "demo_axes":
                        st = manager.send_demo_axes(message.get("values"), active=message.get("active"))
                        self.reply_json(200, {"ok": True, "status": st})
                    elif action == "refresh":
                        self.reply_json(200, {"ok": True, "status": manager.status()})
                    else:
                        raise ValueError(f"不支持的 Sharpa 操作: {action}")
                else:
                    self.send_error(404)
            except (ValueError, RuntimeError, OSError) as exc:
                self.reply_json(400, {"error": str(exc), "status": manager.status()})

    class Server(ThreadingHTTPServer):
        allow_reuse_address = True

    server = Server(("0.0.0.0", port), Handler)
    server.timeout = 0.25
    try:
        while not stop_event.is_set():
            server.handle_request()
    finally:
        manager.stop()
        scene_bridge.close()
        server.server_close()
