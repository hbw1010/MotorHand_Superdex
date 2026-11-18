"""Quest 右腕输入和桌面/头显共用的场景位姿接口。"""

import json
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock, Thread
import time

import numpy as np
from scipy.spatial.transform import Rotation

# WebXR 的右、上、后转换到场景的前、左、上。
XR_TO_WORLD = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
IDENTITY = [0., 0., 0., 0., 0., 0., 1.]


MISSING_POSE_GRACE_S = .075
LEFT_POSE_TIMEOUT_S = .12
LEFT_HAND_POINT_COUNT = 16
MISSING_POSE_STATUS = {
    "no_right_controller": "头显未报告右手柄",
    "right_hand_only": "头显只报告右手光学追踪",
    "no_controller_grip": "右侧输入没有可用的握持位姿",
    "no_right_hand": "头显未报告右手",
    "emulated_position": "手柄位置未被追踪",
    "pose_unavailable": "头显未收到有效 6DoF 位姿",
}


class WristBridge:
    def __init__(self):
        self.lock = Lock()
        self.space = None
        self.reference = None
        self.sample = None
        self.missing_pose = False
        self.packet_time = None
        self.source_status = None
        self.sequence = -1
        self.received = 0.
        self.registration = None
        self.root = IDENTITY.copy()
        self.scene = None
        self.align_requested = False
        self.left_sample = None
        self.left_received = 0.

    def receive(self, msg):
        space, seq, pose = msg["space"], msg["seq"], msg["pose"]
        if not isinstance(space, str) or not 0 < len(space) <= 80 or type(seq) is not int or seq < 0:
            raise ValueError("参考空间或序号无效")
        if pose is not None:
            pose = np.asarray(pose, dtype=float)
            if pose.shape != (7,) or not np.isfinite(pose).all() or abs(np.linalg.norm(pose[3:]) - 1) > .01:
                raise ValueError("右腕必须提供米制位置和单位四元数 xyzw")
        left = msg.get("left")
        if left is not None:
            if not isinstance(left, dict) or left.get("kind") not in ("hand", "controller"):
                raise ValueError("未知的左手输入类型")
            points = np.asarray(left.get("points"), dtype=float)
            count = LEFT_HAND_POINT_COUNT if left["kind"] == "hand" else 1
            if points.shape != (count, 3) or not np.isfinite(points).all() or np.max(np.abs(points)) > 10.:
                raise ValueError("左手输入须提供有限的米制坐标")
            left = (left["kind"], points)
        with self.lock:
            if space != self.space:
                self.space, self.reference, self.sequence = space, None, -1
                self.sample, self.received = None, 0.
                self.registration = None
                self.align_requested = False
            if seq <= self.sequence:
                return
            self.sequence = seq
            self.packet_time = time.monotonic()
            self.left_sample = None if msg.get("paused") is True else left
            self.left_received = self.packet_time
            if pose is None:
                status = msg.get("source_status")
                self.source_status = MISSING_POSE_STATUS.get(status) if isinstance(status, str) else None
                self.missing_pose = True
                if msg.get("paused") is True:
                    self.source_status = "头显沉浸场景已暂停"
                    self.sample = None
                    self.align_requested = False
            else:
                self.missing_pose = False
                self.source_status = None
                self.sample, self.received = pose, self.packet_time
                if msg.get("align") is True:
                    self.align_requested = True

    def wrist(self, calibrate=False):
        with self.lock:
            max_age = MISSING_POSE_GRACE_S if self.missing_pose else .3
            if self.sample is None or time.monotonic() - self.received > max_age:
                return self.root, "等待右腕追踪", self.registration, 0
            stamp = round(self.received * 1e9)
            if not self.missing_pose and (calibrate or self.align_requested):
                self.align_requested = False
                self.reference = self.sample.copy()
                self.registration = {"space": self.space, "xr_from_world": [
                    *self.reference[:3], *Rotation.from_matrix(XR_TO_WORLD.T).as_quat()]}
            if self.reference is None:
                return self.root, "右腕已连接，请对齐参考姿态", None, stamp
            p, ref = self.sample, self.reference
            delta = Rotation.from_quat(p[3:]) * Rotation.from_quat(ref[3:]).inv()
            rot = Rotation.from_matrix(XR_TO_WORLD @ delta.as_matrix() @ XR_TO_WORLD.T)
            self.root = [*(XR_TO_WORLD @ (p[:3] - ref[:3])), *rot.as_quat()]
            return self.root, "右腕跟踪中", self.registration, stamp

    def reset_scene(self):
        with self.lock:
            self.root = IDENTITY.copy()
            self.reference = None
            self.registration = None
            self.left_sample = None
            self.align_requested = (self.sample is not None and not self.missing_pose
                                    and time.monotonic() - self.received <= .3)

    def left_hand(self, now=None):
        with self.lock:
            if self.reference is None or self.left_sample is None:
                return None
            if (time.monotonic() if now is None else now) - self.left_received > LEFT_POSE_TIMEOUT_S:
                return None
            kind, points = self.left_sample
            local = (points - self.reference[:3]) @ XR_TO_WORLD.T
            return {"kind": kind, "points": local.tolist()}

    def tracking_detail(self):
        with self.lock:
            now = time.monotonic()
            if self.packet_time is None or now - self.packet_time > .3:
                return "等待头显位姿数据"
            max_age = MISSING_POSE_GRACE_S if self.missing_pose else .3
            if self.sample is None or now - self.received > max_age:
                return self.source_status or "头显未收到有效 6DoF 位姿"
            if self.reference is None:
                return "右腕已连接，请对齐参考姿态"
            return "右腕跟踪中"


@contextmanager
def wrist_server(bridge, port, glb=None, control=None):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        disable_nagle_algorithm = True

        def reply(self, status, body, kind):
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                self.close_connection = True

        def reply_scene(self):
            with bridge.lock:
                frame = bridge.scene
            self.reply(200 if frame else 503, json.dumps(frame, allow_nan=False).encode(), "application/json")

        def do_GET(self):
            if self.path == "/":
                self.reply(200, Path(__file__).with_name("ve33_wrist.html").read_bytes(), "text/html; charset=utf-8")
            elif self.path == "/quest-scene.js":
                bundle = Path(__file__).with_name("quest_view") / "build/scene.js"
                if not bundle.is_file():
                    return self.send_error(503, "Run npm ci and npm run build in tools/quest_view")
                self.reply(200, bundle.read_bytes(), "text/javascript")
            elif self.path == "/api/v1/model.glb" and glb is not None:
                self.reply(200, glb, "model/gltf-binary")
            elif self.path == "/api/v1/scene":
                self.reply_scene()
            elif self.path == "/api/v1/control" and control is not None:
                state = control.read()
                with bridge.lock:
                    if bridge.scene:
                        for key in ("cube_mode", "scene_revision"):
                            if key in bridge.scene:
                                state[key] = bridge.scene[key]
                self.reply(200, json.dumps(state, ensure_ascii=False).encode(), "application/json")
            else:
                self.send_error(404)

        def do_POST(self):
            is_control = self.path == "/api/v1/control" and control is not None
            if self.path != "/api/v1/wrist" and not is_control:
                return self.send_error(404)
            origin = self.headers.get("Origin")
            allowed = {f"http://{host}:{self.server.server_port}" for host in ("localhost", "127.0.0.1")}
            if origin and origin not in allowed:
                return self.send_error(403)
            if self.headers.get_content_type() != "application/json":
                return self.send_error(415)
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 4096:
                    raise ValueError("包长无效")
                message = json.loads(self.rfile.read(size))
                if not isinstance(message, dict):
                    raise ValueError("请求必须是对象")
                if is_control:
                    control.command(message)
                else:
                    bridge.receive(message)
            except (ValueError, TypeError, KeyError, OverflowError):
                return self.send_error(400)
            if not is_control and message.get("want_scene") is True:
                self.reply_scene()
            else:
                self.reply(204, b"", "text/plain")

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
