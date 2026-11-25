"""Quest 右腕输入和桌面/头显共用的场景位姿接口。"""

import json
import os
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from shutil import copyfileobj
from threading import Condition, Lock, Thread
import time
from urllib.parse import unquote

import numpy as np
from scipy.spatial.transform import Rotation

# WebXR 的右、上、后转换到场景的前、左、上。
XR_TO_WORLD = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
IDENTITY = [0., 0., 0., 0., 0., 0., 1.]


MISSING_POSE_GRACE_S = .075
LEFT_POSE_TIMEOUT_S = .12
LEFT_HAND_POINT_COUNT = 16
SCENE_WAIT_S = .20
MISSING_POSE_STATUS = {
    "no_right_controller": "头显未报告右手柄",
    "right_hand_only": "头显只报告右手光学追踪",
    "no_controller_grip": "右侧输入没有可用的握持位姿",
    "no_right_hand": "头显未报告右手",
    "emulated_position": "手柄位置未被追踪",
    "pose_unavailable": "头显未收到有效 6DoF 位姿",
}


PIVOT_MIN_SAMPLES = 15
PIVOT_MAX_SAMPLES = 600
PIVOT_MIN_ANGLE_STEP_RAD = 0.025  # ~1.4 deg
PIVOT_MIN_EXCITATION = 0.12
PIVOT_MAX_OFFSET_M = 0.45
PIVOT_MAX_RMS_MM = 25.0
PIVOT_TARGET_DEG = 28.0


def parse_controller_offset(offset):
    vec = np.asarray(offset, dtype=float)
    if vec.shape != (3,) or not np.isfinite(vec).all() or np.linalg.norm(vec) > 1.:
        raise ValueError("手柄相对手套偏移须提供 3 维有限米制坐标且模长不超过 1 m")
    return vec


def parse_pose7(pose, label="位姿"):
    arr = np.asarray(pose, dtype=float)
    if arr.shape != (7,) or not np.isfinite(arr).all() or abs(np.linalg.norm(arr[3:]) - 1.) > .02:
        raise ValueError(f"{label}须提供 7 维米制位姿与单位四元数 xyzw")
    quat = arr[3:] / np.linalg.norm(arr[3:])
    return np.concatenate([arr[:3], quat])


def parse_controller_to_hand(pose):
    if pose is None:
        return None
    arr = parse_pose7(pose, "手柄到手套 6DoF 外参")
    if np.linalg.norm(arr[:3]) > 1.:
        raise ValueError("手柄到手套平移模长不应超过 1 m")
    return arr[:3].copy(), Rotation.from_quat(arr[3:])


def solve_pivot_offset(samples):
    """用原地多轴旋转样本线性最小二乘求解手柄相对手套原点的刚体偏移 c_hand。"""
    n = len(samples)
    if n < 3:
        return {"ok": False, "samples": n, "sv_min": 0., "offset_m": None, "rms_mm": None,
                "reason": "样本不足，请固定手腕位置并沿至少两个方向旋转"}
    rots = np.stack([r for r, _ in samples], axis=0)
    pos = np.stack([q for _, q in samples], axis=0)
    a = (rots - rots.mean(axis=0, keepdims=True)).reshape(3 * n, 3)
    b = (pos - pos.mean(axis=0, keepdims=True)).reshape(3 * n)
    u, s, vt = np.linalg.svd(a, full_matrices=False)
    sv_min = float(s[-1] / np.sqrt(n))
    if n < PIVOT_MIN_SAMPLES or sv_min < PIVOT_MIN_EXCITATION:
        return {"ok": False, "samples": n, "sv_min": sv_min, "offset_m": None, "rms_mm": None,
                "reason": "旋转角度或方向不足，请保持手腕原地不动，沿俯仰与滚转两个方向各转动至少 ±30°"}
    offset = vt.T @ ((u.T @ b) / s)
    res = (a @ offset - b).reshape(n, 3)
    rms_mm = float(np.sqrt(np.mean(np.sum(res * res, axis=1))) * 1000.)
    norm = float(np.linalg.norm(offset))
    if not np.isfinite(offset).all() or norm > PIVOT_MAX_OFFSET_M:
        return {"ok": False, "samples": n, "sv_min": sv_min, "offset_m": offset.tolist(), "rms_mm": rms_mm,
                "reason": f"估计偏移过大（{norm * 1000:.0f} mm），请保持手腕/掌心位置不动只做原地旋转"}
    if rms_mm > PIVOT_MAX_RMS_MM:
        return {"ok": False, "samples": n, "sv_min": sv_min, "offset_m": offset.tolist(), "rms_mm": rms_mm,
                "reason": f"残差偏大（{rms_mm:.1f} mm），旋转时请尽量不要平移整条手臂"}
    return {"ok": True, "samples": n, "sv_min": sv_min, "offset_m": offset.tolist(), "rms_mm": rms_mm, "reason": ""}


class WristBridge:
    def __init__(self, controller_offset=(0., 0., 0.), controller_to_hand=None):
        self.lock = Lock()
        self.scene_changed = Condition(self.lock)
        self.controller_to_hand = parse_controller_to_hand(controller_to_hand)
        if self.controller_to_hand is not None:
            t_c2h, r_c2h = self.controller_to_hand
            self.controller_offset = -r_c2h.inv().apply(t_c2h)
        else:
            self.controller_offset = parse_controller_offset(controller_offset)
        self.space = None
        self.reference = None
        self.world_xr_pose = None
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
        self._overlay_cal_active = False
        self._cal_active = False
        self._cal_samples = []
        self._cal_last_rot = None
        self._cal_start_rot = None
        self._cal_pivot_world = None
        self._cal_tilt_deg = [0., 0.]
        self._cal_dir_max = {"up": 0., "down": 0., "left": 0., "right": 0.}
        self._cal_message = ""
        self._cal_rms_mm = None
        self._cal_event_time = 0.

    def _update_registration_locked(self):
        if self.world_xr_pose is not None:
            self.registration = {"space": self.space, "xr_from_world": self.world_xr_pose.tolist()}
            return
        if self.reference is None:
            self.registration = None
            return
        world_origin_xr = self.reference[:3] - XR_TO_WORLD.T @ self.controller_offset
        self.registration = {"space": self.space, "xr_from_world": [
            *world_origin_xr, *Rotation.from_matrix(XR_TO_WORLD.T).as_quat()]}

    def _hand_pose_xr_locked(self, ctrl_pose):
        t_c2h, r_c2h = self.controller_to_hand
        r_c = Rotation.from_quat(ctrl_pose[3:])
        p_h = ctrl_pose[:3] + r_c.apply(t_c2h)
        r_h = r_c * r_c2h
        return p_h, r_h

    def _horizontal_world_xr_from_hand(self, p_h, r_h):
        # 保持世界桌面与真实重力严格水平，同时水平朝向对准用户当前手掌指尖方向
        fwd_xr = r_h.apply([1., 0., 0.])
        horiz = float(np.hypot(fwd_xr[0], fwd_xr[2]))
        yaw = float(np.arctan2(-fwd_xr[0], -fwd_xr[2])) if horiz > 1e-4 else 0.
        r_w = Rotation.from_euler("y", yaw) * Rotation.from_matrix(XR_TO_WORLD.T)
        return np.concatenate([p_h, r_w.as_quat()])

    def _controller_to_hand_list_locked(self):
        if self.controller_to_hand is None:
            return None
        t_c2h, r_c2h = self.controller_to_hand
        return [round(float(x), 6) for x in (*t_c2h, *r_c2h.as_quat())]

    def start_overlay_calibration(self):
        with self.lock:
            now = time.monotonic()
            self._cal_active = False
            self._overlay_cal_active = True
            self.root = IDENTITY.copy()
            self._cal_message = "虚实重合标定中：把真实右手手套伸进眼前虚拟手重合，按【左手扳机】确认保存"
            self._cal_event_time = now
            return self._calibration_status_locked()

    def cancel_overlay_calibration(self):
        with self.lock:
            self._overlay_cal_active = False
            self._cal_message = "已取消虚实重合标定"
            self._cal_event_time = time.monotonic()
            return self._calibration_status_locked()

    def confirm_overlay_calibration(self, space, ctrl_pose, hand_pose, auto_save_path=None):
        ctrl = parse_pose7(ctrl_pose, "右手柄位姿")
        hand = parse_pose7(hand_pose, "目标虚拟手位姿")
        r_c = Rotation.from_quat(ctrl[3:])
        r_h = Rotation.from_quat(hand[3:])
        r_c2h = r_c.inv() * r_h
        t_c2h = r_c.inv().apply(hand[:3] - ctrl[:3])
        if float(np.linalg.norm(t_c2h)) > PIVOT_MAX_OFFSET_M:
            raise ValueError(f"手柄与虚拟手距离过大（{np.linalg.norm(t_c2h) * 1000:.0f} mm），请先将真实手套与虚拟手重合")
        c_hand = r_h.inv().apply(ctrl[:3] - hand[:3])
        with self.lock:
            now = time.monotonic()
            self.controller_to_hand = (t_c2h, r_c2h)
            self.controller_offset = c_hand
            if isinstance(space, str) and 0 < len(space) <= 80:
                self.space = space
            self.reference = ctrl.copy()
            self.world_xr_pose = hand.copy()
            self.sample = ctrl.copy()
            self.received = now
            self.missing_pose = False
            self.align_requested = False
            self._overlay_cal_active = False
            self._cal_active = False
            self.root = IDENTITY.copy()
            self._update_registration_locked()
            ox, oy, oz = c_hand * 1000.
            if auto_save_path is not None:
                save_wrist_offset_config(auto_save_path, c_hand, self._controller_to_hand_list_locked())
                self._cal_message = f"虚实重合标定已保存！杆臂 ({ox:+.1f}, {oy:+.1f}, {oz:+.1f}) mm"
            else:
                self._cal_message = f"虚实重合标定已应用！杆臂 ({ox:+.1f}, {oy:+.1f}, {oz:+.1f}) mm"
            self._cal_event_time = now
            return self._calibration_status_locked()

    def _world_rot_locked(self, pose):
        delta = Rotation.from_quat(pose[3:]) * Rotation.from_quat(self.reference[3:]).inv()
        return Rotation.from_matrix(XR_TO_WORLD @ delta.as_matrix() @ XR_TO_WORLD.T)

    def _record_calibration_sample_locked(self, pose):
        if not self._cal_active or self.reference is None:
            return
        rot = self._world_rot_locked(pose)
        r_mat = rot.as_matrix()
        # 世界 FLU 坐标系：+X 指尖、+Y 左/拇指、+Z 手背；平放参考姿态恰在罗盘中心 (0°, 0°)
        up_deg = float(np.degrees(np.arcsin(np.clip(r_mat[2, 0], -1., 1.))))
        right_deg = float(np.degrees(np.arcsin(np.clip(-r_mat[1, 2], -1., 1.))))
        self._cal_tilt_deg = [right_deg, up_deg]
        if up_deg > self._cal_dir_max["up"]:
            self._cal_dir_max["up"] = up_deg
        if -up_deg > self._cal_dir_max["down"]:
            self._cal_dir_max["down"] = -up_deg
        if right_deg > self._cal_dir_max["right"]:
            self._cal_dir_max["right"] = right_deg
        if -right_deg > self._cal_dir_max["left"]:
            self._cal_dir_max["left"] = -right_deg
        if len(self._cal_samples) >= PIVOT_MAX_SAMPLES:
            return
        if self._cal_last_rot is not None:
            step = (rot * self._cal_last_rot.inv()).magnitude()
            if step < PIVOT_MIN_ANGLE_STEP_RAD:
                return
        q = XR_TO_WORLD @ (pose[:3] - self.reference[:3])
        self._cal_samples.append((r_mat, q))
        self._cal_last_rot = rot

    def set_controller_offset(self, offset, message="", clear_6dof=False):
        vec = parse_controller_offset(offset)
        with self.lock:
            if clear_6dof:
                self.controller_to_hand = None
                self.world_xr_pose = None
            self.controller_offset = vec
            self._update_registration_locked()
            if message:
                self._cal_message = message
                self._cal_event_time = time.monotonic()
            return self._calibration_status_locked()

    def start_offset_calibration(self):
        with self.lock:
            now = time.monotonic()
            if self.sample is None or self.missing_pose or now - self.received > .3:
                self._cal_active = False
                self._cal_message = "等待头显右手柄有效 6DoF 位姿，请先在头显中进入场景"
                self._cal_event_time = now
                return self._calibration_status_locked()
            if self.reference is None:
                self.reference = self.sample.copy()
                self._update_registration_locked()
            rot_now = self._world_rot_locked(self.sample)
            self._cal_start_rot = rot_now.as_matrix()
            start_pos = (XR_TO_WORLD @ (self.sample[:3] - self.reference[:3])
                         + self.controller_offset - rot_now.apply(self.controller_offset))
            self._cal_pivot_world = [float(x) for x in start_pos]
            self._cal_tilt_deg = [0., 0.]
            self._cal_dir_max = {"up": 0., "down": 0., "left": 0., "right": 0.}
            self._cal_active = True
            self._cal_samples = []
            self._cal_last_rot = None
            self._cal_rms_mm = None
            self._cal_message = "标定中：手腕固定在绿球处不动，上下翘腕并左右翻腕点亮四个方向灯"
            self._cal_event_time = now
            return self._calibration_status_locked()

    def finish_offset_calibration(self, auto_save_path=None):
        with self.lock:
            now = time.monotonic()
            if not self._cal_active:
                self._cal_message = "当前未处于动作标定状态"
                self._cal_event_time = now
                return self._calibration_status_locked()
            result = solve_pivot_offset(self._cal_samples)
            if not result["ok"]:
                self._cal_message = result["reason"]
                self._cal_event_time = now
                return self._calibration_status_locked()
            self.controller_offset = np.asarray(result["offset_m"], dtype=float)
            self._update_registration_locked()
            self._cal_active = False
            self._cal_rms_mm = result["rms_mm"]
            ox, oy, oz = self.controller_offset * 1000.
            if auto_save_path is not None:
                saved = save_wrist_offset_config(auto_save_path, self.controller_offset)
                self.controller_offset = np.asarray(saved, dtype=float)
                self._update_registration_locked()
                ox, oy, oz = self.controller_offset * 1000.
                self._cal_message = f"标定完成并已保存：({ox:+.1f}, {oy:+.1f}, {oz:+.1f}) mm · 残差 {result['rms_mm']:.1f} mm"
            else:
                self._cal_message = f"标定已应用：偏移 ({ox:+.1f}, {oy:+.1f}, {oz:+.1f}) mm · 残差 {result['rms_mm']:.1f} mm"
            self._cal_event_time = now
            return self._calibration_status_locked()

    def toggle_offset_calibration(self, auto_save_path=None):
        with self.lock:
            active = self._cal_active
        if active:
            return self.finish_offset_calibration(auto_save_path=auto_save_path)
        return self.start_offset_calibration()

    def cancel_offset_calibration(self):
        with self.lock:
            self._cal_active = False
            self._cal_samples = []
            self._cal_last_rot = None
            self._cal_message = "已取消动作标定"
            self._cal_event_time = time.monotonic()
            return self._calibration_status_locked()

    def _calibration_status_locked(self):
        preview = solve_pivot_offset(self._cal_samples) if self._cal_active else None
        now = time.monotonic()
        est = preview["offset_m"] if preview and preview["offset_m"] is not None else None
        drift_mm = None
        if self._cal_active and self._cal_pivot_world is not None and self.sample is not None and self.reference is not None:
            vec = np.asarray(est, dtype=float) if est is not None else self.controller_offset
            rot_now = self._world_rot_locked(self.sample)
            est_pos = XR_TO_WORLD @ (self.sample[:3] - self.reference[:3]) + vec - rot_now.apply(vec)
            drift_mm = float(np.linalg.norm(est_pos - np.asarray(self._cal_pivot_world)) * 1000.)
        dirs = {k: round(min(1., v / PIVOT_TARGET_DEG), 3) for k, v in self._cal_dir_max.items()}
        return {
            "offset_m": [round(float(x), 5) for x in self.controller_offset],
            "pose_6dof": self._controller_to_hand_list_locked(),
            "calibrated_6dof": self.controller_to_hand is not None,
            "overlay_active": self._overlay_cal_active,
            "active": self._cal_active,
            "samples": len(self._cal_samples),
            "ready": bool(preview and preview["ok"]),
            "excitation": round(float(preview["sv_min"]), 3) if preview else 0.,
            "estimated_offset_m": [round(float(x), 5) for x in est] if est is not None else None,
            "rms_mm": round(float(preview["rms_mm"]), 2) if preview and preview["rms_mm"] is not None else (
                round(float(self._cal_rms_mm), 2) if self._cal_rms_mm is not None else None),
            "tilt_right_deg": round(float(self._cal_tilt_deg[0]), 1),
            "tilt_up_deg": round(float(self._cal_tilt_deg[1]), 1),
            "directions": dirs,
            "pivot_world": [round(float(x), 4) for x in self._cal_pivot_world] if self._cal_pivot_world is not None else None,
            "drift_mm": round(drift_mm, 1) if drift_mm is not None else None,
            "recent_feedback": bool(self._cal_message and (now - self._cal_event_time) < 6.0),
            "message": self._cal_message,
        }

    def notify(self, message):
        with self.lock:
            self._cal_message = str(message)
            self._cal_event_time = time.monotonic()
            return self._calibration_status_locked()

    def calibration_status(self):
        with self.lock:
            return self._calibration_status_locked()

    def publish_scene(self, frame):
        with self.scene_changed:
            self.scene = frame
            self.scene_changed.notify_all()

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
                self.world_xr_pose = None
                self.sample, self.received = None, 0.
                self.registration = None
                self.align_requested = False
                if self._cal_active:
                    self._cal_active = False
                    self._cal_samples = []
                    self._cal_last_rot = None
                    self._cal_message = "头显参考空间已重置，请重新开始动作标定"
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
                self._record_calibration_sample_locked(pose)

    def wrist(self, calibrate=False):
        with self.lock:
            if self._overlay_cal_active:
                self.root = IDENTITY.copy()
                stamp = round(self.received * 1e9) if self.sample is not None else 0
                return self.root, "虚实重合标定中", self.registration, stamp
            max_age = MISSING_POSE_GRACE_S if self.missing_pose else .3
            if self.sample is None or time.monotonic() - self.received > max_age:
                return self.root, "等待右腕追踪", self.registration, 0
            stamp = round(self.received * 1e9)
            if self.controller_to_hand is not None:
                p_h, r_h = self._hand_pose_xr_locked(self.sample)
                if not self.missing_pose and (calibrate or self.align_requested):
                    self.align_requested = False
                    self.reference = self.sample.copy()
                    self.world_xr_pose = self._horizontal_world_xr_from_hand(p_h, r_h)
                    self._update_registration_locked()
                if self.world_xr_pose is None:
                    return self.root, "右腕已连接，请对齐参考姿态", None, stamp
                p_w = self.world_xr_pose[:3]
                r_w = Rotation.from_quat(self.world_xr_pose[3:])
                root_pos = r_w.inv().apply(p_h - p_w)
                root_rot = r_w.inv() * r_h
                self.root = [*root_pos, *root_rot.as_quat()]
                return self.root, "右腕跟踪中", self.registration, stamp
            if not self.missing_pose and (calibrate or self.align_requested):
                self.align_requested = False
                self.reference = self.sample.copy()
                self._update_registration_locked()
            if self.reference is None:
                return self.root, "右腕已连接，请对齐参考姿态", None, stamp
            p, ref = self.sample, self.reference
            delta = Rotation.from_quat(p[3:]) * Rotation.from_quat(ref[3:]).inv()
            rot = Rotation.from_matrix(XR_TO_WORLD @ delta.as_matrix() @ XR_TO_WORLD.T)
            root_pos = (XR_TO_WORLD @ (p[:3] - ref[:3])
                        + self.controller_offset - rot.apply(self.controller_offset))
            self.root = [*root_pos, *rot.as_quat()]
            return self.root, "右腕跟踪中", self.registration, stamp

    def reset_scene(self):
        with self.lock:
            self.root = IDENTITY.copy()
            self.reference = None
            self.world_xr_pose = None
            self.registration = None
            self.left_sample = None
            self.align_requested = (self.sample is not None and not self.missing_pose
                                    and time.monotonic() - self.received <= .3)

    def left_hand(self, now=None):
        with self.lock:
            if (self.reference is None and self.world_xr_pose is None) or self.left_sample is None:
                return None
            if (time.monotonic() if now is None else now) - self.left_received > LEFT_POSE_TIMEOUT_S:
                return None
            kind, points = self.left_sample
            if self.world_xr_pose is not None:
                r_w = Rotation.from_quat(self.world_xr_pose[3:])
                local = (points - self.world_xr_pose[:3]) @ r_w.as_matrix()
            else:
                local = (points - self.reference[:3]) @ XR_TO_WORLD.T + self.controller_offset
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


def save_wrist_offset_config(config_path, offset, pose_6dof=None):
    vec = [round(float(x), 5) for x in parse_controller_offset(offset)]
    path = Path(config_path)
    data = json.loads(path.read_text(encoding="utf-8"))
    data["wrist_tracker_offset_m"] = vec
    data["wrist_tracker_pose"] = [round(float(x), 6) for x in parse_pose7(pose_6dof)] if pose_6dof is not None else None
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    tmp.replace(path)
    return vec


@contextmanager
def wrist_server(bridge, port, glb=None, control=None, *, scene_feedback=None, offset_config_path=None):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        disable_nagle_algorithm = True

        def reply(self, status, body, kind):
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                self.close_connection = True

        def reply_scene(self, after=None):
            with bridge.scene_changed:
                if after is not None and bridge.scene:
                    bridge.scene_changed.wait_for(
                        lambda: bridge.scene is None or bridge.scene.get("frame") != after, timeout=SCENE_WAIT_S)
                frame = bridge.scene
            if frame and after is not None and frame.get("frame") == after:
                return self.reply(204, b"", "application/json")
            if frame and ("wrist_status" in frame or bridge._cal_active or bridge._overlay_cal_active or bridge._cal_event_time > 0):
                frame = {**frame, "wrist_offset": bridge.calibration_status()}
            if frame and scene_feedback is not None:
                frame = {**frame, "motor_feedback": scene_feedback(frame)}
            feedback = frame.get("motor_feedback") if frame else None
            if feedback and "expires_ns" in feedback:
                remaining_ms = (feedback["expires_ns"] - time.monotonic_ns()) / 1e6
                frame = {**frame, "motor_feedback": {**feedback, "remaining_ms": remaining_ms}
                         if remaining_ms > 0 else None}
            self.reply(200 if frame else 503, json.dumps(frame, allow_nan=False).encode(), "application/json")

        def reply_recording(self):
            path = control.record_ready.value
            if not path or control.record_requested.is_set():
                return self.send_error(404)
            path = Path(os.fsdecode(path))
            try:
                file = path.open("rb")
            except OSError:
                return self.send_error(404)
            with file:
                self.send_response(200)
                self.send_header("Content-Type", "text/csv; charset=utf-8")
                self.send_header("Content-Disposition", f'attachment; filename="{path.name}"')
                self.send_header("Content-Length", str(os.fstat(file.fileno()).st_size))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    copyfileobj(file, self.wfile)
                except (BrokenPipeError, ConnectionResetError):
                    self.close_connection = True

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/":
                bundle = Path(__file__).with_name("quest_view") / "build/scene.js"
                ver = bundle.stat().st_mtime_ns if bundle.is_file() else 0
                html = Path(__file__).with_name("ve33_wrist.html").read_text(encoding="utf-8")
                html = html.replace('src="/quest-scene.js"', f'src="/quest-scene.js?v={ver}"')
                self.reply(200, html.encode("utf-8"), "text/html; charset=utf-8")
            elif path == "/quest-scene.js":
                bundle = Path(__file__).with_name("quest_view") / "build/scene.js"
                if not bundle.is_file():
                    return self.send_error(503, "Run npm ci and npm run build in tools/quest_view")
                self.reply(200, bundle.read_bytes(), "text/javascript")
            elif path == "/api/v1/model.glb" and glb is not None:
                self.reply(200, glb, "model/gltf-binary")
            elif (self.path.startswith("/api/v1/grasp-model/") and control is not None
                  and getattr(control, "is_local_grasp", lambda: control.grasp is not None)()):
                from ve33_grasp_assets import render_model_path
                path = render_model_path(unquote(self.path.removeprefix("/api/v1/grasp-model/")))
                if path is None:
                    return self.send_error(404)
                try:
                    model = path.read_bytes()
                except OSError:
                    return self.send_error(404)
                self.reply(200, model, "model/gltf-binary")
            elif self.path == "/api/v1/scene":
                self.reply_scene()
            elif self.path.startswith("/api/v1/scene?after="):
                after = self.path.split("=", 1)[1]
                if not 1 <= len(after) <= 20 or not after.isascii() or not after.isdecimal():
                    return self.send_error(400)
                self.reply_scene(int(after))
            elif self.path == "/api/v1/recording.csv" and control is not None and control.record_ready is not None:
                self.reply_recording()
            elif path == "/api/v1/hand-model" and control is not None:
                from motorhand.hand_profiles import AVAILABLE_HAND_MODELS, get_hand_profile
                spec = get_hand_profile(control.current_hand_model())
                self.reply(200, json.dumps({
                    "hand_model": spec.model_id,
                    "label": spec.label,
                    "dof": spec.dof,
                    "available_hand_models": list(AVAILABLE_HAND_MODELS),
                }, ensure_ascii=False).encode(), "application/json")
            elif self.path == "/api/v1/control" and control is not None:
                state = control.read()
                if getattr(control, "is_local_grasp", lambda: control.grasp is not None)():
                    from ve33_grasp_assets import catalog
                    state["grasp_assets"] = catalog()
                with bridge.lock:
                    if bridge.scene:
                        for key in ("cube_mode", "scene_revision"):
                            if key in bridge.scene:
                                state[key] = bridge.scene[key]
                        grasp = bridge.scene.get("grasp_object")
                        if grasp is not None:
                            state["grasp"] = {k: v for k, v in grasp.items() if k not in ("vertices", "indices", "parts")}
                state["wrist_offset"] = bridge.calibration_status()
                state["can_save_wrist_offset"] = offset_config_path is not None
                if "control_sample_ns" in state:
                    from ve33_web_control import panel_snapshot
                    revision = state["grasp"]["contact_revision"] if "grasp" in state else state.get("scene_revision", 0)
                    panel_snapshot(state, time.monotonic_ns(), revision)
                self.reply(200, json.dumps(state, ensure_ascii=False).encode(), "application/json")
            else:
                self.send_error(404)

        def do_POST(self):
            is_hand_model = self.path == "/api/v1/hand-model" and control is not None
            is_demo_axes = self.path == "/api/v1/demo-axes" and control is not None
            is_control = (self.path == "/api/v1/control" or is_hand_model or is_demo_axes) and control is not None
            is_local = control is not None and getattr(control, "is_local_grasp", lambda: control.grasp is not None)()
            if self.path == "/api/v1/wrist" and is_local:
                return self.send_error(404)
            if self.path != "/api/v1/wrist" and not is_control:
                return self.send_error(404)
            origin = self.headers.get("Origin")
            allowed = {f"http://{host}:{self.server.server_port}" for host in ("localhost", "127.0.0.1")}
            if origin and origin not in allowed:
                return self.send_error(403)
            if self.headers.get_content_type() != "application/json":
                return self.send_error(415)
            cal_reply = None
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 8192:
                    raise ValueError("包长无效")
                message = json.loads(self.rfile.read(size))
                if not isinstance(message, dict):
                    raise ValueError("请求必须是对象")
                if is_hand_model:
                    control.command({"action": "switch_hand_model",
                                     "hand_model": message.get("hand_model") or message.get("model")})
                    cal_reply = {"ok": True, "requested_hand_model": message.get("hand_model") or message.get("model")}
                elif is_demo_axes:
                    control.command({"action": "demo_axes", **message})
                    cal_reply = {"ok": True}
                elif is_control:
                    action = message.get("action")
                    if action == "start_overlay_cal":
                        cal_reply = bridge.start_overlay_calibration()
                    elif action == "cancel_overlay_cal":
                        cal_reply = bridge.cancel_overlay_calibration()
                    elif action == "toggle_feedback":
                        current = control.read()
                        is_on = (control.requested.is_set() or bool(current.get("active"))
                                 or bool(current.get("feedback_rearm_required")))
                        if is_on:
                            control.command({"action": "disable"})
                            cal_reply = bridge.notify("力反馈已关闭（安全卸力）")
                        elif current.get("can_feedback") and not control.zero.is_set():
                            control.command({"action": "enable"})
                            cal_reply = bridge.notify("力反馈已开启")
                        else:
                            dof = current.get("dof", control.current_dof())
                            cal_reply = bridge.notify(f"暂不可开启力反馈：请先确认 {dof} 轴在线且已标零")
                    elif action == "confirm_overlay_cal":
                        auto_save = offset_config_path if message.get("auto_save", True) is True else None
                        cal_reply = bridge.confirm_overlay_calibration(
                            message.get("space"), message.get("ctrl_pose"), message.get("hand_pose"),
                            auto_save_path=auto_save)
                    elif action == "start_offset_cal":
                        cal_reply = bridge.start_offset_calibration()
                    elif action == "finish_offset_cal":
                        auto_save = offset_config_path if message.get("auto_save") is True else None
                        cal_reply = bridge.finish_offset_calibration(auto_save_path=auto_save)
                    elif action == "toggle_offset_cal":
                        auto_save = offset_config_path if message.get("auto_save", True) is True else None
                        cal_reply = bridge.toggle_offset_calibration(auto_save_path=auto_save)
                    elif action == "cancel_offset_cal":
                        cal_reply = bridge.cancel_offset_calibration()
                    elif action == "reset_offset_cal":
                        cal_reply = bridge.set_controller_offset((0., 0., 0.), "已清零手柄标定", clear_6dof=True)
                        if offset_config_path is not None and message.get("auto_save") is True:
                            save_wrist_offset_config(offset_config_path, (0., 0., 0.), None)
                    elif action == "set_wrist_offset":
                        offset = message.get("offset_m")
                        cal_reply = bridge.set_controller_offset(offset, "已更新手柄偏移补偿", clear_6dof=True)
                    elif action == "save_offset_cal":
                        if offset_config_path is None:
                            raise ValueError("未配置文件路径")
                        st = bridge.calibration_status()
                        saved = save_wrist_offset_config(offset_config_path, st["offset_m"], st["pose_6dof"])
                        ox, oy, oz = [x * 1000. for x in saved]
                        cal_reply = bridge.set_controller_offset(
                            saved, f"已保存到配置文件：({ox:+.1f}, {oy:+.1f}, {oz:+.1f}) mm")
                    else:
                        control.command(message)
                else:
                    bridge.receive(message)
            except (ValueError, TypeError, KeyError, OverflowError, OSError):
                return self.send_error(400)
            if cal_reply is not None:
                self.reply(200, json.dumps(cal_reply, ensure_ascii=False).encode(), "application/json")
            elif not is_control and message.get("want_scene") is True:
                self.reply_scene()
            else:
                self.reply(204, b"", "text/plain")

        def log_message(self, *_):
            pass

    class _FastThreadingHTTPServer(ThreadingHTTPServer):
        allow_reuse_address = True
        daemon_threads = True

    server = _FastThreadingHTTPServer(("127.0.0.1", port), Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.)
