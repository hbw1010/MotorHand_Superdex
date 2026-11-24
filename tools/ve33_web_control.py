"""G4 采集进程与场景 HTTP 服务共享的操作和状态。"""

import json
import math


def measure_frame_gaps(rows):
    """同帧仿真角减驱动目标角。保留符号，避免把输入传输延迟当成受阻。"""
    largest = None
    for row in rows:
        sim, target = row["sim_rad"], row["contact_input_rad"]
        gap = sim - target if sim is not None and target is not None else None
        if gap is not None and math.isfinite(gap):
            largest = abs(gap) if largest is None else max(largest, abs(gap))
        else:
            gap = None
        row["sim_minus_input_rad"] = gap
    return largest


def panel_snapshot(state, now_ns, revision):
    """使 HTTP 快照在读取时仍能识别观察进程停顿或场景切换。"""
    stamp = state.get("control_sample_ns")
    if stamp is None:
        return state
    age = now_ns - stamp
    stale = not 0 <= age < state["diagnostic_ttl_ns"]
    state["diagnostic_stale"] = stale
    state["snapshot_age_ms"] = age / 1e6
    if stale:
        state.update(active=None, online=None, rows=[], control_state_known=False,
                     max_angle_error_rad=None, contact_frame_ns=None)
    elif revision != state["control_revision"] or now_ns >= state["force_expiry_ns"]:
        for row in state.get("rows", ()):
            for key in ("contact_motor_nm", "contact_input_rad", "contact_encoder_rad", "position_motor_nm",
                        "mapped_contact_nm", "contact_gate", "retreat_weight", "nominal_nm", "sent_motor_nm",
                        "sim_rad", "sim_minus_input_rad"):
                row[key] = None
        state["contact_frame_ns"] = None
        state["max_angle_error_rad"] = None
    return state


class WebControl:
    def __init__(self, context, align, *, default_enabled=True, cube_switch=False, allow_reset=False,
                 local_grasp=False, demo=False, switchable_tasks=(), initial_task=None):
        self.align = align
        self.requested = context.Event()
        self.zero = context.Event()
        self.home = context.Event()
        self.cancel_home = context.Event()
        self.task_names = tuple(switchable_tasks)
        default_task = initial_task or ("local_grasp" if local_grasp else (self.task_names[0] if self.task_names else ""))
        self.task = context.Array("c", 32)
        self.task.value = default_task.encode()
        self.requested_task = context.Array("c", 32)
        self.requested_task.value = b""
        has_local_grasp = local_grasp or ("local_grasp" in self.task_names)
        self.grasp = context.Array("c", 2048) if has_local_grasp else None
        self.record_requested = context.Event() if has_local_grasp else None
        self.record_ready = context.Array("c", 4096) if has_local_grasp else None
        self.record_error = context.Array("c", 256) if has_local_grasp else None
        self.record_dropped = context.Value("Q", 0) if has_local_grasp else None
        self.demo_grip = context.Value("d", -1.) if demo else None
        if has_local_grasp:
            from ve33_local_grasp import DEFAULT_GRASP
            self.grasp.value = json.dumps({**DEFAULT_GRASP, "contact_revision": 0}).encode()
        self.cube_fixed = context.Event() if cube_switch else None
        self.scene_revision = context.Value("Q", 0, lock=False) if allow_reset or cube_switch or has_local_grasp else None
        self.wrist_stamp = context.Value("q", 0, lock=False)
        if self.cube_fixed is not None:
            self.cube_fixed.set()
        self.snapshot = context.Array("c", 16_384)
        self.command_lock = context.Lock()
        self.native_link_path = None
        self._native_link = None
        if default_enabled:
            self.requested.set()

    def __getstate__(self):
        state = self.__dict__.copy()
        state["_native_link"] = None
        return state

    def current_task(self):
        return self.task.value.decode()

    def is_local_grasp(self):
        return self.grasp is not None and (not self.task_names or self.current_task() == "local_grasp")

    def pending_task(self):
        target = self.requested_task.value.decode()
        return target if target and target != self.current_task() else None

    def commit_task(self, task_name):
        with self.command_lock:
            self.requested.clear()
            if self.record_requested is not None:
                self.record_requested.clear()
            self.task.value = task_name.encode()
            self.requested_task.value = b""
            if self.scene_revision is not None:
                self.scene_revision.value += 1
            if self.grasp is not None:
                settings = json.loads(self.grasp.value)
                settings["contact_revision"] += 1
                self.grasp.value = json.dumps(settings).encode()
            self._publish_native_intent()

    def publish(self, status):
        self.snapshot.value = json.dumps(status, ensure_ascii=False, allow_nan=False).encode()

    def read(self):
        return json.loads(self.snapshot.value or b"{}")

    def scene_request(self):
        with self.command_lock:
            fixed = self.cube_fixed.is_set() if self.cube_fixed is not None else None
            return fixed, self.scene_revision.value

    def grasp_request(self):
        return json.loads(self.grasp.value) if self.is_local_grasp() else None

    def grasp_scene_request(self):
        with self.command_lock:
            return self.scene_revision.value, self.grasp_request()

    def return_grasp_when_clear(self, expected):
        """只有仍是物理线程刚检查的摆位，才使旧力帧失效并调度归位。"""
        with self.command_lock:
            settings = self.grasp_request()
            if settings != expected:
                return False
            settings["home"] = settings.get("home", 0) + 1
            settings["contact_revision"] += 1
            self.grasp.value = json.dumps(settings).encode()
            self._publish_native_intent()
            return True

    def _publish_native_intent(self):
        if self.native_link_path:
            if self._native_link is None:
                from motorhand.native_runtime import load_native
                self._native_link = load_native().SharedLink(self.native_link_path)
            revision = (self.grasp_request()["contact_revision"] if self.is_local_grasp()
                        else self.scene_revision.value if self.scene_revision is not None else 0)
            self._native_link.command(self.requested.is_set() and not self.zero.is_set() and not self.home.is_set(),
                                      revision,
                                      self.demo_grip.value if self.demo_grip is not None else -1.)

    def command(self, message):
        with self.command_lock:
            action = message.get("action")
            if action == "enable" and self.read().get("can_feedback"):
                if self.zero.is_set() or self.home.is_set():
                    raise ValueError("正在标零或自动回零")
                self.requested.set()
            elif action == "disable":
                self.requested.clear()
                if self.home.is_set():
                    self.cancel_home.set()
            elif action == "align" and self.align is not None and not self.is_local_grasp():
                self.align.set()
            elif action == "switch_task" and self.task_names:
                target = message.get("task")
                if target not in self.task_names:
                    raise ValueError("未知的仿真任务")
                if self.zero.is_set() or self.home.is_set():
                    raise ValueError("标零或自动回零期间不能切换任务")
                if target != self.current_task():
                    self.requested.clear()
                    if self.record_requested is not None:
                        self.record_requested.clear()
                    self.requested_task.value = target.encode()
            elif action == "configure_grasp" and self.is_local_grasp():
                from ve33_local_grasp import grasp_needs_rebuild, grasp_settings
                current = self.grasp_request()
                if "expected_kind" in message and message["expected_kind"] != current["kind"]:
                    return
                settings = grasp_settings(current, message.get("values"))
                if grasp_needs_rebuild(current, settings):
                    settings["contact_revision"] += 1
                self.grasp.value = json.dumps(settings).encode()
            elif action == "demo_grip" and self.demo_grip is not None:
                value = message.get("value")
                if value is not None and (type(value) not in (int, float) or not 0. <= value <= 1.):
                    raise ValueError("演示抓握比例应在 0 到 1 之间")
                self.demo_grip.value = -1. if value is None else value
            elif action == "home_grasp" and self.is_local_grasp():
                settings = self.grasp_request()
                settings.update(layout="palm", drag=None, angle_deg=0., home=settings.get("home", 0) + 1)
                settings["contact_revision"] += 1
                self.grasp.value = json.dumps(settings).encode()
            elif action == "start_recording" and self.is_local_grasp() and self.record_requested is not None:
                if not self.record_requested.is_set():
                    self.record_ready.value = b""
                    self.record_error.value = b""
                    self.record_dropped.value = 0
                    self.record_requested.set()
            elif action == "stop_recording" and self.record_requested is not None:
                self.record_requested.clear()
            elif action == "cube_mode" and self.cube_fixed is not None:
                mode = message.get("mode")
                if mode == "fixed":
                    self.cube_fixed.set()
                elif mode == "free":
                    self.cube_fixed.clear()
                else:
                    raise ValueError("魔方模式必须是固定中心或自由抓取")
            elif action == "reset_scene" and self.scene_revision is not None:
                if self.cube_fixed is not None:
                    self.cube_fixed.set()
                if self.is_local_grasp():
                    settings = self.grasp_request()
                    settings.update(layout="palm", drag=None, angle_deg=0.)
                    settings["contact_revision"] += 1
                    self.grasp.value = json.dumps(settings).encode()
                self.scene_revision.value += 1
            elif action == "zero":
                if self.demo_grip is not None:
                    raise ValueError("演示模式没有真机零位")
                if self.zero.is_set() or self.home.is_set():
                    raise ValueError("正在标零或自动回零")
                current = self.read()
                for key in ("device", "zero_path", "axes"):
                    if key in message and message[key] != current.get(key):
                        raise ValueError("请求与当前网关、标零文件或轴映射不同")
                self.zero.set()
                self.requested.clear()
            elif action == "home":
                if self.demo_grip is not None:
                    raise ValueError("演示模式没有真机自动回零")
                if self.zero.is_set() or self.home.is_set():
                    raise ValueError("正在标零或自动回零")
                current = self.read()
                for key in ("device", "zero_path", "axes"):
                    if key in message and message[key] != current.get(key):
                        raise ValueError("请求与当前网关、标零文件或轴映射不同")
                self.cancel_home.clear()
                self.home.set()
                self.requested.clear()
            else:
                raise ValueError("当前模式不支持该操作")
            if action not in ("start_recording", "stop_recording"):
                self._publish_native_intent()
