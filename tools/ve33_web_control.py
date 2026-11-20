"""G4 采集进程与场景 HTTP 服务共享的操作和状态。"""

import json


class WebControl:
    def __init__(self, context, align, *, default_enabled=True, cube_switch=False, allow_reset=False,
                 local_grasp=False, demo=False):
        self.align = align
        self.requested = context.Event()
        self.zero = context.Event()
        self.grasp = context.Array("c", 2048) if local_grasp else None
        self.record_requested = context.Event() if local_grasp else None
        self.record_ready = context.Array("c", 4096) if local_grasp else None
        self.record_error = context.Array("c", 256) if local_grasp else None
        self.record_dropped = context.Value("Q", 0) if local_grasp else None
        self.demo_grip = context.Value("d", -1.) if demo else None
        if local_grasp:
            from ve33_local_grasp import DEFAULT_GRASP
            self.grasp.value = json.dumps({**DEFAULT_GRASP, "contact_revision": 0}).encode()
        self.cube_fixed = context.Event() if cube_switch else None
        self.scene_revision = context.Value("Q", 0, lock=False) if allow_reset or cube_switch or local_grasp else None
        if self.cube_fixed is not None:
            self.cube_fixed.set()
        self.snapshot = context.Array("c", 16_384)
        self.command_lock = context.Lock()
        if default_enabled:
            self.requested.set()

    def publish(self, status):
        self.snapshot.value = json.dumps(status, ensure_ascii=False, allow_nan=False).encode()

    def read(self):
        return json.loads(self.snapshot.value or b"{}")

    def scene_request(self):
        with self.command_lock:
            fixed = self.cube_fixed.is_set() if self.cube_fixed is not None else None
            return fixed, self.scene_revision.value

    def grasp_request(self):
        return json.loads(self.grasp.value) if self.grasp is not None else None

    def grasp_scene_request(self):
        with self.command_lock:
            return self.scene_revision.value, self.grasp_request()

    def command(self, message):
        with self.command_lock:
            action = message.get("action")
            if action == "enable" and self.read().get("can_feedback"):
                if self.zero.is_set():
                    raise ValueError("正在标零")
                self.requested.set()
            elif action == "disable":
                self.requested.clear()
            elif action == "align" and self.align is not None:
                self.align.set()
            elif action == "configure_grasp" and self.grasp is not None:
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
            elif action == "home_grasp" and self.grasp is not None:
                settings = self.grasp_request()
                settings.update(layout="palm", drag=None, angle_deg=0., home=settings.get("home", 0) + 1)
                settings["contact_revision"] += 1
                self.grasp.value = json.dumps(settings).encode()
            elif action == "start_recording" and self.record_requested is not None:
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
                if self.grasp is not None:
                    settings = self.grasp_request()
                    settings.update(layout="palm", drag=None, angle_deg=0.)
                    settings["contact_revision"] += 1
                    self.grasp.value = json.dumps(settings).encode()
                self.scene_revision.value += 1
            elif action == "zero":
                if self.demo_grip is not None:
                    raise ValueError("演示模式没有真机零位")
                if self.zero.is_set():
                    raise ValueError("正在标零")
                current = self.read()
                for key in ("device", "zero_path", "axes"):
                    if key in message and message[key] != current.get(key):
                        raise ValueError("请求与当前网关、标零文件或轴映射不同")
                self.zero.set()
                self.requested.clear()
            else:
                raise ValueError("当前模式不支持该操作")
