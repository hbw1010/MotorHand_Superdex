"""G4 采集进程与场景 HTTP 服务共享的操作和状态。"""

import json


class WebControl:
    def __init__(self, context, align, *, default_enabled=True, cube_switch=False, allow_reset=False):
        self.align = align
        self.requested = context.Event()
        self.zero = context.Event()
        self.feedback_gain = context.Value("d", 1.)
        self.cube_fixed = context.Event() if cube_switch else None
        self.scene_revision = context.Value("Q", 0, lock=False) if allow_reset or cube_switch else None
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

    def command(self, message):
        with self.command_lock:
            action = message.get("action")
            if action == "enable" and self.read().get("can_feedback"):
                if self.zero.is_set():
                    raise ValueError("正在标零")
                self.requested.set()
            elif action == "disable":
                self.requested.clear()
            elif action == "feedback_gain" and self.read().get("can_feedback"):
                gain = message.get("value")
                if self.zero.is_set() or type(gain) not in (int, float) or gain not in (1., 1.5, 2., 2.5, 3., 3.5, 4., 5., 6., 7., 8.):
                    raise ValueError("力反馈倍率必须是网页可选的 1 到 8 倍，标零时不可改变")
                self.feedback_gain.value = float(gain)
            elif action == "align" and self.align is not None:
                self.align.set()
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
                self.scene_revision.value += 1
            elif action == "zero":
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
