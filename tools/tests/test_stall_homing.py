"""验证支架小电流自动顶限位回零与 USB 重连保留零位逻辑。"""
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from threading import Event
import time
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motorhand.stall_homing import load_homing_config, run_stall_homing


class FakeMechanicalGlove:
    """模拟 12 轴手套：每轴带机械硬限位、初始断电跨圈偏置，以及受小力矩匀速滑向限位的物理行为。"""

    def __init__(self, stops, initial, ratios, *, speed_deg_per_nm_s=450.0, alarm_axis=None, alarm_after_ticks=2):
        self.stops = dict(stops)
        self.angles = dict(initial)
        self.ratios = dict(ratios)
        self.speed = speed_deg_per_nm_s
        self.alarm_axis = alarm_axis
        self.alarm_after_ticks = alarm_after_ticks
        self.ticks = 0
        self.sent_masks = []
        self.released = 0
        self.last_t = time.monotonic()

    def read_raw(self):
        now_ns = time.monotonic_ns()
        out = {}
        for mid, deg in self.angles.items():
            err = 1 if (self.alarm_axis == mid and self.ticks >= self.alarm_after_ticks) else 0
            out[mid] = SimpleNamespace(id=mid, bus=1, valid=1, error=err, angle_deg=deg, received_ns=now_ns)
        return out

    def send_mit_batch(self, batch, active_mask, ttl_ns):
        self.ticks += 1
        self.sent_masks.append(active_mask)
        now = time.monotonic()
        dt = min(0.05, max(0.005, now - self.last_t))
        self.last_t = now
        for mid in range(1, 13):
            if not (active_mask & (1 << (mid - 1))):
                continue
            tor = batch[(mid - 1) * 5 + 4]
            delta = tor * self.speed * dt
            nxt = self.angles[mid] + delta
            stop = self.stops[mid]
            # 沿运动方向撞上机械硬限位即停住
            if delta < 0 and nxt <= stop:
                nxt = stop
            elif delta > 0 and nxt >= stop:
                nxt = stop
            self.angles[mid] = nxt
        return 1

    def release_all(self):
        self.released += 1


class StallHomingTest(unittest.TestCase):
    def setUp(self):
        self.axes = {mid: (f"joint_{mid}", SimpleNamespace(sign=(1.0 if mid in (1, 3, 5, 7, 9, 11) else -1.0), scale=1.0))
                     for mid in range(1, 13)}
        self.ratios = [self.axes[mid][1].sign for mid in range(1, 13)]

    def test_stall_homing_recovers_exact_stops_with_disabled_j4_and_early_release(self):
        # 模拟 122.4:1 断电后各轴停在各种弯曲/跨圈角度，但机械硬限位固定在 stops
        stops = {mid: (-30.0 if self.ratios[mid - 1] > 0 else 40.0) for mid in range(1, 13)}
        initial = {}
        for mid in range(1, 13):
            # sign > 0 时弯曲在正方向；sign < 0 时弯曲在负方向
            offset = (15.0 + 4.0 * mid) * self.ratios[mid - 1]
            initial[mid] = stops[mid] + offset
        initial[4] = -37.25  # J4 被屏蔽，只读保持原值

        glove = FakeMechanicalGlove(stops, initial, {mid: self.ratios[mid - 1] for mid in range(1, 13)})
        stop_offsets = [0.0] * 12
        stop_offsets[0] = 5.0  # J1 机械限位相对 CAD 零位偏 5°
        cfg = load_homing_config({
            "stall_dwell_s": 0.08,
            "min_run_s": 0.06,
            "timeout_s": 3.0,
            "stop_offset_deg": stop_offsets,
        }, disabled_motor_ids=[4])

        accepted = []
        with TemporaryDirectory() as tmp:
            zero_path = Path(tmp) / "zero.json"
            result = run_stall_homing(
                glove.read_raw, glove.send_mit_batch, glove.release_all,
                self.axes, self.ratios, cfg,
                zero_path=zero_path,
                mapping_identity=lambda a: {str(k): [v[0], v[1].sign, 1.0] for k, v in a.items()},
                accept=accepted.append,
                tick_s=0.005, settle_s=0.0,
            )
            self.assertEqual(glove.released, 1)
            self.assertEqual(len(accepted), 1)
            # J4 全程绝不下发 MIT
            for mask in glove.sent_masks:
                self.assertEqual(mask & (1 << 3), 0)
            # 先到位的轴必须提前释放，而不是所有轴一起顶到最后
            self.assertTrue(any(bin(m).count("1") < 11 for m in glove.sent_masks))
            # 验证 J1 扣除了 stop_offset_deg=5°，J4 保持原角度，其余 10 轴精准停在机械限位
            self.assertAlmostEqual(result[1], stops[1] - 5.0, places=1)
            self.assertAlmostEqual(result[4], -37.25, places=3)
            for mid in (2, 3, 5, 6, 7, 8, 9, 10, 11, 12):
                self.assertAlmostEqual(result[mid], stops[mid], places=1)
            saved = json.loads(zero_path.read_text())
            self.assertEqual(saved["method"], "stall_homing")
            self.assertAlmostEqual(saved["zero_deg"]["5"], stops[5], places=1)

    def test_stall_homing_aborts_and_releases_on_alarm_runaway_or_cancel(self):
        stops = {mid: (-30.0 if self.ratios[mid - 1] > 0 else 40.0) for mid in range(1, 13)}
        initial = {mid: stops[mid] + 40.0 * self.ratios[mid - 1] for mid in range(1, 13)}
        cfg = load_homing_config({"stall_dwell_s": 0.08, "min_run_s": 0.06, "timeout_s": 2.0, "max_travel_deg": 25.0})

        # 1) 电机报警立即卸力中止
        glove_alarm = FakeMechanicalGlove(stops, initial, {mid: self.ratios[mid - 1] for mid in range(1, 13)},
                                          alarm_axis=7, alarm_after_ticks=2)
        with self.assertRaisesRegex(ValueError, "J7 报警"):
            run_stall_homing(glove_alarm.read_raw, glove_alarm.send_mit_batch, glove_alarm.release_all,
                             self.axes, self.ratios, cfg, tick_s=0.005, settle_s=0.0)
        self.assertEqual(glove_alarm.released, 1)

        # 2) 超行程保护立即卸力中止
        glove_runaway = FakeMechanicalGlove(stops, initial, {mid: self.ratios[mid - 1] for mid in range(1, 13)})
        with self.assertRaisesRegex(ValueError, "超过安全行程"):
            run_stall_homing(glove_runaway.read_raw, glove_runaway.send_mit_batch, glove_runaway.release_all,
                             self.axes, self.ratios, cfg, tick_s=0.005, settle_s=0.0)
        self.assertEqual(glove_runaway.released, 1)

        # 3) 用户急停取消立即卸力中止
        cancel = Event()
        cancel.set()
        glove_cancel = FakeMechanicalGlove(stops, initial, {mid: self.ratios[mid - 1] for mid in range(1, 13)})
        with self.assertRaisesRegex(ValueError, "取消"):
            run_stall_homing(glove_cancel.read_raw, glove_cancel.send_mit_batch, glove_cancel.release_all,
                             self.axes, self.ratios, cfg, cancel_event=cancel, tick_s=0.005, settle_s=0.0)
        self.assertEqual(glove_cancel.released, 1)

    def test_native_loop_preserve_zero_on_usb_reconnect_still_disarms_feedback(self):
        import multiprocessing as mp
        from multiprocessing.reduction import DupFd
        import os
        import pty
        import tty
        from motorhand.feedback import MAPPING
        from motorhand.mapping import GloveMapping
        from motorhand.native_control import make_feedback_control
        from motorhand.native_kinematics import NativeGloveKinematics
        from motorhand.native_runtime import load_native
        from test_native_runtime_pty import simulate
        from ve33_g4_view import MODEL_DIR

        native = load_native()
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        kin = NativeGloveKinematics(MODEL_DIR, mapping)
        lo, hi = zip(*(m.model_rad_limits() for m in mapping.motors))

        with TemporaryDirectory() as tmp:
            tty_link = Path(tmp) / "g4"
            shared_path = str(Path(tmp) / "stream")
            link = native.SharedLink(shared_path, len(kin.joint_names))

            def connect():
                master, slave = pty.openpty()
                tty.setraw(slave)
                tty_link.symlink_to(os.ttyname(slave))
                ctx = mp.get_context("spawn")
                pipe, child = ctx.Pipe()
                peer = ctx.Process(target=simulate, args=(DupFd(master), shared_path, child), daemon=True)
                try:
                    peer.start()
                finally:
                    child.close()
                    os.close(master)
                self.assertTrue(pipe.poll(8))
                self.assertEqual(pipe.recv(), "ready")
                return peer, pipe, slave

            def disconnect(conn):
                peer, pipe, slave = conn
                pipe.send("exit")
                peer.join(timeout=3)
                if peer.is_alive():
                    peer.terminate()
                    peer.join()
                pipe.close()
                os.close(slave)
                if tty_link.exists() or tty_link.is_symlink():
                    tty_link.unlink()

            conn = connect()
            loop = native.NativeLoop(
                kin.native, make_feedback_control(axes, native), link, str(tty_link),
                [m.sign * m.scale for m in mapping.motors], lo, hi, [0.0] * 12,
                10_000_000, 0, True,
            )
            try:
                loop.start([0.0] * 12)

                def until(pred, timeout=4.0):
                    end = time.monotonic() + timeout
                    while time.monotonic() < end:
                        rows = loop.samples(0, True)
                        if rows and pred(rows[-1]):
                            return rows[-1]
                        time.sleep(0.01)
                    self.fail("未在超时内观察到期望的重连状态")

                link.command(True, 0, 0.0)
                until(lambda s: s["feedback"].valid_sent and s["feedback"].nominal[4] < -0.5)

                # 模拟 USB 闪断
                disconnect(conn)
                conn = None
                lost = until(lambda s: not s["transport_online"])
                self.assertTrue(lost["calibrated"])
                self.assertFalse(lost["requires_new_zero"])
                self.assertFalse(lost["requested"] or lost["feedback"].valid_sent)

                # USB 重连后：保留零位（无需重新标零），且力反馈保持安全关闭直到用户显式重开
                conn = connect()
                seen = until(lambda s: s["reconnect_count"] == 1 and all(v == 1 for v in s["axis_status"]))
                self.assertTrue(seen["calibrated"])
                self.assertFalse(seen["requires_new_zero"])
                self.assertFalse(seen["requested"] or seen["feedback"].valid_sent)
                # 允许用户直接再次启用力反馈而无需重新标零
                link.command(True, 0, 0.0)
                armed = until(lambda s: s["feedback"].valid_sent and s["feedback"].nominal[4] < -0.5)
                self.assertTrue(armed["calibrated"])
            finally:
                loop.close()
                if conn is not None:
                    disconnect(conn)

    def test_web_control_home_and_cancel_with_disable(self):
        import multiprocessing as mp
        from ve33_web_control import WebControl

        ctx = mp.get_context("spawn")
        ctrl = WebControl(ctx, None, default_enabled=False, local_grasp=True, demo=False)
        ctrl.publish({"can_feedback": True, "device": "/dev/ttyUSB0", "zero_path": "/tmp/z.json", "axes": {}})
        ctrl.command({"action": "home"})
        self.assertTrue(ctrl.home.is_set())
        self.assertFalse(ctrl.requested.is_set())
        with self.assertRaisesRegex(ValueError, "正在标零或自动回零"):
            ctrl.command({"action": "enable"})
        ctrl.command({"action": "disable"})
        self.assertTrue(ctrl.cancel_home.is_set())


if __name__ == "__main__":
    unittest.main()
