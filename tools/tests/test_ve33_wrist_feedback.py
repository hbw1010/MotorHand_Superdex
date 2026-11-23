"""验证 Quest 位姿中断与 G4 指令租期的边界。"""
import json
import math
import multiprocessing as mp
from pathlib import Path
import socket
import sys
from threading import Timer
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motorhand.feedback import ContactForceFeedbackLoop, FeedbackLoop, MAPPING
from motorhand.mapping import GloveMapping
from ve33_quest import WristBridge, wrist_server
from ve33_web_control import WebControl


class Gateway:
    def __init__(self):
        self.commands = []
        self.released = False

    def send(self, batch, deadline):
        self.commands.append((batch, deadline))

    def release(self):
        self.released = True


class WristFeedbackTest(unittest.TestCase):
    def test_fresh_frame_after_expired_output_requires_manual_rearm(self):
        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        gateway = Gateway()
        feedback = ContactForceFeedbackLoop(gateway, axes)
        q = np.zeros(12)
        reaction = q.copy()
        reaction[4] = -1.
        raw = dict.fromkeys(axes, 0.)
        with patch("motorhand.feedback.time.monotonic_ns", return_value=1_095_000_000):
            feedback.start()
            feedback.started -= 200_000_000
            feedback.follow_request(q, raw, 1_094_000_000,
                                    [1_000_000_000, 1_093_000_000, True, *reaction, *q, *q, *q], True)
        self.assertEqual(gateway.commands[0][1], 1_100_000_000)
        self.assertNotEqual(gateway.commands[0][0][4][4], 0.)
        frame = [1_055_000_000, 1_104_000_000, True, *reaction, *q, *q, *q]
        with patch("motorhand.feedback.time.monotonic_ns", return_value=1_105_000_000):
            feedback.follow_request(q, raw, 1_104_000_000, frame, True)
        self.assertFalse(feedback.active)
        self.assertTrue(feedback.rearm_required)
        self.assertTrue(gateway.released)
        self.assertEqual(len(gateway.commands), 1)
        for j in range(35):
            t = 1_115_000_000 + j*10_000_000
            frame[:2] = [t, t]
            with patch("motorhand.feedback.time.monotonic_ns", return_value=t):
                feedback.follow_request(q, raw, t, frame, True)
        self.assertTrue(feedback.rearm_required)
        self.assertEqual(len(gateway.commands), 1)
        feedback.stop()
        self.assertFalse(feedback.rearm_required)

    def test_expired_batch_with_long_gap_or_short_new_lease_pauses(self):
        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        q = np.zeros(12)
        raw = dict.fromkeys(axes, 0.)
        for now, sample in ((1_130_000_000, 1_088_000_000), (1_105_000_000, 1_008_000_000)):
            with self.subTest(now=now, sample=sample):
                gateway = Gateway()
                feedback = ContactForceFeedbackLoop(gateway, axes)
                with patch("motorhand.feedback.time.monotonic_ns", return_value=1_095_000_000):
                    feedback.start()
                    feedback.follow_request(q, raw, 1_094_000_000,
                                            [1_000_000_000, 1_093_000_000, True, *q, *q, *q, *q], True)
                with patch("motorhand.feedback.time.monotonic_ns", return_value=now):
                    feedback.follow_request(q, raw, now-1_000_000, [sample, now-2_000_000, True, *q, *q, *q, *q], True)
                self.assertEqual(len(gateway.commands), 1)
                self.assertFalse(feedback.active)
                self.assertEqual(feedback.interruption_count, 1)

    def test_new_contact_frame_uses_same_encoder_sample_without_extending_lease(self):
        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        gateway = Gateway()
        feedback = ContactForceFeedbackLoop(gateway, axes)
        q = np.zeros(12)
        reaction = q.copy()
        reaction[4] = -.3
        raw = dict.fromkeys(axes, 0.)
        stamp = 1_000_000_000
        with patch("motorhand.feedback.time.monotonic_ns", return_value=stamp):
            feedback.start()
        feedback.started -= 200_000_000
        for t in (stamp + 10_000_000, stamp + 20_000_000, stamp + 20_000_000):
            with patch("motorhand.feedback.time.monotonic_ns", return_value=t):
                feedback.tick(q, raw, stamp, [stamp, t, True, *reaction, *q, *q, *q])
        self.assertEqual(len(gateway.commands), 2)
        self.assertTrue(all(d == stamp + 100_000_000 for _, d in gateway.commands))
        with patch("motorhand.feedback.time.monotonic_ns", return_value=stamp + 110_000_000):
            feedback.follow_request(q, raw, stamp + 110_000_000, [stamp, stamp + 50_000_000, True, *reaction, *q, *q, *q], True)
        self.assertFalse(feedback.active)
        self.assertEqual(feedback.interruption_count, 1)
        self.assertIn("仿真使用的编码器样本", feedback.last_interruption)

    def test_missing_pose_preserves_impedance_until_simulation_frame_expires(self):
        bridge = WristBridge()
        gateway = Gateway()
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        loop = FeedbackLoop(gateway, axes)
        pose = [0., 0., 0., 0., 0., 0., 1.]
        q = np.full(12, .05)
        surfaces = [0.] + [math.nan] * 11
        raw = {m.motor_id: m.zero_deg for m in mapping.motors}
        with patch('ve33_quest.time.monotonic', return_value=1.), patch('motorhand.feedback.time.monotonic_ns', return_value=1_000_000_000):
            bridge.receive({"space": "quest", "seq": 1, "pose": pose, "align": True})
            self.assertEqual(bridge.wrist()[1], "右腕跟踪中")
            loop.start([1_000_000_000, bridge.wrist()[3], 1, *surfaces])
            loop.started -= 200_000_000
        with patch('ve33_quest.time.monotonic', return_value=1.035), patch('motorhand.feedback.time.monotonic_ns', return_value=1_035_000_000):
            bridge.receive({"space": "quest", "seq": 2, "pose": None})
            self.assertEqual(bridge.wrist()[1], "右腕跟踪中")
            loop.tick(q, raw, 1_035_000_000,
                      [1_035_000_000, 1_035_000_000, 1, *surfaces])
            (batch, deadline), = gateway.commands
            self.assertGreater(batch[0][1], 0.)
            self.assertEqual(deadline, 1_135_000_000)
        with patch('ve33_quest.time.monotonic', return_value=1.12), patch('motorhand.feedback.time.monotonic_ns', return_value=1_120_000_000):
            self.assertEqual(bridge.wrist()[1], "等待右腕追踪")
            loop.tick(q, raw, 1_120_000_000, [1_120_000_000, 1_120_000_000, 1, *surfaces])
            self.assertTrue(loop.active)
            self.assertFalse(gateway.released)
            self.assertEqual(len(gateway.commands), 2)
        with patch('motorhand.feedback.time.monotonic_ns', return_value=1_221_000_000):
            loop.tick(q, raw, 1_221_000_000, [1_120_000_000, 1_120_000_000, 1, *surfaces])
        self.assertFalse(gateway.released)
        self.assertFalse(loop.active)
        self.assertIn("仿真接触帧过期", loop.message)

    def test_command_deadline_respects_the_oldest_contact_input(self):
        gateway = Gateway()
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        loop = FeedbackLoop(gateway, axes)
        raw = {m.motor_id: m.zero_deg for m in mapping.motors}
        q = np.full(12, .2)
        surfaces = [0.] + [math.nan] * 11
        with patch("motorhand.feedback.time.monotonic_ns", return_value=1_000_000_000):
            loop.start()
            loop.started -= 200_000_000
            loop.tick(q, raw, 995_000_000, [990_000_000, 975_000_000, 1, *surfaces])
        with patch("motorhand.feedback.time.monotonic_ns", return_value=1_060_000_000):
            loop.tick(q, raw, 1_055_000_000, [1_050_000_000, 1_030_000_000, 1, *surfaces])
        self.assertTrue(loop.active)
        self.assertFalse(gateway.released)
        self.assertEqual([d for _, d in gateway.commands], [1_075_000_000, 1_130_000_000])

    def test_wrist_exchange_returns_the_latest_published_scene(self):
        bridge = WristBridge()
        bridge.scene = {"version": 1, "frame": 42, "wrist_received_ns": 10}
        with socket.socket() as endpoint:
            endpoint.bind(("127.0.0.1", 0))
            port = endpoint.getsockname()[1]
        data = {"space": "headset", "seq": 1, "pose": [0, 0, 0, 0, 0, 0, 1], "want_scene": True}
        request = Request(f"http://127.0.0.1:{port}/api/v1/wrist", json.dumps(data).encode(),
                          {"Content-Type": "application/json"})
        with wrist_server(bridge, port):
            with urlopen(request, timeout=2) as response:
                self.assertEqual(response.status, 200)
                self.assertEqual(json.load(response), bridge.scene)
            self.assertEqual(bridge.sequence, 1)
            data.pop("want_scene")
            data["seq"] = 2
            wrist_only = Request(request.full_url, json.dumps(data).encode(),
                                 {"Content-Type": "application/json"})
            with urlopen(wrist_only, timeout=2) as response:
                self.assertEqual(response.status, 204)
                self.assertEqual(response.read(), b"")
            self.assertEqual(bridge.sequence, 2)
            with urlopen(f"http://127.0.0.1:{port}/api/v1/scene", timeout=2) as response:
                self.assertEqual(json.load(response), bridge.scene)

    def test_scene_request_waits_for_a_new_frame(self):
        bridge = WristBridge()
        bridge.scene = {"version": 1, "frame": 42}
        with socket.socket() as endpoint:
            endpoint.bind(("127.0.0.1", 0))
            port = endpoint.getsockname()[1]
        url = f"http://127.0.0.1:{port}/api/v1/scene?after=42"
        with wrist_server(bridge, port):
            with urlopen(url, timeout=2) as response:
                self.assertEqual(response.status, 204)
                self.assertEqual(response.read(), b"")
            updated = {"version": 1, "frame": 43}
            timer = Timer(.025, bridge.publish_scene, args=(updated,))
            timer.start()
            try:
                with urlopen(url, timeout=2) as response:
                    self.assertEqual(json.load(response), updated)
            finally:
                timer.cancel()
                timer.join()

    def test_web_controls_share_status_and_stop_for_calibration(self):
        ctx = mp.get_context("spawn")
        align = ctx.Event()
        control = WebControl(ctx, align)
        self.assertTrue(control.requested.is_set())
        self.assertFalse(WebControl(ctx, align, default_enabled=False).requested.is_set())
        control.publish({"can_feedback": True, "online": 12, "device": "/dev/ttyUSB0"})
        with socket.socket() as endpoint:
            endpoint.bind(("127.0.0.1", 0))
            port = endpoint.getsockname()[1]
        url = f"http://127.0.0.1:{port}/api/v1/control"

        def command(action, origin=f"http://localhost:{port}", **options):
            request = Request(url, json.dumps({"action": action, **options}).encode(),
                              {"Content-Type": "application/json", "Origin": origin})
            with urlopen(request, timeout=2) as response:
                self.assertEqual(response.status, 204)

        with wrist_server(WristBridge(), port, control=control):
            with urlopen(url, timeout=2) as response:
                state = json.load(response)
                self.assertEqual(state["online"], 12)
                self.assertNotIn("feedback_gains", state)
            command("disable")
            self.assertFalse(control.requested.is_set())
            with self.assertRaises(HTTPError) as error:
                command("feedback_gain", value=64.)
            self.assertEqual(error.exception.code, 400)
            with self.assertRaises(HTTPError) as error:
                command("enable", origin="http://example.test")
            self.assertEqual(error.exception.code, 403)
            self.assertFalse(control.requested.is_set())
            command("align")
            self.assertTrue(align.is_set())
            command("enable")
            with self.assertRaises(HTTPError) as error:
                command("zero", device="/dev/ttyUSB1")
            self.assertEqual(error.exception.code, 400)
            self.assertTrue(control.requested.is_set())
            command("zero", device="/dev/ttyUSB0")
            self.assertTrue(control.zero.is_set())
            self.assertFalse(control.requested.is_set())
            with self.assertRaises(HTTPError) as error:
                command("enable")
            self.assertEqual(error.exception.code, 400)

    def test_missing_controller_is_distinct_from_alignment_and_from_timeout(self):
        bridge = WristBridge()
        self.assertEqual(bridge.tracking_detail(), "等待头显位姿数据")
        bridge.receive({"space": "quest", "seq": 1, "pose": None, "source_status": "no_right_controller"})
        self.assertEqual(bridge.tracking_detail(), "头显未报告右手柄")
        bridge.receive({"space": "quest", "seq": 2, "pose": None, "source_status": "emulated_position"})
        self.assertEqual(bridge.tracking_detail(), "手柄位置未被追踪")
        bridge.receive({"space": "quest", "seq": 3, "pose": [0, 0, 0, 0, 0, 0, 1]})
        self.assertEqual(bridge.tracking_detail(), "右腕已连接，请对齐参考姿态")
        self.assertEqual(bridge.wrist(calibrate=True)[1], "右腕跟踪中")
        self.assertEqual(bridge.tracking_detail(), "右腕跟踪中")
        with patch('ve33_quest.time.monotonic', return_value=bridge.received + .08):
            bridge.receive({"space": "quest", "seq": 4, "pose": None, "source_status": "right_hand_only"})
            self.assertEqual(bridge.tracking_detail(), "头显只报告右手光学追踪")
            self.assertEqual(bridge.wrist()[3], 0)

    def test_default_request_recovers_only_after_stable_frames_and_manual_stop_latches(self):
        gateway = Gateway()
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        loop = FeedbackLoop(gateway, axes)
        raw = {m.motor_id: m.zero_deg for m in mapping.motors}
        q = np.full(12, .2)
        surfaces = [0.] + [math.nan] * 11

        def update(ms, valid=True, requested=True):
            stamp = ms * 1_000_000
            frame = [stamp, stamp if valid else 0, int(valid), *surfaces]
            with patch("motorhand.feedback.time.monotonic_ns", return_value=stamp):
                loop.follow_request(q, raw, stamp, frame, requested)

        update(1000, valid=False)
        update(1010)
        update(1100)
        self.assertFalse(gateway.commands)
        update(1220)
        self.assertTrue(loop.active)
        update(1250, valid=False)
        self.assertFalse(gateway.released)
        self.assertFalse(loop.active)
        update(1300)
        update(1430)
        self.assertFalse(loop.active)
        update(1500)
        self.assertTrue(loop.active)
        update(1520, requested=False)
        sent = len(gateway.commands)
        update(1800, requested=False)
        self.assertFalse(loop.active)
        self.assertEqual(len(gateway.commands), sent)
        self.assertTrue(gateway.released)

    def test_submission_timeout_pauses_without_actively_releasing(self):
        gateway = Gateway()
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        loop = FeedbackLoop(gateway, axes)
        raw = {m.motor_id: m.zero_deg for m in mapping.motors}
        q = np.full(12, .2)
        surfaces = [0.] + [math.nan] * 11

        def update(ms):
            stamp = ms * 1_000_000
            frame = [stamp, stamp, 1, *surfaces]
            with patch("motorhand.feedback.time.monotonic_ns", return_value=stamp):
                loop.follow_request(q, raw, stamp, frame, True)

        update(1000)
        update(1181)
        self.assertTrue(loop.active)
        with patch.object(gateway, "send", side_effect=TimeoutError("G4 指令已过期")):
            update(1200)
        self.assertFalse(gateway.released)
        self.assertFalse(loop.active)
        self.assertFalse(loop.output_error)
        update(1210)
        update(1350)
        self.assertFalse(loop.active)
        update(1391)
        self.assertTrue(loop.active)
        with patch.object(gateway, "send", side_effect=RuntimeError("网关不可用")):
            update(1410)
        self.assertFalse(loop.active)
        self.assertTrue(loop.output_error)
        update(1600)
        update(1800)
        self.assertTrue(loop.output_error)
        self.assertFalse(loop.active)

    def test_pose_expiring_during_planning_is_never_submitted(self):
        gateway = Gateway()
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        loop = FeedbackLoop(gateway, axes)
        raw = {m.motor_id: m.zero_deg for m in mapping.motors}
        q = np.full(12, .2)
        surfaces = [0.] + [math.nan] * 11
        with patch("motorhand.feedback.time.monotonic_ns", return_value=1_000_000_000):
            loop.start()
        with patch("motorhand.feedback.time.monotonic_ns",
                   side_effect=[1_098_000_000, 1_100_000_000, 1_100_000_000]):
            loop.tick(q, raw, 1_098_000_000, [1_080_000_000, 1_000_000_000, 1, *surfaces])
        self.assertFalse(gateway.commands)
        self.assertFalse(gateway.released)
        self.assertFalse(loop.output_error)

    def test_session_pause_cancels_grace(self):
        bridge = WristBridge()
        pose = [0., 0., 0., 0., 0., 0., 1.]
        bridge.receive({"space": "quest", "seq": 1, "pose": pose, "align": True})
        self.assertEqual(bridge.wrist()[1], "右腕跟踪中")
        bridge.receive({"space": "quest", "seq": 2, "pose": None, "paused": True})
        self.assertEqual(bridge.wrist()[1], "等待右腕追踪")
        self.assertEqual(bridge.wrist()[3], 0)


if __name__ == "__main__":
    unittest.main()
