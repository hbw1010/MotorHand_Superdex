"""用固定单调时钟验证 C++ 控制周期，测试网关只收集提交。"""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motorhand.feedback import ContactForceFeedbackLoop, MAPPING
from motorhand.native_control import make_feedback_control
from motorhand.mapping import GloveMapping
from motorhand.native_runtime import load_native


class NativeControlTest(unittest.TestCase):
    def setUp(self):
        self.axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        self.commands = []
        self.releases = 0
        self.loop = make_feedback_control(self.axes, load_native(), "nominal-plus")
        self.q = np.full(12, .8)
        self.now = 1_000_000_000

    def frame(self, reaction=-.27745119178421784, *, stamp=None, revision=0):
        t = self.now if stamp is None else stamp
        return [t, self.now, True, *([reaction] * 12), *([0.] * 12), *(self.q - .09),
                *self.q, *self.q, revision]

    def step(self, frame=None, *, requested=True, encoder=None, revision=0):
        frame = self.frame() if frame is None else frame
        status = self.loop.step(self.now, self.q, self.q if encoder is None else encoder,
                                [self.now] * 12, frame, requested, revision)
        if status.need_release:
            self.releases += 1
            self.loop.released()
        if status.need_send:
            self.commands.append((status.motor, status.deadline))
            self.loop.accepted()
        return self.loop.state()

    def arm_and_settle(self):
        for _ in range(75):
            self.step()
            self.now += 10_000_000

    def test_position_spring_on_all_axes_matches_doubled_reference(self):
        # 0.007 rad 同向受阻减去 0.002 死区；充分加载，且所有轴都尚未触及位置预算。
        frame = self.frame(-.25)
        frame[27:39] = self.q - .007
        state = self.step(frame, requested=False)
        np.testing.assert_allclose(state.position, -.005 * np.array([166.8] + [111.2] * 11))
        self.assertFalse(state.need_send)

    def test_live_factory_routes_finger_profile_without_changing_j1_or_release(self):
        tuned = make_feedback_control(self.axes, load_native(), "nominal-fingers")
        for force in [-.08] * 60 + [.08] * 30 + [0.]:
            frame = self.frame(force)
            args = (self.now, self.q, self.q, [self.now] * 12, frame, True, 0)
            a, b = self.loop.step(*args), tuned.step(*args)
            self.assertEqual(a.need_send, b.need_send)
            if b.need_send:
                self.assertEqual(a.motor[0], b.motor[0])
                if force:
                    np.testing.assert_allclose(b.mapped_contact[1:], np.sign(force) * 6.5, atol=1e-12)
                else:
                    np.testing.assert_array_equal(b.motor, np.zeros(12))
                self.loop.accepted()
                tuned.accepted()
            self.now += 10_000_000

    def test_preview_arming_encoding_dedup_and_release(self):
        self.step(requested=False)
        self.assertFalse(self.commands)
        self.arm_and_settle()
        self.assertTrue(self.loop.state().active)
        self.assertAlmostEqual(self.loop.state().nominal[4], -14.25, places=4)
        self.assertAlmostEqual(self.commands[-1][0][5], 14.25, places=4)
        self.step()
        count = len(self.commands)
        self.step()
        self.assertEqual(len(self.commands), count)
        self.now += 10_000_000
        retreat = self.q.copy()
        retreat[4] -= .07
        self.step(encoder=retreat)
        self.assertEqual(self.loop.state().nominal[4], 0.)
        self.assertLess(self.loop.state().nominal[5], -13.5)
        self.now += 10_000_000
        self.step(self.frame(0.))
        np.testing.assert_array_equal(self.loop.state().nominal, 0.)
        self.step(requested=False)
        self.assertFalse(self.loop.state().active)
        self.assertEqual(self.releases, 1)

    def test_deadline_uses_physics_input_and_scene_revision_releases_live_batch(self):
        self.arm_and_settle()
        old = self.now - 95_000_000
        self.step(self.frame(stamp=old))
        self.assertEqual(self.commands[-1][1], old + 100_000_000)
        count = len(self.commands)
        self.now += 5_000_000
        self.step(self.frame(stamp=old))
        self.assertFalse(self.loop.state().active)
        self.assertEqual(len(self.commands), count)
        self.assertIn("编码器样本过期", self.loop.state().last_interruption)
        self.assertEqual(self.releases, 1)
        self.step(requested=False)
        self.arm_and_settle()
        releases = self.releases
        self.step(self.frame(revision=0), revision=1)
        self.assertEqual(self.releases, releases + 1)
        self.assertFalse(self.loop.state().active)
        self.assertFalse(self.loop.state().valid_nominal)

    def test_invalid_input_never_sends(self):
        self.arm_and_settle()
        count = len(self.commands)
        self.step(self.frame(float("nan")))
        self.assertEqual(len(self.commands), count)
        self.assertFalse(self.loop.state().active)
        self.assertFalse(self.loop.state().valid_nominal)

    def test_expired_output_latches_even_with_a_fresh_replacement(self):
        for gap in (0, 8_000_000, 21_000_000):
            with self.subTest(gap=gap):
                self.arm_and_settle()
                self.now = self.loop.state().deadline + gap
                count = len(self.commands)
                state = self.step()
                self.assertFalse(state.active or state.valid_sent)
                self.assertTrue(state.rearm_required)
                self.assertEqual(state.handoff_count, 0)
                self.assertIn("MIT", state.last_interruption)
                self.arm_and_settle()  # 保持旧使能，经过 750 ms 新鲜帧仍须等待手动复位。
                self.assertEqual(len(self.commands), count)
                self.assertTrue(self.loop.state().rearm_required)
                self.assertTrue(self.step(self.frame(revision=2), revision=2).rearm_required)
                self.step(requested=False)
                self.assertFalse(self.loop.state().rearm_required)
                self.now += 10_000_000
                for _ in range(18):
                    self.step()
                    self.assertFalse(self.loop.state().active)
                    self.now += 10_000_000
                self.assertTrue(self.step().active)
                self.now += 10_000_000

    def test_almost_expired_frames_do_not_arm_after_180_ms(self):
        for _ in range(25):
            frame = self.frame(stamp=self.now - 95_000_000)
            self.step(frame)
            self.now += 10_000_000
        self.assertEqual(len(self.commands), 0)
        self.assertFalse(self.loop.state().rearm_required)

    def test_active_build_after_scheduler_pause_is_at_most_one_period(self):
        reference = ContactForceFeedbackLoop(Mock(), self.axes, profile="nominal-plus")
        raw = dict.fromkeys(self.axes, 0.)
        def both(force):
            frame = self.frame(force)
            state = self.step(frame)
            with patch("motorhand.feedback.time.monotonic_ns", return_value=self.now):
                reference.follow_request(self.q, raw, self.now, frame, True,
                                         sample_stamps=[self.now] * 12, encoder_position=self.q)
            return state
        for _ in range(45):
            state = both(0.)
            self.now += 10_000_000
        self.assertTrue(state.active)
        self.now += 65_000_000  # 距上次送零 75 ms，两批输入都仍在原期限内。
        rebuilt = both(-.4)
        self.assertTrue(rebuilt.valid_sent)
        self.assertGreater(abs(rebuilt.sent[0]), .25)
        self.assertLessEqual(abs(rebuilt.sent[0]), .45)
        np.testing.assert_allclose(rebuilt.nominal, reference.nominal, atol=1e-12)
        self.now += 10_000_000
        np.testing.assert_array_equal(both(0.).sent, np.zeros(12))

    def test_sdk_rejection_cannot_be_reported_as_a_sent_batch(self):
        self.arm_and_settle()
        self.loop.step(self.now, self.q, self.q, [self.now] * 12, self.frame(), True, 0)
        self.loop.failed(False)
        self.assertTrue(self.loop.state().need_release)
        self.now += 10_000_000
        state = self.step()
        self.assertTrue(state.output_error and state.rearm_required)
        self.assertFalse(state.active or state.valid_sent or state.need_send)
        self.assertEqual(state.submitted_until, 0)
        self.assertFalse(self.step(requested=False).rearm_required)

    def test_sdk_expiry_after_sending_requires_release_and_manual_rearm(self):
        self.arm_and_settle()
        accepted = len(self.commands)
        self.loop.step(self.now, self.q, self.q, [self.now] * 12, self.frame(), True, 0)
        self.loop.failed(True)
        state = self.loop.state()
        self.assertTrue(state.need_release and state.rearm_required)
        self.assertFalse(state.valid_sent or state.need_send or state.active)
        self.loop.released()
        self.arm_and_settle()
        self.assertEqual(len(self.commands), accepted)

    def test_python_native_tick_parity_for_asynchronous_sensor_and_contact_updates(self):
        gateway = Mock()
        reference = ContactForceFeedbackLoop(gateway, self.axes, profile="nominal-plus")
        rng = np.random.default_rng(1)
        stamps = np.full(12, self.now, dtype=np.int64)
        enc = self.q.copy()
        frame = self.frame()
        for tick in range(240):
            self.now += 10_000_000
            mask = rng.random(12) < .9
            stamps[mask] = self.now - 700_000
            enc[mask] = self.q[mask] + rng.uniform(-.025, .025, sum(mask))
            if tick % 4 == 0:
                frame = self.frame(0. if tick % 28 == 0 else rng.uniform(-.6, .6),
                                   stamp=int(min(stamps)))
            requested = 13 <= tick < 195
            state = self.loop.step(self.now, self.q, enc, stamps, frame, requested, 0)
            with patch("motorhand.feedback.time.monotonic_ns", return_value=self.now):
                reference.follow_request(self.q, dict.fromkeys(self.axes, 0.), int(min(stamps)),
                                         frame, requested, sample_stamps=stamps, encoder_position=enc)
            if state.need_release:
                self.loop.released()
            if state.need_send:
                batch, deadline = gateway.send.call_args.args
                np.testing.assert_allclose(np.asarray(batch)[:, 4], state.motor, atol=1e-12)
                np.testing.assert_allclose(np.asarray(batch)[:, 2], state.motor_kd, atol=1e-12)
                self.assertEqual(deadline, state.deadline)
                self.loop.accepted()
            self.assertEqual(reference.active, state.active)
            np.testing.assert_allclose(state.nominal, reference.nominal, atol=1e-12)
            np.testing.assert_allclose(state.friction, reference.friction_torque, atol=1e-12)
            np.testing.assert_allclose(state.dq, reference.velocity.dq, atol=1e-12)
            np.testing.assert_allclose(state.position, reference.position_torque, atol=1e-12)
            np.testing.assert_allclose(state.mapped_contact, reference.mapped_contact, atol=1e-12)
            np.testing.assert_array_equal(state.gate, reference.contact_gate)


if __name__ == "__main__":
    unittest.main()
