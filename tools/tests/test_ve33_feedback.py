"""真实 G4 协议在伪终端上的验收。不会打开物理串口。"""
import binascii
import math
import os
from pathlib import Path
import pty
import select
import struct
import sys
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motorhand.feedback import (CONFIG, PROXY_FRAME_SIZE, ContactForceFeedbackLoop, ContactProxy,
                                FeedbackLoop, MAPPING, motor_command)
from motorhand.impedance import DriverImpedancePlanner, ImpedanceCommand, VelocityEstimator
from motorhand.mapping import MOTOR_FINGER, GloveMapping
from ve33_g4_view import MeasuredFingers, open_reader, publish_contact_frame


class FeedbackTest(unittest.TestCase):
    def test_cpp_feedback_matches_python_and_releases_all_axes(self):
        from motorhand.native_runtime import load_native

        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        cpp = ContactForceFeedbackLoop(Mock(), axes, native=load_native())
        python = ContactForceFeedbackLoop(Mock(), axes)
        rng = np.random.default_rng(33)
        for tick in range(256):
            q = rng.uniform(0., .6, 12)
            dq, sim_speed = rng.normal(0., .5, (2, 12))
            reaction = rng.uniform(-2., 2., 12) if tick % 23 else np.zeros(12)
            sim_pos = q + rng.normal(0., .03, 12)
            sampled = q + rng.normal(0., .01, 12)
            frame = [1., 1., True, *reaction, *sim_speed, *sim_pos, *sampled]
            ramp, dt = min(1., tick / 15), .008 + .001 * (tick % 5)
            a, message_a = cpp.plan(q, dq, frame, ramp, dt, 0)
            b, message_b = python.plan(q, dq, frame, ramp, dt, 0)
            np.testing.assert_allclose([c.tau_ff for c in a], [c.tau_ff for c in b], atol=1e-12)
            np.testing.assert_allclose(cpp.position_torque, python.position_torque, atol=1e-12)
            np.testing.assert_allclose(cpp.contact_damping, python.contact_damping, atol=1e-12)
            np.testing.assert_allclose(cpp.friction_torque, python.friction_torque, atol=1e-12)
            self.assertEqual(message_a, message_b)
            if not reaction.any():
                np.testing.assert_array_equal(cpp.contact_damping, np.zeros(12))
                np.testing.assert_allclose([c.tau_ff for c in a], cpp.friction_torque, atol=1e-12)
                self.assertTrue(all(abs(c.tau_ff) <= limit + 1e-12 for c, limit in zip(a, cpp.friction.limit)))

    def test_hybrid_feedback_adds_only_position_effort_matching_physical_contact(self):
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        zero = np.zeros(12)
        real = np.full(12, .4)
        contact = zero.copy()
        contact[0] = -.3

        def settle(sim, reaction):
            loop = ContactForceFeedbackLoop(Mock(), axes)
            for _ in range(48):
                cmds, _ = loop.plan(real, zero, [1, 1, 1, *reaction, *zero, *sim], 1., .01, 0)
            return cmds[0].tau_ff

        self.assertAlmostEqual(settle(real - .1, contact), -3.3, places=5)
        self.assertAlmostEqual(settle(real - .003, contact), -.3 - .001 * 166.8, places=5)
        self.assertAlmostEqual(settle(real - .001, contact), -.3, places=5)
        self.assertAlmostEqual(settle(real + .1, contact), -.3, places=5)
        self.assertAlmostEqual(settle(real - .3, contact), -3.3, places=5)
        grazing = contact * .05
        contact_only = settle(real, grazing)
        with patch.dict(CONFIG["driver_impedance"], position_compensation_gain=1.):
            original = settle(real - .3, grazing)
        self.assertLess(abs(original), .05)
        self.assertAlmostEqual(settle(real - .3, grazing), contact_only + 2 * (original - contact_only))
        self.assertEqual(settle(real - .1, zero), 0.)

    def test_recorded_j1_position_term_has_an_independent_torque_cap(self):
        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        zero = np.zeros(12)
        q = zero.copy()
        q[0] = .681
        sim = q.copy()
        sim[0] = .1893
        reaction = zero.copy()
        reaction[0] = -.1097
        loop = ContactForceFeedbackLoop(Mock(), axes)
        for _ in range(48):
            commands, _ = loop.plan(q, zero, [1, 1, True, *reaction, *zero, *sim, *q], 1., .01, 0)
        self.assertAlmostEqual(loop.position_torque[0], -3., places=6)
        self.assertAlmostEqual(commands[0].tau_ff, -3. - .1097, places=6)

    def test_contact_uses_projected_torque_inside_the_existing_limit(self):
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        loop = ContactForceFeedbackLoop(Mock(), axes)
        zeros = np.zeros(12)

        def settled(load, ramp=1.):
            loop.reset()
            raw = zeros.copy()
            raw[0] = load
            for _ in range(70):
                cmds, _ = loop.plan(zeros, zeros, [1, 1, 1, *raw, *zeros], ramp, .01, 0)
            return cmds[0].tau_ff

        self.assertAlmostEqual(settled(.19), .19, places=6)
        self.assertAlmostEqual(settled(-5.), -5., places=6)
        self.assertAlmostEqual(settled(25.), 14.25, places=6)
        self.assertAlmostEqual(settled(25., ramp=.25), 3.5625, places=6)
        self.assertEqual(settled(0.), 0.)
        self.assertEqual(settled(.00215), 0.)

    def test_position_feedback_uses_the_pose_that_drove_the_contact_frame(self):
        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        zero = np.zeros(12)
        driven = np.full(12, .4)
        live = driven + .025
        raw = zero.copy()
        raw[4] = -.3
        loop = ContactForceFeedbackLoop(Mock(), axes)
        frame = [1, 1, True, *raw, *zero, *driven, *driven]
        loop.plan(live, zero, frame, 1., .01, 0)
        self.assertEqual(loop.position_torque[4], 0.)
        frame = [1, 1, True, *raw, *zero, *(driven - .02), *driven]
        loop.plan(live, zero, frame, 1., .01, 0)
        self.assertAlmostEqual(loop.position_torque[4], -.018 * 111.2, places=7)
        rebound = zero.copy()
        rebound[4] = -1.
        loop.plan(driven, rebound, frame, 1., .01, 0)
        self.assertEqual(loop.position_torque[4], 0.)
        loop.plan(driven - .03, zero, frame, 1., .01, 0)
        self.assertEqual(loop.position_torque[4], 0.)

    def test_strong_contact_damps_rebound_and_grazing_contact_enters_smoothly(self):
        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        zero = np.zeros(12)

        def steady(force, speed):
            loop = ContactForceFeedbackLoop(Mock(), axes)
            raw, dq = zero.copy(), zero.copy()
            raw[4], dq[4] = force, speed
            for _ in range(40):
                commands, _ = loop.plan(zero, dq, [1, 1, True, *raw, *zero], 1., .01, 0)
            return commands[4].tau_ff

        self.assertGreater(steady(-.5, -1.) - steady(-.5, 0.), .25)
        self.assertLess(abs(steady(-.0105, 0.)), .001)
        self.assertEqual(steady(0., 0.), 0.)
        self.assertLess(steady(0., -1.), -.2)
        self.assertGreaterEqual(steady(0., -1.), -.48)

    def test_stale_contact_fades_on_retreat_and_restores_with_slew(self):
        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        held = np.full(12, .4)
        zero = np.zeros(12)
        for direction in (-1., 1.):
            with self.subTest(direction=direction):
                loop = ContactForceFeedbackLoop(Mock(), axes)
                reaction = zero.copy()
                reaction[4] = direction * 8.
                reaction[5] = direction * 14.25

                def plan(q, sampled=held, dq=zero):
                    frame = [1, 1, True, *reaction, *zero, *sampled, *sampled]
                    commands, _ = loop.plan(q, dq, frame, 1., .01, 0)
                    return direction * commands[5].tau_ff

                for _ in range(48):
                    force = plan(held)
                self.assertGreater(force, 14.15)

                still_pressing = held.copy()
                still_pressing[5] -= direction * math.radians(6.)
                self.assertGreater(plan(still_pressing), 14.15)
                partial = held.copy()
                partial[5] += direction * math.radians(2.)
                self.assertAlmostEqual(plan(partial), 7.125, places=5)

                withdrawn = held.copy()
                withdrawn[5] += direction * math.radians(6.)
                self.assertEqual(plan(withdrawn), 0.)
                self.assertEqual(plan(withdrawn), 0.)
                self.assertAlmostEqual(loop.filtered_torque[4] * direction, 8., places=5)

                # A new force frame passively damps fast retreat (no kick-back) and restores once steady.
                returning = zero.copy()
                returning[5] = direction * 2.
                self.assertGreater(plan(withdrawn, sampled=withdrawn, dq=returning), 0.)
                self.assertLessEqual(loop.filtered_torque[5] * direction, .45)
                for _ in range(24):
                    damped_force = plan(withdrawn, sampled=withdrawn, dq=returning)
                self.assertGreater(damped_force, 1.5)
                self.assertLess(damped_force, 4.)
                for _ in range(45):
                    force = plan(withdrawn, sampled=withdrawn, dq=zero)
                self.assertGreater(force, 13.)

                # Deep in contact, a small 2.5 deg yield should not slam torque to zero.
                deep_sim = held.copy()
                deep_sim[5] += direction * math.radians(12.)
                slight_yield = held.copy()
                slight_yield[5] += direction * math.radians(2.5)
                deep_frame = [1, 1, True, *reaction, *zero, *deep_sim, *held]
                for _ in range(30):
                    loop.plan(held, zero, deep_frame, 1., .01, 0)
                cmds, _ = loop.plan(slight_yield, zero, deep_frame, 1., .01, 0)
                self.assertGreater(direction * cmds[5].tau_ff, 6.)

                # Pushing firmly against the feedback builds resistance noticeably faster than idle.
                press_dq = zero.copy()
                press_dq[5] = -direction * 1.0
                idle_loop = ContactForceFeedbackLoop(Mock(), axes)
                push_loop = ContactForceFeedbackLoop(Mock(), axes)
                f_idle, _ = idle_loop.plan(held, zero, deep_frame, 1., .01, 0)
                f_push, _ = push_loop.plan(held, press_dq, deep_frame, 1., .01, 0)
                self.assertGreater(direction * f_push[5].tau_ff, direction * f_idle[5].tau_ff * 1.5)
                self.assertGreater(push_loop.contact_damping[5], 0.)
                self.assertLessEqual(push_loop.contact_damping[5], .28)

    def test_each_motor_velocity_uses_its_own_position_sample_time(self):
        velocity = VelocityEstimator(cutoff_hz=12)
        initial = np.full(12, 1_000_000_000, dtype=np.int64)
        zero = np.zeros(12)
        velocity.update_samples(zero, initial)
        stamps = initial.copy()
        q = zero.copy()
        stamps[0] += 10_000_000
        q[0] = .02
        first = velocity.update_samples(q, stamps).copy()
        stamps[1] += 25_000_000
        q[1] = .025
        result = velocity.update_samples(q, stamps)
        self.assertAlmostEqual(result[0], first[0])
        self.assertAlmostEqual(result[1], 1. - math.exp(-2. * math.pi * 12. * .025))

    def test_encoder_motion_past_model_limit_reduces_stale_force(self):
        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        loop = ContactForceFeedbackLoop(Mock(), axes)
        q = np.zeros(12)
        q[9] = math.pi / 2
        sampled_encoder = q.copy()
        sampled_encoder[9] = math.radians(94.5)
        old_force = np.zeros(12)
        old_force[9] = -1.0
        raw = dict.fromkeys(axes, 0.)
        stamp = 1_000_000_000

        for _ in range(26):
            stamp += 10_000_000
            frame = [stamp, stamp, True, *old_force, *np.zeros(12), *q, *q, *sampled_encoder]
            with patch("motorhand.feedback.time.monotonic_ns", return_value=stamp):
                loop.tick(q, raw, stamp, frame, sample_stamps=[stamp] * 12,
                          encoder_position=sampled_encoder)
        self.assertAlmostEqual(loop.nominal[9], -1., places=6)

        latest_encoder = sampled_encoder.copy()
        latest_encoder[9] = math.radians(91.)
        stamp += 10_000_000
        with patch("motorhand.feedback.time.monotonic_ns", return_value=stamp):
            loop.tick(q, raw, stamp, frame, sample_stamps=[stamp] * 12,
                      encoder_position=latest_encoder)
        self.assertLess(loop.velocity.dq[9], 0.)
        self.assertEqual(loop.nominal[9], 0.)

    def test_grasp_revision_releases_old_command_and_rejects_old_frame(self):
        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        gateway = Mock()
        loop = ContactForceFeedbackLoop(gateway, axes)
        q = np.zeros(12)
        load = q.copy()
        load[4] = -8.
        stamp = 1_000_000_000
        frame = [stamp, stamp, True, *load, *q, *q, *q, *q, 0.]
        self.assertTrue(loop.match_grasp_frame(frame, 0))
        with patch("motorhand.feedback.time.monotonic_ns", return_value=stamp):
            loop.start()
        loop.started = stamp - 1_000_000_000
        for _ in range(24):
            stamp += 10_000_000
            frame[:2] = [stamp, stamp]
            with patch("motorhand.feedback.time.monotonic_ns", return_value=stamp):
                loop.tick(q, dict.fromkeys(axes, 0.), stamp, frame)
        self.assertLess(loop.sent_nm[4], -7.5)

        self.assertFalse(loop.match_grasp_frame(frame, 1))
        gateway.release.assert_called_once()
        self.assertFalse(loop.active)
        self.assertIsNone(loop.nominal)
        frame[-1] = 1.
        self.assertTrue(loop.match_grasp_frame(frame, 1))
        self.assertFalse(loop.active)
        self.assertIsNone(loop.ready_since)

    def test_grasp_revision_releases_after_invalid_old_frame(self):
        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        gateway = Mock()
        loop = ContactForceFeedbackLoop(gateway, axes)
        q = np.zeros(12)
        load = q.copy()
        load[4] = -8.
        raw = dict.fromkeys(axes, 0.)
        stamp = 1_000_000_000
        frame = [stamp, stamp, True, *load, *q, *q, *q, *q, 0.]
        self.assertTrue(loop.match_grasp_frame(frame, 0))
        with patch("motorhand.feedback.time.monotonic_ns", return_value=stamp):
            loop.start()
        loop.started = stamp - 1_000_000_000
        with patch("motorhand.feedback.time.monotonic_ns", return_value=stamp):
            loop.tick(q, raw, stamp, frame)
        gateway.send.assert_called_once()

        frame[2] = False
        with patch("motorhand.feedback.time.monotonic_ns", return_value=stamp + 10_000_000):
            loop.tick(q, raw, stamp, frame)
            self.assertFalse(loop.active)
            gateway.release.assert_called_once()
            self.assertTrue(loop.rearm_required)
            self.assertFalse(loop.match_grasp_frame(frame, 1))
            self.assertTrue(loop.rearm_required)
        gateway.release.assert_called_once()

    def test_contact_force_sends_pure_torque_and_releases_in_free_space(self):
        gateway = Mock()
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        loop = ContactForceFeedbackLoop(gateway, axes)
        raw = dict.fromkeys(axes, 120.)
        reaction = np.zeros(12)
        reaction[5] = -1.2
        stamp = time.monotonic_ns()
        loop.start()
        loop.started -= 1_000_000_000
        q = np.full(12, .3)
        for i in range(5):
            t = stamp + i * 10_000_000
            with patch("motorhand.feedback.time.monotonic_ns", return_value=t):
                loop.tick(q, raw, t, [t, t, 1, *reaction, *np.zeros(12)])
        batch, _ = gateway.send.call_args.args
        self.assertTrue(all(command[:2] == (0., 0.) and command[3] == 0. for command in batch))
        self.assertTrue(all(0. <= command[2] <= .28 for command in batch))
        self.assertGreater(batch[5][2], 0.)
        self.assertTrue(all(command[2] == 0. for i, command in enumerate(batch) if i != 5))
        sign = axes[6][1].sign * CONFIG["torque_sign"][5]
        self.assertGreater(-sign * batch[5][4], .9)
        self.assertEqual(sum(command[4] != 0. for command in batch), 1)
        t += 10_000_000
        with patch("motorhand.feedback.time.monotonic_ns", return_value=t):
            loop.tick(q + .02, raw, t, [t, t, 1, *np.zeros(12), *np.zeros(12)])
        batch, _ = gateway.send.call_args.args
        # 刚脱离接触的轴立刻归零（避免松手弹回），其余自由运动轴仅输出安全限幅内的顺向摩擦补偿。
        self.assertEqual(batch[5], (0., 0., 0., 0., 0.))
        self.assertTrue(all(command[:4] == (0., 0., 0., 0.) for command in batch))
        self.assertTrue(all(abs(command[4]) <= limit + 1e-12 for command, limit in zip(batch, loop.friction.limit)))
        for _ in range(25):
            t += 10_000_000
            with patch("motorhand.feedback.time.monotonic_ns", return_value=t):
                loop.tick(q + .02, raw, t, [t, t, 1, *np.zeros(12), *np.zeros(12)])
        batch, _ = gateway.send.call_args.args
        self.assertEqual(batch, [(0., 0., 0., 0., 0.)] * 12)
        self.assertTrue(loop.active)

    def test_free_space_friction_compensation_is_bounded_directional_and_smooth_after_release(self):
        from motorhand.native_runtime import load_native

        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        cpp = ContactForceFeedbackLoop(Mock(), axes, native=load_native())
        py = ContactForceFeedbackLoop(Mock(), axes)
        zero = np.zeros(12)
        free_frame = [1, 1, True, *zero, *zero]

        # 死区与静止保持精确为零。
        small_dq = np.full(12, .02)
        for loop in (cpp, py):
            cmds, _ = loop.plan(zero, small_dq, free_frame, 1., .01, 0)
            np.testing.assert_array_equal([c.tau_ff for c in cmds], zero)

        # 自由运动双向顺向补偿，且严格低于实测动摩擦（<= max_nm），J2 上限更低。
        for sign in (1., -1.):
            cpp.reset()
            py.reset()
            dq = np.full(12, sign * 1.5)
            for _ in range(20):
                a, _ = cpp.plan(zero, dq, free_frame, 1., .01, 0)
                b, _ = py.plan(zero, dq, free_frame, 1., .01, 0)
            tau_a = np.asarray([c.tau_ff for c in a])
            tau_b = np.asarray([c.tau_ff for c in b])
            np.testing.assert_allclose(tau_a, tau_b, atol=1e-12)
            np.testing.assert_array_equal(cpp.contact_damping, zero)
            self.assertTrue(np.all(sign * tau_a > .12))
            self.assertTrue(np.all(np.abs(tau_a) <= cpp.friction.limit + 1e-12))
            self.assertLess(abs(tau_a[1]), abs(tau_a[4]))

        # 接触时关闭摩擦补偿；脱离接触首拍归零不弹手，随后 80 ms 内平滑渐入。
        contact = zero.copy()
        contact[4] = -2.0
        contact_frame = [1, 1, True, *contact, *zero]
        retreat_dq = zero.copy()
        retreat_dq[4] = -1.0
        for _ in range(10):
            cpp.plan(zero, retreat_dq, contact_frame, 1., .01, 0)
            py.plan(zero, retreat_dq, contact_frame, 1., .01, 0)
        self.assertEqual(cpp.friction_torque[4], 0.)
        self.assertEqual(py.friction_torque[4], 0.)

        first_a, _ = cpp.plan(zero, retreat_dq, free_frame, 1., .01, 0)
        first_b, _ = py.plan(zero, retreat_dq, free_frame, 1., .01, 0)
        self.assertEqual(first_a[4].tau_ff, 0.)
        self.assertEqual(first_b[4].tau_ff, 0.)
        for _ in range(15):
            settled_a, _ = cpp.plan(zero, retreat_dq, free_frame, 1., .01, 0)
            settled_b, _ = py.plan(zero, retreat_dq, free_frame, 1., .01, 0)
        self.assertAlmostEqual(settled_a[4].tau_ff, settled_b[4].tau_ff, places=12)
        self.assertLess(settled_a[4].tau_ff, -.3)
        self.assertGreaterEqual(settled_a[4].tau_ff, -.48)

    def test_close_hold_and_release(self):
        p = ContactProxy()
        q = np.zeros(12)
        p.update(q, set(), set())
        q[4:6] = [.1, .2]
        p.update(q, {"index"}, set())
        np.testing.assert_array_equal(p.visible(q), np.zeros(12))
        q[4] = -.02
        p.update(q, set(), set())
        self.assertNotIn(4, p.blocks)
        self.assertIn(5, p.blocks)
        p.update(q, set(), {"index"})
        self.assertFalse(p.blocks)

    def test_extension_only_blocks_when_contact_deepens(self):
        proxy = ContactProxy()
        previous = np.full(12, .2)
        proxy.update(previous, set(), set())
        extending = previous.copy()
        extending[5] = .16
        proxy.update(extending, {"index"}, set(), deepening=set())
        self.assertFalse(proxy.blocks)
        extending[5] = .12
        proxy.update(extending, {"index"}, set(), deepening={"index"})
        self.assertEqual(proxy.blocks[5][:2], (-1, .16))
        self.assertEqual(proxy.visible(extending)[5], .16)
        extending[5] = .18
        proxy.update(extending, {"index"}, set(), deepening=set())
        self.assertNotIn(5, proxy.blocks)

    def test_extension_impedance_resists_in_model_and_motor_frames(self):
        gateway = Mock()
        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        loop = FeedbackLoop(gateway, axes)
        q = np.full(12, .12)
        raw = dict.fromkeys(axes, 30.)
        surfaces, directions = [math.nan] * 12, [0] * 12
        surfaces[5], directions[5] = .16, -1
        now = time.monotonic_ns()
        loop.start()
        loop.started -= 1_000_000_000
        loop.tick(q, raw, now, [now, now, 1, *surfaces, *directions, 0])
        self.assertGreater(loop.nominal[5], 2.)
        batch, _ = gateway.send.call_args.args
        self.assertAlmostEqual(batch[5][0], 30. - math.degrees(.04))
        self.assertIn("1 轴伸展受阻", loop.message)

    def test_palm_contact_produces_one_bounded_cue_and_clears_on_detach(self):
        gateway = Mock()
        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        loop = FeedbackLoop(gateway, axes)
        q = np.full(12, .2)
        raw = dict.fromkeys(axes, 30.)
        loop.start()
        start = time.monotonic_ns()
        loop.started -= 1_000_000_000
        def tick(ms, palm):
            ns = start + ms * 1_000_000
            with patch("motorhand.feedback.time.monotonic_ns", return_value=ns):
                loop.tick(q, raw, ns, [ns, ns, 1, *([math.nan] * 12), *([0] * 12), palm])

        tick(0, 0)
        tick(20, 1)
        tick(50, 1)
        self.assertGreater(abs(loop.nominal[4]), .2)
        self.assertGreater(abs(loop.nominal[6]), .2)
        self.assertLessEqual(max(map(abs, loop.nominal)), 55.6 * .016 + 1e-6)
        self.assertEqual(loop.nominal[5], 0.)
        tick(75, 0)
        np.testing.assert_array_equal(loop.nominal, np.zeros(12))
        tick(95, 1)  # 接触抖动不会启动第二次触觉提示。
        tick(120, 1)
        np.testing.assert_array_equal(loop.nominal, np.zeros(12))
        tick(140, 0)
        tick(200, 1)
        tick(235, 1)
        self.assertGreater(abs(loop.nominal[4]), .2)
        tick(305, 1)
        tick(335, 1)
        np.testing.assert_array_equal(loop.nominal, np.zeros(12))

    def test_spring_budget_all_axes(self):
        self.assertEqual(CONFIG["torque_limit_nm"], [15.] * 12)
        self.assertEqual(CONFIG["driver_impedance"]["torque_margin"], .95)
        planner = DriverImpedancePlanner(CONFIG["driver_impedance"], CONFIG["torque_limit_nm"])
        q = np.full(12, 1.2)
        cmds = planner.plan(q, np.zeros(12), {i: (1, 0., 55.6) for i in range(12)})
        for cmd, pos in zip(cmds, q):
            self.assertAlmostEqual(cmd.k * (pos-cmd.target_rad), 14.25)

    def test_negative_axis_and_scale(self):
        from dataclasses import replace
        motor = replace(GloveMapping.load(MAPPING).motors[5], sign=-1, scale=2.)
        result = motor_command(motor, ImpedanceCommand(.3, 20, .1, .12), .5, 35.)
        np.testing.assert_allclose(result, [35 + math.degrees(.1), 80, .4, 0, -.24])

    def test_reported_current_has_its_own_timestamp(self):
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        def reply(payload):
            packet = bytearray(32)
            packet[:4] = bytes.fromhex("a55a0190")
            struct.pack_into("<BBH", packet, 6, 2, len(payload), 6)
            packet[16:16+len(payload)] = payload
            struct.pack_into("<H", packet, 30, binascii.crc_hqx(packet[:30], 65535))
            os.write(master, packet)

        def expect(reader, condition):
            until = time.monotonic() + .2
            while time.monotonic() < until:
                axis = reader().get(6)
                if axis is not None and condition(axis):
                    return axis
                time.sleep(.003)
            self.fail("没有读到伪网关的电机回包")

        with open_reader(Path(os.ttyname(slave))) as device:
            reply(struct.pack(">BfhB", 0x40, 42., 123, 60))
            current = expect(device, lambda j: j.valid & 2).current_received_ns
            self.assertGreater(current, 0)
            reply(struct.pack(">BBf", 0xa0, 1, 43.))
            axis = expect(device, lambda j: j.angle_deg == 43.)
            self.assertAlmostEqual(axis.current_a, 1.23, places=4)
            self.assertEqual(axis.current_received_ns, current)
            self.assertGreater(axis.received_ns, current)

    def test_g4_frames_and_command_expiry(self):
        motors = GloveMapping.load(MAPPING).motors
        axes = {m.motor_id: (m.model_joint, m) for m in motors}
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        data = bytearray()

        def collect(duration):
            frames = []
            end = time.monotonic() + duration
            while time.monotonic() < end:
                if select.select([master], [], [], .005)[0]:
                    data.extend(os.read(master, 65536))
                while len(data) >= 32:
                    frame = bytes(data[:32])
                    del data[:32]
                    self.assertEqual(frame[:4], bytes.fromhex('a55a0102'))
                    frames.append(frame)
            return frames

        with open_reader(Path(os.ttyname(slave)), True) as device:
            self.assertTrue(all(f[7] == 2 for f in collect(.02)))
            bad_batch = [(0., 0., 0., 0., .3)] * 11 + [(0., 0., 0., 0., math.nan)]
            with self.assertRaises(ValueError):
                device.send(bad_batch, time.monotonic_ns() + 90_000_000)
            self.assertTrue(all(f[7] == 2 for f in collect(.02)))
            loop = FeedbackLoop(device, axes)
            loop.start([0, 0, 0])
            self.assertFalse(loop.active)
            self.assertIn("未启用", loop.message)
            # 只有连续几何扫描拦下了食指，未限位目标的端点没有接触。
            fingers = MeasuredFingers.__new__(MeasuredFingers)
            fingers.last = np.zeros(12)
            fingers.last[5] = .20
            fingers.motor_rows = list(range(12))
            fingers.finger_names = np.array(MOTOR_FINGER)
            fingers.proxy = ContactProxy()
            fingers.project = Mock()
            guard = Mock(fingers=np.array(MOTOR_FINGER))
            guard.sweep.return_value = (0.5, None, {"corner4": [5]})
            q = fingers.last.copy()
            q[5] = .24
            fingers.guarded_angles(q, guard, publish_blocks=True)
            self.assertEqual(fingers.last[5], .20)
            self.assertEqual(fingers.proxy.blocks[5][:2], (1, .20))
            self.assertEqual(set(fingers.proxy.blocks), {5})

            fresh = time.monotonic_ns()
            loop.start([fresh, fresh, 1])
            self.assertTrue(loop.active)
            loop.started -= 1_000_000_000
            raw = {m.motor_id: 30. for m in motors}
            stamp = time.monotonic_ns()
            proxy_frame = [0.] * PROXY_FRAME_SIZE
            publish_contact_frame(proxy_frame, fingers.proxy, stamp, stamp, True, False)
            loop.tick(q, raw, stamp, proxy_frame)
            self.assertIn("1 轴受阻", loop.message)
            self.assertAlmostEqual(loop.nominal[5], -2.224, places=3)
            active = collect(.04)
            cmds = {int.from_bytes(f[8:10], 'little'):f for f in active if f[7] == 8}
            self.assertEqual(set(cmds), set(range(1,13)))
            word = int.from_bytes(cmds[6][16:24], 'big')
            kp = (word >> 49 & 4095) * 500 / 4095
            kd = (word >> 40 & 511) * 5 / 511
            deg = math.degrees((word >> 24 & 65535)*10/65535 - 5)
            self.assertAlmostEqual(kp, 55.6, delta=.07)
            self.assertAlmostEqual(kd, .008*55.6, delta=.006)
            self.assertAlmostEqual(deg, 30+math.degrees(.04), delta=.006)
            self.assertEqual(cmds[6][6], 2)
            self.assertEqual(int.from_bytes(cmds[5][16:24],'big') >> 40, 0)
            after = collect(.17)
            self.assertTrue(any(f[7] == 8 and int.from_bytes(f[16:24],'big') >> 40 == 0 for f in after))
            self.assertTrue(all(f[7] == 2 for f in after[-12:]))
            now = time.monotonic_ns()
            loop.tick(q, raw, now, proxy_frame)
            self.assertFalse(loop.active)

            force_loop = ContactForceFeedbackLoop(device, axes)
            force_loop.start()
            force_loop.started -= 1_000_000_000
            stamp = time.monotonic_ns()
            reaction = np.zeros(12)
            reaction[5] = -1.2
            force_loop.tick(q, raw, stamp, [stamp, stamp, 1, *reaction, *np.zeros(12)])
            active = collect(.04)
            packets = [bytes_ for bytes_ in active if bytes_[7] == 8 and bytes_[8] == 6]
            self.assertTrue(packets)
            bits = int.from_bytes(packets[-1][16:24], 'big')
            self.assertEqual(bits >> 49, 0)
            kd = ((bits >> 40) & 511) * 5. / 511
            self.assertGreater(kd, 0.)
            self.assertLessEqual(kd, .3)
            torque = (bits & 4095) * 40. / 4095 - 20.
            self.assertGreater(torque, .25)  # J6 的反向电机映射


if __name__ == "__main__":
    unittest.main()
