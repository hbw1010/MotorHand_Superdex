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
            self.assertEqual(message_a, message_b)
            if not reaction.any():
                self.assertTrue(all(c.tau_ff == 0. for c in a))

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

        self.assertAlmostEqual(settle(real - .1, contact), -2.3, places=5)
        self.assertAlmostEqual(settle(real - .003, contact), -.3 - .001 * 55.6 * 1.5, places=5)
        self.assertAlmostEqual(settle(real - .001, contact), -.3, places=5)
        self.assertAlmostEqual(settle(real + .1, contact), -.3, places=5)
        self.assertAlmostEqual(settle(real - .3, contact), -2.3, places=5)
        grazing = contact * .05
        self.assertLess(abs(settle(real - .3, grazing)), .05)
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
        self.assertAlmostEqual(loop.position_torque[0], -2., places=6)
        self.assertAlmostEqual(commands[0].tau_ff, -2. - .1097, places=6)

    def test_contact_uses_projected_torque_inside_the_existing_limit(self):
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        loop = ContactForceFeedbackLoop(Mock(), axes)
        zeros = np.zeros(12)

        def settled(load, ramp=1.):
            loop.reset()
            raw = zeros.copy()
            raw[0] = load
            for _ in range(48):
                cmds, _ = loop.plan(zeros, zeros, [1, 1, 1, *raw, *zeros], ramp, .01, 0)
            return cmds[0].tau_ff

        self.assertAlmostEqual(settled(.19), .19, places=6)
        self.assertAlmostEqual(settled(-5.), -5., places=6)
        self.assertAlmostEqual(settled(12.), 9.5, places=6)
        self.assertAlmostEqual(settled(12., ramp=.25), 2.375, places=6)
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
        self.assertAlmostEqual(loop.position_torque[4], -.018 * 55.6, places=7)
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
        self.assertEqual(steady(0., -1.), 0.)

    def test_stale_contact_fades_on_retreat_and_restores_with_slew(self):
        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        held = np.full(12, .4)
        zero = np.zeros(12)
        for direction in (-1., 1.):
            with self.subTest(direction=direction):
                loop = ContactForceFeedbackLoop(Mock(), axes)
                reaction = zero.copy()
                reaction[4] = direction * 8.
                reaction[5] = direction * 9.5

                def plan(q, sampled=held, dq=zero):
                    frame = [1, 1, True, *reaction, *zero, *sampled, *sampled]
                    commands, _ = loop.plan(q, dq, frame, 1., .01, 0)
                    return direction * commands[5].tau_ff

                for _ in range(24):
                    force = plan(held)
                self.assertGreater(force, 9.4)

                still_pressing = held.copy()
                still_pressing[5] -= direction * math.radians(6.)
                self.assertGreater(plan(still_pressing), 9.4)
                partial = held.copy()
                partial[5] += direction * math.radians(2.)
                self.assertAlmostEqual(plan(partial), 4.75, places=5)

                withdrawn = held.copy()
                withdrawn[5] += direction * math.radians(6.)
                self.assertEqual(plan(withdrawn), 0.)
                self.assertEqual(plan(withdrawn), 0.)
                self.assertAlmostEqual(loop.filtered_torque[4] * direction, 8., places=5)

                # A new force frame can still reflect elastic force while the hand moves.
                returning = zero.copy()
                returning[5] = direction * 2.
                self.assertGreater(plan(withdrawn, sampled=withdrawn, dq=returning), 0.)
                self.assertLessEqual(loop.filtered_torque[5] * direction, .6)
                for _ in range(24):
                    force = plan(withdrawn, sampled=withdrawn, dq=returning)
                self.assertGreater(force, 8.)

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
            gateway.release.assert_not_called()
            self.assertFalse(loop.match_grasp_frame(frame, 1))
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
        self.assertTrue(all(command[:4] == (0., 0., 0., 0.) for command in batch))
        sign = axes[6][1].sign * CONFIG["torque_sign"][5]
        self.assertGreater(-sign * batch[5][4], .9)
        self.assertEqual(sum(command[4] != 0. for command in batch), 1)
        t += 10_000_000
        with patch("motorhand.feedback.time.monotonic_ns", return_value=t):
            loop.tick(q + .02, raw, t, [t, t, 1, *np.zeros(12), *np.zeros(12)])
        batch, _ = gateway.send.call_args.args
        self.assertEqual(batch, [(0., 0., 0., 0., 0.)] * 12)
        self.assertTrue(loop.active)

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
        self.assertEqual(CONFIG["torque_limit_nm"], [10.] * 12)
        self.assertEqual(CONFIG["driver_impedance"]["torque_margin"], .95)
        planner = DriverImpedancePlanner(CONFIG["driver_impedance"], CONFIG["torque_limit_nm"])
        q = np.full(12, 1.2)
        cmds = planner.plan(q, np.zeros(12), {i: (1, 0., 55.6) for i in range(12)})
        for cmd, pos in zip(cmds, q):
            self.assertAlmostEqual(cmd.k * (pos-cmd.target_rad), 9.5)

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
            self.assertEqual(bits >> 40, 0)
            torque = (bits & 4095) * 20. / 4095 - 10.
            self.assertGreater(torque, .4)  # J6 的反向电机映射


if __name__ == "__main__":
    unittest.main()
