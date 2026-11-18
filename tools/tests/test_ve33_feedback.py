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
from motorhand.impedance import DriverImpedancePlanner, ImpedanceCommand
from motorhand.mapping import MOTOR_FINGER, GloveMapping
from ve33_g4_view import MeasuredFingers, open_reader, publish_contact_frame


class FeedbackTest(unittest.TestCase):
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

        self.assertAlmostEqual(settle(real - .1, contact), -.3 - .098 * 55.6 * 1.5, places=5)
        self.assertAlmostEqual(settle(real - .003, contact), -.3 - .001 * 55.6 * 1.5, places=5)
        self.assertAlmostEqual(settle(real - .001, contact), -.3, places=5)
        self.assertAlmostEqual(settle(real + .1, contact), -.3, places=5)
        self.assertAlmostEqual(settle(real - .3, contact), -9.5, places=5)
        grazing = contact * .05
        self.assertLess(abs(settle(real - .3, grazing)), .05)
        self.assertEqual(settle(real - .1, zero), 0.)

    def test_contact_gain_amplifies_load_inside_the_existing_limit(self):
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        loop = ContactForceFeedbackLoop(Mock(), axes)
        zeros = np.zeros(12)

        def settled(load, gain):
            loop.reset()
            loop.gain = gain
            raw = zeros.copy()
            raw[0] = load
            for _ in range(48):
                cmds, _ = loop.plan(zeros, zeros, [1, 1, 1, *raw, *zeros], 1., .01, 0)
            return cmds[0].tau_ff

        self.assertAlmostEqual(settled(.19, 2.), 2. * settled(.19, 1.), places=6)
        self.assertAlmostEqual(settled(5., 4.), 9.5, places=6)
        self.assertEqual(settled(0., 4.), 0.)

    def test_feedback_gain_scales_position_compensation_and_contact_together(self):
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        real = np.full(12, .4)
        velocity = np.zeros(12)
        reaction = velocity.copy()
        reaction[0] = -.3

        def settle(gain, error=.012):
            loop = ContactForceFeedbackLoop(Mock(), axes)
            loop.gain = gain
            for _ in range(48):
                cmd, _ = loop.plan(real, velocity, [1, 1, 1, *reaction, *velocity, *(real - error)], 1., .01, 0)
            return cmd[0].tau_ff, loop.position_torque[0]

        first, first_position = settle(1.)
        second, second_position = settle(2.)
        self.assertAlmostEqual(second, first * 2., places=6)
        self.assertAlmostEqual(second_position, first_position * 2., places=6)
        self.assertAlmostEqual(settle(8., error=.02)[0], -9.5, places=6)

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
