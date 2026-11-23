"""原生合成路径与分段标尺的回归。"""
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motorhand.native_runtime import load_native
from motorhand.feedback import ContactForceFeedbackLoop, MAPPING
from motorhand.mapping import GloveMapping


class NativeNominalTest(unittest.TestCase):
    def test_axis_curve_preserves_other_axes_and_zero_release(self):
        curve = [[0., 0.], [.03, .03], [.28, 5.], [1., 9.]]
        limits = [9.5] * 12
        zero, weights = [0.] * 12, [1.] * 12
        kernel = load_native().ContactFeedback(limits, zero, zero, .002, .01, .03, .25, 0., .02, curve)
        baseline = load_native().ContactFeedback(limits, zero, zero, .002, .01, .03, .25, 0., .02, curve)
        kernel.set_axis_curve(12, [[0., 0.], [.03, .03], [.1, 2.9], [.28, 5.], [1., 9.]])
        for reaction in (.02, -.10, 0.):
            for _ in range(60):
                a = kernel.plan(zero, zero, [reaction] * 12, zero, zero, weights, 1., .01)[0]
                b = baseline.plan(zero, zero, [reaction] * 12, zero, zero, weights, 1., .01)[0]
                self.assertEqual(a[:11], b[:11])
                if reaction != -.1:
                    self.assertEqual(a[11], b[11])
            if reaction == -.1:
                self.assertAlmostEqual(a[11], -2.9)
                self.assertLess(a[11], b[11])

    def test_bad_axis_curve_is_rejected(self):
        zero = [0.] * 12
        f = load_native().ContactFeedback([9.5] * 12, zero, zero, .002, .01, .03, .25, 0., .02)
        valid = [[0., 0.], [.03, .03], [1., 9.]]
        for axis, anchors in ((0, valid), (13, valid), (1, []), (1, [[0, 1], [.03, .03]]),
                              (2, [[0, 0], [.015, .019], [.03, .03], [.2, 3.]]),
                              (12, [[0, 0], [.03, .03], [.2, 10.]])):
            with self.subTest(axis=axis, anchors=anchors), self.assertRaises(ValueError):
                f.set_axis_curve(axis, anchors)

    def test_python_and_cpp_match_during_mapped_contact_and_retreat(self):
        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        python = ContactForceFeedbackLoop(Mock(), axes, profile="nominal")
        cpp = ContactForceFeedbackLoop(Mock(), axes, native=load_native(), profile="nominal")
        rng = np.random.default_rng(17)
        for tick in range(250):
            q = rng.uniform(0., .6, 12)
            dq = rng.normal(0., .5, 12)
            reaction = rng.uniform(-1., 1., 12) if tick % 19 else np.zeros(12)
            sim = q + rng.normal(0., .03, 12)
            sampled = q + rng.normal(0., .01, 12)
            frame = [1., 1., True, *reaction, *np.zeros(12), *sim, *sampled]
            a, _ = python.plan(q, dq, frame, 1., .01, 0)
            b, _ = cpp.plan(q, dq, frame, 1., .01, 0)
            np.testing.assert_allclose([c.tau_ff for c in a], [c.tau_ff for c in b], atol=1e-12)
            np.testing.assert_allclose(python.position_torque, cpp.position_torque, atol=1e-12)

    def test_curve_maps_contact_and_preserves_release(self):
        native = load_native()
        feedback = native.ContactFeedback([9.5] * 12, [0.] * 12, [2.] * 12,
                                          .002, .01, .03, .1, 0., .02,
                                          [[0., 0.], [.03, .03], [.28, 5.], [1., 9.]])
        zero = [0.] * 12
        load = [-.28] + [0.] * 11
        for _ in range(100):
            torque, _, _ = feedback.plan(zero, zero, load, zero, zero, [1.] * 12, 1., .01)
        self.assertAlmostEqual(torque[0], -5., places=6)
        torque, _, _ = feedback.plan(zero, zero, zero, zero, zero, [1.] * 12, 1., .01)
        self.assertEqual(torque[0], 0.)


if __name__ == '__main__':
    unittest.main()
