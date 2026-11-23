"""离线名义映射的配置和合成回归。"""
import sys
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motorhand.feedback import ContactForceFeedbackLoop, MAPPING
from motorhand.mapping import GloveMapping


def loop(profile='nominal', config=None):
    axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
    return ContactForceFeedbackLoop(Mock(), axes, profile=profile, profile_config=config)


def settle(instance, reaction, delta=-.1):
    q = np.full(12, .8)
    zero = np.zeros(12)
    raw = zero.copy()
    raw[4] = reaction
    sim = q.copy()
    sim[4] += delta
    frame = [1, 1, True, *raw, *zero, *sim, *q]
    for _ in range(80):
        cmds, _ = instance.plan(q, zero, frame, 1., .01, 0)
    return cmds[4].tau_ff



class NominalFeedbackTest(unittest.TestCase):
    def test_finger_profile_boosts_only_loaded_middle_range_and_preserves_j1(self):
        original, tuned = loop('nominal-plus'), loop('nominal-fingers')
        zero = np.zeros(12)
        for force in (.02, .08, -.08, .28, -.8, 0.):
            frame = [1, 1, True, *np.full(12, force), *zero, *zero, *zero]
            for _ in range(30):
                a, _ = original.plan(zero, zero, frame, 1., .01, 0)
                b, _ = tuned.plan(zero, zero, frame, 1., .01, 0)
                self.assertEqual(a[0].tau_ff, b[0].tau_ff)
            if abs(force) == .08:
                np.testing.assert_allclose(tuned.mapped_contact[1:], np.sign(force) * 6.5)
                self.assertGreater(abs(b[4].tau_ff), abs(a[4].tau_ff) + 1.)
            else:
                np.testing.assert_array_equal(tuned.mapped_contact, original.mapped_contact)
        self.assertTrue(all(command.tau_ff == 0. for command in b))

    def test_axis_overrides_reject_bad_ids_and_duplicate_assignments(self):
        anchors = [[0., 0.], [.03, .03], [.28, 5.]]
        for ids in ([], [0], [13], [True], [2, 2], [5.5]):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                loop(config={'anchors_nm': anchors, 'axis_curves': [{'motor_ids': ids, 'anchors_nm': anchors}]})

    def test_nominal_plus_increases_loaded_contact_and_releases_on_no_contact(self):
        nominal, stronger = loop(), loop('nominal-plus')
        reference = -.27745119178421784
        self.assertAlmostEqual(settle(nominal, reference), -12.)
        self.assertAlmostEqual(settle(stronger, reference), -14.25, places=6)
        self.assertEqual(settle(stronger, .02), settle(nominal, .02))
        self.assertAlmostEqual(settle(stronger, -4.), -14.25, places=6)
        self.assertEqual(settle(stronger, 0.), 0.)

    def test_nominal_maps_contact_only_with_raw_gate_and_position_budget(self):
        cfg = {'anchors_nm': [[0, 0], [.03, .03], [.28, 5], [1, 14.25]]}
        nominal = loop(config=cfg)
        self.assertAlmostEqual(settle(nominal, -.28), -8, places=6)
        self.assertAlmostEqual(nominal.position_torque[4], -3)
        self.assertEqual(settle(nominal, 0), 0)
        self.assertAlmostEqual(settle(nominal, -.28, delta=.1), -5, places=6)
        self.assertAlmostEqual(settle(nominal, -.95), -14.25, places=6)
        self.assertAlmostEqual(settle(loop('identity'), -.28), -3.28, places=6)

    def test_nominal_curve_is_monotone_signed_and_preserves_light_touch(self):
        nominal, identity = loop(), loop('identity')
        loads = [0., .009, .015, .02, .03, .05, .1, .28, .5, 1., 2.]
        values = [settle(nominal, load, delta=0.) for load in loads]
        self.assertTrue(np.all(np.diff(values) >= 0.))
        for load in (.015, .02, .03):
            with self.subTest(load=load):
                self.assertAlmostEqual(settle(nominal, load, delta=0.), settle(identity, load, delta=0.))
        self.assertAlmostEqual(settle(nominal, -.28, delta=0.), -settle(nominal, .28, delta=0.))

    def test_invalid_anchors_rejected(self):
        cases = [
            [[0, 0], [0, 1]], [[0, 0], [.2, 2], [.1, 3]],
            [[0, 0], [.1, 2], [.2, 1]], [[0, 0], [.1, 15]],
            [[0, 0], [float('nan'), 1]], [[.01, 0], [.1, 1]],
        ]
        for anchors in cases:
            with self.subTest(anchors=anchors), self.assertRaises(ValueError):
                loop(config={'anchors_nm': anchors})


if __name__ == '__main__':
    unittest.main()
