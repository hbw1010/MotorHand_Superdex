"""完整闭链、投影及失败后原状态的离线回归。"""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motorhand.kinematics import GloveKinematics, KinematicsError
from motorhand.native_kinematics import NativeGloveKinematics
from ve33_g4_view import GloveMapping, MAPPING, MODEL_DIR


class NativeKinematicsTest(unittest.TestCase):
    def test_trajectory_derivative_projection_and_failure_are_consistent(self):
        mapping = GloveMapping.load(MAPPING)
        legacy = GloveKinematics(MODEL_DIR, mapping)
        native = NativeGloveKinematics(MODEL_DIR, mapping)
        pattern = np.array([.2, .15, .6, .6, .8, 1.2, .8, 1.2, .8, 1.2, .8, 1.2])
        loads = np.random.default_rng(52).uniform(-1., 1., len(legacy.joint_names))
        for grip in (.0, .02, .08, .16, .35, .5, .65, .5, .35, .2, .0):
            q = grip * pattern
            with self.subTest(grip=grip):
                a = legacy.full_pose(q, None, mapping)
                b = native.full_pose(q, None, mapping)
                np.testing.assert_allclose(a, b, rtol=0, atol=2e-7)
                np.testing.assert_allclose(native.tree_derivative(), legacy.tree_derivative(), atol=1e-6)
                np.testing.assert_allclose(native.project_tree(loads, q), loads @ legacy.tree_derivative(), atol=1e-6)
                self.assertLessEqual(native.last_gap_m, 1e-7)
        native.copy_slot("target", "visual")
        before = native.tree_derivative()
        native.full_pose(pattern * .2, None, mapping, slot="visual")
        np.testing.assert_array_equal(native.tree_derivative(), before)
        for invalid in (np.full(12, np.nan), np.full(12, 1e10)):
            with self.assertRaises(KinematicsError):
                native.full_pose(invalid, None, mapping)
        np.testing.assert_array_equal(native.tree_derivative(), before)
