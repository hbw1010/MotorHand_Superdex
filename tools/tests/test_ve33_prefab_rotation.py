"""预制件在原物理场景中连续旋转。"""
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ve33_grasp_assets import GRASP_ASSETS, GraspPrefab


class PrefabRotationTest(unittest.TestCase):
    def test_rotation_after_free_motion_uses_current_object_center(self):
        import superdex.physics as sdp

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene('解锁后绕当前位置转动')
            scene.set_gravity([0., 0., 0.])
            settings = dict(layout='desk', angle_deg=0., size_mm=65., locked=False, drag=None)
            obj = GraspPrefab(sdp, scene, GRASP_ASSETS['official_block_red'], settings,
                              1, Rotation.identity(), None)
            body = obj.roots[0]
            obj.step(settings, 1/240)
            transform = body.get_root_transform()
            position = np.array(transform.translation) + [.05, 0., 0.]
            body.set_root_transform(sdp.TransformRT(rotation=transform.rotation, translation=position))
            def center():
                box = body.get_aabb_world()
                return (np.asarray(box.min) + box.max) / 2.
            initial_center = center()
            settings['angle_deg'] = 30.
            for _ in range(140):
                obj.step(settings, 1/240)
                scene.step(1/240)
            np.testing.assert_allclose(center(), initial_center, atol=.002)
            obj.destroy()
        finally:
            sdp.shutdown()

    def test_locked_block_tracks_rotation_over_multiple_physics_steps(self):
        import superdex.physics as sdp

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene('预制件平滑转动')
            scene.set_gravity([0., 0., 0.])
            settings = dict(layout='desk', angle_deg=0., size_mm=70., locked=True, drag=None)
            obj = GraspPrefab(sdp, scene, GRASP_ASSETS['official_block_red'], settings,
                              1, Rotation.identity(), None)
            part = obj.parts[0][0]
            initial = Rotation.from_quat(part.get_root_transform().rotation)
            settings['angle_deg'] = 30.
            angles = []
            for _ in range(100):
                obj.step(settings, 1/240)
                scene.step(1/240)
                rot = Rotation.from_quat(part.get_root_transform().rotation) * initial.inv()
                angles.append(rot.as_rotvec(degrees=True)[2])
            self.assertGreater(angles[12], 1.)
            self.assertLess(angles[12], 15.)
            self.assertTrue(np.isfinite(angles).all())
            self.assertAlmostEqual(angles[-1], 30., delta=2.)
            obj.destroy()
            self.assertEqual(scene.get_num_constraints(), 0)
        finally:
            sdp.shutdown()

    def test_unlocked_board_moves_the_static_base_and_loose_parts_together(self):
        import superdex.physics as sdp

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene('桌面插销板调角')
            scene.set_gravity([0., 0., 0.])
            settings = dict(layout='desk', angle_deg=0., size_mm=90., locked=False, drag=None)
            obj = GraspPrefab(sdp, scene, GRASP_ASSETS['official_pegboard'], settings,
                              1, Rotation.identity(), None)
            bodies = [obj.static_roots[0], obj.roots[0]]
            starts = [body.get_root_transform() for body in bodies]
            settings['angle_deg'] = 30.
            for _ in range(220):
                obj.step(settings, 1/240)
                scene.step(1/240)
            yaw = Rotation.from_euler('z', 30., degrees=True)
            for body, start in zip(bodies, starts):
                pose = body.get_root_transform()
                np.testing.assert_allclose(pose.translation, obj.center + yaw.apply(start.translation - obj.center), atol=.001)
                actual = Rotation.from_quat(pose.rotation) * Rotation.from_quat(start.rotation).inv()
                self.assertLess((yaw.inv() * actual).magnitude(), .02)
            self.assertFalse(obj.position_pins or obj.rotation_pins)
            obj.destroy()
        finally:
            sdp.shutdown()

    def test_soft_duck_surface_follows_the_target_then_releases_extra_pins(self):
        import superdex.physics as sdp

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene('软体鸭灯调角')
            scene.set_gravity([0., 0., 0.])
            settings = dict(layout='desk', angle_deg=0., size_mm=65., locked=True, drag=None)
            obj = GraspPrefab(sdp, scene, GRASP_ASSETS['official_duck_lamp'], settings,
                              1, Rotation.identity(), None)
            obj.step(settings, 1/240)
            scene.step(1/240)
            def surface():
                xyz = np.array(obj.packet()['vertices']).reshape(-1, 3)
                return xyz - xyz.mean(axis=0)
            start = surface()
            settings['angle_deg'] = 30.
            for _ in range(220):
                obj.step(settings, 1/240)
                scene.step(1/240)
            angle, _ = Rotation.align_vectors(surface(), start)
            self.assertAlmostEqual(angle.as_rotvec(degrees=True)[2], 30., delta=2.)
            self.assertEqual(scene.get_num_constraints(), 1)
            obj.destroy()
            self.assertEqual(scene.get_num_constraints(), 0)
        finally:
            sdp.shutdown()
