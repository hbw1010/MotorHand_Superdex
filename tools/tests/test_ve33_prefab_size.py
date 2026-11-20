"""用实际碰撞包围盒验证官方物体的整体缩放。"""
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ve33_grasp_assets import GRASP_ASSETS, GraspPrefab, nested_parts


class PrefabSizeTest(unittest.TestCase):
    def test_physical_extent_and_display_match_for_scaled_assets(self):
        import superdex.physics as sdp
        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("预制件尺寸回归")
            scene.set_gravity([0., 0., 0.])
            for kind in ("official_block_blue", "official_duck_lamp", "official_pegboard"):
                small_scales = None
                for size_mm in (20., 150.):
                    with self.subTest(kind=kind, size_mm=size_mm):
                        settings = dict(layout="desk", size_mm=size_mm, angle_deg=0., locked=True, drag=None)
                        prefab = GraspPrefab(sdp, scene, GRASP_ASSETS[kind], settings, 1,
                                             Rotation.identity(), None)
                        try:
                            boxes = [part.get_aabb_world() for actor in prefab.instance.actors
                                     for part in nested_parts(sdp, scene, actor)
                                     if len(part.get_surface_mesh().coordinates)]
                            extent = (np.max([list(box.max) for box in boxes], axis=0)
                                      - np.min([list(box.min) for box in boxes], axis=0))
                            self.assertAlmostEqual(extent.max(), size_mm / 1000., delta=.0001)
                            prefab.step(settings, 1/240)
                            scene.step(1/240)
                            packet = prefab.packet()
                            if prefab.soft is not None:
                                verts = np.array(packet['vertices']).reshape(-1, 3)
                                self.assertAlmostEqual(np.ptp(verts, axis=0).max(), size_mm / 1000., delta=.0001)
                            else:
                                scales = np.array([part["scale"] for part in packet["parts"]])
                                self.assertGreater(len(scales), 0)
                                if size_mm == 20.:
                                    small_scales = scales
                                else:
                                    np.testing.assert_allclose(scales, small_scales * 7.5)
                            self.assertNotEqual(scene.get_solver_stats().convergence_status,
                                                sdp.ConvergenceStatus.DIVERGED)
                        finally:
                            prefab.destroy()
                        self.assertEqual(scene.get_num_actors(), 0)
                        self.assertEqual(scene.get_num_constraints(), 0)
        finally:
            sdp.shutdown()

    def test_hardness_sets_contact_and_selects_rigid_duck_at_one(self):
        import superdex.physics as sdp
        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("官方资产硬度")
            scene.set_gravity([0., 0., 0.])
            energies = []
            for kind in ('official_duck_lamp', 'official_pegboard'):
                for h in (0., .8, 1.):
                    with self.subTest(kind=kind, h=h):
                        settings = dict(layout="desk", size_mm=90., angle_deg=0., locked=True,
                                        drag=None, hardness=h)
                        prefab = GraspPrefab(sdp, scene, GRASP_ASSETS[kind], settings,
                                             1, Rotation.identity(), None)
                        try:
                            actor = prefab.contacts[0][1]
                            for _, part in prefab.contacts:
                                self.assertAlmostEqual(part.get_contact_params().penalty_coefficient /
                                                       (1e7 * 10**(4*h)), 1., places=5)
                            if kind == 'official_duck_lamp':
                                self.assertEqual(actor.get_type(), sdp.ActorType.RIGID if h == 1 else sdp.ActorType.SOFT)
                                settings['angle_deg'] = 30
                                prefab.step(settings, 1/240)
                                scene.step(1/240)
                                vertices = np.array(prefab.packet()['vertices']).reshape(-1, 3)
                                self.assertAlmostEqual(np.ptp(vertices, axis=0).max(), .09, delta=.001)
                                if h < 1.:
                                    prefab.release_pins()
                                    actor.register_query(sdp.QueryType.ELASTIC_ENERGY)
                                    nodes = np.asarray(actor.get_node_positions_local()).reshape(-1, 3).copy()
                                    nodes[:, 0] *= .91
                                    actor.set_node_positions_local(nodes.ravel())
                                    scene.step(.0001)
                                    energies.append(actor.get_elastic_energy())
                        finally:
                            prefab.destroy()
            self.assertGreater(energies[1], 2 * energies[0])
        finally:
            sdp.shutdown()
