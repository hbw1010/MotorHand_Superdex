"""原版魔方保留转层所用的内部接触，仅让手查询外壳。"""
from pathlib import Path
import sys
import unittest

from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ve33_grasp_assets import GRASP_ASSETS, GraspPrefab


class OriginalCubeContactsTest(unittest.TestCase):
    def test_hand_queries_shells_and_internals_remain_connected(self):
        import superdex.physics as sdp
        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("原版魔方接触方向")
            cfg = dict(size_mm=73, hardness=1., locked=False, angle_deg=0., layout="palm", drag=None)
            cube = GraspPrefab(sdp, scene, GRASP_ASSETS["official_puzzle_cube"], cfg, 1, Rotation.identity(), None)
            try:
                shell = [p for _, p in cube.contacts if p.get_name().endswith("/corner_shell_link")]
                core = [p for _, p in cube.contacts if p not in shell]
                self.assertEqual((len(cube.instance.actors), len(shell), len(core)), (8, 8, 8))
                for p in shell:
                    layer = p.get_contact_layer()
                    self.assertEqual(p.get_collider_type(), sdp.ColliderType.SDF)
                    self.assertTrue(scene.is_layer_contact_enabled("RightHand", layer))
                    self.assertFalse(scene.is_layer_contact_enabled(layer, "RightHand"))
                for p in core:
                    layer = p.get_contact_layer()
                    self.assertFalse(scene.is_layer_contact_enabled("RightHand", layer))
                    self.assertFalse(scene.is_layer_contact_enabled(layer, "RightHand"))
                    for _, other in cube.contacts:
                        self.assertTrue(scene.is_layer_contact_enabled(layer, other.get_contact_layer()))
                        self.assertTrue(scene.is_layer_contact_enabled(other.get_contact_layer(), layer))
            finally:
                cube.destroy()
        finally:
            sdp.shutdown()
