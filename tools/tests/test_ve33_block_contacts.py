"""验证官方四色方块的手部查询方向，保持原碰撞网格及其他物体交互。"""
from pathlib import Path
import sys
import unittest

from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ve33_grasp_assets import GRASP_ASSETS, GraspPrefab


class BlockContactsTest(unittest.TestCase):
    def test_hand_queries_each_block_sdf_without_reverse_queries(self):
        import superdex.physics as sdp

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("官方方块接触方向")
            settings = dict(layout="palm", size_mm=117., angle_deg=0., hardness=1.)
            for color in ("blue", "green", "red", "yellow"):
                asset = GRASP_ASSETS[f"official_block_{color}"]
                with self.subTest(color=color):
                    block = GraspPrefab(sdp, scene, asset, settings, 1, Rotation.identity(), None)
                    try:
                        self.assertEqual(len(block.contacts), 1)
                        part = block.contacts[0][1]
                        layer = part.get_contact_layer()
                        self.assertEqual(part.get_collider_type(), sdp.ColliderType.SDF)
                        self.assertTrue(scene.is_layer_contact_enabled("RightHand", layer))
                        self.assertFalse(scene.is_layer_contact_enabled(layer, "RightHand"))
                        self.assertTrue(scene.is_layer_contact_enabled("tabletop", layer))
                        self.assertTrue(scene.is_layer_contact_enabled(layer, "tabletop"))
                    finally:
                        block.destroy()
            for kind in ("official_sphere", "official_paper_cup", "official_peg", "official_fidget_box"):
                with self.subTest(kind=kind):
                    prefab = GraspPrefab(sdp, scene, GRASP_ASSETS[kind], settings, 2, Rotation.identity(), None)
                    try:
                        for _, part in prefab.contacts:
                            layer = part.get_contact_layer()
                            self.assertEqual(part.get_collider_type(), sdp.ColliderType.SDF)
                            self.assertTrue(scene.is_layer_contact_enabled("RightHand", layer))
                            self.assertFalse(scene.is_layer_contact_enabled(layer, "RightHand"))
                            self.assertTrue(scene.is_layer_contact_enabled("tabletop", layer))
                            self.assertTrue(scene.is_layer_contact_enabled(layer, "tabletop"))
                            self.assertTrue(scene.is_layer_contact_enabled(layer, layer))
                    finally:
                        prefab.destroy()
        finally:
            sdp.shutdown()
