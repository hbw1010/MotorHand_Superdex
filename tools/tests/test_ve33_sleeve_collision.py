"""指环填实使用实际资产，保留相同的 URDF 可视轮廓。"""

from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest

import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ve33_closed_chain_view import ROOT, attach_compound_visuals


class SleeveCollisionTest(unittest.TestCase):
    def test_single_part_finger_ring_receives_solid_convex_collision(self):
        import superdex.physics as sdp
        import superdex.robotics as sdr

        model_dir = ROOT / "assets/hand_motor_ve40/hand_motor_ve40_R"
        prefab = sdr.load_bot_prefab_from_file(str(model_dir / "hand_motor_ve40_right_closed_chain.superdex_bot"))
        links = {link.name: link for link in prefab.links}
        sleeve = links["index_pip_link"]
        rod_path = links["index_pip_connecting_rod_1"].shape_file
        original = sdp.model.load_from_file(sleeve.shape_file)

        def mesh(data):
            return trimesh.Trimesh(vertices=np.asarray(data.coordinates).reshape(-1, 3),
                                   faces=np.asarray(data.connectivity).reshape(-1, 3))

        with TemporaryDirectory() as directory:
            attach_compound_visuals(prefab, model_dir, directory, sdp, full_collision=True)
            filled = sdp.model.load_from_file(sleeve.shape_file)
            ring, collider = mesh(original.mesh), mesh(filled.mesh)
            self.assertTrue(collider.is_volume and collider.is_convex)
            self.assertGreater(collider.volume, 3 * ring.volume)
            # 原始碰撞文件含约 0.06 mm 的烘焙余量，填实按同一份 CAD 的外轮廓。
            np.testing.assert_allclose(collider.bounds, ring.bounds, atol=1e-4)
            np.testing.assert_allclose(collider.bounds, mesh(filled.visual_mesh).bounds, atol=2e-6)
            self.assertEqual(links["index_pip_connecting_rod_1"].shape_file, rod_path)


if __name__ == "__main__":
    unittest.main()
