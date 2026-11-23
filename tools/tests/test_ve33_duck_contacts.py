"""鸭灯在软、硬两端使用各自有效的手部碰撞方向。"""
from pathlib import Path
import sys
import unittest

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ve33_grasp_assets import ASSET_ROOT, GRASP_ASSETS, GraspPrefab


class DuckContactsTest(unittest.TestCase):
    def test_only_rigid_duck_uses_hand_to_duck_sdf(self):
        import superdex.physics as sdp

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("鸭灯接触方向")
            for hardness in (1., .99):
                with self.subTest(hardness=hardness):
                    settings = dict(layout="palm", size_mm=108., hardness=hardness, angle_deg=0.)
                    duck = GraspPrefab(sdp, scene, GRASP_ASSETS["official_duck_lamp"], settings,
                                       1, Rotation.identity(), None)
                    try:
                        actor = duck.surface_actor
                        layer = actor.get_contact_layer()
                        if hardness == 1.:
                            self.assertEqual(actor.get_collider_type(), sdp.ColliderType.SDF)
                            self.assertTrue(scene.is_layer_contact_enabled("RightHand", layer))
                            self.assertFalse(scene.is_layer_contact_enabled(layer, "RightHand"))
                        else:
                            self.assertEqual(actor.get_collider_type(), sdp.ColliderType.NONE)
                            self.assertTrue(scene.is_layer_contact_enabled(layer, "RightHand"))
                        for a, b in ((layer, "tabletop"), ("tabletop", layer)):
                            self.assertTrue(scene.is_layer_contact_enabled(a, b))
                    finally:
                        duck.destroy()
        finally:
            sdp.shutdown()

    def test_contact_skin_preserves_fem_and_tracks_deforming_surface_nodes(self):
        import superdex.physics as sdp

        folder = ASSET_ROOT / "duck_lamp"
        original = sdp.model.load_from_file(str(folder / "duck_lamp.mochi.h5"))
        baked = sdp.model.load_from_file(str(folder / "duck_lamp_hand_contact.mochi.h5"))
        np.testing.assert_array_equal(baked.mesh.coordinates, original.mesh.coordinates)
        np.testing.assert_array_equal(baked.mesh.connectivity, original.mesh.connectivity)
        indices = np.array(baked.contact_skin_mesh.skinning.indices)
        original_nodes = np.asarray(original.mesh.coordinates).reshape(-1, 3)
        np.testing.assert_array_equal(original_nodes[indices].min(axis=0), original_nodes.min(axis=0))
        np.testing.assert_array_equal(original_nodes[indices].max(axis=0), original_nodes.max(axis=0))
        self.assertEqual(baked.contact_skin_mesh.skinning.weights_per_node, 1)
        np.testing.assert_array_equal(baked.contact_skin_mesh.skinning.weights, np.ones(len(indices)))

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("柔体接触蒙皮随物体形变")
            scene.set_gravity([0., 0., 0.])
            settings = dict(layout="desk", size_mm=108., hardness=.98, angle_deg=0., locked=False, drag=None)
            duck = GraspPrefab(sdp, scene, GRASP_ASSETS["official_duck_lamp"], settings, 1, Rotation.identity(), None)
            try:
                actor = duck.soft
                self.assertEqual(actor.get_type(), sdp.ActorType.SOFT)
                skin = actor.get_contact_skin_mesh()
                faces = np.array(skin.connectivity).reshape(-1, 3)
                self.assertEqual(len(faces), 672)
                rest = np.array(skin.coordinates).reshape(-1, 3)
                mesh = trimesh.Trimesh(rest, faces, process=False)
                self.assertTrue(mesh.is_watertight and mesh.is_winding_consistent)
                self.assertGreater(mesh.volume, 0.)
                actor.register_query_and_compute(sdp.QueryType.CONTACT_SKIN_NODE_POSITIONS)
                actor.register_query_and_compute(sdp.QueryType.NODE_POSITIONS)
                nodes = np.array(actor.get_node_positions_local()).reshape(-1, 3)
                center = nodes.mean(axis=0)
                deformed = (nodes - center) @ [[.90, .03, 0.], [0., 1.03, 0.], [0., 0., 1.02]] + center
                actor.set_node_positions_local(deformed.ravel())
                scene.step(1./240)
                new_nodes = np.array(actor.get_node_positions_local()).reshape(-1, 3)
                skin_nodes = np.array(actor.get_contact_skin_mesh_node_positions_local()).reshape(-1, 3)
                np.testing.assert_allclose(skin_nodes, new_nodes[indices], atol=1e-7, rtol=0.)
                self.assertGreater(np.max(abs(skin_nodes-rest)), .001)
                new_mesh = trimesh.Trimesh(skin_nodes, faces, process=False)
                self.assertTrue(np.all(np.einsum("ij,ij->i", mesh.face_normals, new_mesh.face_normals) > 0.))
            finally:
                duck.destroy()
        finally:
            sdp.shutdown()
