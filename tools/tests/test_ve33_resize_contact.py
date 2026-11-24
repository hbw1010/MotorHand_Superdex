"""合拢姿态下扩大掌心方块时，检查初始几何及真实物理子步的反力。"""
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motorhand.native_kinematics import NativeGloveKinematics
from motorhand.native_runtime import load_native
from ve33_g4_view import HandContactDrive, ShellContacts, attach_compound_visuals, MODEL_DIR, MAPPING, GloveMapping
from ve33_local_grasp import DEFAULT_GRASP, UPRIGHT_WRIST_POSE, DemoGlove, LocalGrasp, grasp_settings, open_hand_surface


class ResizeContactTest(unittest.TestCase):
    def test_enlarging_locked_red_block_with_fingers_closed(self):
        import superdex.physics as sdp
        import superdex.robotics as sdr

        sdp.initialize(num_worker_threads=2)
        self.addCleanup(sdp.shutdown)
        scene = sdp.create_scene("握持时增大红方块")
        scene.set_gravity([0., 0., 0.])
        from ve33_g4_view import MODEL_BOT
        prefab = sdr.load_bot_prefab_from_file(str(MODEL_BOT))
        prefab.world_from_root = sdp.TransformRT(rotation=UPRIGHT_WRIST_POSE[3:], translation=UPRIGHT_WRIST_POSE[:3])
        prefab.joints[0].type = sdp.ArticulatedJointType.HARD
        for link in prefab.links:
            link.layer = "RightHand"
            link.boundary_element_type = sdp.ActorBoundaryElementType.P1Q1
            link.boundary_subsampling = sdp.BoundarySubsamplingParams(
                strategy=sdp.BoundarySubsamplingStrategy.AREA_PROPORTIONAL, subsampling_density=.10)
        scene.enable_layer_contact_symmetric("RightHand", "RightHand", False)
        with tempfile.TemporaryDirectory() as tmp:
            attach_compound_visuals(prefab, MODEL_DIR, tmp, sdp, full_collision=True, scene=scene)
            bot = sdr.create_bot(scene, prefab, sdr.create_context())
        self.addCleanup(sdr.destroy_bot, scene, bot)
        hand = bot.get_articulated_actor()
        info = hand.get_articulated_shape_info()
        mapping = GloveMapping.load(MAPPING)
        kin = NativeGloveKinematics(MODEL_DIR, mapping)
        projection = NativeGloveKinematics(MODEL_DIR, mapping)
        offsets = {name: dof.offset for name, dof in zip(info.joint_names, info.dof_info)}
        tree_indices = [offsets[name] for name in kin.joint_names]

        settings = grasp_settings(DEFAULT_GRASP, {"kind": "official_block_red", "size_mm": 76., "hardness": 1.})
        env = LocalGrasp(sdp, scene, settings, hand=hand)
        drive = HandContactDrive(sdp, hand, info, mapping.model_joint_names, fixed_wrist=True)
        drive.set_free_grip(True)
        links, parts = zip(*ShellContacts(sdp, scene, hand, env.contact_actors).hand)
        physics = load_native().GraspPhysics(scene, hand, parts, tree_indices,
            [offsets[n] for n in mapping.model_joint_names], links.index("base_link") if "base_link" in links else -1)
        target = np.zeros(hand.get_num_dofs())
        def advance(grip):
            target[tree_indices] = kin.full_pose(grip * np.asarray(DemoGlove.GRIP), None, mapping)
            env.step(1/240)
            physics.step(target, 1/240)

        for grip in np.linspace(0., .85, 40):
            advance(grip)
        for _ in range(20):
            advance(.85)

        surface = open_hand_surface(sdp, scene, hand, include_target=True)
        self.assertTrue(env.configure(grasp_settings(settings, {"size_mm": 123.})))
        center = Rotation.from_quat(UPRIGHT_WRIST_POSE[3:]).inv().apply(env.prefab.center)
        np.testing.assert_allclose(center[:2], [.072, -.005], atol=1e-6)
        clearance = np.max(np.abs(surface - center), axis=1) - .123 / 2
        self.assertGreater(clearance.min(), .0015)
        self.assertTrue(env.awaiting_hand_clearance)
        self.assertFalse(env.ready_to_return(False, 1.))
        for _ in range(24):
            advance(.85)
            usable, angles, _, tree_torque = physics.effort()
            self.assertTrue(usable)
            self.assertLess(np.max(np.abs(projection.project_tree(tree_torque, angles))), .03)

        for grip in [*np.linspace(.85, 0., 40), *([0.] * 60)]:
            advance(grip)
        self.assertFalse(env.ready_to_return(True, 2.))  # 仍有接触时不得归位
        self.assertTrue(env.ready_to_return(False, 3.))
        env.configure(env.settings, reset=True)
        self.assertFalse(env.awaiting_hand_clearance)
        for grip in np.linspace(0., .85, 40):
            advance(grip)
        contact = []
        for _ in range(12):
            advance(.85)
            usable, angles, _, tree_torque = physics.effort()
            self.assertTrue(usable)
            contact.append(np.max(np.abs(projection.project_tree(tree_torque, angles))))
        self.assertGreater(np.median(contact), .03)

    def test_delayed_home_rejects_intervening_user_change(self):
        import multiprocessing as mp
        from ve33_web_control import WebControl
        control = WebControl(mp.get_context("spawn"), None, local_grasp=True)
        checked = control.grasp_request()
        control.command({"action": "configure_grasp", "values": {"angle_deg": 15.}})
        self.assertFalse(control.return_grasp_when_clear(checked))
        checked = control.grasp_request()
        self.assertTrue(control.return_grasp_when_clear(checked))
        self.assertEqual(control.grasp_request()["contact_revision"], checked["contact_revision"] + 1)
        self.assertEqual(control.grasp_request()["angle_deg"], 15.)


if __name__ == "__main__":
    unittest.main()
