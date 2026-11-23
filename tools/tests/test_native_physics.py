"""用无串口采样器验证原生子步帧、拖动、转动及静态/软体约束。"""
import sys
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motorhand.native_control import make_feedback_control
from motorhand.native_kinematics import NativeGloveKinematics
from motorhand.native_runtime import load_native
from ve33_g4_view import MODEL_DIR, MAPPING, GloveMapping, HandContactDrive
from ve33_local_grasp import DEFAULT_GRASP, DemoGlove, LocalGrasp, grasp_settings


class NativePhysicsTest(unittest.TestCase):
    def test_frame_uses_driving_input_and_motion_releases_constraints(self):
        import superdex.physics as sdp
        import superdex.robotics as sdr
        native = load_native()
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        names = [name for name, _ in axes.values()]
        limits = np.array([m.model_rad_limits() for m in mapping.motors])
        ctl = make_feedback_control(axes, native, 'nominal-plus')
        kin = NativeGloveKinematics(MODEL_DIR, mapping)
        producer_kin = NativeGloveKinematics(MODEL_DIR, mapping)

        with tempfile.TemporaryDirectory() as tmp:
            link = native.SharedLink(str(Path(tmp) / 'mailbox'), len(kin.joint_names))
            loop = native.NativeLoop(producer_kin.native, ctl, link, '', [m.sign*m.scale for m in mapping.motors],
                                     limits[:, 0], limits[:, 1], DemoGlove.GRIP, 10_000_000)
            link.command(False, 7, .25)
            loop.start([0.] * 12)
            self.addCleanup(loop.close)
            sdp.initialize(num_worker_threads=0)
            self.addCleanup(sdp.shutdown)
            scene = sdp.create_scene("抓握原生子步")
            scene.set_gravity([0., 0., 0.])
            prefab = sdr.load_bot_prefab_from_file(str(MODEL_DIR / "hand_motor_ve33_right_closed_chain.superdex_bot"))
            prefab.joints[0].type = sdp.ArticulatedJointType.HARD
            prefab.world_from_root = sdp.TransformRT(translation=[2., 0., 0.])
            bot = sdr.create_bot(scene, prefab, sdr.create_context())
            self.addCleanup(sdr.destroy_bot, scene, bot)
            hand = bot.get_articulated_actor()
            info = hand.get_articulated_shape_info()
            offsets = {n: dof.offset for n, dof in zip(info.joint_names, info.dof_info)}
            tree = [offsets[n] for n in kin.joint_names]
            drive = HandContactDrive(sdp, hand, info, names, fixed_wrist=True)
            drive.set_free_grip(True)
            physics = native.GraspPhysics(scene, hand, [], tree, [offsets[n] for n in names], -1)
            self.addCleanup(physics.clear_motion)
            settings = {**DEFAULT_GRASP, 'kind':'box', 'hardness':1., 'layout':'desk'}
            obj = LocalGrasp(sdp, scene, settings)
            rigid = obj.actor
            origin = np.asarray(rigid.get_root_transform().translation).copy()
            target = origin + [.015, -.02, .02]
            constraints = scene.get_num_constraints()
            physics.set_motion([rigid], [], [rigid], [], -1, origin, [0.,0.,1.], 0.)
            physics.motion_command(True, 30., target)
            dt = 1/240
            for _ in range(160):
                published, stamp = physics.step_contact(link, kin.native, dt, 0, 1, True, 7)
            self.assertTrue(published)
            snapshot = physics.last_target()
            self.assertEqual(snapshot[0], stamp)
            time.sleep(.035)
            samples = loop.samples(0, False)
            observed = next(s['force'] for s in samples if s['force'][0] == stamp)
            self.assertEqual(observed[2], 1)
            self.assertEqual(observed[-1], 7)
            self.assertGreaterEqual(observed[1], stamp)
            np.testing.assert_array_equal(observed[39:51], [snapshot[1+kin.joint_names.index(n)] for n in names])
            np.testing.assert_array_equal(observed[51:63], snapshot[-12:])
            np.testing.assert_allclose(rigid.get_root_transform().translation, target, atol=.002)
            self.assertAlmostEqual(Rotation.from_quat(rigid.get_root_transform().rotation).as_rotvec(degrees=True)[2],
                                   30., delta=2.)
            self.assertEqual(scene.get_num_constraints(), constraints + 2)
            physics.clear_motion()
            self.assertEqual(scene.get_num_constraints(), constraints)

            settings = grasp_settings(settings, {'kind':'official_duck_lamp', 'layout':'desk',
                                                 'size_mm':65., 'hardness':.25})
            obj.configure(settings)
            duck = obj.prefab
            scene.step(dt)  # 先完成一次求解，渲染表面查询才有效。
            original = np.asarray(duck.packet()['vertices']).reshape(-1, 3)
            physics.set_motion([], [], [duck.soft], [duck.soft], duck.node, duck.center, duck.yaw_axis, 0.)
            physics.motion_command(True, 30., [])
            for _ in range(180):
                physics.step_contact(link, kin.native, dt, 0, 1, True, 7)
            points = np.asarray(duck.packet()['vertices']).reshape(-1, 3)
            rot, _ = Rotation.align_vectors(points - points.mean(0), original - original.mean(0))
            self.assertAlmostEqual(rot.as_rotvec(degrees=True)[2], 30., delta=3.)
            physics.motion_command(False, 30., [])
            physics.step_contact(link, kin.native, dt, 0, 1, True, 7)
            self.assertEqual(scene.get_num_constraints(), constraints)
            physics.clear_motion()

            settings = grasp_settings(settings, {'kind':'official_pegboard','layout':'desk','hardness':1.})
            obj.configure(settings)
            board = obj.prefab
            static, dynamic = board.static_roots[0], board.roots[0]
            initial = [actor.get_root_transform() for actor in (static,dynamic)]
            constraints = scene.get_num_constraints()
            physics.set_motion(board.roots, board.static_roots, [a for _,a in board.contacts], [],
                               -1, board.center, board.yaw_axis, 0.)
            physics.motion_command(False, 30., [])
            for _ in range(190):
                physics.step_contact(link, kin.native, dt, 0, 1, True, 7)
            yaw = Rotation.from_euler('z', 30., degrees=True)
            for start,actor in zip(initial,(static,dynamic)):
                np.testing.assert_allclose(actor.get_root_transform().translation,
                    board.center + yaw.apply(np.asarray(start.translation)-board.center), atol=.0015)
            self.assertEqual(scene.get_num_constraints(), constraints)


if __name__ == '__main__':
    unittest.main()
