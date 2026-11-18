"""验证左手与右手共用空间、丢帧后清除，以及力的方向和单位。"""
from pathlib import Path
import sys
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ve33_quest import WristBridge


class XRDiagnosticsTest(unittest.TestCase):
    def test_left_hand_shares_right_wrist_alignment_and_clears_on_new_space(self):
        bridge = WristBridge()
        msg = {"space": "xr", "seq": 1, "pose": [1., 2., 3., 0., 0., 0., 1.], "align": True,
               "left": {"kind": "hand", "points": [[.9, 2.1, 2.8]] * 16}}
        bridge.receive(msg)
        self.assertIsNone(bridge.left_hand())
        bridge.wrist()
        self.assertEqual(bridge.left_hand()["kind"], "hand")
        np.testing.assert_allclose(bridge.left_hand()["points"], [[.2, .1, .1]] * 16)
        bridge.receive({"space": "reset", "seq": 1, "pose": None})
        self.assertIsNone(bridge.left_hand())

    def test_invalid_or_expired_left_probes_are_unavailable(self):
        bridge = WristBridge()
        pose = [0., 0., 0., 0., 0., 0., 1.]
        with self.assertRaises(ValueError):
            bridge.receive({"space": "xr", "seq": 1, "pose": pose,
                            "left": {"kind": "controller", "points": [[float("nan"), 0, 0]]}})
        bridge.receive({"space": "xr", "seq": 1, "pose": pose, "align": True,
                        "left": {"kind": "controller", "points": [[.10, 0, -.20]]}})
        bridge.wrist()
        np.testing.assert_allclose(bridge.left_hand()["points"], [[.2, -.1, 0]])
        self.assertIsNone(bridge.left_hand(time.monotonic() + .5))

    def test_scene_reset_rebases_live_wrist_and_discards_old_left_probes(self):
        bridge = WristBridge()
        identity = [0., 0., 0., 0., 0., 0., 1.]
        moved = [.06, .04, -.08, .0, .0, .0, 1.]
        left = {"kind": "controller", "points": [[.1, .1, -.1]]}
        bridge.receive({"space": "xr", "seq": 1, "pose": identity, "align": True})
        bridge.wrist()
        bridge.receive({"space": "xr", "seq": 2, "pose": moved, "left": left})
        self.assertGreater(np.linalg.norm(bridge.wrist()[0][:3]), .05)
        self.assertIsNotNone(bridge.left_hand())
        bridge.reset_scene()
        np.testing.assert_allclose(bridge.wrist()[0], identity)
        self.assertIsNone(bridge.left_hand())
        bridge.receive({"space": "xr", "seq": 3, "pose": moved, "left": left})
        np.testing.assert_allclose(bridge.wrist()[0], identity)
        self.assertIsNotNone(bridge.left_hand())


    def test_left_controller_proxy_can_push_the_official_cube(self):
        from types import SimpleNamespace
        from scipy.spatial.transform import Rotation
        import superdex.physics as sdp
        import vtk
        from ve33_cube import PuzzleCube, link_poses
        from ve33_left_hand import LeftHandSupport

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("左手支撑测试")
            scene.set_gravity([0, 0, 0])
            folder = Path(__file__).resolve().parents[2] / "assets/prefabs/puzzle_cube_quest"
            cube = PuzzleCube(sdp, scene, folder, SimpleNamespace(scene=vtk.vtkRenderer()))
            left = LeftHandSupport(sdp, scene, cube.contact_actors)
            initial = link_poses(cube.names, cube.transforms)
            for x in np.linspace(.16, .211, 75):
                left.update({"kind": "controller", "points": [[x, -.032, .050]]}, 1/240)
                scene.step(1 / 240)
            cube.update()
            moved = link_poses(cube.names, cube.transforms)
            angle = max((Rotation.from_quat(moved[n][3:]) *
                         Rotation.from_quat(initial[n][3:]).inv()).magnitude()
                        for n in cube.names if n.endswith("_shell_link"))
            self.assertGreater(angle, .01)
            self.assertEqual(left.update(None, 1/240), [None] * 6)
        finally:
            sdp.shutdown()

    def test_cube_switch_keeps_layers_and_can_pin_after_physical_motion(self):
        from types import SimpleNamespace
        from scipy.spatial.transform import Rotation
        import superdex.physics as sdp
        import vtk
        from ve33_cube import PuzzleCube, link_poses
        from ve33_left_hand import LeftHandSupport

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("魔方固定与自由抓取测试")
            scene.set_gravity([0, 0, 0])
            folder = Path(__file__).resolve().parents[2] / "assets/prefabs/puzzle_cube_quest"
            cube = PuzzleCube(sdp, scene, folder, SimpleNamespace(scene=vtk.vtkRenderer()), switchable=True)
            left = LeftHandSupport(sdp, scene, cube.contact_actors)

            def read():
                cube.update()
                return link_poses(cube.names, cube.transforms)

            self.assertEqual(cube.mode, "fixed")
            initial_links = read()
            initial = initial_links["center_link"]
            fixed_constraints = scene.get_num_constraints()
            cube.set_fixed(False)
            self.assertEqual(cube.mode, "free")
            self.assertEqual(scene.get_num_constraints(), fixed_constraints - 2)
            for x in np.linspace(.16, .245, 95):
                left.update({"kind": "controller", "points": [[x, -.032, .050]]}, 1/240)
                scene.step(1 / 240)
            left.update(None, 1/240)
            moved = read()["center_link"]
            self.assertGreater(np.linalg.norm(np.subtract(initial[:3], moved[:3])), .01)
            self.assertGreater((Rotation.from_quat(moved[3:]) * Rotation.from_quat(initial[3:]).inv()).magnitude(), .01)
            cube.set_fixed(True)
            self.assertEqual(cube.mode, "fixed")
            self.assertEqual(scene.get_num_constraints(), fixed_constraints)
            np.testing.assert_allclose(read()["center_link"], moved, atol=1e-5)
            for _ in range(40):
                scene.step(1 / 240)
            np.testing.assert_allclose(read()["center_link"], moved, atol=1e-4)
            cube.set_fixed(False)
            cube.reset()
            self.assertEqual(cube.mode, "fixed")
            reset_links = read()
            for link, pose in initial_links.items():
                np.testing.assert_allclose(reset_links[link], pose, atol=1e-5)
        finally:
            sdp.shutdown()

    def test_free_cube_lands_and_settles_on_table_after_a_bump(self):
        from types import SimpleNamespace
        import superdex.physics as sdp
        import vtk
        from ve33_cube import PuzzleCube, link_poses
        from ve33_tabletop import Tabletop

        sdp.initialize(num_worker_threads=0)
        try:
            scene = sdp.create_scene("魔方桌面着陆测试")
            scene.set_gravity([0., 0., -9.81])
            view = SimpleNamespace(scene=vtk.vtkRenderer())
            folder = Path(__file__).resolve().parents[2] / "assets/prefabs/puzzle_cube_quest"
            cube = PuzzleCube(sdp, scene, folder, view, switchable=True)
            table = Tabletop(sdp, scene, view, puzzle_shells=cube.contact_actors)
            _, table_actor = table.contact_actors[0]
            top = float(table_actor.get_aabb_world().max[2])
            cube.set_fixed(False)
            root = cube.actor.get_articulated_shape_info().dof_info[0]
            pos = root.get_trans_offset()
            raised = list(cube.initial_pose)
            raised[pos + 2] += .09
            cube.actor.set_articulated_pose_from_joints(raised)
            velocities = [0.] * cube.actor.get_num_dofs()
            velocities[pos] = .24
            cube.actor.set_articulated_joint_velocities(velocities)

            def center():
                cube.update()
                return np.asarray(link_poses(cube.names, cube.transforms)["center_link"][:3])

            start = center()
            for _ in range(220):
                scene.step(1/240)
            landed = center()
            self.assertGreater(landed[0] - start[0], .01)
            self.assertGreater(start[2] - landed[2], .06)
            self.assertGreater(landed[2] - top, .024)
            self.assertLess(landed[2] - top, .05)
            self.assertGreater(min(shell.get_aabb_world().min[2] for _, shell in cube.contact_actors), top - .004)
            for _ in range(96):
                scene.step(1/240)
            self.assertLess(np.linalg.norm(center() - landed), .004)
        finally:
            sdp.shutdown()


if __name__ == "__main__":
    unittest.main()
