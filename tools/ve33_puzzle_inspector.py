"""在官方 Physics Debugger 中镜像固定腕部测试的手部和魔方位姿。"""

import argparse
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time
from types import SimpleNamespace
from urllib.request import urlopen

import superdex.physics as sdp
import superdex.robotics as sdr
import vtk

from ve33_closed_chain_view import attach_compound_visuals
from ve33_cube import PuzzleCube
from ve33_g4_view import MODEL_DIR


def poses(actor, state):
    order = actor.get_articulated_shape_info().link_names
    return [sdp.TransformRT(sdp.Quaternion(*state[name][3:]), sdp.Real3(*state[name][:3]))
            for name in order]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--cube-dir", type=Path, required=True)
    args = parser.parse_args()
    endpoint = f"http://127.0.0.1:{args.port}/api/v1/scene"
    deadline = time.monotonic() + 25
    while True:
        try:
            with urlopen(endpoint, timeout=2) as response:
                sample = json.load(response)
            if sample["scene_kind"] == "official_puzzle":
                break
        except OSError:
            pass
        if time.monotonic() >= deadline:
            raise RuntimeError("没有找到实机魔方会话")
        time.sleep(.15)

    sdp.initialize(num_worker_threads=0)
    bot = None
    try:
        scene = sdp.create_scene("Puzzle Cube · 实机姿态只读镜像")
        scene.set_gravity((0, 0, 0))
        context = sdr.create_context()
        from ve33_g4_view import MODEL_BOT
        prefab = sdr.load_bot_prefab_from_file(str(MODEL_BOT))
        prefab.joints[0].type = sdp.ArticulatedJointType.HARD
        for link in prefab.links:
            link.layer = "RightHand"
        scene.enable_layer_contact_symmetric("RightHand", "RightHand", False)
        scene.enable_layer_contact_asymmetric("puzzle_cube_shell", "RightHand", False)
        with TemporaryDirectory(prefix="ve33-inspector-") as folder:
            attach_compound_visuals(prefab, MODEL_DIR, folder, sdp)
            bot = sdr.create_bot(scene, prefab, context)
        hand = bot.get_articulated_actor()
        cube = PuzzleCube(sdp, scene, args.cube_dir.resolve(),
                          SimpleNamespace(scene=vtk.vtkRenderer()), official_grip=True)
        hand.set_articulated_pose_from_links(poses(hand, sample["right_hand"]))
        cube.actor.set_articulated_pose_from_links(poses(cube.actor, sample["puzzle_cube"]))
        if not sdp.debugger.attach():
            raise RuntimeError("官方 Physics Debugger 未连接")
        while sdp.debugger.is_attached():
            try:
                with urlopen(endpoint, timeout=.4) as response:
                    sample = json.load(response)
            except OSError:
                break
            if sample["scene_kind"] != "official_puzzle":
                break
            hand.set_articulated_pose_from_links(poses(hand, sample["right_hand"]))
            cube.actor.set_articulated_pose_from_links(poses(cube.actor, sample["puzzle_cube"]))
            scene.step(1e-6)
            time.sleep(.05)
    finally:
        if bot is not None:
            sdr.destroy_bot(scene, bot)
        sdp.shutdown()


if __name__ == "__main__":
    main()
