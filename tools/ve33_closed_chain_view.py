#!/usr/bin/env python3
"""Interactive 12-input VE33 kinematic display in SuperDex Physics Debugger.

Run with Python 3.12 (matching Motorhand_FFB's native geometry extension), for example:
  /tmp/superdex-studio-env/bin/python tools/ve33_closed_chain_view.py --hand right
  /tmp/superdex-studio-env/bin/python tools/ve33_closed_chain_view.py --hand left --mode joint

Requires superdex-physics, superdex-robotics, superdex-physics-debugger,
NumPy, SciPy, PyYAML, tkinter and the Motorhand_FFB source/native extension.
Set --motorhand-src if the checkout is elsewhere. This is a kinematic display,
not a simulation of motor forces or physical hardware limits. The debugger is
not Studio; Studio's standalone binary has no exposed Python control channel.
"""

import argparse
import importlib
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import types
import tkinter as tk
from tkinter import ttk
from xml.etree import ElementTree

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
FINGER_SLEEVE_LINKS = frozenset(
    f"{finger}_{joint}_link"
    for finger in ("index", "middle", "ring", "pinky")
    for joint in ("mcp", "pip", "dip")
) | {"thumb_mcp_link", "thumb_ip_link"}


def preserve_link_inertia(sdp, scene, link, shape_path, collider_type):
    """以填实前的碰撞几何确定惯量，封住空隙只影响接触。"""
    if link.center_of_mass is not None and link.moment_of_inertia is not None:
        return
    shape = sdp.load_shape_from_file(str(shape_path), link.shape_scale,
                                     sdp.TransformRT(link.shape_rotation, link.shape_translation))
    body = scene.create_rigid_actor(shape=shape, collider_type=collider_type,
                                    density=link.density, mass=link.mass)
    try:
        link.mass = body.get_mass()
        link.center_of_mass = body.get_rigid_center_of_mass_local()
        link.moment_of_inertia = body.get_rigid_moment_of_inertia_local()
        link.density = None
    finally:
        scene.destroy_actor(body)


def attach_compound_visuals(prefab, model_dir, temporary_dir, sdp, *, full_collision=False, scene=None):
    """组合多零件可视面，为仿真生成完整的碰撞面并填实指环。"""
    links = {link.name: link for link in prefab.links}
    for node in ElementTree.parse(model_dir / "hand_tree_only.urdf").findall("link"):
        visuals = node.findall("visual")
        fill_sleeve = full_collision and node.attrib["name"] in FINGER_SLEEVE_LINKS
        if len(visuals) < 2 and not fill_sleeve:
            continue
        link = links[node.attrib["name"]]
        shape_rotation = Rotation.from_quat(np.asarray(link.shape_rotation))
        vertices = []
        triangles = []
        offset = 0
        collision_parts = []
        original_parts = []
        for visual in visuals:
            mesh = visual.find("geometry/mesh")
            source = sdp.model.load_from_file(str(model_dir / mesh.attrib["filename"]))
            xyz = np.asarray(source.mesh.coordinates).reshape(-1, 3)
            xyz = xyz * np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
            origin = visual.find("origin")
            if origin is not None:
                rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                xyz = Rotation.from_euler("xyz", rpy).apply(xyz)
                xyz += np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            # 载入 shape 时引擎会统一应用其变换，先把可视顶点转回 shape 局部系。
            xyz = shape_rotation.inv().apply(xyz - np.asarray(link.shape_translation))
            vertices.append(xyz / np.asarray(link.shape_scale))
            triangles.append(np.asarray(source.mesh.connectivity) + offset)
            if full_collision:
                import trimesh
                part = trimesh.Trimesh(vertices=vertices[-1],
                                       faces=np.asarray(source.mesh.connectivity).reshape(-1, 3),
                                       process=True)
                # 每个 CAD 零件独立封闭，填满指环的通孔，不跨零件或关节搭桥。
                if not part.is_volume:
                    part = part.convex_hull
                if fill_sleeve:
                    original_parts.append(part)
                collision_parts.append(part.convex_hull if fill_sleeve else part)
            offset += len(xyz)

        model = sdp.model.load_from_file(link.shape_file)
        model.visual_mesh = sdp.MeshData(
            nodes_per_element=3,
            coordinates=np.concatenate(vertices).ravel(),
            connectivity=np.concatenate(triangles),
        )
        if full_collision:
            if fill_sleeve and scene is not None:
                original_path, original_type = link.shape_file, link.collider_type
                if len(visuals) > 1:
                    original = trimesh.util.concatenate(original_parts)
                    model.mesh = sdp.MeshData(nodes_per_element=3, coordinates=original.vertices.ravel(),
                                              connectivity=original.faces.ravel())
                    model.sdf = None
                    original_path = Path(temporary_dir) / f"{link.name}_original.mochi.h5"
                    original_type = sdp.ColliderType.MESH
                    sdp.model.save_to_file(model, str(original_path), sdp.FileFormat.H5)
                preserve_link_inertia(sdp, scene, link, original_path, original_type)
            # 原始碰撞文件只含第一个零件；这里使用同一局部系的完整三角面。
            collision = trimesh.util.concatenate(collision_parts)
            model.mesh = sdp.MeshData(nodes_per_element=3, coordinates=collision.vertices.ravel(),
                                      connectivity=collision.faces.ravel())
            model.sdf = None
            link.collider_type = sdp.ColliderType.MESH
            if fill_sleeve:
                link.boundary_subsampling = sdp.BoundarySubsamplingParams(
                    strategy=sdp.BoundarySubsamplingStrategy.AREA_PROPORTIONAL,
                    subsampling_density=.16,
                )
        shape_path = Path(temporary_dir) / f"{link.name}.mochi.h5"
        sdp.model.save_to_file(model, str(shape_path), sdp.FileFormat.H5)
        link.shape_file = str(shape_path)


def load_kinematics(src):
    # Import just the upstream solver and its relative geometry module; the
    # package's __init__ imports unrelated hardware drivers.
    package_dir = src.resolve() / "motorhand_teleop"
    if not (package_dir / "ve33_kinematics.py").is_file():
        raise FileNotFoundError(f"Motorhand_FFB solver not found in {package_dir}")
    package = types.ModuleType("motorhand_teleop")
    package.__path__ = [str(package_dir)]
    package.__package__ = "motorhand_teleop"
    sys.modules[package.__name__] = package
    return importlib.import_module("motorhand_teleop.ve33_kinematics").Ve33Kinematics


class ControlWindow:
    def __init__(self, root, solver, actor, scene, sdp, mode):
        self.root, self.solver, self.actor, self.scene, self.sdp = root, solver, actor, scene, sdp
        self.mode = tk.StringVar(value=mode)
        self.status = tk.StringVar()
        self.values = []
        info = actor.get_articulated_shape_info()
        nlinks = len(info.link_names)
        indices = {}
        for name, dof in zip(list(info.joint_names)[:nlinks], list(info.dof_info)[:nlinks]):
            if name in solver.tree_names:
                if name in indices or dof.trans_size != 0 or dof.rot_size != 1:
                    raise ValueError(f"Unexpected SuperDex joint layout for {name}")
                indices[name] = dof.offset
        if set(indices) != set(solver.tree_names):
            raise ValueError(f"SuperDex joint names do not match solver: missing {set(solver.tree_names) - set(indices)}")
        self.indices = indices
        self.pose = sdp.DynamicArrayReal(actor.get_num_dofs())
        actor.get_articulated_pose(self.pose)
        root.title(f"VE33 {solver.hand} — 12-input closed-chain control")
        ttk.Label(root, text="12 个输入，单位 rad；点击应用。拒绝的目标保留上一有效姿态。").grid(row=0, column=0, columnspan=3, sticky="w")
        modes = ttk.Frame(root)
        modes.grid(row=1, column=0, columnspan=3, sticky="w")
        ttk.Radiobutton(modes, text="物理电机相对角", variable=self.mode, value="motor", command=self.rebuild).pack(side="left")
        ttk.Radiobutton(modes, text="真实相邻指节角", variable=self.mode, value="joint", command=self.rebuild).pack(side="left")
        self.entries = ttk.Frame(root)
        self.entries.grid(row=2, column=0, columnspan=3, sticky="nsew")
        ttk.Button(root, text="应用 12 个输入", command=self.apply).grid(row=3, column=0, sticky="ew")
        ttk.Button(root, text="重置输入到当前姿态", command=self.rebuild).grid(row=3, column=1, sticky="ew")
        ttk.Label(root, textvariable=self.status, wraplength=620).grid(row=4, column=0, columnspan=3, sticky="w")
        self.rebuild()
        self.display(solver.current)
        self.tick()

    def rebuild(self):
        for child in self.entries.winfo_children():
            child.destroy()
        self.values.clear()
        pose = self.solver.current
        positions = pose.motor_positions if self.mode.get() == "motor" else pose.joint_positions
        names = self.solver.motor_names if self.mode.get() == "motor" else self.solver.joint_names
        for row, name in enumerate(names):
            ttk.Label(self.entries, text=name).grid(row=row, column=0, sticky="w")
            value = tk.StringVar(value=f"{positions[name]:.12g}")
            ttk.Entry(self.entries, textvariable=value, width=20).grid(row=row, column=1, sticky="ew")
            self.values.append(value)
        self.status.set(f"当前闭环最大间隙 {pose.max_gap_m:.3g} m；模式切换不会改变姿态。")

    def display(self, result):
        for name, index in self.indices.items():
            self.pose[index] = result.all_positions[name]
        self.actor.set_articulated_pose_from_joints(self.pose)

    def apply(self):
        names = self.solver.motor_names if self.mode.get() == "motor" else self.solver.joint_names
        try:
            numbers = [float(v.get()) for v in self.values]
            if not all(math.isfinite(v) for v in numbers):
                raise ValueError("所有输入必须是有限实数")
            targets = dict(zip(names, numbers))
            # Evaluate first, and commit only after the complete pose has been
            # successfully handed to the display.
            result = (self.solver.evaluate_motors(targets) if self.mode.get() == "motor"
                      else self.solver.evaluate(targets))
            self.display(result)
            self.solver.current = result
            self.status.set(f"已应用；闭环最大间隙 {result.max_gap_m:.3g} m")
        except ValueError as error:
            self.status.set(f"未应用，保持上一有效姿态：{error}")

    def tick(self):
        if not self.sdp.debugger.is_attached():
            self.root.destroy()
            return
        # Publish a kinematic pose, not an independently simulated set of
        # 68 actuators.
        # Periodic debugger sync is emitted only after a simulation step. Restore
        # the solved pose and zero velocities on every tick; a tiny step is used
        # only to publish it, not to simulate the hand's dynamics.
        self.display(self.solver.current)
        self.actor.set_articulated_joint_velocities(self.sdp.DynamicArrayReal(self.actor.get_num_dofs()))
        self.scene.step(1e-6)
        self.root.after(30, self.tick)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--mode", choices=("motor", "joint"), default="motor")
    parser.add_argument("--motorhand-src", type=Path, default=Path("/home/boen/hand/Motorhand_FFB/src"))
    args = parser.parse_args()
    import superdex.physics as sdp
    import superdex.robotics as sdr
    side = "L" if args.hand == "left" else "R"
    model_dir = ROOT / "assets" / "hand_motor_ve33" / f"hand_motor_ve33_{side}"
    bot_path = model_dir / f"hand_motor_ve33_{args.hand}_closed_chain.superdex_bot"
    solver = load_kinematics(args.motorhand_src)(args.hand, model_dir=model_dir)
    sdp.initialize(num_worker_threads=0)
    bot = None
    scene = None
    try:
        scene = sdp.create_scene(f"VE33 {args.hand} closed chain")
        scene.set_gravity([0, 0, 0])
        context = sdr.create_context()
        prefab = sdr.load_bot_prefab_from_file(str(bot_path))
        with TemporaryDirectory(prefix="ve33-") as temporary_dir:
            attach_compound_visuals(prefab, model_dir, temporary_dir, sdp)
            bot = sdr.create_bot(scene, prefab, context)
        sdp.get_debug_server().set_coordinate_space(sdp.CoordinateSpace(axes=sdp.CoordinateSpaceAxes.FLU))
        if not sdp.debugger.attach():
            raise RuntimeError("无法连接 SuperDex Physics Debugger；确认 debugger wheel 和图形桌面可用")
        root = tk.Tk()
        ControlWindow(root, solver, bot.get_articulated_actor(), scene, sdp, args.mode)
        root.mainloop()
    finally:
        if bot is not None:
            sdr.destroy_bot(scene, bot)
        sdp.shutdown()


if __name__ == "__main__":
    main()
