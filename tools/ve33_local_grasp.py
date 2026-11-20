"""本地抓握物体和物理线程内的摆放、固定与拖动。"""

from functools import lru_cache
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from motorhand.g4_types import G4Joint
from ve33_grasp_assets import GRASP_ASSETS, GraspPrefab


GRASP_KINDS = {f"{shape}_{kind}": hardness
               for shape in ("ball", "box", "cylinder")
               for kind, hardness in (("soft", .25), ("hard", .92))}
DEFAULT_GRASP = {"kind": "ball_soft", "size_mm": 45., "hardness": .25,
                 "layout": "palm", "locked": True, "angle_deg": 0., "drag": None}
# 沿用参考仿真朝向，手的 +X 指向世界 +Z，掌面 -Z 指向世界 -Y。
UPRIGHT_WRIST_POSE = (0., 0., 0., -.5, -.5, -.5, .5)
DESK_SIZE = (.46, .42, .025)
DESK_CENTER = (.15, 0., -.155)
SPHERE_MESH = Path(__file__).resolve().parents[1] / "superdex_physics/assets/sphere/icosphere_3subdiv.1.mochi.json"


def grasp_settings(current, changes):
    if not isinstance(changes, dict) or changes.keys() - DEFAULT_GRASP.keys():
        raise ValueError("无效的抓握设置")
    result = {**current, **changes}
    kind = result["kind"]
    if not isinstance(kind, str) or (kind not in GRASP_KINDS and kind not in GRASP_ASSETS):
        raise ValueError("未知抓握物体")
    asset = GRASP_ASSETS.get(kind)
    if kind != current["kind"]:
        if asset is not None and "locked" not in changes:
            result["locked"] = False
        if asset is None and "hardness" not in changes:
            result["hardness"] = GRASP_KINDS[kind]
    if result["layout"] not in ("palm", "desk") or type(result["locked"]) is not bool:
        raise ValueError("无效的场景或锁定状态")
    if asset is not None and asset.desk_only:
        result["layout"] = "desk"
    for key, low, high in (("size_mm", 20., 90.), ("hardness", 0., 1.), ("angle_deg", -180., 180.)):
        value = result[key]
        if type(value) not in (int, float) or not low <= value <= high:
            raise ValueError(f"{key} 超出范围")
        result[key] = float(value)
    if (result["layout"] != current["layout"] or kind != current["kind"]) and "drag" not in changes:
        result["drag"] = None
    drag = result["drag"]
    if drag is not None and (result["layout"] != "desk" or not isinstance(drag, (list, tuple)) or len(drag) != 3
                             or any(type(x) not in (int, float) or not -.15 <= x <= .45 for x in drag)):
        raise ValueError("拖动位置超出桌面工作区")
    return result


def grasp_needs_rebuild(old, settings):
    keys = ("kind", "layout", "angle_deg") if settings["kind"] in GRASP_ASSETS else (
        "kind", "size_mm", "hardness", "layout")
    return old is None or any(old[key] != settings[key] for key in keys)


@lru_cache(maxsize=1)
def sphere_mesh(sdp):
    mesh = sdp.model.load_from_file(str(SPHERE_MESH)).mesh
    points = np.array(mesh.coordinates).reshape(-1, 3)
    points -= (points.min(axis=0) + points.max(axis=0)) / 2.
    return points / np.ptp(points[:, 0]), np.array(mesh.connectivity)


def open_hand_surface(sdp, scene, hand):
    """获取张手时各碰撞表面的腕部局部坐标。"""
    parts = []
    for handle in hand.get_nested_link_actors():
        part = scene.get_actor(handle)
        if not part.is_query_supported(sdp.QueryType.TOTAL_CONTACT_FORCE):
            continue
        points = np.asarray(part.get_surface_mesh().coordinates).reshape(-1, 3)
        pose = part.get_root_transform()
        parts.append(Rotation.from_quat(list(pose.rotation)).apply(points) + np.asarray(pose.translation))
    root = hand.get_root_transform()
    inverse = Rotation.from_quat(list(root.rotation)).inv()
    return inverse.apply(np.concatenate(parts) - np.asarray(root.translation))


class LocalGrasp:
    def __init__(self, sdp, scene, settings, *, open_surface=None):
        self.sdp = sdp
        self.scene = scene
        self.open_surface = open_surface
        self.wrist_rotation = Rotation.from_quat(UPRIGHT_WRIST_POSE[3:])
        self.actor = None
        self.prefab = None
        self.pins = ()
        self.mesh_revision = 0
        self.settings = None
        self.configure(settings)

    @property
    def contact_actors(self):
        if self.prefab is not None:
            return self.prefab.contacts
        return [("grasp_object", self.actor)]

    def configure(self, settings, *, reset=False):
        old = self.settings
        rebuild = reset or grasp_needs_rebuild(old, settings)
        if rebuild:
            previous = self.actor
            previous_prefab = self.prefab
            asset = GRASP_ASSETS.get(settings["kind"])
            if asset is not None:
                self.prefab = GraspPrefab(self.sdp, self.scene, asset, settings, self.mesh_revision + 1,
                                          self.wrist_rotation, self.open_surface)
                self.actor = None
            else:
                home = self.home(settings)
                if not reset and previous is not None and settings["layout"] == old["layout"] == "desk":
                    bounds = previous.get_aabb_world()
                    home = (np.asarray(bounds.min) + np.asarray(bounds.max)) / 2.
                self.create(settings, home)
                self.anchor = home
                self.angle = settings["angle_deg"]
                self.prefab = None
            self.release_pins()
            if previous is not None:
                self.scene.destroy_actor(previous)
            if previous_prefab is not None:
                previous_prefab.destroy()
            self.mesh_revision += 1
        self.settings = settings.copy()
        return rebuild

    def home(self, settings):
        if settings["layout"] == "desk":
            return np.array([.18, -.06, -.05])
        return self.wrist_rotation.apply(self.palm_center(settings)) + UPRIGHT_WRIST_POSE[:3]

    def palm_center(self, settings):
        radius = settings["size_mm"] / 2000.
        center = np.array([.072, -.005, -.007 - radius])
        if self.open_surface is None:
            return center
        surface = Rotation.from_euler("z", -settings["angle_deg"], degrees=True).apply(self.open_surface - center)
        x, y, z = surface.T
        # 初始落点留出张手间隙，之后由实际接触解算。
        margin = .0015
        extent = radius + margin
        shape = settings["kind"].split("_")[0]
        if shape == "box":
            footprint = (abs(x) <= extent) & (abs(y) <= extent)
            top = extent
        else:
            horizontal2 = x*x + y*y if shape == "ball" else x*x
            footprint = horizontal2 <= extent**2
            if shape == "cylinder":
                footprint &= abs(y) <= radius * 1.4 + margin
            top = np.sqrt(extent**2 - horizontal2[footprint])
        if footprint.any():
            center[2] += min(0., float(np.min(z[footprint] - top)))
        return center

    def object_rotation(self, layout, angle):
        rotation = Rotation.from_euler("z", angle, degrees=True)
        return self.wrist_rotation * rotation if layout == "palm" else rotation

    def create(self, settings, center):
        sdp = self.sdp
        size = settings["size_mm"] / 1000.
        kind, hardness, angle = (settings[k] for k in ("kind", "hardness", "angle_deg"))
        self.soft = kind == "ball_soft"
        self.origin = np.asarray(center)
        self.rotation = self.object_rotation(settings["layout"], angle)
        transform = sdp.TransformRT(rotation=self.rotation.as_quat(), translation=center)
        params = sdp.ContactParams(penalty_coefficient=1e7 * 10**(4. * hardness),
                                   penalty_smoothing_half_distance=.001,
                                   normal_viscous_damping_coefficient=.5, coulomb_friction_coefficient=.8)
        if self.soft:
            vertices, tets = sphere_mesh(sdp)
            material = sdp.SoftMaterialParams(type=sdp.SoftMaterialType.NEO_HOOKEAN,
                neo_hookean=sdp.NeoHookeanMaterialParams(youngs_modulus=10000. * 16.**hardness, poisson_ratio=.45),
                density=1000., stiffness_damping_coefficient=.025)
            shape = sdp.create_model_shape(mesh=sdp.MeshData(nodes_per_element=4,
                coordinates=(vertices * size).ravel(), connectivity=tets))
            self.actor = self.scene.create_soft_actor(name=kind, layer="GraspSoft", shape=shape,
                world_from_local=transform, material=material, contact=params, has_gravity=False,
                boundary_element_type=sdp.ActorBoundaryElementType.P1Q1)
            self.actor.register_query(sdp.QueryType.NODE_POSITIONS)
            self.actor.register_query(sdp.QueryType.SURFACE_NODE_POSITIONS)
            self.node = int(np.argmin(np.linalg.norm(vertices, axis=1)))
            self.node_rest = self.rotation.apply(vertices[self.node] * size)
            self.indices = list(self.actor.get_surface_mesh().connectivity)
            self.has_frame = False
            return
        if kind.startswith("ball"):
            mesh, collider = trimesh.creation.icosphere(subdivisions=2, radius=size / 2.), sdp.ColliderType.SPHERE
        else:
            if kind.startswith("box"):
                mesh, collider = trimesh.creation.box(extents=[size] * 3), sdp.ColliderType.BOX
            else:
                mesh, collider = trimesh.creation.cylinder(radius=size / 2., height=size * 1.4, sections=32), sdp.ColliderType.MESH
                mesh.vertices = Rotation.from_euler("x", 90, degrees=True).apply(mesh.vertices)
        shape = sdp.create_model_shape(mesh=sdp.MeshData(nodes_per_element=3,
            coordinates=mesh.vertices.ravel(), connectivity=mesh.faces.ravel()))
        self.actor = self.scene.create_rigid_actor(name=kind, layer="GraspRigid", shape=shape,
            world_from_local=transform, collider_type=collider, contact=params, has_gravity=False)

    def release_pins(self):
        for pin in self.pins:
            self.scene.destroy_constraint(pin)
        self.pins = ()

    def pin_position(self):
        if not self.soft:
            return np.asarray(self.actor.get_root_transform().translation)
        if not self.has_frame:
            return self.origin + self.node_rest
        points = np.asarray(self.actor.get_node_positions_local()).reshape(-1, 3)
        if len(points) <= self.node:
            return self.origin + self.node_rest
        return self.origin + self.rotation.apply(points[self.node])

    def step(self, dt):
        if self.prefab is not None:
            self.prefab.step(self.settings, dt)
            return
        dragging = self.settings["drag"]
        target_angle = self.settings["angle_deg"]
        rotate = not self.soft and abs(target_angle - self.angle) > .01
        if not (self.settings["locked"] or dragging is not None or rotate):
            self.release_pins()
            return
        if not self.pins:
            handle = self.actor.get_handle()
            self.anchor = self.pin_position()
            if self.soft:
                pull = self.scene.create_deformable_node_position_constraint(actor=handle,
                    node_index=self.node, position=self.anchor, stiffness=4000., damping=1.)
                self.pins = (pull,)
            else:
                pose = self.actor.get_root_transform()
                current = Rotation.from_quat(list(pose.rotation))
                pull = self.scene.create_rigid_pivot_position_constraint(actor=handle, local_position=[0., 0., 0.],
                    target_position=self.anchor, stiffness=4000., damping=12.)
                twist = self.scene.create_rigid_pivot_rotation_constraint(actor=handle, local_rotation=[0., 0., 0.],
                    target_rotation=current.as_rotvec(), stiffness=1., damping=.03)
                self.pins = (pull, twist)
        if dragging is not None:
            desired = np.asarray(dragging) + (self.node_rest if self.soft else 0.)
            delta = desired - self.anchor
            self.anchor += delta * min(1., .35 * dt / max(np.linalg.norm(delta), 1e-9))
        self.pins[0].set_target_position(self.anchor)
        if not self.soft and rotate:
            self.angle += float(np.clip(target_angle - self.angle, -150. * dt, 150. * dt))
            self.pins[1].set_target_rotation(self.object_rotation(self.settings["layout"], self.angle).as_quat())

    def packet(self):
        payload = {**self.settings, "mesh_revision": self.mesh_revision}
        if self.prefab is not None:
            return {**payload, **self.prefab.packet()}
        if self.soft:
            local = np.asarray(self.actor.get_surface_mesh_node_positions_local()).reshape(-1, 3)
            self.has_frame = True
            if not len(local):
                local = np.asarray(self.actor.get_surface_mesh().coordinates).reshape(-1, 3)
            world = self.rotation.apply(local) + self.origin
            payload.update(vertices=world.round(6).ravel().tolist(), indices=self.indices,
                           pose=[*world.mean(axis=0), 0., 0., 0., 1.])
        else:
            pose = self.actor.get_root_transform()
            payload["pose"] = [*map(float, pose.translation), *map(float, pose.rotation)]
        return payload


class DemoGlove:
    """本地演示编码器，无法向串口发送任何指令。"""

    GRIP = (.20, .15, .60, .60, .80, 1.20, .80, 1.20, .80, 1.20, .80, 1.20)

    def __init__(self, axes, manual):
        self.axes = axes
        self.manual = manual
        self.start = time.monotonic()

    def __call__(self):
        fraction = self.manual.value
        if fraction < 0:
            fraction = .5 - .5 * math.cos((time.monotonic() - self.start) * math.tau / 7.)
        stamp = time.monotonic_ns()
        return {mid: G4Joint(id=mid, bus=3 - (mid - 1) // 4, valid=1, received_ns=stamp,
                            angle_deg=math.degrees(self.GRIP[mid - 1] * fraction) / (m.sign * m.scale))
                for mid, (_, m) in self.axes.items()}

    def release(self):
        pass  # 演示中没有硬件令牌或输出

    def send(self, batch, deadline):
        raise RuntimeError("演示模式没有真机输出")
