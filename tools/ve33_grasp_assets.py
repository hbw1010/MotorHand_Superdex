"""固定腕部抓握场景可加载的 SuperDex 官方物体。"""

from dataclasses import dataclass
from functools import lru_cache
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


ASSET_ROOT = Path(__file__).resolve().parents[1] / "assets/prefabs"


@dataclass(frozen=True)
class GraspAsset:
    label: str
    file: str


GRASP_ASSETS = {
    "official_sphere": GraspAsset("官方刚性球", "sphere/sphere"),
    "official_block_blue": GraspAsset("蓝色小方块", "box_and_blocks/block_blue"),
    "official_block_green": GraspAsset("绿色小方块", "box_and_blocks/block_green"),
    "official_block_red": GraspAsset("红色小方块", "box_and_blocks/block_red"),
    "official_block_yellow": GraspAsset("黄色小方块", "box_and_blocks/block_yellow"),
    "official_paper_cup": GraspAsset("单个纸杯", "paper_cups/paper_cup"),
    "official_duck_lamp": GraspAsset("软体鸭灯", "duck_lamp/duck_lamp_recumbent"),
    "official_peg": GraspAsset("独立插销", "functional_dexterity_test/fdt_peg"),
    "official_chain": GraspAsset("链条", "chain/chain"),
    "official_fidget_box": GraspAsset("Fidget Box", "fidget_box/fidget_box"),
    "official_puzzle_cube": GraspAsset("二阶魔方 · 原版", "puzzle_cube/puzzle_cube"),
    "official_quest_cube": GraspAsset("二阶魔方 · Quest 版", "puzzle_cube_quest/puzzle_cube"),
    "official_box_and_blocks": GraspAsset("分隔盒与方块", "box_and_blocks/box_and_blocks"),
    "official_cup_pyramid": GraspAsset("纸杯金字塔", "paper_cups/paper_cup_pyramid"),
    "official_pegboard": GraspAsset("灵巧度插销板", "functional_dexterity_test/functional_dexterity_test"),
    "official_nine_hole": GraspAsset("九孔插销板", "nine_hole_peg_test/nine_hole_peg_test"),
    "official_shape_box": GraspAsset("形状配对盒", "shape_box/shape_box"),
}


def catalog():
    return [{"kind": kind, "label": asset.label} for kind, asset in GRASP_ASSETS.items()]


def prefab_path(asset):
    return ASSET_ROOT / f"{asset.file}.mochi_prefab"


@lru_cache(maxsize=None)
def render_models(path):
    """按 prefab 生成的 actor 路径关联原版 GLB 和渲染缩放。"""
    source = json.loads(path.read_text())
    models = {}
    for reference in source.get("prefabs", ()):
        child = (path.parent / reference["path"]).resolve()
        for name, (model, scale) in render_models(child).items():
            models[f"{reference['name']}/{name}"] = model, scale * reference.get("scale", 1.)
    actors = source.get("actors", {})
    links = [(actor["name"], actor, 1.) for actor in actors.get("rigid", ())]
    links.extend((f"{actor['name']}/{link['name']}", link, actor.get("scale", 1.))
                 for actor in actors.get("articulated", ()) for link in actor["links"])
    for name, link, scale in links:
        if link.get("renderModel"):
            model = (path.parent / link["renderModel"]).resolve().relative_to(ASSET_ROOT)
            models[name] = model.as_posix(), np.array(link.get("renderModelScale", (1., 1., 1.))) * scale
    return models


@lru_cache(maxsize=1)
def render_model_files():
    return {model: ASSET_ROOT / model for asset in GRASP_ASSETS.values()
            for model, _ in render_models(prefab_path(asset)).values()}


def render_model_path(name):
    return render_model_files().get(name)


def nested_parts(sdp, scene, actor):
    if actor.get_type() == sdp.ActorType.ARTICULATED:
        return [scene.get_actor(handle) for handle in actor.get_nested_link_actors()]
    return [actor]


def contact_stiffness(hardness):
    return 1e7 * 10. ** (4. * hardness)


def instantiate(sdp, scene, path, *, hardness=None, **placement):
    params = sdp.prefab.PrefabParams(apply_scene_settings=False, **placement)
    if path == prefab_path(GRASP_ASSETS["official_duck_lamp"]):
        # 鸭灯只有一个软体。先缩放网格和原点，再加载形状以保留本地接触积分。
        data = json.loads(path.read_text())
        duck = data["actors"]["soft"][0]
        duck["scale"] = (np.array(duck["scale"]) * params.scale).tolist()
        duck["translation"] = (np.array(duck["translation"]) * params.scale).tolist()
        duck["boundaryElementType"] = "P1Q1"
        if hardness == 1.:
            del duck["material"]
            data["actors"] = {"rigid": [duck]}
        elif hardness is not None:
            # 0.25 对应鸭灯原始材料，随后按与基础软体相同的倍率变化。
            duck["material"]["neoHookean"]["youngsModulus"] *= 16. ** (hardness - .25)
        prefab = sdp.prefab.load_from_json_string(json.dumps(data), str(path.parent))
        params.scale = 1.
        return sdp.prefab.add_to_scene(prefab, scene, params)
    return sdp.prefab.add_to_scene(str(path), str(path.parent), scene, params)


@lru_cache(maxsize=None)
def authored_bounds(sdp, path):
    scene = sdp.create_scene("抓握资产尺寸")
    try:
        instance = instantiate(sdp, scene, path)
        bounds = [part.get_aabb_world() for actor in instance.actors
                  for part in nested_parts(sdp, scene, actor) if len(part.get_surface_mesh().coordinates)]
        return (np.min([list(box.min) for box in bounds], axis=0),
                np.max([list(box.max) for box in bounds], axis=0))
    finally:
        sdp.destroy_scene(scene)


def pose_values(pose):
    return [*map(float, pose.translation), *map(float, pose.rotation)]


def yaw_step_degrees(current, target, dt):
    """以有界速度调角，并在目标附近减速，避免解锁时带着转速越过目标。"""
    error = target - current
    if abs(error) <= .0001:
        return error
    return float(np.clip(error * (-np.expm1(-dt / .05)), -150. * dt, 150. * dt))


class SoftRotationPins:
    """转动期间用三个不共线的节点引导方向，目标从当前变形姿态开始。"""

    def __init__(self, scene, actor, nodes_local, center_node):
        self.scene, self.actor = scene, actor
        self.rest = np.asarray(nodes_local)
        self.center_node = center_node
        self.pins = []

    def update(self, step_rotation, pivot_world):
        if step_rotation is None:
            self.release()
            return
        if not self.pins:
            current = np.asarray(self.actor.get_node_positions_local()).reshape(-1, 3)
            nodes = current if len(current) else self.rest
            pose = self.actor.get_root_transform()
            rotation = Rotation.from_quat(list(pose.rotation))
            offsets = rotation.apply(nodes - nodes[self.center_node])
            axis = step_rotation.as_rotvec()
            axis /= np.linalg.norm(axis)
            radial = offsets - np.outer(offsets @ axis, axis)
            first = np.argmax(np.linalg.norm(radial, axis=1))
            second = np.argmax(np.linalg.norm(np.cross(radial[first], radial), axis=1))
            for index in (first, second):
                pin = self.scene.create_deformable_node_position_constraint(
                    actor=self.actor.get_handle(), node_index=int(index),
                    position=pivot_world + offsets[index], stiffness=4000., damping=8.)
                self.pins.append((pin, offsets[index].copy()))
        for pin, offset in self.pins:
            offset[:] = step_rotation.apply(offset)
            pin.set_target_position(pivot_world + offset)

    def release(self):
        for pin, _ in self.pins:
            self.scene.destroy_constraint(pin)
        self.pins.clear()


class GraspPrefab:
    def __init__(self, sdp, scene, asset, settings, revision, wrist_rotation, open_surface):
        self.sdp, self.scene = sdp, scene
        path = prefab_path(asset)
        low, high = authored_bounds(sdp, path)
        span = high - low
        # size_mm 为初始姿态下整组碰撞包围盒的最长边。
        scale = settings["size_mm"] / (1000. * np.max(span))
        half_span = span * scale / 2.
        yaw = Rotation.from_euler("z", settings["angle_deg"], degrees=True)
        if settings["layout"] == "palm":
            palm_center = np.array([.072, -.005, -.007 - half_span[2]])
            if open_surface is not None:
                x, y, z = yaw.inv().apply(open_surface - palm_center).T
                footprint = (abs(x) <= half_span[0] + .002) & (abs(y) <= half_span[1] + .002)
                if footprint.any():
                    palm_center[2] += min(0., float(z[footprint].min() - half_span[2] - .002))
            rotation = wrist_rotation * yaw
            self.center = wrist_rotation.apply(palm_center)
        else:
            extent = abs(yaw.as_matrix()) @ half_span
            rotation = yaw
            self.center = np.array([.11, -.025 - extent[1], .045])
            if open_surface is not None:
                x, y, z = (wrist_rotation.apply(open_surface) - self.center).T
                footprint = (abs(x) <= extent[0] + .008) & (abs(z) <= extent[2] + .008)
                if footprint.any():
                    self.center[1] += min(0., float(y[footprint].min() - extent[1] - .008))
        translation = self.center - rotation.apply((low + high) * (scale / 2.))
        prefix = f"grasp{revision}"
        hardness = settings.get("hardness", .25)
        self.instance = instantiate(sdp, scene, path, hardness=hardness, name=prefix, scale=scale,
                                    rotation=rotation.as_quat(), translation=translation)
        self.parts = []
        self.contacts = []
        self.roots = []
        self.position_pins = []
        self.rotation_pins = []
        self.static_roots = []
        self.angle = settings["angle_deg"]
        self.was_free = False
        self.yaw_axis = rotation.apply([0., 0., 1.])
        self.soft = None
        self.surface_actor = None
        self.soft_rotation = None
        models = render_models(path)
        for actor in self.instance.actors:
            leaves = nested_parts(sdp, scene, actor)
            if asset == GRASP_ASSETS["official_duck_lamp"]:
                self.surface_actor = actor
                surface = actor.get_surface_mesh()
                self.rest_surface = np.asarray(surface.coordinates).copy().reshape(-1, 3)
                self.indices = list(surface.connectivity)
                self.contacts.append(("grasp_object", actor))
                if actor.get_type() == sdp.ActorType.SOFT:
                    self.soft = actor
                    actor.register_query_and_compute(sdp.QueryType.NODE_POSITIONS)
                    actor.register_query(sdp.QueryType.SURFACE_NODE_POSITIONS)
                    nodes = np.asarray(actor.get_mesh().coordinates).reshape(-1, 3)
                    self.node = int(np.argmin(np.linalg.norm(nodes - nodes.mean(axis=0), axis=1)))
                    self.rest_node = nodes[self.node].copy()
                    self.soft_rotation = SoftRotationPins(scene, actor, nodes, self.node)
                else:
                    self.roots.append(actor)
                continue
            root = leaves[0]
            if not root.is_static():
                self.roots.append(root)
            else:
                self.static_roots.append(actor)
            for part in leaves:
                name = part.get_name().removeprefix(prefix + "/")
                if name not in models:
                    continue
                model, model_scale = models[name]
                self.parts.append((part, model, (scale * model_scale).tolist()))
                self.contacts.append(("grasp_object", part))
        for _, part in self.contacts:
            params = part.get_contact_params()
            params.penalty_coefficient = contact_stiffness(hardness)
            part.set_contact_params(params)
        self.asset_scale = float(scale)

    def release_pins(self):
        for pins in (self.position_pins, self.rotation_pins):
            for pin, _ in pins:
                self.scene.destroy_constraint(pin)
            pins.clear()
        if self.soft_rotation:
            self.soft_rotation.release()

    def destroy(self):
        self.release_pins()
        for constraint in self.instance.constraints:
            self.scene.destroy_constraint(constraint)
        for actor in self.instance.actors:
            self.scene.destroy_actor(actor)

    def step(self, settings, dt):
        dragging = settings["drag"]
        delta = yaw_step_degrees(self.angle, settings["angle_deg"], dt)
        if not settings["locked"] and dragging is None and not delta:
            self.release_pins()
            self.was_free = True
            return
        if self.was_free:
            bounds = [part.get_aabb_world() for _, part in self.contacts]
            low = np.min([box.min for box in bounds], axis=0)
            high = np.max([box.max for box in bounds], axis=0)
            self.center = (low + high) / 2.
            self.was_free = False
        if not self.position_pins:
            for actor in self.roots:
                pose = actor.get_root_transform()
                handle = actor.get_handle()
                position = np.asarray(pose.translation)
                pull = self.scene.create_rigid_pivot_position_constraint(
                    actor=handle, local_position=[0., 0., 0.], target_position=position,
                    stiffness=4000., damping=12.)
                self.position_pins.append((pull, position - self.center))
                rotation = self.scene.create_rigid_pivot_rotation_constraint(
                    actor=handle, local_rotation=[0., 0., 0.],
                    target_rotation=Rotation.from_quat(list(pose.rotation)).as_rotvec(),
                    stiffness=1., damping=.03)
                self.rotation_pins.append((rotation, Rotation.from_quat(list(pose.rotation))))
            if self.soft is not None:
                nodes = np.asarray(self.soft.get_node_positions_local()).reshape(-1, 3)
                node = nodes[self.node] if len(nodes) else self.rest_node
                pose = self.soft.get_root_transform()
                point = Rotation.from_quat(list(pose.rotation)).apply(node) + np.asarray(pose.translation)
                pull = self.scene.create_deformable_node_position_constraint(
                    actor=self.soft.get_handle(), node_index=self.node, position=point, stiffness=4000., damping=1.)
                self.position_pins.append((pull, point - self.center))
        old_center = self.center.copy()
        if dragging is not None:
            move = np.asarray(dragging) - self.center
            self.center += move * min(1., .35 * dt / max(np.linalg.norm(move), 1e-9))
        step_rotation = Rotation.from_rotvec(self.yaw_axis * np.deg2rad(delta)) if delta else None
        self.angle += delta
        if step_rotation is not None:
            for i, (pin, rotation) in enumerate(self.rotation_pins):
                target = step_rotation * rotation
                pin.set_target_rotation(target.as_quat())
                self.rotation_pins[i] = pin, target
        if delta or dragging is not None:
            for pin, offset in self.position_pins:
                if step_rotation is not None:
                    offset[:] = step_rotation.apply(offset)
                pin.set_target_position(self.center + offset)
            for actor in self.static_roots:
                pose = actor.get_root_transform()
                offset = np.asarray(pose.translation) - old_center
                rotation = Rotation.from_quat(list(pose.rotation))
                if step_rotation is not None:
                    offset = step_rotation.apply(offset)
                    rotation = step_rotation * rotation
                actor.set_root_transform(self.sdp.TransformRT(rotation=rotation.as_quat(), translation=self.center + offset))
        if self.soft_rotation:
            self.soft_rotation.update(step_rotation, self.center + self.position_pins[-1][1])

    def packet(self):
        result = {"asset": True, "asset_scale": self.asset_scale, "desk_only": False,
                  "pose": [*self.center, 0., 0., 0., 1.]}
        if self.surface_actor is not None:
            points = (np.asarray(self.soft.get_surface_mesh_node_positions_local()).reshape(-1, 3)
                      if self.soft is not None else self.rest_surface)
            pose = self.surface_actor.get_root_transform()
            world = Rotation.from_quat(list(pose.rotation)).apply(points if len(points) else self.rest_surface)
            world += np.asarray(pose.translation)
            result.update(vertices=world.round(6).ravel().tolist(), indices=self.indices)
        else:
            result["parts"] = [{"model": model, "scale": scale, "pose": pose_values(part.get_root_transform())}
                               for part, model, scale in self.parts]
        return result
