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
    desk_only: bool = False


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
    "official_box_and_blocks": GraspAsset("分隔盒与方块", "box_and_blocks/box_and_blocks", True),
    "official_cup_pyramid": GraspAsset("纸杯金字塔", "paper_cups/paper_cup_pyramid", True),
    "official_pegboard": GraspAsset("灵巧度插销板", "functional_dexterity_test/functional_dexterity_test", True),
    "official_nine_hole": GraspAsset("九孔插销板", "nine_hole_peg_test/nine_hole_peg_test", True),
    "official_shape_box": GraspAsset("形状配对盒", "shape_box/shape_box", True),
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


def instantiate(sdp, scene, path, **placement):
    params = sdp.prefab.PrefabParams(apply_scene_settings=False, **placement)
    if path == prefab_path(GRASP_ASSETS["official_duck_lamp"]):
        # 沿用本地软球的接触积分，减少鸭灯表面的碰撞查询。
        prefab = sdp.prefab.load_from_file(str(path), str(path.parent))
        prefab.actors.soft[0].boundary_element_type = sdp.ActorBoundaryElementType.P1Q1
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


class GraspPrefab:
    def __init__(self, sdp, scene, asset, settings, revision, wrist_rotation, open_surface):
        self.sdp, self.scene = sdp, scene
        path = prefab_path(asset)
        low, high = authored_bounds(sdp, path)
        span = high - low
        # 两套大场景在固定腕部的桌面工作区统一缩放，保留各部件相对尺寸。
        scale = min(1., .32 / np.max(span)) if asset.desk_only else 1.
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
        self.instance = instantiate(sdp, scene, path, name=prefix, scale=scale,
                                    rotation=rotation.as_quat(), translation=translation)
        self.parts = []
        self.contacts = []
        self.roots = []
        self.pins = []
        self.soft = None
        models = render_models(path)
        for actor in self.instance.actors:
            leaves = nested_parts(sdp, scene, actor)
            if actor.get_type() == sdp.ActorType.SOFT:
                self.soft = actor
                actor.register_query_and_compute(sdp.QueryType.NODE_POSITIONS)
                actor.register_query(sdp.QueryType.SURFACE_NODE_POSITIONS)
                surface = actor.get_surface_mesh()
                self.rest_surface = np.asarray(surface.coordinates).copy().reshape(-1, 3)
                self.indices = list(surface.connectivity)
                nodes = np.asarray(actor.get_mesh().coordinates).reshape(-1, 3)
                self.node = int(np.argmin(np.linalg.norm(nodes - nodes.mean(axis=0), axis=1)))
                self.rest_node = nodes[self.node].copy()
                self.contacts.append(("grasp_object", actor))
                continue
            root = leaves[0]
            if not root.is_static():
                self.roots.append(root)
            for part in leaves:
                name = part.get_name().removeprefix(prefix + "/")
                if name not in models:
                    continue
                model, model_scale = models[name]
                self.parts.append((part, model, (scale * model_scale).tolist()))
                self.contacts.append(("grasp_object", part))
        self.asset_scale = float(scale)
        self.desk_only = asset.desk_only

    def release_pins(self):
        for pin, _ in self.pins:
            self.scene.destroy_constraint(pin)
        self.pins.clear()

    def destroy(self):
        self.release_pins()
        for constraint in self.instance.constraints:
            self.scene.destroy_constraint(constraint)
        for actor in self.instance.actors:
            self.scene.destroy_actor(actor)

    def step(self, settings, dt):
        dragging = settings["drag"]
        if not settings["locked"] and dragging is None:
            self.release_pins()
            return
        if not self.pins:
            for actor in self.roots:
                pose = actor.get_root_transform()
                handle = actor.get_handle()
                position = np.asarray(pose.translation)
                pull = self.scene.create_rigid_pivot_position_constraint(
                    actor=handle, local_position=[0., 0., 0.], target_position=position,
                    stiffness=4000., damping=12.)
                self.pins.append((pull, position - self.center))
                rotation = self.scene.create_rigid_pivot_rotation_constraint(
                    actor=handle, local_rotation=[0., 0., 0.],
                    target_rotation=Rotation.from_quat(list(pose.rotation)).as_rotvec(),
                    stiffness=1., damping=.03)
                self.pins.append((rotation, None))
            if self.soft is not None:
                nodes = np.asarray(self.soft.get_node_positions_local()).reshape(-1, 3)
                node = nodes[self.node] if len(nodes) else self.rest_node
                pose = self.soft.get_root_transform()
                point = Rotation.from_quat(list(pose.rotation)).apply(node) + np.asarray(pose.translation)
                pull = self.scene.create_deformable_node_position_constraint(
                    actor=self.soft.get_handle(), node_index=self.node, position=point, stiffness=4000., damping=1.)
                self.pins.append((pull, point - self.center))
        if dragging is not None:
            delta = np.asarray(dragging) - self.center
            self.center += delta * min(1., .35 * dt / max(np.linalg.norm(delta), 1e-9))
            for pin, offset in self.pins:
                if offset is not None:
                    pin.set_target_position(self.center + offset)

    def packet(self):
        result = {"asset": True, "asset_scale": self.asset_scale, "desk_only": self.desk_only,
                  "pose": [*self.center, 0., 0., 0., 1.]}
        if self.soft is not None:
            points = np.asarray(self.soft.get_surface_mesh_node_positions_local()).reshape(-1, 3)
            pose = self.soft.get_root_transform()
            world = Rotation.from_quat(list(pose.rotation)).apply(points if len(points) else self.rest_surface)
            world += np.asarray(pose.translation)
            result.update(vertices=world.round(6).ravel().tolist(), indices=self.indices)
        else:
            result["parts"] = [{"model": model, "scale": scale, "pose": pose_values(part.get_root_transform())}
                               for part, model, scale in self.parts]
        return result
