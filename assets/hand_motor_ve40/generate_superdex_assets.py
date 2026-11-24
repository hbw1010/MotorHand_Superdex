#!/usr/bin/env python3
"""Generate SuperDex collision (.mochi.h5), render (.glb), and closed-chain
(.superdex_bot) assets for hand_motor_ve40 from URDF, meshes, and loop closures.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
from pathlib import Path, PureWindowsPath
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import superdex.physics as sdp
import trimesh
import yaml


HERE = Path(__file__).resolve().parent
WITNESS_SEPARATION = 0.03  # meters


def is_metal(component: dict) -> bool:
    part = PureWindowsPath(component["source_part"]).stem
    return (
        "connecting_rod" in component["link"]
        or "drive_link" in component["link"]
        or "连杆" in part
        or "动子" in part
        or "定子" in part
        or part == "指节3-模型"
    )


def _origin_matrix(origin_elem) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if origin_elem is None:
        return np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]), np.eye(4)
    xyz = np.fromstring(origin_elem.get("xyz", "0 0 0"), sep=" ")
    rpy = np.fromstring(origin_elem.get("rpy", "0 0 0"), sep=" ")
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite(np.r_[xyz, rpy]).all():
        raise ValueError(f"Invalid origin xyz/rpy: {origin_elem.attrib}")
    rot = Rotation.from_euler("xyz", rpy)
    mat = np.eye(4)
    mat[:3, :3] = rot.as_matrix()
    mat[:3, 3] = xyz
    return xyz, rot.as_quat(), mat


def parse_urdf(urdf_path: Path):
    root = ET.parse(urdf_path).getroot()
    link_nodes = {link.attrib["name"]: link for link in root.findall("link")}
    parents = {}
    children = {name: [] for name in link_nodes}
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        if child in parents or child not in link_nodes or parent not in link_nodes:
            raise ValueError(f'Invalid URDF tree at joint {joint.attrib["name"]}')
        xyz, quat, mat = _origin_matrix(joint.find("origin"))
        axis_elem = joint.find("axis")
        axis = np.fromstring(axis_elem.get("xyz", "0 0 1"), sep=" ") if axis_elem is not None else np.array([0.0, 0.0, 1.0])
        norm = float(np.linalg.norm(axis))
        if norm > 0.0:
            axis = axis / norm
        lim = joint.find("limit")
        limits = None
        if lim is not None:
            lower = float(lim.get("lower", "0.0"))
            upper = float(lim.get("upper", "0.0"))
            effort = float(lim.get("effort", "1.0"))
            limits = (lower, upper, effort if effort > 0.0 else 1.0)
        parents[child] = {
            "parent": parent,
            "joint_name": joint.attrib["name"],
            "joint_type": joint.attrib["type"],
            "xyz": xyz,
            "quat": quat,
            "transform": mat,
            "axis": axis,
            "limits": limits,
        }
        children[parent].append(child)

    roots = [name for name in link_nodes if name not in parents]
    if len(roots) != 1:
        raise ValueError(f"URDF must have exactly one root, found {roots}")
    root_name = roots[0]

    # Topological BFS order so parentLink < child index for every link.
    ordered_links = []
    queue = deque([root_name])
    while queue:
        cur = queue.popleft()
        ordered_links.append(cur)
        for ch in children[cur]:
            queue.append(ch)
    if len(ordered_links) != len(link_nodes):
        raise ValueError("URDF contains disconnected links")

    poses = {root_name: np.eye(4)}
    for name in ordered_links[1:]:
        info = parents[name]
        poses[name] = poses[info["parent"]] @ info["transform"]

    return root_name, ordered_links, link_nodes, parents, poses


def build_link_meshes(model_dir: Path, link_node, by_mesh: dict, collision_dir: Path, render_dir: Path):
    name = link_node.attrib["name"]
    visuals = link_node.findall("visual")
    if not visuals:
        raise ValueError(f"Link {name} has no <visual> geometry")

    vis_verts = []
    vis_faces = []
    offset = 0
    col_parts = []
    render_scene = trimesh.Scene()
    first_stl = None

    metal_mat = trimesh.visual.material.PBRMaterial(
        name="metal", baseColorFactor=[0.83, 0.84, 0.86, 1.0], metallicFactor=0.97, roughnessFactor=0.34
    )
    body_mat = trimesh.visual.material.PBRMaterial(
        name="body", baseColorFactor=[0.04, 0.045, 0.052, 1.0], metallicFactor=0.0, roughnessFactor=0.80
    )

    for idx, visual in enumerate(visuals):
        mesh_elem = visual.find("geometry/mesh")
        stl_path = model_dir / mesh_elem.attrib["filename"]
        if first_stl is None:
            first_stl = stl_path
        scale = np.fromstring(mesh_elem.get("scale", "1 1 1"), sep=" ")
        _, _, mat = _origin_matrix(visual.find("origin"))

        raw = trimesh.load_mesh(stl_path, process=False)
        if not isinstance(raw, trimesh.Trimesh):
            raw = trimesh.util.concatenate(tuple(raw.geometry.values()))
        verts = np.asarray(raw.vertices, dtype=np.float64) * scale
        verts = (mat[:3, :3] @ verts.T).T + mat[:3, 3]
        faces = np.asarray(raw.faces, dtype=np.int32)

        vis_verts.append(verts)
        vis_faces.append(faces + offset)
        offset += len(verts)

        part = trimesh.Trimesh(vertices=verts, faces=faces, process=True)
        if not part.is_volume:
            part = part.convex_hull
        col_parts.append(part)

        comp = by_mesh.get(stl_path.name)
        mat_pbr = metal_mat if (comp is not None and is_metal(comp)) else body_mat
        render_mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
        render_mesh.visual = trimesh.visual.TextureVisuals(material=mat_pbr)
        render_scene.add_geometry(render_mesh, node_name=f"{name}_{idx}")

    collision_mesh = trimesh.util.concatenate(col_parts)
    model = sdp.model.load_from_file(str(first_stl))
    model.mesh = sdp.MeshData(
        nodes_per_element=3,
        coordinates=np.ascontiguousarray(collision_mesh.vertices, dtype=np.float64).ravel(),
        connectivity=np.ascontiguousarray(collision_mesh.faces, dtype=np.int32).ravel(),
    )
    model.visual_mesh = sdp.MeshData(
        nodes_per_element=3,
        coordinates=np.ascontiguousarray(np.concatenate(vis_verts, axis=0), dtype=np.float64).ravel(),
        connectivity=np.ascontiguousarray(np.concatenate(vis_faces, axis=0), dtype=np.int32).ravel(),
    )
    model.sdf = None
    sdp.model.bake_sdf(model)
    col_rel = f"collision/{name}.mochi.h5"
    sdp.model.save_to_file(model, str(collision_dir / f"{name}.mochi.h5"), sdp.FileFormat.H5)

    render_rel = f"render/{name}.glb"
    (render_dir / f"{name}.glb").write_bytes(render_scene.export(file_type="glb"))
    return col_rel, render_rel


def generate(model_dir: Path, bot_prefix: str):
    urdf_path = model_dir / "hand_tree_only.urdf"
    closures_path = model_dir / "config/loop_closures.yaml"
    cad_path = model_dir / "config/cad_components.json"

    by_mesh = {p["mesh"]: p for p in json.loads(cad_path.read_text(encoding="utf-8"))}
    collision_dir = model_dir / "collision"
    render_dir = model_dir / "render"
    collision_dir.mkdir(exist_ok=True)
    render_dir.mkdir(exist_ok=True)

    _, ordered_links, link_nodes, parents, poses = parse_urdf(urdf_path)
    indices = {name: i for i, name in enumerate(ordered_links)}

    links_json = []
    joints_json = [{"name": "world_joint", "type": "Free"}]
    dof_count = 0

    for i, name in enumerate(ordered_links):
        col_rel, render_rel = build_link_meshes(model_dir, link_nodes[name], by_mesh, collision_dir, render_dir)
        entry = {
            "name": name,
            "renderModel": render_rel,
            "shape": col_rel,
            "shapeRotation": [0.0, 0.0, 0.0, 1.0],
            "shapeTranslation": [0.0, 0.0, 0.0],
        }
        if i > 0:
            info = parents[name]
            entry["parentLink"] = indices[info["parent"]]
            jtype = "Hard" if info["joint_type"] == "fixed" else "Revolute"
            j_entry = {
                "name": info["joint_name"],
                "parentLinkFromJoint": {
                    "rotation": info["quat"].tolist(),
                    "translation": info["xyz"].tolist(),
                },
                "type": jtype,
            }
            if jtype == "Revolute":
                dof_count += 1
                j_entry["axis"] = info["axis"].tolist()
                if info["limits"] is not None:
                    lower, upper, effort = info["limits"]
                    j_entry["effortLimit"] = effort
                    j_entry["minLimit"] = (info["axis"] * lower).tolist()
                    j_entry["maxLimit"] = (info["axis"] * upper).tolist()
            joints_json.append(j_entry)
        links_json.append(entry)

    tree_bot = {
        "name": f"{bot_prefix}_visual_repaired",
        "defaultPose": [0.0] * dof_count,
        "joints": joints_json,
        "links": links_json,
    }
    tree_path = model_dir / f"{bot_prefix}_visual_repaired.superdex_bot"
    tree_path.write_text(json.dumps(tree_bot, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")

    closures_cfg = yaml.safe_load(closures_path.read_text(encoding="utf-8"))
    cycles = []
    for closure in closures_cfg["closures"]:
        a, b = closure["link_a"], closure["link_b"]
        if a not in indices or b not in indices or a == b:
            raise ValueError(f"Invalid closure links: {a}, {b}")
        point = np.asarray(closure["point"], dtype=float)
        axis = np.asarray(closure["axis"], dtype=float)
        norm = float(np.linalg.norm(axis))
        if norm < 1e-12:
            raise ValueError(f"Zero closure axis: {a}, {b}")
        for witness in (point, point + WITNESS_SEPARATION * axis / norm):
            child_pose = poses[b]
            child_local = child_pose[:3, :3].T @ (witness - child_pose[:3, 3])
            cycles.append({
                "parentLink": indices[a],
                "childLink": indices[b],
                "jointFromChildLink": {"translation": (-child_local).tolist()},
            })

    closed_bot = {**tree_bot, "name": f"{bot_prefix}_closed_chain", "cycles": cycles}
    closed_path = model_dir / f"{bot_prefix}_closed_chain.superdex_bot"
    closed_path.write_text(json.dumps(closed_bot, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Generated {len(links_json)} links, {len(joints_json)} joints ({dof_count} DOFs), {len(cycles)} cycle joints -> {closed_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", type=Path, default=HERE / "hand_motor_ve40_R")
    parser.add_argument("--prefix", default="hand_motor_ve40_right")
    args = parser.parse_args()
    sdp.initialize(num_worker_threads=4)
    generate(args.model_dir, args.prefix)


if __name__ == "__main__":
    main()
