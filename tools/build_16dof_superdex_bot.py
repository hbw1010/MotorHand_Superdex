#!/usr/bin/env python3
"""从 assets/hand_motor_16dof_R/hand_tree_only.urdf 生成 SuperDex 闭链资产（.superdex_bot、collision/*.mochi.h5、render/*.glb）。"""
from __future__ import annotations

import json
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
from scipy.spatial.transform import Rotation
import superdex.physics as sdp
import superdex.robotics as sdr
import trimesh
import yaml

ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "assets/hand_motor_16dof_R"
BOT_NAME = "hand_motor_16dof_right_closed_chain"
BOT_PATH = SRC_DIR / f"{BOT_NAME}.superdex_bot"


def _parse_vec(text: str | None, default=(0.0, 0.0, 0.0)) -> np.ndarray:
    if not text:
        return np.array(default, dtype=float)
    return np.fromstring(text, sep=" ", dtype=float)


def _rpy_to_mat(rpy: np.ndarray) -> np.ndarray:
    return Rotation.from_euler("xyz", rpy).as_matrix()


def _mat_to_quat_xyzw(mat: np.ndarray) -> list[float]:
    return Rotation.from_matrix(mat).as_quat().tolist()


def _load_link_meshes(model_dir: Path, link_elem: ElementTree.Element) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """返回 (visual_mesh, collision_mesh)：可视化保持完整细节，碰撞保证闭合流形且面片轻量。"""
    vis_parts: list[trimesh.Trimesh] = []
    col_parts: list[trimesh.Trimesh] = []
    for vis in link_elem.findall("visual"):
        mesh_el = vis.find("geometry/mesh")
        if mesh_el is None:
            continue
        mesh_path = model_dir / mesh_el.attrib["filename"]
        m = trimesh.load(str(mesh_path), force="mesh", process=True)
        m.merge_vertices()
        scale = _parse_vec(mesh_el.get("scale"), (1.0, 1.0, 1.0))
        if not np.allclose(scale, 1.0):
            m = m.copy()
            m.vertices = m.vertices * scale
        origin = vis.find("origin")
        if origin is not None:
            m = m.copy()
            xyz = _parse_vec(origin.get("xyz"))
            rpy = _parse_vec(origin.get("rpy"))
            tf = np.eye(4)
            tf[:3, :3] = _rpy_to_mat(rpy)
            tf[:3, 3] = xyz
            m.apply_transform(tf)
        vis_parts.append(m)
        cm = m if (m.is_volume and len(m.faces) <= 2500) else m.convex_hull
        col_parts.append(cm)
    if not vis_parts:
        box = trimesh.creation.box(extents=(0.004, 0.004, 0.004))
        return box, box
    vis_mesh = trimesh.util.concatenate(vis_parts) if len(vis_parts) > 1 else vis_parts[0]
    col_mesh = trimesh.util.concatenate(col_parts) if len(col_parts) > 1 else col_parts[0]
    return vis_mesh, col_mesh


def _save_link_assets(model_dir: Path, link_name: str, vis_mesh: trimesh.Trimesh, col_mesh: trimesh.Trimesh) -> tuple[str, str]:
    col_dir = model_dir / "collision"
    ren_dir = model_dir / "render"
    col_dir.mkdir(parents=True, exist_ok=True)
    ren_dir.mkdir(parents=True, exist_ok=True)

    # 渲染 GLB
    ren_rel = f"render/{link_name}.glb"
    mesh_ren = vis_mesh.copy()
    mesh_ren.visual = trimesh.visual.ColorVisuals(mesh=mesh_ren, face_colors=[180, 195, 210, 255])
    mesh_ren.export(str(model_dir / ren_rel))

    # 碰撞 mochi.h5（闭合网格 + 预烘焙 Grid SDF，避免运行期卡顿与非闭合告警）
    col_rel = f"collision/{link_name}.mochi.h5"
    c_verts = np.asarray(col_mesh.vertices, dtype=np.float64).ravel()
    c_faces = np.asarray(col_mesh.faces, dtype=np.int32).ravel()
    v_verts = np.asarray(vis_mesh.vertices, dtype=np.float64).ravel()
    v_faces = np.asarray(vis_mesh.faces, dtype=np.int32).ravel()
    model = sdp.ModelData()
    model.mesh = sdp.MeshData(nodes_per_element=3, coordinates=c_verts, connectivity=c_faces)
    model.visual_mesh = sdp.MeshData(nodes_per_element=3, coordinates=v_verts, connectivity=v_faces)
    sdf_params = sdp.GridSdfParams()
    sdf_params.resolution_mode = sdp.GridSdfResolutionMode.EXPLICIT
    sdf_params.resolution_delta = np.array([0.002, 0.002, 0.002], dtype=np.float32)
    sdf_params.boundary_padding_dist = 0.004
    sdp.model.bake_sdf(model, sdf_params)
    sdp.model.save_to_file(model, str(model_dir / col_rel), sdp.FileFormat.H5)
    return col_rel, ren_rel


def build_bot(model_dir: Path = SRC_DIR) -> Path:
    urdf_path = model_dir / "hand_tree_only.urdf"
    lc_path = model_dir / "config/loop_closures.yaml"
    root = ElementTree.parse(urdf_path).getroot()
    lc = yaml.safe_load(lc_path.read_text(encoding="utf-8"))

    link_elems = {el.attrib["name"]: el for el in root.findall("link")}
    joints_by_child: dict[str, ElementTree.Element] = {}
    children_of: dict[str, list[ElementTree.Element]] = {}
    all_children: set[str] = set()
    for jel in root.findall("joint"):
        parent = jel.find("parent").attrib["link"]
        child = jel.find("child").attrib["link"]
        joints_by_child[child] = jel
        children_of.setdefault(parent, []).append(jel)
        all_children.add(child)

    roots = [name for name in link_elems if name not in all_children]
    if len(roots) != 1:
        raise RuntimeError(f"Expected 1 root link, got {roots}")
    root_link = roots[0]

    # BFS 拓扑序构建 links、joints 与零位世界变换
    ordered_links: list[str] = []
    world_T: dict[str, np.ndarray] = {root_link: np.eye(4)}
    links_json: list[dict] = []
    joints_json: list[dict] = []
    link_index: dict[str, int] = {}

    queue = [root_link]
    while queue:
        lname = queue.pop(0)
        idx = len(ordered_links)
        ordered_links.append(lname)
        link_index[lname] = idx

        vis_mesh, col_mesh = _load_link_meshes(model_dir, link_elems[lname])
        col_rel, ren_rel = _save_link_assets(model_dir, lname, vis_mesh, col_mesh)

        if lname == root_link:
            joints_json.append({"name": "world_joint", "type": "Free"})
            links_json.append({
                "name": lname,
                "renderModel": ren_rel,
                "shape": col_rel,
                "shapeRotation": [0.0, 0.0, 0.0, 1.0],
                "shapeTranslation": [0.0, 0.0, 0.0],
            })
        else:
            jel = joints_by_child[lname]
            pname = jel.find("parent").attrib["link"]
            origin = jel.find("origin")
            xyz = _parse_vec(origin.get("xyz") if origin is not None else None)
            rpy = _parse_vec(origin.get("rpy") if origin is not None else None)
            R_local = _rpy_to_mat(rpy)
            T_local = np.eye(4)
            T_local[:3, :3] = R_local
            T_local[:3, 3] = xyz
            world_T[lname] = world_T[pname] @ T_local

            jtype = jel.attrib["type"]
            jentry: dict = {
                "name": jel.attrib["name"],
                "parentLinkFromJoint": {
                    "rotation": _mat_to_quat_xyzw(R_local),
                    "translation": xyz.tolist(),
                },
            }
            if jtype == "fixed":
                jentry["type"] = "Hard"
            elif jtype in ("revolute", "continuous"):
                jentry["type"] = "Revolute"
                axis_el = jel.find("axis")
                ax = _parse_vec(axis_el.get("xyz") if axis_el is not None else None, (0.0, 0.0, 1.0))
                ax = ax / np.linalg.norm(ax)
                jentry["axis"] = ax.tolist()
                lim = jel.find("limit")
                if jtype == "revolute" and lim is not None:
                    if lim.get("lower") is not None:
                        jentry["minLimit"] = [0.0, 0.0, float(lim.attrib["lower"])]
                    if lim.get("upper") is not None:
                        jentry["maxLimit"] = [0.0, 0.0, float(lim.attrib["upper"])]
                    if lim.get("effort") is not None:
                        jentry["effortLimit"] = float(lim.attrib["effort"])
            else:
                raise RuntimeError(f"Unsupported joint type: {jtype} ({jel.attrib['name']})")
            joints_json.append(jentry)
            links_json.append({
                "name": lname,
                "renderModel": ren_rel,
                "shape": col_rel,
                "shapeRotation": [0.0, 0.0, 0.0, 1.0],
                "shapeTranslation": [0.0, 0.0, 0.0],
                "parentLink": link_index[pname],
            })

        for child_j in children_of.get(lname, ()):
            queue.append(child_j.find("child").attrib["link"])

    # 32 处闭链转轴 -> 64 条点连接 cycles
    cycles_json: list[dict] = []
    for cl in lc["closures"]:
        la = cl["link_a"]
        lb = cl["link_b"]
        p0 = np.asarray(cl["point"], dtype=float)
        ax = np.asarray(cl["axis"], dtype=float)
        ax = ax / np.linalg.norm(ax)
        T_b_inv = np.linalg.inv(world_T[lb])
        for step in (0.0, 0.03):
            pw = p0 + step * ax
            pl_b = (T_b_inv @ np.array([pw[0], pw[1], pw[2], 1.0]))[:3]
            cycles_json.append({
                "parentLink": link_index[la],
                "childLink": link_index[lb],
                # SuperDex cycle 使用 jointFromChildLink = T_joint_from_child = -p_in_child
                "jointFromChildLink": {"translation": (-pl_b).tolist()},
            })

    n_dof = sum(1 for j in joints_json if j["type"] == "Revolute")
    bot = {
        "name": BOT_NAME,
        "defaultPose": [0.0] * n_dof,
        "joints": joints_json,
        "links": links_json,
        "cycles": cycles_json,
    }
    BOT_PATH.write_text(json.dumps(bot, indent=2) + "\n", encoding="utf-8")

    # 校验 SuperDex Robotics 能否原生加载
    prefab = sdr.load_bot_prefab_from_file(str(BOT_PATH))
    print(f"构建完成: {BOT_PATH}")
    print(f"  links={len(prefab.links)} joints={len(prefab.joints)} revolute={n_dof} cycles={len(cycles_json)}")
    return BOT_PATH


if __name__ == "__main__":
    build_bot()
