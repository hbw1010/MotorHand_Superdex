#!/usr/bin/env python3
"""导出 MotorHand (12-DOF VE40 / 16-DOF) + Sharpa WaveSE 并排 GLB，并以低延迟通过 stdio 提供实时 FK 姿态。"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import struct
import sys
from xml.etree import ElementTree

import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
SHARPA_ROOT = ROOT / "sharpa_teleop"


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=float)


def _quat_rot(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    u = np.array([x, y, z], dtype=float)
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def _rpy_to_mat(rpy: np.ndarray) -> np.ndarray:
    rx, ry, rz = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    return Rz @ Ry @ Rx


def _load_binary_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """纯 NumPy 读取二进制 STL，返回 (verts[N,3] float32, faces[M,3] uint32)。"""
    raw = path.read_bytes()
    if len(raw) < 84:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.uint32)
    tri_count = struct.unpack_from("<I", raw, 80)[0]
    expected = 84 + 50 * tri_count
    if expected != len(raw) or tri_count == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.uint32)
    dt = np.dtype([("normal", "<3f4"), ("v", "<3f4", (3,)), ("attr", "<u2")])
    arr = np.frombuffer(raw, dtype=dt, count=tri_count, offset=84)
    verts = np.ascontiguousarray(arr["v"].reshape(-1, 3), dtype=np.float32)
    faces = np.arange(tri_count * 3, dtype=np.uint32).reshape(-1, 3)
    return verts, faces


def _write_glb(out_path: Path, body_primitives: list[tuple[str, list[tuple[np.ndarray, np.ndarray, tuple[float, float, float, float], bool, str]]]]) -> None:
    """将按 body 分组的网格原语写出为二进制 glTF 2.0 (.glb)。"""
    bin_chunks = bytearray()
    buffer_views = []
    accessors = []
    materials = []
    mat_index: dict[tuple, int] = {}
    meshes = []
    nodes = []

    def append_bytes(data_bytes: bytes, target: int | None = None) -> int:
        while len(bin_chunks) % 4:
            bin_chunks.append(0)
        offset = len(bin_chunks)
        bin_chunks.extend(data_bytes)
        bv = {"buffer": 0, "byteOffset": offset, "byteLength": len(data_bytes)}
        if target is not None:
            bv["target"] = target
        idx = len(buffer_views)
        buffer_views.append(bv)
        return idx

    def get_material(rgba, is_sharpa: bool, gname: str) -> int:
        r, g, b, a = [float(x) for x in rgba]
        if is_sharpa:
            if "elastomer" in gname.lower() or "pad" in gname.lower():
                r, g, b, a = 0.28, 0.72, 0.78, 1.0
            elif r > 0.75 and g > 0.75 and b > 0.75:
                r, g, b, a = 0.82, 0.86, 0.90, 1.0
        key = (round(r, 3), round(g, 3), round(b, 3), round(a, 3), is_sharpa)
        if key not in mat_index:
            mat_index[key] = len(materials)
            mat = {
                "pbrMetallicRoughness": {
                    "baseColorFactor": [r, g, b, a],
                    "metallicFactor": 0.18 if is_sharpa else 0.28,
                    "roughnessFactor": 0.55 if is_sharpa else 0.48,
                },
                "doubleSided": True,
            }
            if a < 0.99:
                mat["alphaMode"] = "BLEND"
            materials.append(mat)
        return mat_index[key]

    for bname, prim_specs in body_primitives:
        is_sharpa = bname.startswith("sharpa_")
        primitives = []
        for verts_body, faces, rgba, is_sh, gname in prim_specs:
            if verts_body.shape[0] == 0 or faces.shape[0] == 0:
                continue
            v_min = verts_body.min(axis=0).tolist()
            v_max = verts_body.max(axis=0).tolist()
            pos_bv = append_bytes(np.ascontiguousarray(verts_body, dtype=np.float32).tobytes(), 34962)
            pos_acc = len(accessors)
            accessors.append({
                "bufferView": pos_bv,
                "componentType": 5126,
                "count": int(verts_body.shape[0]),
                "type": "VEC3",
                "min": v_min,
                "max": v_max,
            })
            idx_flat = np.ascontiguousarray(faces.reshape(-1), dtype=np.uint32)
            idx_bv = append_bytes(idx_flat.tobytes(), 34963)
            idx_acc = len(accessors)
            accessors.append({
                "bufferView": idx_bv,
                "componentType": 5125,
                "count": int(idx_flat.size),
                "type": "SCALAR",
            })
            primitives.append({
                "attributes": {"POSITION": pos_acc},
                "indices": idx_acc,
                "material": get_material(rgba, is_sh, gname),
            })
        if not primitives:
            continue
        mesh_idx = len(meshes)
        meshes.append({"name": bname, "primitives": primitives})
        nodes.append({"name": f"body__{bname}", "mesh": mesh_idx})

    while len(bin_chunks) % 4:
        bin_chunks.append(0)

    gltf = {
        "asset": {"version": "2.0", "generator": "superdex-sharpa-fk-worker"},
        "scene": 0,
        "scenes": [{"nodes": list(range(len(nodes)))}],
        "nodes": nodes,
        "meshes": meshes,
        "materials": materials,
        "accessors": accessors,
        "bufferViews": buffer_views,
        "buffers": [{"byteLength": len(bin_chunks)}],
    }
    json_bytes = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    while len(json_bytes) % 4:
        json_bytes += b" "

    total_len = 12 + 8 + len(json_bytes) + 8 + len(bin_chunks)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_path.with_suffix(".tmp")
    with tmp.open("wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, total_len))
        f.write(struct.pack("<II", len(json_bytes), 0x4E4F534A))
        f.write(json_bytes)
        f.write(struct.pack("<II", len(bin_chunks), 0x004E4942))
        f.write(bin_chunks)
    tmp.replace(out_path)


def _extract_mujoco_body_primitives(model) -> tuple[list[int], list[tuple[str, list]]]:
    body_geoms: dict[int, list[int]] = {}
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        if model.geom_group[g] == 3:
            continue
        if model.geom_rgba[g][3] <= 0.01:
            continue
        bid = int(model.geom_bodyid[g])
        body_geoms.setdefault(bid, []).append(g)

    body_ids = sorted(body_geoms.keys())
    out = []
    for bid in body_ids:
        bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid) or f"body_{bid}"
        is_sharpa = bname.startswith("sharpa_")
        prims = []
        for g in body_geoms[bid]:
            mid = int(model.geom_dataid[g])
            if mid < 0:
                continue
            v_adr = int(model.mesh_vertadr[mid])
            v_num = int(model.mesh_vertnum[mid])
            f_adr = int(model.mesh_faceadr[mid])
            f_num = int(model.mesh_facenum[mid])
            if v_num <= 0 or f_num <= 0:
                continue
            verts = np.array(model.mesh_vert[v_adr:v_adr + v_num], dtype=np.float64)
            faces = np.array(model.mesh_face[f_adr:f_adr + f_num], dtype=np.uint32)
            gpos = np.asarray(model.geom_pos[g], dtype=np.float64)
            gquat = np.asarray(model.geom_quat[g], dtype=np.float64)
            rmat = np.empty(9, dtype=np.float64)
            mujoco.mju_quat2Mat(rmat, gquat)
            R = rmat.reshape(3, 3)
            verts_body = (verts @ R.T + gpos).astype(np.float32)
            gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            rgba = tuple(float(x) for x in model.geom_rgba[g])
            prims.append((verts_body, faces, rgba, is_sharpa, gname))
        if prims:
            out.append((bname, prims))
    return body_ids, out


def _mujoco_body_poses(model, data, body_ids: list[int]) -> dict[str, list[float]]:
    out = {}
    for bid in body_ids:
        bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid) or f"body_{bid}"
        pos = data.xpos[bid]
        w, x, y, z = data.xquat[bid]
        out[bname] = [
            round(float(pos[0]), 5), round(float(pos[1]), 5), round(float(pos[2]), 5),
            round(float(x), 5), round(float(y), 5), round(float(z), 5), round(float(w), 5),
        ]
    return out


class CombinedScene12:
    """12-DOF MotorHand (VE40/VE33 MuJoCo 闭链) + Sharpa WaveSE 并排场景。"""

    def __init__(self, hand: str, glb_out: Path):
        side = "L" if hand == "left" else "R"
        ve40_dir = ROOT / "assets/hand_motor_ve40" / f"hand_motor_ve40_{side}"
        ve40_targets = SHARPA_ROOT / "assets/hand_motor_ve40" / f"hand_motor_ve40_{side}" / "config/retarget_targets.json"
        if (ve40_dir / "hand_closed_chain.xml").is_file() and ve40_targets.is_file():
            glove_dir = ve40_dir
            targets_path = ve40_targets
        else:
            glove_dir = ROOT / "assets/hand_motor_ve33" / f"hand_motor_ve33_{side}"
            targets_path = SHARPA_ROOT / "assets/hand_motor_ve33" / f"hand_motor_ve33_{side}" / "config/retarget_targets.json"

        sharpa_xml = SHARPA_ROOT / "assets/sharpa-hand" / f"{hand}.xml"
        targets = json.loads(targets_path.read_text(encoding="utf-8"))
        rotation = np.asarray(targets["canonical_to_model"], dtype=float).T
        wrist = targets["sites"][0]
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, rotation.ravel())
        glove_pos = np.array([0.0, -0.15, 0.12]) - rotation @ np.asarray(wrist["local_xyz_m"], dtype=float)

        hand_spec = mujoco.MjSpec.from_file(str(sharpa_xml))
        orig = hand_spec.compile()
        orig_data = mujoco.MjData(orig)
        mujoco.mj_forward(orig, orig_data)
        palm_id = orig.body(f"{hand}_hand_C_MC").id
        hand_pos = np.array([0.0, 0.15, 0.12]) - orig_data.xpos[palm_id]

        spec = mujoco.MjSpec.from_string("<mujoco><worldbody/></mujoco>")
        glove_spec = mujoco.MjSpec.from_file(str(glove_dir / "hand_closed_chain.xml"))
        for child, prefix, pos, orientation in (
            (glove_spec, "glove_", glove_pos, quat),
            (hand_spec, "sharpa_", hand_pos, [1.0, 0.0, 0.0, 0.0]),
        ):
            child.option.timestep = spec.option.timestep
            child.option.gravity[:] = spec.option.gravity
            child.option.integrator = spec.option.integrator
            for geom in child.geoms:
                geom.contype = geom.conaffinity = 0
            mount = spec.worldbody.add_frame(pos=pos, quat=orientation)
            spec.attach(child, prefix=prefix, frame=mount)

        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        closure = yaml.safe_load((glove_dir / "config/loop_closures.yaml").read_text(encoding="utf-8"))
        tree_names = closure["independent"] + closure["dependent"]
        self.glove_idx = np.array([int(self.model.joint("glove_" + n).qposadr[0]) for n in tree_names], dtype=int)
        self.sharpa_by_name = {}
        self.sharpa_idx = np.empty(orig.nq, dtype=int)
        for j in range(orig.njnt):
            jname = orig.joint(j).name
            addr = int(self.model.joint("sharpa_" + jname).qposadr[0])
            self.sharpa_by_name[jname] = addr
            self.sharpa_idx[orig.jnt_qposadr[j]] = addr

        self.object_pos = (hand_pos + np.array([0.065, 0.0, 0.14])).tolist()
        self.body_ids, body_prims = _extract_mujoco_body_primitives(self.model)
        _write_glb(glb_out, body_prims)

    def bodies(self) -> dict[str, list[float]]:
        return _mujoco_body_poses(self.model, self.data, self.body_ids)

    def step(self, req: dict) -> dict[str, list[float]]:
        tree = req.get("glove_tree_rad")
        if isinstance(tree, list) and len(tree) == len(self.glove_idx):
            self.data.qpos[self.glove_idx] = tree
        qpos = req.get("qpos")
        if req.get("simulated") and isinstance(qpos, list) and len(qpos) == len(self.sharpa_idx):
            self.data.qpos[self.sharpa_idx] = qpos
        else:
            names = req.get("joint_names")
            pos = req.get("position_rad")
            if isinstance(names, list) and isinstance(pos, list) and len(names) == len(pos):
                for n, v in zip(names, pos):
                    addr = self.sharpa_by_name.get(n)
                    if addr is not None:
                        self.data.qpos[addr] = v
        mujoco.mj_forward(self.model, self.data)
        return self.bodies()


class CombinedScene16:
    """16-DOF MotorHand (Pinocchio 80-树关节 + STL) + Sharpa WaveSE (MuJoCo) 并排场景。"""

    def __init__(self, hand: str, glb_out: Path):
        import pinocchio as pin
        self._pin = pin

        side = "L" if hand == "left" else "R"
        glove_dir = ROOT / f"assets/hand_motor_16dof_{side}"
        urdf_path = glove_dir / "hand_tree_only.urdf"
        targets_path = SHARPA_ROOT / "assets/hand_motor_16dof" / f"hand_motor_16dof_{side}" / "config/retarget_targets.json"
        if not urdf_path.is_file():
            raise FileNotFoundError(f"缺少 16-DOF URDF: {urdf_path}")

        targets = json.loads(targets_path.read_text(encoding="utf-8"))
        self.glove_rot = np.asarray(targets["canonical_to_model"], dtype=float).T
        wrist = targets["sites"][0]
        self.glove_pos = np.array([0.0, -0.15, 0.12]) - self.glove_rot @ np.asarray(wrist["local_xyz_m"], dtype=float)
        self.glove_quat_wxyz = np.empty(4)
        mujoco.mju_mat2Quat(self.glove_quat_wxyz, self.glove_rot.ravel())

        # Pinocchio 16-DOF 运动学模型（注意 continuous 关节 nq==2 使用 [cos, sin]）
        self.pin_model = pin.buildModelFromUrdf(str(urdf_path))
        self.pin_data = self.pin_model.createData()
        closure = yaml.safe_load((glove_dir / "config/loop_closures.yaml").read_text(encoding="utf-8"))
        tree_names = closure["independent"] + closure["dependent"]
        self.pin_joints: list[tuple[int, int]] = []
        for n in tree_names:
            jid = self.pin_model.getJointId(n)
            j = self.pin_model.joints[jid]
            self.pin_joints.append((int(j.idx_q), int(j.nq)))

        # 解析 16-DOF URDF visual STL 网格并生成 glove_<link> 原语
        urdf_root = ElementTree.parse(urdf_path).getroot()
        glove_prims: list[tuple[str, list]] = []
        self.glove_frames: list[tuple[str, int]] = []
        for link_el in urdf_root.findall("link"):
            lname = link_el.attrib["name"]
            if not self.pin_model.existFrame(lname):
                continue
            prims = []
            for vis in link_el.findall("visual"):
                mesh_el = vis.find("geometry/mesh")
                if mesh_el is None:
                    continue
                mesh_file = (glove_dir / mesh_el.attrib["filename"]).resolve()
                if not mesh_file.is_file():
                    continue
                verts, faces = _load_binary_stl(mesh_file)
                if verts.shape[0] == 0:
                    continue
                scale_str = mesh_el.get("scale")
                if scale_str:
                    scale = np.fromstring(scale_str, sep=" ", dtype=np.float64)
                    verts = (verts.astype(np.float64) * scale).astype(np.float32)
                orig_el = vis.find("origin")
                if orig_el is not None:
                    xyz = np.fromstring(orig_el.get("xyz", "0 0 0"), sep=" ", dtype=np.float64)
                    rpy = np.fromstring(orig_el.get("rpy", "0 0 0"), sep=" ", dtype=np.float64)
                    R = _rpy_to_mat(rpy)
                    verts = (verts.astype(np.float64) @ R.T + xyz).astype(np.float32)
                rgba = (0.72, 0.78, 0.85, 1.0)
                col_el = vis.find("material/color")
                if col_el is not None and col_el.get("rgba"):
                    vals = np.fromstring(col_el.get("rgba"), sep=" ", dtype=float)
                    if len(vals) == 4:
                        rgba = tuple(float(x) for x in vals)
                if "abd" in lname:
                    # 给新增的四指侧摆驱动件一抹醒目的青蓝色调，便于 3D 视窗直观区分 16-DOF 结构
                    rgba = (0.35, 0.75, 0.88, 1.0)
                prims.append((verts, faces, rgba, False, lname))
            if prims:
                bname = f"glove_{lname}"
                glove_prims.append((bname, prims))
                self.glove_frames.append((bname, int(self.pin_model.getFrameId(lname))))

        # Sharpa WaveSE (MuJoCo)
        sharpa_xml = SHARPA_ROOT / "assets/sharpa-hand" / f"{hand}.xml"
        hand_spec = mujoco.MjSpec.from_file(str(sharpa_xml))
        orig = hand_spec.compile()
        orig_data = mujoco.MjData(orig)
        mujoco.mj_forward(orig, orig_data)
        palm_id = orig.body(f"{hand}_hand_C_MC").id
        hand_pos = np.array([0.0, 0.15, 0.12]) - orig_data.xpos[palm_id]

        spec = mujoco.MjSpec.from_string("<mujoco><worldbody/></mujoco>")
        hand_spec.option.timestep = spec.option.timestep
        hand_spec.option.gravity[:] = spec.option.gravity
        hand_spec.option.integrator = spec.option.integrator
        for geom in hand_spec.geoms:
            geom.contype = geom.conaffinity = 0
        mount = spec.worldbody.add_frame(pos=hand_pos, quat=[1.0, 0.0, 0.0, 0.0])
        spec.attach(hand_spec, prefix="sharpa_", frame=mount)

        self.sharpa_model = spec.compile()
        self.sharpa_data = mujoco.MjData(self.sharpa_model)
        mujoco.mj_forward(self.sharpa_model, self.sharpa_data)
        self.sharpa_by_name = {}
        self.sharpa_idx = np.empty(orig.nq, dtype=int)
        for j in range(orig.njnt):
            jname = orig.joint(j).name
            addr = int(self.sharpa_model.joint("sharpa_" + jname).qposadr[0])
            self.sharpa_by_name[jname] = addr
            self.sharpa_idx[orig.jnt_qposadr[j]] = addr

        self.object_pos = (hand_pos + np.array([0.065, 0.0, 0.14])).tolist()
        self.sharpa_body_ids, sharpa_prims = _extract_mujoco_body_primitives(self.sharpa_model)
        _write_glb(glb_out, glove_prims + sharpa_prims)

        # 初始零位 FK
        q0 = pin.neutral(self.pin_model)
        pin.framesForwardKinematics(self.pin_model, self.pin_data, q0)

    def bodies(self) -> dict[str, list[float]]:
        out = _mujoco_body_poses(self.sharpa_model, self.sharpa_data, self.sharpa_body_ids)
        quat = np.empty(4)
        for bname, fid in self.glove_frames:
            omf = self.pin_data.oMf[fid]
            wpos = self.glove_pos + self.glove_rot @ omf.translation
            wrot = self.glove_rot @ omf.rotation
            mujoco.mju_mat2Quat(quat, np.ascontiguousarray(wrot).ravel())
            w, x, y, z = quat
            out[bname] = [
                round(float(wpos[0]), 5), round(float(wpos[1]), 5), round(float(wpos[2]), 5),
                round(float(x), 5), round(float(y), 5), round(float(z), 5), round(float(w), 5),
            ]
        return out

    def step(self, req: dict) -> dict[str, list[float]]:
        tree = req.get("glove_tree_rad")
        if isinstance(tree, list) and len(tree) == len(self.pin_joints):
            q = self._pin.neutral(self.pin_model)
            for (idx, nq), val in zip(self.pin_joints, tree):
                v = float(val)
                if nq == 2:
                    q[idx] = math.cos(v)
                    q[idx + 1] = math.sin(v)
                else:
                    q[idx] = v
            self._pin.framesForwardKinematics(self.pin_model, self.pin_data, q)
        qpos = req.get("qpos")
        if req.get("simulated") and isinstance(qpos, list) and len(qpos) == len(self.sharpa_idx):
            self.sharpa_data.qpos[self.sharpa_idx] = qpos
        else:
            names = req.get("joint_names")
            pos = req.get("position_rad")
            if isinstance(names, list) and isinstance(pos, list) and len(names) == len(pos):
                for n, v in zip(names, pos):
                    addr = self.sharpa_by_name.get(n)
                    if addr is not None:
                        self.sharpa_data.qpos[addr] = v
        mujoco.mj_forward(self.sharpa_model, self.sharpa_data)
        return self.bodies()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hand", choices=("right", "left"), default="right")
    parser.add_argument("--glove-model", choices=("12dof", "16dof"), default="12dof")
    parser.add_argument("--glb-out", type=Path, required=True)
    args = parser.parse_args()

    scene = (
        CombinedScene16(args.hand, args.glb_out)
        if args.glove_model == "16dof"
        else CombinedScene12(args.hand, args.glb_out)
    )
    ready = {
        "ready": True,
        "hand": args.hand,
        "glove_model": args.glove_model,
        "glb_path": str(args.glb_out),
        "object_pos": scene.object_pos,
        "object_radius": 0.035,
        "bodies": scene.bodies(),
    }
    sys.stdout.write(json.dumps(ready, separators=(",", ":")) + "\n")
    sys.stdout.flush()

    for line in sys.stdin:
        text = line.strip()
        if not text:
            continue
        try:
            req = json.loads(text)
            resp = {"bodies": scene.step(req)}
        except Exception as exc:
            resp = {"error": str(exc)}
        sys.stdout.write(json.dumps(resp, separators=(",", ":")) + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
