#!/usr/bin/env python3
"""从 assets/hand_motor_ve40/hand_motor_ve40_R 提取运动学资产并生成精确指腹 retarget_targets.json。"""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import struct
import numpy as np
import pinocchio as pin
import yaml

ROOT = Path(__file__).resolve().parents[2]
SHARPA_ROOT = ROOT / "sharpa_teleop"
VE40_SRC = ROOT / "assets/hand_motor_ve40/hand_motor_ve40_R"
VE40_DST = SHARPA_ROOT / "assets/hand_motor_ve40/hand_motor_ve40_R"
REF_TEMPLATE = SHARPA_ROOT / "assets/hand_motor_ve33/hand_motor_ve33_R/config/retarget_targets.json"

TIP_FRACTION = 0.35
SLICE_M, LATERAL_M = 0.0025, 0.004


def _stl_verts(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    if raw[:5] == b"solid" and b"facet" in raw[:400]:
        return np.array([list(map(float, line.split()[1:4])) for line in raw.decode(errors="ignore").splitlines()
                         if line.strip().startswith("vertex")])
    n = struct.unpack("<I", raw[80:84])[0]
    rec = np.frombuffer(raw[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3).astype(float)


def build_retarget_targets(model_dir: Path, ref_template: dict) -> dict:
    urdf = model_dir / "hand_tree_only.urdf"
    m = pin.buildModelFromUrdf(str(urdf))
    g = pin.buildGeomFromUrdf(m, str(urdf), pin.GeometryType.VISUAL, package_dirs=[str(model_dir)])
    d = m.createData()
    q = pin.neutral(m)
    pin.framesForwardKinematics(m, d, q)
    gd = pin.GeometryData(g)
    pin.updateGeometryPlacements(m, d, g, gd, q)

    meshes: dict[str, list[np.ndarray]] = {}
    for i, obj in enumerate(g.geometryObjects):
        body = m.frames[obj.parentFrame].name
        om = gd.oMg[i]
        meshes.setdefault(body, []).append(_stl_verts(Path(obj.meshPath)) * np.asarray(obj.meshScale) @ om.rotation.T + om.translation)
    mesh_map = {k: np.vstack(v) for k, v in meshes.items()}

    lc = yaml.safe_load((model_dir / "config/loop_closures.yaml").read_text(encoding="utf-8"))
    closures = {(c["link_a"], c["link_b"]): c for c in lc["closures"]}

    def closure_line(a: str, b: str):
        c = closures.get((a, b)) or closures[(b, a)]
        p = np.asarray(c["point"], float)
        u = np.asarray(c["axis"], float)
        return p, u / np.linalg.norm(u)

    def frame_pose(name: str):
        return d.oMf[m.getFrameId(name)]

    def link_pivot(link_name: str) -> np.ndarray:
        jid = m.frames[m.getFrameId(link_name)].parentJoint
        return d.oMi[jid].translation.copy()

    def project_onto_line(pt: np.ndarray, line_p: np.ndarray, line_u: np.ndarray) -> np.ndarray:
        return line_p + np.dot(pt - line_p, line_u) * line_u

    p_thumb_cmc = link_pivot("thumb_cmc_flex_link")
    c_p, c_u = closure_line("thumb_mcp_connecting_rod_1", "thumb_mcp_connecting_rod_3")
    rc_thumb_mcp = link_pivot("thumb_mcp_connecting_rod_3") + link_pivot("thumb_mcp_link") - c_p
    ip_mesh = mesh_map["thumb_ip_link"]
    s_vals = (ip_mesh - rc_thumb_mcp) @ c_u
    p_thumb_mcp = rc_thumb_mcp + 0.5 * (s_vals.min() + s_vals.max()) * c_u

    ip_p, ip_u = closure_line("thumb_mcp_link", "thumb_ip_link")
    s_ip = (ip_mesh - ip_p) @ ip_u
    p_thumb_ip = ip_p + 0.5 * (s_ip.min() + s_ip.max()) * ip_u

    dir_t = p_thumb_ip - p_thumb_mcp
    dir_t /= np.linalg.norm(dir_t)
    p_thumb_tip = p_thumb_ip + float(((ip_mesh - p_thumb_ip) @ dir_t).max()) * dir_t

    axis_world = {
        "thumb_cmc": ("base_link", p_thumb_cmc),
        "thumb_mcp": ("thumb_cmc_abd_link", p_thumb_mcp),
        "thumb_ip": ("thumb_ip_link", p_thumb_ip),
        "thumb_tip": ("thumb_ip_link", p_thumb_tip),
    }

    mcp_z = []
    for f in ("index", "middle", "ring", "pinky"):
        mount_p = frame_pose(f"{f}_mount_link").translation.copy()
        cp, cu = closure_line(f"{f}_mcp_connecting_rod_1", f"{f}_mcp_connecting_rod_3")
        rc = link_pivot(f"{f}_mcp_connecting_rod_3") + link_pivot(f"{f}_mcp_link") - cp
        p_mcp = project_onto_line(mount_p, rc, cu)
        if f in ("index", "middle", "ring"):
            mcp_z.append(p_mcp[2])
        pp, pu = closure_line(f"{f}_mcp_link", f"{f}_pip_link")
        p_pip = project_onto_line(mount_p, pp, pu)
        dp, du = closure_line(f"{f}_pip_link", f"{f}_dip_link")
        p_dip = project_onto_line(mount_p, dp, du)
        d_f = p_dip - p_pip
        d_f /= np.linalg.norm(d_f)
        dip_mesh = mesh_map[f"{f}_dip_link"]
        p_tip = p_dip + float(((dip_mesh - p_dip) @ d_f).max()) * d_f
        axis_world[f"{f}_mcp"] = (f"{f}_mount_link", p_mcp)
        axis_world[f"{f}_pip"] = (f"{f}_pip_link", p_pip)
        axis_world[f"{f}_dip"] = (f"{f}_dip_link", p_dip)
        axis_world[f"{f}_tip"] = (f"{f}_dip_link", p_tip)

    axis_world["wrist"] = ("base_link", np.array([0.0, 0.0, float(np.mean(mcp_z))]))

    def flex_motion(link: str, local: np.ndarray) -> np.ndarray:
        jid = m.frames[m.getFrameId(link)].parentJoint
        dd = m.createData()
        pts = []
        for dq in (0.0, 0.05):
            qq = pin.neutral(m)
            qq[m.joints[jid].idx_q] += dq
            pin.framesForwardKinematics(m, dd, qq)
            pts.append(dd.oMf[m.getFrameId(link)].act(local))
        return pts[1] - pts[0]

    def pad_depth(verts: np.ndarray, origin: np.ndarray, axis: np.ndarray, normal: np.ndarray, s_centre: float) -> float:
        lateral_axis = np.cross(axis, normal)
        rel = verts - origin
        s, lateral, depth = rel @ axis, rel @ lateral_axis, rel @ normal
        for k in (1, 2, 3, 4):
            sel = (np.abs(s - s_centre) < SLICE_M * k) & (np.abs(lateral) < LATERAL_M * k)
            if sel.sum() >= 3:
                return float(depth[sel].max())
        raise RuntimeError("指腹截面没有网格顶点")

    pad_world = {}
    pad_report = {}
    for f in ("thumb", "index", "middle", "ring", "pinky"):
        names = ("thumb_mcp", "thumb_ip", "thumb_tip") if f == "thumb" else (f"{f}_pip", f"{f}_dip", f"{f}_tip")
        pip, dip, tip = [axis_world[n][1] for n in names]
        a_tip = (tip - dip) / np.linalg.norm(tip - dip)
        a_pip = (dip - pip) / np.linalg.norm(dip - pip)
        tip_link = axis_world[names[2]][0]
        tip_local = frame_pose(tip_link).actInv(tip)
        motion = flex_motion(tip_link, tip_local)
        n_tip = motion - (motion @ a_tip) * a_tip
        n_tip /= np.linalg.norm(n_tip)
        check = axis_world["index_tip"][1] - tip if f == "thumb" else np.array([0.0, 0.0, -1.0])
        if n_tip @ check <= 0:
            n_tip = -n_tip
        n_pip = n_tip - (n_tip @ a_pip) * a_pip
        n_pip /= np.linalg.norm(n_pip)
        distal = mesh_map["thumb_ip_link" if f == "thumb" else f"{f}_dip_link"]
        s_end = float(((distal - dip) @ a_tip).max())
        s_centre = s_end * (1.0 - TIP_FRACTION)
        depth = pad_depth(distal, dip, a_tip, n_tip, s_centre)
        pads = (pip + n_pip * depth, dip + n_tip * depth, dip + a_tip * s_centre + n_tip * depth)
        for name, p in zip(names, pads):
            pad_world[name] = p
        pad_report[f] = {
            "depth_mm": round(depth * 1e3, 2),
            "back_from_tip_mm": float(round((np.linalg.norm(tip - dip) - s_centre) * 1e3, 2)),
        }

    out = dict(ref_template)
    new_sites = []
    for item in ref_template["sites"]:
        name = item["name"]
        link, ax_w = axis_world[name]
        ax_l = frame_pose(link).actInv(ax_w)
        entry = {
            "index": item["index"],
            "name": name,
            "link": link,
            "source": item["source"],
        }
        if name in pad_world:
            pw = pad_world[name]
            pl = frame_pose(link).actInv(pw)
            entry["local_xyz_m"] = pl.tolist()
            entry["zero_world_xyz_m"] = pw.tolist()
            entry["axis_local_xyz_m"] = ax_l.tolist()
            entry["axis_zero_world_xyz_m"] = ax_w.tolist()
            entry["task_point"] = "pad"
        else:
            entry["local_xyz_m"] = ax_l.tolist()
            entry["zero_world_xyz_m"] = ax_w.tolist()
        new_sites.append(entry)
    out["sites"] = new_sites
    out["pad_report"] = pad_report
    out["model_version"] = "hand_motor_ve40_R"
    bl = []
    for seq in (
        ("thumb_cmc", "thumb_mcp", "thumb_ip", "thumb_tip"),
        ("index_mcp", "index_pip", "index_dip", "index_tip"),
        ("middle_mcp", "middle_pip", "middle_dip", "middle_tip"),
        ("ring_mcp", "ring_pip", "ring_dip", "ring_tip"),
        ("pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip"),
    ):
        pts = [axis_world[n][1] for n in seq]
        bl.append([float(np.linalg.norm(pts[k + 1] - pts[k])) for k in range(3)])
    out["bone_lengths_m"] = bl
    return out


def main() -> None:
    (VE40_DST / "config").mkdir(parents=True, exist_ok=True)
    shutil.copy2(VE40_SRC / "hand_tree_only.urdf", VE40_DST / "hand_tree_only.urdf")
    for name in ("joint_space.json", "loop_closures.yaml"):
        shutil.copy2(VE40_SRC / "config" / name, VE40_DST / "config" / name)
    ref_template = json.loads(REF_TEMPLATE.read_text(encoding="utf-8"))
    targets = build_retarget_targets(VE40_SRC, ref_template)
    (VE40_DST / "config/retarget_targets.json").write_text(
        json.dumps(targets, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(f"已生成 VE40_R 运动学配置：{VE40_DST}")
    print("pad_report:", targets["pad_report"])


if __name__ == "__main__":
    main()
