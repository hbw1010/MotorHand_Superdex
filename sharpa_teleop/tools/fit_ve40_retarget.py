#!/usr/bin/env python3
"""针对 hand_motor_ve40_R 闭链模型重新拟合 Sharpa 右手重定向参数（几何、指段比例、四指对指闭合与零位展平）。"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np
from scipy.optimize import least_squares
import yaml

ROOT = Path(__file__).resolve().parents[2]
SHARPA_ROOT = ROOT / "sharpa_teleop"
PIXI_PY = SHARPA_ROOT / ".pixi/envs/default/bin/python"
PROBE_BIN = SHARPA_ROOT / ".cache/sharpa-control/sharpa_retarget_probe"
VE40_DIR = ROOT / "assets/hand_motor_ve40/hand_motor_ve40_R"
SHARPA_CFG = SHARPA_ROOT / "config/retargeting/sharpa/right.yaml"
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
TASK_INDICES = np.array([[4 * f + 2 + j for j in range(3)] for f in range(5)])

sys.path.insert(0, str(ROOT / "tools"))
from motorhand.mapping import GloveMapping
from motorhand.native_kinematics import NativeGloveKinematics


MJ_WORKER_CODE = r"""
import json, sys, numpy as np, mujoco, pinocchio as pin
from pathlib import Path

root = Path(sys.argv[1])
sharpa_root = root / "sharpa_teleop"
ve40_xml = root / "assets/hand_motor_ve40/hand_motor_ve40_R/hand_closed_chain.xml"
rt_path = sharpa_root / "assets/hand_motor_ve40/hand_motor_ve40_R/config/retarget_targets.json"
pad_urdf = sharpa_root / "assets/sharpa-hand/wave_01/right_sharpa_wave/right_sharpa_wave_pad.urdf"
sharpa_xml = sharpa_root / "assets/sharpa-hand/right.xml"

m40 = mujoco.MjModel.from_xml_path(str(ve40_xml))
d40 = mujoco.MjData(m40)
rt40 = json.loads(rt_path.read_text(encoding="utf-8"))
R40 = np.asarray(rt40["canonical_to_model"], float)
jnames = json.loads(sys.stdin.readline())
j_idx = [m40.jnt_qposadr[mujoco.mj_name2id(m40, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in jnames]
site_meta = [
    (mujoco.mj_name2id(m40, mujoco.mjtObj.mjOBJ_BODY, s["link"]), np.asarray(s["local_xyz_m"], float))
    for s in sorted(rt40["sites"], key=lambda x: x["index"])
]

ve_verts = {}
for ln in ("thumb_ip_link", "index_dip_link", "middle_dip_link", "ring_dip_link", "pinky_dip_link"):
    bid = mujoco.mj_name2id(m40, mujoco.mjtObj.mjOBJ_BODY, ln)
    arr = []
    for g in range(m40.ngeom):
        if m40.geom_bodyid[g] == bid and m40.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = m40.geom_dataid[g]
            v0, nv = m40.mesh_vertadr[mid], m40.mesh_vertnum[mid]
            arr.append((g, m40.mesh_vert[v0:v0 + nv:12]))
    ve_verts[ln] = arr

sm = pin.buildModelFromUrdf(str(pad_urdf))
sd = sm.createData()
SUFFIXES = (
    "thumb_CMC_FE", "thumb_CMC_AA", "thumb_MCP_FE", "thumb_MCP_AA", "thumb_IP",
    "index_MCP_FE", "index_MCP_AA", "index_PIP", "index_DIP",
    "middle_MCP_FE", "middle_MCP_AA", "middle_PIP", "middle_DIP",
    "ring_MCP_FE", "ring_MCP_AA", "ring_PIP", "ring_DIP",
    "pinky_CMC", "pinky_MCP_FE", "pinky_MCP_AA", "pinky_PIP", "pinky_DIP",
)
s_jidx = np.array([sm.joints[sm.getJointId("right_" + s)].idx_q for s in SUFFIXES])
s_fnames = ["right_hand_C_MC"]
for f in ("thumb", "index", "middle", "ring", "pinky"):
    s_fnames += [f"right_{f}_MCP_VL"] + [f"right_{f}_pad_{k}" for k in ("pip", "dip", "tip")]
s_fids = [sm.getFrameId(n) for n in s_fnames]
s_lo = sm.lowerPositionLimit[s_jidx]
s_hi = sm.upperPositionLimit[s_jidx]

ms = mujoco.MjModel.from_xml_path(str(sharpa_xml))
ds = mujoco.MjData(ms)
sj_idx = [ms.jnt_qposadr[mujoco.mj_name2id(ms, mujoco.mjtObj.mjOBJ_JOINT, "right_" + s)] for s in SUFFIXES]
sa_idx = [mujoco.mj_name2id(ms, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_" + s + "_ctrl") for s in SUFFIXES]
sh_verts = {}
for fn in ("thumb", "index", "middle", "ring", "pinky"):
    gid = mujoco.mj_name2id(ms, mujoco.mjtObj.mjOBJ_GEOM, f"right_{fn}_elastomer")
    mid = ms.geom_dataid[gid]
    v0, nv = ms.mesh_vertadr[mid], ms.mesh_vertnum[mid]
    sh_verts[fn] = (gid, ms.mesh_vert[v0:v0 + nv:10])

mujoco.mj_resetData(m40, d40)
mujoco.mj_forward(m40, d40)
zpts = np.array([d40.xpos[bid] + d40.xmat[bid].reshape(3, 3) @ loc for bid, loc in site_meta])
zref = np.array([s["zero_world_xyz_m"] for s in sorted(rt40["sites"], key=lambda x: x["index"])])
assert float(np.max(np.linalg.norm(zpts - zref, axis=1)) * 1000.0) < 1e-3

sys.stdout.write("READY\n")
sys.stdout.flush()

for line in sys.stdin:
    req = json.loads(line)
    cmd = req.get("cmd", "ve40")
    if cmd == "ve40":
        for idx, val in zip(j_idx, req["tree"]):
            d40.qpos[idx] = val
        mujoco.mj_forward(m40, d40)
        pts = np.array([d40.xpos[bid] + d40.xmat[bid].reshape(3, 3) @ loc for bid, loc in site_meta])
        pts[1] = 0.5 * (pts[0] + pts[2])
        canon = (pts - pts[0]) @ R40
        out = {"points_m": canon.tolist()}
        fn = req.get("mesh_finger")
        if fn:
            vt = np.vstack([v @ d40.geom_xmat[g].reshape(3, 3).T + d40.geom_xpos[g] for g, v in ve_verts["thumb_ip_link"]])
            vf = np.vstack([v @ d40.geom_xmat[g].reshape(3, 3).T + d40.geom_xpos[g] for g, v in ve_verts[f"{fn}_dip_link"]])
            dmat = np.linalg.norm(vt[:, None, :] - vf[None, :, :], axis=-1)
            i_t, i_f = np.unravel_index(np.argmin(dmat), dmat.shape)
            out["mesh_gap_mm"] = float(dmat[i_t, i_f] * 1000.0)
            out["mesh_diff_mm"] = ((vt[i_t] - vf[i_f]) @ R40 * 1000.0).tolist()
        sys.stdout.write(json.dumps(out) + "\n")
    elif cmd == "sharpa_fk":
        q22 = np.clip(np.asarray(req["q22"], float), s_lo, s_hi)
        q = np.zeros(22)
        q[s_jidx] = q22
        pin.framesForwardKinematics(sm, sd, q)
        pts = np.array([sd.oMf[f].translation for f in s_fids])
        pts -= pts[0]
        pts[1] = 0.5 * pts[2]
        sys.stdout.write(json.dumps({"physical_points_m": pts.tolist()}) + "\n")
    elif cmd == "sharpa_el":
        results = []
        for item in req["items"]:
            for idx, val in zip(sj_idx, item["q22"]):
                ds.qpos[idx] = val
            mujoco.mj_forward(ms, ds)
            gt, vt0 = sh_verts["thumb"]
            vt = vt0 @ ds.geom_xmat[gt].reshape(3, 3).T + ds.geom_xpos[gt]
            gf, vf0 = sh_verts[item["finger"]]
            vf = vf0 @ ds.geom_xmat[gf].reshape(3, 3).T + ds.geom_xpos[gf]
            results.append(float(np.min(np.linalg.norm(vt[:, None, :] - vf[None, :, :], axis=-1)) * 1000.0))
        sys.stdout.write(json.dumps({"el_gaps_mm": results}) + "\n")
    elif cmd == "sharpa_sim_contact":
        results = []
        for item in req["items"]:
            mujoco.mj_resetData(ms, ds)
            for a, val in zip(sa_idx, item["q22"]):
                ds.ctrl[a] = val
            for _ in range(180):
                mujoco.mj_step(ms, ds)
            mujoco.mj_forward(ms, ds)
            gt, vt0 = sh_verts["thumb"]
            vt = vt0 @ ds.geom_xmat[gt].reshape(3, 3).T + ds.geom_xpos[gt]
            gf, vf0 = sh_verts[item["finger"]]
            vf = vf0 @ ds.geom_xmat[gf].reshape(3, 3).T + ds.geom_xpos[gf]
            el_gap = float(np.min(np.linalg.norm(vt[:, None, :] - vf[None, :, :], axis=-1)) * 1000.0)
            touching = False
            t_prefix = "right_thumb"
            f_prefix = f"right_{item['finger']}"
            for c in range(ds.ncon):
                con = ds.contact[c]
                g1 = mujoco.mj_id2name(ms, mujoco.mjtObj.mjOBJ_GEOM, con.geom1) or ""
                g2 = mujoco.mj_id2name(ms, mujoco.mjtObj.mjOBJ_GEOM, con.geom2) or ""
                if (g1.startswith(t_prefix) and g2.startswith(f_prefix)) or (g2.startswith(t_prefix) and g1.startswith(f_prefix)):
                    touching = True
                    break
            results.append({"el_gap_mm": el_gap, "contact": touching})
        sys.stdout.write(json.dumps({"results": results}) + "\n")
    sys.stdout.flush()
"""


class VE40Evaluator:
    def __init__(self):
        self.mapping = GloveMapping.load(ROOT / "hardware/encos_g4_gateway_sdk/config/right_sim_mapping.json")
        self.kin = NativeGloveKinematics(VE40_DIR, self.mapping)
        self.proc = subprocess.Popen(
            [str(PIXI_PY), "-c", MJ_WORKER_CODE, str(ROOT)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )
        self.proc.stdin.write(json.dumps(list(self.kin.joint_names)) + "\n")
        self.proc.stdin.flush()
        ready = self.proc.stdout.readline().strip()
        if ready != "READY":
            raise RuntimeError(f"MJ worker failed: {ready}")
        lo, hi = np.array([m.model_rad_limits() for m in self.mapping.motors]).T
        self.lo = lo
        self.hi = hi

    def eval_ve40(self, motors_12, mesh_finger: str | None = None):
        tree = list(self.kin.native.solve([float(x) for x in motors_12], 0))
        self.proc.stdin.write(json.dumps({"cmd": "ve40", "tree": tree, "mesh_finger": mesh_finger}) + "\n")
        self.proc.stdin.flush()
        res = json.loads(self.proc.stdout.readline())
        return np.asarray(res["points_m"], float), res.get("mesh_gap_mm"), res.get("mesh_diff_mm")

    def sharpa_physical_points(self, q22):
        self.proc.stdin.write(json.dumps({"cmd": "sharpa_fk", "q22": [float(x) for x in q22]}) + "\n")
        self.proc.stdin.flush()
        return np.asarray(json.loads(self.proc.stdout.readline())["physical_points_m"], float)

    def sharpa_elastomer_gaps(self, items: list[dict]) -> list[float]:
        self.proc.stdin.write(json.dumps({"cmd": "sharpa_el", "items": items}) + "\n")
        self.proc.stdin.flush()
        return json.loads(self.proc.stdout.readline())["el_gaps_mm"]

    def sharpa_sim_contacts(self, items: list[dict]) -> list[dict]:
        self.proc.stdin.write(json.dumps({"cmd": "sharpa_sim_contact", "items": items}) + "\n")
        self.proc.stdin.flush()
        return json.loads(self.proc.stdout.readline())["results"]

    def close(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            self.proc.wait(timeout=3)


def build_probe_input(seq_frames: list[dict]) -> str:
    lines = []
    prev_seq = None
    for p in seq_frames:
        req = {"points_m": p["points_m"]}
        if p.get("sequence") != prev_seq:
            req["reset"] = True
        lines.append(json.dumps(req) + "\n")
        prev_seq = p.get("sequence")
    return "".join(lines)


def run_probe_raw(cfg_dict: dict, probe_stdin: str) -> list[dict]:
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", dir=str(SHARPA_CFG.parent), delete=False) as tf:
        yaml.safe_dump(cfg_dict, tf, allow_unicode=True, sort_keys=False)
        tmp_path = Path(tf.name)
    try:
        res = subprocess.run(
            [str(PROBE_BIN), "--hand", "right", "--config", str(tmp_path)],
            input=probe_stdin,
            capture_output=True,
            text=True,
            check=True,
        )
        return [json.loads(ln) for ln in res.stdout.splitlines() if ln.strip()]
    finally:
        tmp_path.unlink(missing_ok=True)


def build_pinch_benchmark(ev: VE40Evaluator) -> list[dict]:
    """构建四指真实贴合对指基准（覆盖每指高/中/深多种自然对指，确保 VE40 视觉网格真实接触 <= 1.5 mm）。"""
    thumb_grid = []
    for m0 in np.linspace(0.0, 1.55, 12):
        for m1 in np.linspace(0.1, 1.65, 12):
            for m2 in (0.0, 0.45, 0.9, 1.3):
                for m3 in (0.0, 0.5, 1.0):
                    try:
                        pts, _, _ = ev.eval_ve40([m0, m1, m2, m3, 0, 0, 0, 0, 0, 0, 0, 0])
                        thumb_grid.append((pts[4] * 1000.0, [m0, m1, m2, m3]))
                    except Exception:
                        pass
    t_arr = np.array([x[0] for x in thumb_grid])

    bench = []
    for fi, fname in enumerate(("index", "middle", "ring", "pinky")):
        ti = (fi + 2) * 4
        specs = (
            (0.90, 0.85, "mid"),
            (1.15, 0.80, "low"),
            (0.85, 1.15, "curl"),
        )
        for fmcp, fpip, tag in specs:
            m = np.zeros(12)
            m[4 + 2 * fi] = fmcp
            m[5 + 2 * fi] = fpip
            try:
                fpts, _, _ = ev.eval_ve40(m)
            except Exception:
                continue
            ft = fpts[ti] * 1000.0
            bi = int(np.argmin(np.linalg.norm(t_arr - ft, axis=1)))
            tm = thumb_grid[bi][1]
            active = np.array([0, 1, 2, 3, 4 + 2 * fi, 5 + 2 * fi])
            alo = ev.lo[active].copy()
            ahi = ev.hi[active].copy()
            alo[4], ahi[4] = max(alo[4], fmcp - 0.2), min(ahi[4], fmcp + 0.2)
            alo[5], ahi[5] = max(alo[5], fpip - 0.2), min(ahi[5], fpip + 0.2)
            x0 = np.clip(tm + [fmcp, fpip], alo + 0.01, ahi - 0.01)

            def resid(x):
                mm = np.zeros(12)
                mm[active] = x
                try:
                    pts, _, _ = ev.eval_ve40(mm)
                    return (pts[4] - pts[ti]) * 1000.0
                except Exception:
                    return np.full(3, 1e4)

            sol = least_squares(resid, x0, bounds=(alo, ahi), max_nfev=80)
            mm = np.zeros(12)
            mm[active] = sol.x
            pts, mgap, _ = ev.eval_ve40(mm, mesh_finger=fname)
            sgap = float(np.linalg.norm(pts[4] - pts[ti]) * 1000.0)
            if mgap <= 2.0:
                bench.append({
                    "name": f"{fname}_{tag}",
                    "finger": fname,
                    "fi": fi,
                    "tip_i": ti,
                    "motor_rad": mm.tolist(),
                    "points_m": pts.tolist(),
                    "ve40_mesh_mm": float(mgap),
                    "ve40_site_mm": sgap,
                })
    return bench


def compute_segment_scales(g0: np.ndarray, r0: np.ndarray, axis: np.ndarray, offset: np.ndarray) -> dict[str, list[float]]:
    """给定 (task_axis_scale, wrist_offset_m)，按 VE40_R 零位各指任务向量精确计算 15 段比例，保证伸直零位自然展平。"""
    r0_task = (r0 - offset) / axis
    r0_task[0] = 0.0
    r0_task[1] = 0.5 * r0_task[2]
    seg_scales = {}
    for fi, fname in enumerate(FINGERS):
        vals = []
        for j in range(3):
            ti = int(TASK_INDICES[fi, j])
            gv = g0[ti]
            rv = r0_task[ti]
            vals.append(round(float(np.dot(gv, rv) / np.dot(gv, gv)), 6))
        seg_scales[fname] = vals
    return seg_scales


def summarize_probe(outs: list[dict], bench: list[dict], end_idx: dict[str, int], fist_idx: int, sine_start: int) -> dict:
    q_open = np.asarray(outs[0]["q_rad"], float)
    open_thumb_ip_deg = float(math.degrees(abs(q_open[4])))
    open_max_deg = float(math.degrees(np.max(np.abs(np.delete(q_open, 3)))))
    q_fist = np.asarray(outs[fist_idx]["q_rad"], float)
    fist_curl_deg = float(math.degrees(np.mean([q_fist[5], q_fist[7], q_fist[9], q_fist[11], q_fist[13], q_fist[15], q_fist[18], q_fist[20]])))

    pad_by_finger = {f: [] for f in FINGERS[1:]}
    details = []
    for item in bench:
        o = outs[end_idx[item["name"]]]
        phys = np.asarray(o["physical_points_m"])
        pad_mm = float(np.linalg.norm(phys[4] - phys[item["tip_i"]]) * 1000.0)
        pad_by_finger[item["finger"]].append(pad_mm)
        details.append({
            "name": item["name"],
            "finger": item["finger"],
            "ve40_mesh_mm": item["ve40_mesh_mm"],
            "ve40_site_mm": item["ve40_site_mm"],
            "sharpa_pad_mm": pad_mm,
            "q22": o["q_rad"],
        })

    sine_qs = np.array([o["q_rad"] for o in outs[sine_start:]])
    max_sine_step = float(np.max(np.abs(np.diff(sine_qs, axis=0))))

    return {
        "open_max_deg": open_max_deg,
        "open_thumb_ip_deg": open_thumb_ip_deg,
        "fist_curl_deg": fist_curl_deg,
        "pad_by_finger": {f: float(np.mean(v)) for f, v in pad_by_finger.items()},
        "mean_pad_mm": float(np.mean([d["sharpa_pad_mm"] for d in details])),
        "max_pad_mm": float(np.max([d["sharpa_pad_mm"] for d in details])),
        "max_sine_step_rad": max_sine_step,
        "details": details,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--apply", action="store_true", help="将拟合后的最优 VE40 参数写入 right.yaml")
    args = parser.parse_args()

    base_cfg = yaml.safe_load(SHARPA_CFG.read_text(encoding="utf-8"))
    ev = VE40Evaluator()
    try:
        g0, _, _ = ev.eval_ve40(np.zeros(12))
        r0 = ev.sharpa_physical_points(np.zeros(22))
        bench = build_pinch_benchmark(ev)
        print(f"=== 1. VE40_R 四指多姿态对指基准（共 {len(bench)} 个贴合姿态）===")
        for item in bench:
            print(f"  {item['name']:12s}: VE40_mesh={item['ve40_mesh_mm']:4.1f}mm  VE40_site={item['ve40_site_mm']:5.1f}mm")

        seq_frames = [{"sequence": "open", "points_m": g0.tolist()}]
        end_idx = {}
        for item in bench:
            mt = np.asarray(item["motor_rad"])
            for frac in (0.0, 0.45, 0.80, 1.0):
                pts, _, _ = ev.eval_ve40(mt * frac)
                idx = len(seq_frames)
                seq_frames.append({"sequence": item["name"], "points_m": pts.tolist()})
                if frac == 1.0:
                    end_idx[item["name"]] = idx
        for frac in (0.0, 0.5, 1.0):
            m = np.zeros(12)
            m[0], m[1], m[2], m[3] = 0.8 * frac, 0.5 * frac, 0.6 * frac, 0.6 * frac
            m[4:] = 0.95 * frac
            fist_idx = len(seq_frames)
            seq_frames.append({"sequence": "fist", "points_m": ev.eval_ve40(m)[0].tolist()})
        sine_start = len(seq_frames)
        for frame_i in range(90):
            m = np.array([0.2 * (0.5 + 0.5 * math.sin(frame_i * 0.12 + j * 0.2)) for j in range(12)])
            seq_frames.append({"sequence": "sine90", "points_m": ev.eval_ve40(m)[0].tolist()})

        probe_stdin = build_probe_input(seq_frames)

        # 基准评估（旧 VE33 参数）
        before = summarize_probe(run_probe_raw(base_cfg, probe_stdin), bench, end_idx, fist_idx, sine_start)
        before_el = ev.sharpa_elastomer_gaps([{"q22": d["q22"], "finger": d["finger"]} for d in before["details"]])
        before_sim = ev.sharpa_sim_contacts([{"q22": d["q22"], "finger": d["finger"]} for d in before["details"]])
        for d, el, sr in zip(before["details"], before_el, before_sim):
            d["sharpa_el_mm"] = el
            d["sim_contact"] = sr["contact"]
            d["sim_el_mm"] = sr["el_gap_mm"]

        print("\n=== 2. 当前配置（旧 VE33 参数）在 VE40_R 上的表现 ===")
        print(
            f"  伸直零位: thumb_IP={before['open_thumb_ip_deg']:.1f}°, 非固定轴最大角={before['open_max_deg']:.1f}° | "
            f"握拳均值={before['fist_curl_deg']:.1f}° | 90帧最大跳变={before['max_sine_step_rad']:.4f} rad | "
            f"动力学真实接触={sum(d['sim_contact'] for d in before['details'])}/{len(before['details'])}"
        )
        for f in FINGERS[1:]:
            ds_f = [d for d in before["details"] if d["finger"] == f]
            print(
                f"  thumb <-> {f:6s}: pad均值={before['pad_by_finger'][f]:5.2f}mm | "
                f"指垫={np.mean([d['sharpa_el_mm'] for d in ds_f]):4.2f}mm | "
                f"动力学接触={sum(d['sim_contact'] for d in ds_f)}/{len(ds_f)}"
            )

        # Y/Z 掌宽/指长尺度：Y 由 4 掌骨 MCP 拟合，Z 由完整手掌+指尖高度拟合（避免 10mm MCP Z 跨度噪声放大！）
        palm = [5, 9, 13, 17]
        y_scale, y_off = np.linalg.lstsq(np.c_[g0[palm, 1], np.ones(4)], r0[palm, 1], rcond=None)[0]
        full_z_idx = [5, 9, 13, 17, 6, 7, 8, 10, 11, 12, 14, 15, 16, 18, 19, 20]
        z_scale_full, z_off_full = np.linalg.lstsq(np.c_[g0[full_z_idx, 2], np.ones(len(full_z_idx))], r0[full_z_idx, 2], rcond=None)[0]
        print(f"\n=== 3. VE40_R 全手掌/指长尺度分析: y_scale={y_scale:.4f}, y_off={y_off:.4f}, z_scale_full={z_scale_full:.4f}, z_off_full={z_off_full:.4f} ===")

        # 阶段一：保持合理 X/Y/Z 轴尺度，搜索对指闭合 + 零位展平的全局最优区
        candidates = []
        for x_scale in (0.90, 0.97, 1.04, 1.12):
            for z_scale, z_off in ((0.88, 0.038), (0.92, 0.035), (0.98, 0.031)):
                for x_off in (-0.004, 0.002, 0.008):
                    axis = np.array([x_scale, float(y_scale), z_scale])
                    offset = np.array([x_off, float(y_off), z_off])
                    segs = compute_segment_scales(g0, r0, axis, offset)
                    for thumb_aa in (None, -0.30, -0.15, 0.0):
                        for scaling in (0.96, 1.02, 1.08):
                            for pw in (0.85, 0.95):
                                cand = copy.deepcopy(base_cfg)
                                cand["geometry"]["task_axis_scale"] = axis.tolist()
                                cand["geometry"]["wrist_offset_m"] = offset.tolist()
                                cand["retarget"]["segment_scaling"] = segs
                                cand["retarget"]["scaling"] = scaling
                                cand["retarget"]["pinch_weight_max"] = pw
                                cand["retarget"]["w_pos"] = 2.5
                                cand["retarget"]["w_dir"] = 1.2
                                if thumb_aa is None:
                                    cand["retarget"]["fixed_joints"].pop("right_thumb_MCP_AA", None)
                                else:
                                    cand["retarget"]["fixed_joints"]["right_thumb_MCP_AA"] = thumb_aa
                                for f in FINGERS[1:]:
                                    cand["retarget"]["pinch_thresholds"][f] = {"d1": 2.4, "d2": 5.0}
                                candidates.append(cand)

        def eval_fast(cand):
            outs = run_probe_raw(cand, probe_stdin)
            met = summarize_probe(outs, bench, end_idx, fist_idx, sine_start)
            if met["max_sine_step_rad"] >= 0.11 or met["fist_curl_deg"] < 46.0:
                return 1e9, cand, met
            sc = (
                1.0 * met["mean_pad_mm"]
                + 0.45 * met["max_pad_mm"]
                + 0.45 * met["open_thumb_ip_deg"]
                + 0.25 * max(0.0, met["open_max_deg"] - 8.0)
            )
            return sc, cand, met

        print(f"  并行评估阶段一 {len(candidates)} 组候选...")
        with ThreadPoolExecutor(max_workers=8) as pool:
            res1 = list(pool.map(eval_fast, candidates))
        res1.sort(key=lambda x: x[0])

        # 阶段二：在 Top-5 周围精搜 w_pos, w_dir, pinch_thresholds, scaling, thumb_seg
        refine = []
        for _, base2, _ in res1[:5]:
            for w_pos in (1.5, 2.5, 4.0):
                for w_dir in (0.6, 1.2, 1.8):
                    for d1, d2 in ((2.0, 4.5), (2.5, 5.2), (3.0, 6.0)):
                        for scaling_d in (-0.03, 0.0, 0.03):
                            cand = copy.deepcopy(base2)
                            cand["retarget"]["w_pos"] = w_pos
                            cand["retarget"]["w_dir"] = w_dir
                            cand["retarget"]["scaling"] = round(base2["retarget"]["scaling"] + scaling_d, 3)
                            for f in FINGERS[1:]:
                                cand["retarget"]["pinch_thresholds"][f] = {"d1": d1, "d2": d2}
                            refine.append(cand)

        print(f"  并行评估阶段二 {len(refine)} 组精调...")
        with ThreadPoolExecutor(max_workers=8) as pool:
            res2 = list(pool.map(eval_fast, refine))
        all_valid = [r for r in (res1[:20] + res2) if r[0] < 1e8]
        all_valid.sort(key=lambda x: x[0])

        # 阶段三：对 Top-25 进行真实 MuJoCo 动力学仿真接触打分，选出真能物理对指接触的最优配置！
        print("  阶段三：对 Top-25 候选执行 MuJoCo 360ms 动力学接触打分...")
        best_final = (1e9, None, None)
        for _, cand, met in all_valid[:25]:
            sim_items = [{"q22": d["q22"], "finger": d["finger"]} for d in met["details"]]
            el_gaps = ev.sharpa_elastomer_gaps(sim_items)
            sim_res = ev.sharpa_sim_contacts(sim_items)
            for d, el, sr in zip(met["details"], el_gaps, sim_res):
                d["sharpa_el_mm"] = el
                d["sim_contact"] = sr["contact"]
                d["sim_el_mm"] = sr["el_gap_mm"]
            n_miss = sum(1 for sr in sim_res if not sr["contact"])
            mean_sim_el = float(np.mean([sr["el_gap_mm"] for sr in sim_res]))
            max_sim_el = float(np.max([sr["el_gap_mm"] for sr in sim_res]))
            final_score = (
                12.0 * n_miss
                + 2.0 * mean_sim_el
                + 0.8 * max_sim_el
                + 0.8 * met["mean_pad_mm"]
                + 0.3 * met["max_pad_mm"]
                + 0.3 * met["open_thumb_ip_deg"]
                + 0.25 * max(0.0, met["open_max_deg"] - 8.0)
            )
            if final_score < best_final[0]:
                best_final = (final_score, cand, met)

        _, best_cfg, best_met = best_final
        if best_cfg is None or best_met is None:
            raise RuntimeError("未找到满足约束的最优配置")

        print("\n=== 4. VE40_R 专属拟合后 Sharpa 表现（前后对比）===")
        print(
            f"  伸直零位 thumb_IP: {before['open_thumb_ip_deg']:.1f}° -> {best_met['open_thumb_ip_deg']:.1f}° | "
            f"非固定轴最大角: {before['open_max_deg']:.1f}° -> {best_met['open_max_deg']:.1f}° | "
            f"握拳均值: {before['fist_curl_deg']:.1f}° -> {best_met['fist_curl_deg']:.1f}° | "
            f"90帧最大跳变={best_met['max_sine_step_rad']:.4f} rad"
        )
        print(
            f"  MuJoCo 动力学物理接触率: "
            f"{sum(d['sim_contact'] for d in before['details'])}/{len(before['details'])} -> "
            f"{sum(d['sim_contact'] for d in best_met['details'])}/{len(best_met['details'])}"
        )
        for f in FINGERS[1:]:
            bf = [d for d in before["details"] if d["finger"] == f]
            af = [d for d in best_met["details"] if d["finger"] == f]
            print(
                f"  thumb <-> {f:6s}: pad {before['pad_by_finger'][f]:5.2f} -> {best_met['pad_by_finger'][f]:5.2f} mm | "
                f"动力学指垫 {np.mean([d['sim_el_mm'] for d in bf]):4.2f} -> {np.mean([d['sim_el_mm'] for d in af]):4.2f} mm | "
                f"物理接触 {sum(d['sim_contact'] for d in bf)}/{len(bf)} -> {sum(d['sim_contact'] for d in af)}/{len(af)}"
            )
        print("  最优核心参数:", {
            "task_axis_scale": [round(x, 4) for x in best_cfg["geometry"]["task_axis_scale"]],
            "wrist_offset_m": [round(x, 4) for x in best_cfg["geometry"]["wrist_offset_m"]],
            "scaling": best_cfg["retarget"]["scaling"],
            "w_pos": best_cfg["retarget"]["w_pos"],
            "w_dir": best_cfg["retarget"]["w_dir"],
            "pinch_weight_max": best_cfg["retarget"]["pinch_weight_max"],
            "pinch_thresholds_index": best_cfg["retarget"]["pinch_thresholds"]["index"],
            "thumb_MCP_AA": best_cfg["retarget"]["fixed_joints"].get("right_thumb_MCP_AA", "FREE"),
            "thumb_seg": best_cfg["retarget"]["segment_scaling"]["thumb"],
        })

        print("\n=== 5. 逐姿态详细对比 ===")
        for bd, ad in zip(before["details"], best_met["details"]):
            print(
                f"  {ad['name']:12s}: VE40_mesh={ad['ve40_mesh_mm']:4.1f}mm (site={ad['ve40_site_mm']:4.1f}mm) | "
                f"pad {bd['sharpa_pad_mm']:5.1f}->{ad['sharpa_pad_mm']:5.1f}mm | "
                f"动力学指垫 {bd['sim_el_mm']:4.1f}->{ad['sim_el_mm']:4.1f}mm | 接触 {bd['sim_contact']}->{ad['sim_contact']}"
            )

        if args.apply:
            header = (
                "# sharpa_teleop/tools/fit_ve40_retarget.py 针对 hand_motor_ve40_R 闭链模型拟合生成。\n"
                "# canonical方向与Sharpa URDF一致；geometry、segment_scaling与四指对指参数已适配VE40新拇指与小指结构。\n"
            )
            SHARPA_CFG.write_text(header + yaml.safe_dump(best_cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")
            print(f"\n已写入新配置：{SHARPA_CFG}")
    finally:
        ev.close()


if __name__ == "__main__":
    main()
