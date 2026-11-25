#!/usr/bin/env python3
"""针对 hand_motor_16dof_R 闭链模型拟合 16-DOF 专属 Sharpa 右手重定向参数（释放全部 22 个 Sharpa 自由度）。"""
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
HAND16_DIR = ROOT / "assets/hand_motor_16dof_R"
BASE_CFG = SHARPA_ROOT / "config/retargeting/sharpa/right.yaml"
OUT_CFG = SHARPA_ROOT / "config/retargeting/sharpa/right_16dof.yaml"
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
TASK_INDICES = np.array([[4 * f + 2 + j for j in range(3)] for f in range(5)])
# Sharpa 22-DOF indices for four-finger MCP_AA:
# 0..4 thumb, 5 index_FE, 6 index_AA, 7 index_PIP, 8 index_DIP,
# 9 middle_FE, 10 middle_AA, 11 middle_PIP, 12 middle_DIP,
# 13 ring_FE, 14 ring_AA, 15 ring_PIP, 16 ring_DIP,
# 17 pinky_CMC, 18 pinky_FE, 19 pinky_AA, 20 pinky_PIP, 21 pinky_DIP
SHARPA_AA_IDX = (6, 10, 14, 19)

sys.path.insert(0, str(ROOT / "tools"))
from motorhand.hand_profiles import get_hand_runtime


WORKER_CODE = r"""
import json, math, sys, numpy as np, mujoco, pinocchio as pin
from pathlib import Path

root = Path(sys.argv[1])
sharpa_root = root / "sharpa_teleop"
h16_urdf = sharpa_root / "assets/hand_motor_16dof/hand_motor_16dof_R/hand_tree_only.urdf"
rt_path = sharpa_root / "assets/hand_motor_16dof/hand_motor_16dof_R/config/retarget_targets.json"
pad_urdf = sharpa_root / "assets/sharpa-hand/wave_01/right_sharpa_wave/right_sharpa_wave_pad.urdf"
sharpa_xml = sharpa_root / "assets/sharpa-hand/right.xml"

m16 = pin.buildModelFromUrdf(str(h16_urdf))
d16 = m16.createData()
rt16 = json.loads(rt_path.read_text(encoding="utf-8"))
R16 = np.asarray(rt16["canonical_to_model"], float)
jnames = json.loads(sys.stdin.readline())
j_idx = [(m16.joints[m16.getJointId(n)].idx_q, m16.joints[m16.getJointId(n)].nq) for n in jnames]
site_meta = [
    (m16.getFrameId(s["link"]), np.asarray(s["local_xyz_m"], float))
    for s in sorted(rt16["sites"], key=lambda x: x["index"])
]

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

q0 = pin.neutral(m16)
pin.framesForwardKinematics(m16, d16, q0)
zpts = np.array([d16.oMf[fid].translation + d16.oMf[fid].rotation @ loc for fid, loc in site_meta])
zref = np.array([s["zero_world_xyz_m"] for s in sorted(rt16["sites"], key=lambda x: x["index"])])
assert float(np.max(np.linalg.norm(zpts - zref, axis=1)) * 1000.0) < 1e-3

sys.stdout.write("READY\n")
sys.stdout.flush()

for line in sys.stdin:
    req = json.loads(line)
    cmd = req.get("cmd", "h16")
    if cmd == "h16":
        q = pin.neutral(m16)
        for (idx, nq), val in zip(j_idx, req["tree"]):
            if nq == 2:
                q[idx] = math.cos(val)
                q[idx + 1] = math.sin(val)
            else:
                q[idx] = val
        pin.framesForwardKinematics(m16, d16, q)
        pts = np.array([d16.oMf[fid].translation + d16.oMf[fid].rotation @ loc for fid, loc in site_meta])
        pts[1] = 0.5 * (pts[0] + pts[2])
        canon = (pts - pts[0]) @ R16
        sys.stdout.write(json.dumps({"points_m": canon.tolist()}) + "\n")
    elif cmd == "sharpa_fk":
        q22 = np.clip(np.asarray(req["q22"], float), s_lo, s_hi)
        q = np.zeros(22)
        q[s_jidx] = q22
        pin.framesForwardKinematics(sm, sd, q)
        pts = np.array([sd.oMf[f].translation for f in s_fids])
        pts -= pts[0]
        pts[1] = 0.5 * pts[2]
        sys.stdout.write(json.dumps({"physical_points_m": pts.tolist()}) + "\n")
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


class Hand16Evaluator:
    def __init__(self):
        rt = get_hand_runtime("16dof")
        self.mapping = rt.load_mapping()
        self.kin = rt.create_kinematics(self.mapping)
        self.proc = subprocess.Popen(
            [str(PIXI_PY), "-c", WORKER_CODE, str(ROOT)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )
        self.proc.stdin.write(json.dumps(list(self.kin.joint_names)) + "\n")
        self.proc.stdin.flush()
        ready = self.proc.stdout.readline().strip()
        if ready != "READY":
            raise RuntimeError(f"Hand16 worker failed: {ready}")
        lo, hi = np.array([m.model_rad_limits() for m in self.mapping.motors]).T
        self.lo = lo
        self.hi = hi

    def eval_16dof(self, motors_16):
        tree = list(self.kin.native.solve([float(x) for x in motors_16], 0))
        self.proc.stdin.write(json.dumps({"cmd": "h16", "tree": tree}) + "\n")
        self.proc.stdin.flush()
        res = json.loads(self.proc.stdout.readline())
        return np.asarray(res["points_m"], float)

    def sharpa_physical_points(self, q22):
        self.proc.stdin.write(json.dumps({"cmd": "sharpa_fk", "q22": list(map(float, q22))}) + "\n")
        self.proc.stdin.flush()
        return np.asarray(json.loads(self.proc.stdout.readline())["physical_points_m"], float)

    def sharpa_sim_contacts(self, items):
        self.proc.stdin.write(json.dumps({"cmd": "sharpa_sim_contact", "items": items}) + "\n")
        self.proc.stdin.flush()
        return json.loads(self.proc.stdout.readline())["results"]

    def close(self):
        if self.proc.poll() is None:
            self.proc.stdin.close()
            self.proc.wait(timeout=5)


def build_probe_input(seq_frames: list[dict]) -> str:
    lines = []
    prev_seq = None
    for row in seq_frames:
        item = {"points_m": row["points_m"]}
        if row["sequence"] != prev_seq:
            item["reset"] = True
            prev_seq = row["sequence"]
        lines.append(json.dumps(item))
    return "\n".join(lines) + "\n"


def run_probe_raw(cfg: dict, probe_stdin: str) -> list[dict]:
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", dir=str(SHARPA_ROOT / "config/retargeting/sharpa"), delete=False) as tf:
        yaml.safe_dump(cfg, tf, sort_keys=False, allow_unicode=True)
        cfg_path = Path(tf.name)
    try:
        res = subprocess.run(
            [str(PROBE_BIN), "--hand", "right", "--config", str(cfg_path), "--root", str(SHARPA_ROOT)],
            input=probe_stdin, text=True, capture_output=True, check=True,
        )
        outs = [json.loads(line) for line in res.stdout.splitlines() if line.strip()]
        if any("error" in o for o in outs):
            raise RuntimeError(f"probe error: {[o for o in outs if 'error' in o][:2]}")
        return outs
    finally:
        cfg_path.unlink(missing_ok=True)


def compute_segment_scales(g0: np.ndarray, r0: np.ndarray, axis: np.ndarray, offset: np.ndarray) -> dict[str, list[float]]:
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


def build_pinch_benchmark(ev: Hand16Evaluator) -> list[dict]:
    """构建 16-DOF 右手四指真实对指基准（网格搜索拇指初值 + 最小二乘精确贴合）。"""
    thumb_grid = []
    for m0 in np.linspace(max(ev.lo[0], -0.4), min(ev.hi[0], 1.55), 11):
        for m1 in np.linspace(max(ev.lo[1], -1.6), min(ev.hi[1], 1.65), 13):
            for m2 in (0.0, 0.45, 0.9, 1.3):
                for m3 in (0.0, 0.5, 1.0):
                    m = np.zeros(16)
                    m[0], m[1], m[2], m[3] = m0, m1, m2, m3
                    try:
                        pts = ev.eval_16dof(m)
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
            m = np.zeros(16)
            m[4 + 2 * fi] = fmcp
            m[5 + 2 * fi] = fpip
            try:
                fpts = ev.eval_16dof(m)
            except Exception:
                continue
            ft = fpts[ti] * 1000.0
            bi = int(np.argmin(np.linalg.norm(t_arr - ft, axis=1)))
            tm = thumb_grid[bi][1]
            active = np.array([0, 1, 2, 3, 4 + 2 * fi, 5 + 2 * fi])
            alo = ev.lo[active].copy()
            ahi = ev.hi[active].copy()
            alo[4], ahi[4] = max(alo[4], fmcp - 0.25), min(ahi[4], fmcp + 0.25)
            alo[5], ahi[5] = max(alo[5], fpip - 0.25), min(ahi[5], fpip + 0.25)
            x0 = np.clip(tm + [fmcp, fpip], alo + 0.01, ahi - 0.01)

            def resid(x):
                mm = np.zeros(16)
                mm[active] = x
                try:
                    pts = ev.eval_16dof(mm)
                    return (pts[4] - pts[ti]) * 1000.0
                except Exception:
                    return np.full(3, 1e4)

            sol = least_squares(resid, x0, bounds=(alo, ahi), max_nfev=80)
            mm = np.zeros(16)
            mm[active] = sol.x
            pts = ev.eval_16dof(mm)
            gap_mm = float(np.linalg.norm(pts[4] - pts[ti]) * 1000.0)
            if gap_mm <= 16.0:
                bench.append({
                    "name": f"{fname}_{tag}",
                    "finger": fname,
                    "tip_i": ti,
                    "motor_rad": mm.tolist(),
                    "glove_site_mm": gap_mm,
                })
    return bench


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--apply", action="store_true", help="将拟合结果写入 right_16dof.yaml")
    args = parser.parse_args()

    base_cfg = yaml.safe_load(BASE_CFG.read_text(encoding="utf-8"))
    base_cfg["retarget"]["fixed_joints"] = {}
    base_cfg["retarget"]["allow_free_finger_aa"] = True

    ev = Hand16Evaluator()
    try:
        g0 = ev.eval_16dof(np.zeros(16))
        r0 = ev.sharpa_physical_points(np.zeros(22))
        bench = build_pinch_benchmark(ev)
        print("=== 1. 16-DOF 右手四指对指基准 ===")
        for b in bench:
            print(f"  {b['name']:12s}: glove_site_gap={b['glove_site_mm']:5.2f} mm")

        seq_frames = [{"sequence": "open", "points_m": g0.tolist()}]
        end_idx = {}
        for b in bench:
            mt = np.asarray(b["motor_rad"])
            for frac in (0.0, 0.45, 0.80, 1.0):
                idx = len(seq_frames)
                seq_frames.append({"sequence": b["name"], "points_m": ev.eval_16dof(mt * frac).tolist()})
                if frac == 1.0:
                    end_idx[b["name"]] = idx

        # 四指侧摆测试（正/负侧摆）
        abd_idx = {}
        for sign_name, sgn in (("abd_pos", 1.0), ("abd_neg", -1.0)):
            m = np.zeros(16)
            m[12:16] = sgn * np.array([0.25, 0.20, 0.20, 0.25])
            abd_idx[sign_name] = len(seq_frames)
            seq_frames.append({"sequence": sign_name, "points_m": ev.eval_16dof(m).tolist()})

        # 90 帧连续正弦测试（同时包含屈伸与 13–16 侧摆）
        sine_start = len(seq_frames)
        for fi in range(90):
            m = np.zeros(16)
            for j in range(12):
                m[j] = 0.22 * (0.5 + 0.5 * math.sin(fi * 0.12 + j * 0.2))
            for j in range(12, 16):
                m[j] = 0.22 * math.sin(fi * 0.12 + (j - 12) * 0.5)
            seq_frames.append({"sequence": "sine90", "points_m": ev.eval_16dof(m).tolist()})

        probe_stdin = build_probe_input(seq_frames)

        palm = [5, 9, 13, 17]
        y_scale, y_off = np.linalg.lstsq(np.c_[g0[palm, 1], np.ones(4)], r0[palm, 1], rcond=None)[0]
        print(f"=== 2. 16-DOF 掌宽线性尺度: y_scale={y_scale:.4f}, y_off={y_off:.4f} ===")

        candidates = []
        for x_scale in (0.98, 1.04, 1.10):
            for z_scale, z_off in ((0.90, 0.036), (0.94, 0.034)):
                for x_off in (-0.005, 0.0, 0.005):
                    axis = np.array([x_scale, float(y_scale), z_scale])
                    offset = np.array([x_off, float(y_off), z_off])
                    segs = compute_segment_scales(g0, r0, axis, offset)
                    for scaling in (0.98, 1.02, 1.06):
                        cfg = copy.deepcopy(base_cfg)
                        cfg["geometry"]["task_axis_scale"] = axis.tolist()
                        cfg["geometry"]["wrist_offset_m"] = offset.tolist()
                        cfg["retarget"]["scaling"] = scaling
                        cfg["retarget"]["segment_scaling"] = segs
                        cfg["retarget"]["w_tip_scale"] = 4.0
                        cfg["retarget"]["pinch_close_gain"] = 1.0
                        cfg["retarget"]["thumb_offset_cm"] = [0.0, 0.0, 0.55]
                        candidates.append(cfg)

        def eval_cfg(cfg):
            outs = run_probe_raw(cfg, probe_stdin)
            q_open = np.asarray(outs[0]["q_rad"], float)
            open_rms_deg = float(math.degrees(np.sqrt(np.mean(q_open ** 2))))
            open_max_deg = float(math.degrees(np.max(np.abs(q_open))))
            open_aa_max_deg = float(math.degrees(np.max(np.abs(q_open[list(SHARPA_AA_IDX)]))))
            pad_gaps = []
            for b in bench:
                phys = np.asarray(outs[end_idx[b["name"]]]["physical_points_m"])
                pad_gaps.append(float(np.linalg.norm(phys[4] - phys[b["tip_i"]]) * 1000.0))
            q_pos = np.asarray(outs[abd_idx["abd_pos"]]["q_rad"], float)[list(SHARPA_AA_IDX)]
            q_neg = np.asarray(outs[abd_idx["abd_neg"]]["q_rad"], float)[list(SHARPA_AA_IDX)]
            aa_span_deg = float(math.degrees(np.mean(np.abs(q_pos - q_neg))))
            sine_qs = np.array([o["q_rad"] for o in outs[sine_start:]])
            max_step = float(np.max(np.abs(np.diff(sine_qs, axis=0))))
            score = (
                2.5 * float(np.mean(pad_gaps))
                + 1.5 * float(np.max(pad_gaps))
                + 1.2 * open_max_deg
                + 2.0 * open_rms_deg
                + 1.5 * open_aa_max_deg
                + max(0.0, 12.0 - aa_span_deg) * 1.5
                + max(0.0, max_step - 0.12) * 200.0
            )
            return {
                "score": score, "cfg": cfg, "outs": outs,
                "open_rms_deg": open_rms_deg, "open_max_deg": open_max_deg, "open_aa_max_deg": open_aa_max_deg,
                "pad_gaps": pad_gaps, "aa_span_deg": aa_span_deg, "max_step": max_step,
                "q_pos_aa_deg": np.degrees(q_pos).tolist(), "q_neg_aa_deg": np.degrees(q_neg).tolist(),
            }

        with ThreadPoolExecutor(max_workers=6) as ex:
            results = list(ex.map(eval_cfg, candidates))
        results.sort(key=lambda r: r["score"])
        best = results[0]

        # 阶段二：精细微调 thumb_offset_cm、w_tip_scale、pinch_close_gain
        fine = []
        for tz in (0.40, 0.55, 0.70, 0.85):
            for ty in (-0.25, -0.10, 0.05):
                for tx in (-0.20, 0.0, 0.20):
                    for w_tip in (3.0, 4.0, 5.0):
                        cfg = copy.deepcopy(best["cfg"])
                        cfg["retarget"]["thumb_offset_cm"] = [tx, ty, tz]
                        cfg["retarget"]["w_tip_scale"] = w_tip
                        cfg["retarget"]["pinch_close_gain"] = 1.0
                        fine.append(cfg)

        with ThreadPoolExecutor(max_workers=6) as ex:
            fine_results = list(ex.map(eval_cfg, fine))
        fine_results.sort(key=lambda r: r["score"])
        best = fine_results[0]

        # MuJoCo 动力学真实接触验收
        sim_items = [{"finger": b["finger"], "q22": best["outs"][end_idx[b["name"]]]["q_rad"]} for b in bench]
        sim_res = ev.sharpa_sim_contacts(sim_items)
        n_contact = sum(1 for r in sim_res if r["contact"])

        print("\n=== 3. 16-DOF 全 22 轴自由 Sharpa 最优拟合结果 ===")
        print(f"  伸直零位: RMS={best['open_rms_deg']:.2f}°, max={best['open_max_deg']:.2f}°, 四指AA最大={best['open_aa_max_deg']:.2f}°")
        print(f"  四指侧摆响应 (J13..J16 ±0.20..0.25 rad): 平均张合跨度={best['aa_span_deg']:.2f}°")
        print(f"    正侧摆 AA(deg): {[round(x, 2) for x in best['q_pos_aa_deg']]}")
        print(f"    负侧摆 AA(deg): {[round(x, 2) for x in best['q_neg_aa_deg']]}")
        print(f"  90帧混合屈伸+侧摆最大跳变: {best['max_step']:.4f} rad")
        print(f"  四指对指 MuJoCo 动力学接触: {n_contact}/{len(sim_res)}")
        for b, gap, sr in zip(bench, best["pad_gaps"], sim_res):
            print(f"    {b['name']:12s}: pad_gap={gap:5.2f} mm | sim_el={sr['el_gap_mm']:4.2f} mm | contact={sr['contact']}")

        if args.apply:
            header = (
                "# sharpa_teleop/tools/fit_16dof_retarget.py 针对 hand_motor_16dof_R 闭链模型联合拟合生成。\n"
                "# 16-DOF 模式释放全部 22 个 Sharpa 自由度（含四指 MCP_AA 侧摆），不修改 12-DOF right.yaml。\n"
            )
            OUT_CFG.write_text(header + yaml.safe_dump(best["cfg"], sort_keys=False, allow_unicode=True), encoding="utf-8")
            print(f"\n已写入 16-DOF 重定向配置: {OUT_CFG}")
    finally:
        ev.close()


if __name__ == "__main__":
    main()
