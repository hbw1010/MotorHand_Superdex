"""通过本地无串口演示测量抓握保持、张手期间的力帧新鲜度。"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
from urllib.error import URLError
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[2]
# 选用容易出现接触帧过期的软鸭灯以及用于反馈标定的硬方块。
CASES = {"duck": ("official_duck_lamp", 90., .25, .65), "box": ("box", 89., 1., .8),
         "cube": ("official_puzzle_cube", 73., 1., .8)}


def request(port, path="control", **command):
    body = json.dumps(command).encode() if command else None
    url = f"http://127.0.0.1:{port}/api/v1/{path}"
    req = Request(url, body, {"Content-Type": "application/json"})
    with urlopen(req, timeout=3) as response:
        return json.load(response) if body is None else None


def wait_control(proc, port, predicate, timeout=60):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if proc.poll() is not None:
            raise RuntimeError(f"演示提前退出，状态 {proc.returncode}")
        try:
            state = request(port)
            if predicate(state):
                return state
        except (URLError, TimeoutError):
            pass
        time.sleep(.1)
    raise TimeoutError("控制台状态等待超时")


def quantiles(values):
    if not values:
        return None
    x = sorted(values)
    return {name: round(x[math.ceil(len(x) * p) - 1], 4) for name, p in
            (("p50", .5), ("p95", .95), ("p99", .99), ("max", 1.))}


def collect(proc, port, state_dir, seconds):
    request(port, action="start_recording")
    wait_control(proc, port, lambda s: s.get("recording"))
    time.sleep(seconds)
    request(port, action="stop_recording")
    d = wait_control(proc, port, lambda s: s.get("recording_file") and not s.get("recording"))
    path = state_dir / "superdex/recordings" / d["recording_file"]
    with path.open(encoding="utf-8-sig") as handle:
        all_rows = list(csv.DictReader(handle))
    rows = [r for r in all_rows if r["id"] == "5"]
    if not rows:
        raise RuntimeError("没有 J5 控制采样")
    ages = [float(r["contact_ms"]) for r in rows if r["contact_ms"]]
    frames = sorted({int(r["contact_frame_ns"]) for r in rows if r["contact_frame_ns"]})
    gaps = [(b - a) / 1e6 for a, b in zip(frames, frames[1:])]
    return {"cycles": len(rows), "unavailable_cycles": sum(not r["contact_frame_ns"] for r in rows),
            "expired_cycles": sum(a >= 100. for a in ages), "distinct_frames": len(frames),
            "age_ms": quantiles(ages), "frame_gap_ms": quantiles(gaps),
            "j5_abs_nm": quantiles([abs(float(r["nominal_nm"])) for r in rows if r["nominal_nm"]]),
            "all_axes_peak_nm": max((abs(float(r["nominal_nm"])) for r in all_rows if r["nominal_nm"]), default=None),
            "record_drops": d["recording_dropped"],
            "output_rows": sum(bool(r["sent_motor_nm"]) for r in all_rows)}


def run(root, case, seconds):
    kind, size, hardness, grip = CASES[case]
    with tempfile.TemporaryDirectory(prefix="ve33-link-bench-") as temp:
        state_dir = Path(temp)
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        args = [sys.executable, str(root / "tools/ve33_g4_view.py"), "--local-grasp", "--demo",
                "--feedback-profile", "nominal-plus", "--quest-port", str(port)]
        log = state_dir / "server.log"
        env = {**os.environ, "XDG_STATE_HOME": temp}
        with log.open("w") as out:
            proc = subprocess.Popen(args, cwd=root, env=env, stdout=out, stderr=out, start_new_session=True)
            try:
                wait_control(proc, port, lambda s: s.get("demo") and s.get("online") == 12)
                request(port, action="demo_grip", value=0)
                request(port, action="configure_grasp", values={"kind": kind, "size_mm": size,
                        "hardness": hardness, "layout": "palm", "locked": True, "angle_deg": 0})
                wait_control(proc, port, lambda s: s.get("grasp", {}).get("kind") == kind and s.get("contact_frame_ns"))
                for i in range(1, round(grip / .05) + 1):
                    request(port, action="demo_grip", value=i * .05)
                    time.sleep(.1)
                time.sleep(1.)
                held = collect(proc, port, state_dir, seconds)
                request(port, action="demo_grip", value=0)
                time.sleep(.7)
                released = collect(proc, port, state_dir, 1.)
                return dict(case=case, seconds=seconds, held=held, released=released)
            except Exception as exc:
                raise RuntimeError(f"离线场景 {case} 失败：{log.read_text()[-1800:]}") from exc
            finally:
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGINT)
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT, help="要测量的已构建工作副本")
    parser.add_argument("--case", choices=tuple(CASES), action="append", help="不指定时依次测量两个物体")
    parser.add_argument("--seconds", type=float, default=8., help="每个物体的握持采样秒数")
    options = parser.parse_args()
    if not math.isfinite(options.seconds) or options.seconds <= 0:
        parser.error("采样时长须为有限正值")
    for case in options.case or ("duck", "box"):
        print(json.dumps(run(options.root.resolve(), case, options.seconds), ensure_ascii=False), flush=True)
