#!/usr/bin/env python3
"""比较不同物理工作线程数的无硬件抓握帧龄；不会打开串口。"""
import csv
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[2]
ENTRY = ROOT / 'tools/ve33_g4_view.py'
WORKER_EXPRESSION = 'workers = min(6, max(0, len(os.sched_getaffinity(0)) - 2)) if local_grasp else 0'


def request(base, path, payload=None):
    data = json.dumps(payload).encode() if payload is not None else None
    req = Request(base + path, data, {'Content-Type': 'application/json'})
    with urlopen(req, timeout=2) as response:
        return json.load(response) if payload is None else response.status


def quantiles(values):
    values = sorted(values)
    if not values:
        raise RuntimeError('没有可用帧龄或物理帧间隔')
    return [round(values[int(q * (len(values) - 1))], 2) for q in (.5, .95, .99, 1)]


def run(workers, source):
    with tempfile.TemporaryDirectory(prefix='worker-comparison-') as tmpdir:
        tmp = Path(tmpdir)
        script = tmp / 've33_bench.py'
        script.write_text(source.replace(WORKER_EXPRESSION, f'workers = {workers} if local_grasp else 0'))
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            port = sock.getsockname()[1]
        base = f'http://127.0.0.1:{port}/api/v1/'
        env = {**os.environ, 'PYTHONPATH': str(ROOT / 'tools'), 'XDG_STATE_HOME': str(tmp),
               'OPENBLAS_NUM_THREADS': '1'}
        with (tmp / 'server.log').open('w') as log:
            proc = subprocess.Popen([str(ROOT / '.venv/bin/python'), str(script), '--local-grasp', '--demo',
                                     '--quest-port', str(port)], cwd=ROOT, env=env, stdout=log, stderr=log,
                                    start_new_session=True)
            try:
                deadline = time.monotonic() + 90
                while True:
                    if proc.poll() is not None:
                        raise RuntimeError('demo 退出，日志：' + (tmp / 'server.log').read_text()[-1000:])
                    try:
                        status = request(base, 'control')
                        if status.get('online') == 12 and status.get('grasp'):
                            break
                    except (OSError, ValueError):
                        pass
                    if time.monotonic() > deadline:
                        raise TimeoutError('demo 启动超时')
                    time.sleep(.2)
                request(base, 'control', {'action': 'configure_grasp', 'values': {
                    'kind': 'ball', 'size_mm': 90, 'hardness': .99, 'layout': 'palm', 'locked': True}})
                request(base, 'control', {'action': 'demo_grip', 'value': .8})
                time.sleep(5)
                print(f'开始 {workers} 线程组', flush=True)
                request(base, 'control', {'action': 'start_recording'})
                time.sleep(8)
                request(base, 'control', {'action': 'stop_recording'})
                time.sleep(.5)
                status = request(base, 'control')
                name = status.get('recording_file')
                if not name:
                    raise RuntimeError(f'录制失败：{status.get("recording_error")}')
                with (tmp / 'superdex/recordings' / name).open(encoding='utf-8-sig') as file:
                    rows = [row for row in csv.DictReader(file) if row['id'] == '1']
                ages = [float(row['contact_ms']) for row in rows if row['contact_ms']]
                frames = sorted({int(row['contact_frame_ns']) for row in rows if row['contact_frame_ns']})
                gaps = [(b - a) / 1e6 for a, b in zip(frames, frames[1:])]
                print(json.dumps({'workers': workers, 'samples': len(rows), 'frames': len(frames),
                                  'age_ms_p50_p95_p99_max': quantiles(ages),
                                  'frame_gap_ms_p50_p95_p99_max': quantiles(gaps),
                                  'dropped': status.get('recording_dropped'), 'grasp': status.get('grasp')},
                                 ensure_ascii=False), flush=True)
            finally:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    proc.wait()


def main():
    source = ENTRY.read_text()
    if source.count(WORKER_EXPRESSION) != 1:
        raise RuntimeError('工作线程初始化方式已变化，无法可靠应用测试配置')
    for workers in (0, 2, 6):
        run(workers, source)


if __name__ == '__main__':
    main()
