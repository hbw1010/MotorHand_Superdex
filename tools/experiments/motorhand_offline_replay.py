#!/usr/bin/env python3
"""通过正常 demo 入口记录固定腕部抓握回放；无需任何手套或串口。"""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
from urllib.error import URLError
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=['identity', 'nominal'], default='identity')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    base = f'http://127.0.0.1:{port}/api/v1/'
    def api(path, payload=None):
        req = Request(base + path, json.dumps(payload).encode() if payload is not None else None,
                      {'Content-Type': 'application/json'})
        with urlopen(req, timeout=5) as response:
            return json.load(response) if payload is None else None
    command = [sys.executable, str(ROOT / 'tools/ve33_g4_view.py'), '--local-grasp', '--demo', '--quest-port', str(port)]
    # 旧入口默认 identity，保持旧版也能运行。
    if args.profile != 'identity':
        command += ['--feedback-profile', args.profile]
    with tempfile.TemporaryDirectory(prefix='superdex-offline-') as state, open(args.output / 'server.log', 'w') as log:
        proc = subprocess.Popen(command, cwd=ROOT, env={**os.environ, 'XDG_STATE_HOME': state}, stdout=log, stderr=log)
        try:
            deadline = time.monotonic() + 120
            while True:
                if proc.poll() is not None:
                    raise RuntimeError('演示进程退出，详见 server.log')
                try:
                    status = api('control')
                    if status.get('grasp') and status.get('online') == 12:
                        break
                except (URLError, TimeoutError):
                    pass
                if time.monotonic() > deadline:
                    raise TimeoutError('演示进程启动超时')
                time.sleep(.1)
            metadata = {'profile': args.profile, 'command': command, 'backend': status.get('grasp_backend'),
                        'commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                        'feedback_config': json.loads((ROOT / 'hardware/encos_g4_gateway_12dof/config/right_sim_feedback_12dof.json').read_text()),
                        'physics_step_s': 1 / 240, 'max_substeps': 4, 'control_hz': 100,
                        'precision': 'float32', 'size_mm': 89, 'locked': True, 'layout': 'palm',
                        'timeline': '每 0.2 秒变更 grip 0.05，合拢到 0.8，保持 2 秒，再对称松开',
                        'timing': '实时调度，非固定仿真时间；时间戳为本机 monotonic_ns'}
            (args.output / 'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
            for label, kind, hardness in (('box_hard', 'box', 1.), ('ball_hard', 'ball', 1.),
                                          ('ball_soft', 'ball', .25)):
                api('control', {'action': 'demo_grip', 'value': 0})
                api('control', {'action': 'configure_grasp', 'values': {'kind': kind, 'hardness': hardness,
                                                                       'size_mm': 89, 'locked': True, 'layout': 'palm'}})
                api('control', {'action': 'reset_scene'})
                until = time.monotonic() + 30
                while True:
                    scene = api('scene')
                    obj = scene.get('grasp_object') or {}
                    if obj.get('kind') == kind and obj.get('hardness') == hardness and obj.get('size_mm') == 89:
                        break
                    if time.monotonic() > until:
                        raise TimeoutError(f'{kind} 场景未就绪')
                    time.sleep(.1)
                time.sleep(.5)
                events = [(i / 20, 'close') for i in range(17)] + [(0.8, 'hold')] * 10 + [(i / 20, 'open') for i in range(15, -1, -1)] + [(0, 'released')] * 5
                with (args.output / (label + '.jsonl')).open('w') as out:
                    for grip, phase in events:
                        started = time.monotonic()
                        api('control', {'action': 'demo_grip', 'value': grip})
                        for _ in range(2):
                            sample = {'monotonic_ns': time.monotonic_ns(), 'phase': phase, 'grip': grip,
                                      'control': api('control'), 'scene': api('scene')}
                            out.write(json.dumps(sample, ensure_ascii=False, allow_nan=False) + '\n')
                            remaining = started + (_ + 1) * .1 - time.monotonic()
                            if remaining > 0:
                                time.sleep(remaining)
                print(kind, '已记录', flush=True)
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()


if __name__ == '__main__':
    main()
