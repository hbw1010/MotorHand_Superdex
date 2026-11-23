"""用伪串口回放单场景的 12 轴编码器记录，再检查持续握持和松开。"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import pty
import signal
import socket
import subprocess
import sys
import tempfile
import time
import tty

from motorhand_native_link_benchmark import quantiles, request, wait_control
from motorhand_native_pty_acceptance import (ROOT, MAPPING, DemoGlove, GloveMapping,
                                            SimulatedGateway, mapping_identity)


def read_trajectory(path):
    trajectory = {}
    settings = None
    profile = None
    with path.open(encoding='utf-8-sig') as f:
        for r in csv.DictReader(f):
            config = dict(kind=r['kind'], size_mm=float(r['size_mm']), hardness=float(r['hardness']),
                          layout=r['layout'])
            name = r['feedback_profile']
            if settings is None:
                profile, settings = name, config
            if settings != config or profile != name:
                raise ValueError('需选用物体、尺寸、硬度、布局和反馈档位均不变的录制')
            t, mid = int(r['monotonic_ns']), int(r['id'])
            axes = trajectory.setdefault(t, {})
            if mid in axes or not 1 <= mid <= 12:
                raise ValueError('同一采样的轴重复或编号超出 1–12')
            angle = float(r['encoder_rad']) if r['encoder_rad'] else None
            axes[mid] = angle if angle is not None and math.isfinite(angle) else None
    if not trajectory:
        raise ValueError('录制没有控制周期')
    first = min(trajectory)
    return settings, profile, [((t-first)/1e9, [axes.get(mid) for mid in range(1, 13)])
                               for t, axes in sorted(trajectory.items())]


def wire_stats(wire, start, end):
    per_axis = [[] for _ in range(12)]
    for t, mid, v in wire:
        if start <= t <= end:
            per_axis[mid-1].append((t, v))
    # 超过四个 200 Hz 总线节拍的相邻 MIT 包分开报告，避免把提交空档内的未知输出当作零。
    jumps, gaps = [], 0
    for samples in per_axis:
        for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
            if t1-t0 <= 20_000_000:
                jumps.append(abs(v1-v0))
            else:
                gaps += 1
    return dict(wire_peaks=[round(max((abs(v) for _, v in axis)),4) if axis else None for axis in per_axis],
                wire_counts=[len(axis) for axis in per_axis], wire_short_gap_jumps_nm=quantiles(jumps),
                wire_gaps_over_20ms=gaps, wire_short_gap_jumps_above_1nm=sum(j > 1. for j in jumps))


def run(root, trace, settings, profile, temp, hand_first):
    first_pose = next((pose for _, pose in trace if all(x is not None for x in pose)), None)
    last_pose = next((pose for _, pose in reversed(trace) if all(x is not None for x in pose)), None)
    if first_pose is None:
        raise ValueError('没有完整的 12 轴姿态可供放置物体')

    motors = GloveMapping.load(MAPPING).motors
    zero = temp/'right-zero.json'
    zero.write_text(json.dumps({'hand': 'right', 'axes': mapping_identity({m.motor_id:(m.model_joint,m) for m in motors}),
                                'zero_deg': {m.motor_id:0. for m in motors}}))
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    sim = SimulatedGateway(master, motors)
    def pose_packet(pose):
        # SDK 各轴保留各自上次位置戳；没有采样的轴不给伪网关发送位置报文。
        packed = sim.pose_packets([x if x is not None else 0. for x in pose])
        return b''.join(packed[i*32:(i+1)*32] for i,x in enumerate(pose) if x is not None)

    packets = [(offset, pose_packet(pose)) for offset, pose in trace]
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    phases = {}
    def transition(start, end):
        for k in range(1, 21):
            sim.positions = sim.pose_packets([a+(b-a)*k/20 for a,b in zip(start,end)])
            time.sleep(.05)
    try:
        with (temp/'app.log').open('w') as log:
            proc = subprocess.Popen([sys.executable, str(root/'tools/ve33_g4_view.py'), '--local-grasp',
                                     '--port', os.ttyname(slave), '--preview', '--zero-file', str(zero),
                                     '--feedback-profile', profile, '--quest-port', str(port)], cwd=root,
                                    env={**os.environ,'XDG_RUNTIME_DIR':str(temp),'XDG_STATE_HOME':str(temp)},
                                    stdout=log,stderr=log,start_new_session=True)
            try:
                wait_control(proc,port,lambda s:s.get('online')==12 and s.get('calibrated'))
                if hand_first:
                    transition([0.]*12, first_pose)
                    time.sleep(.7)
                request(port,action='configure_grasp',values={**settings, 'locked':True, 'angle_deg':0})
                wait_control(proc,port,lambda s:all(s.get('grasp',{}).get(k)==v for k,v in settings.items()) and s.get('contact_frame_ns'))
                request(port,action='enable')
                wait_control(proc,port,lambda s:s.get('active'))
                if not hand_first:
                    transition([0.]*12, first_pose)
                time.sleep(.7)
                request(port,action='start_recording')
                wait_control(proc,port,lambda s:s.get('recording'))
                started = time.monotonic_ns()
                for offset,packet in packets:
                    wait = offset-(time.monotonic_ns()-started)/1e9
                    if wait>0:
                        time.sleep(wait)
                    sim.positions = packet
                phases['recorded'] = (started,time.monotonic_ns())
                closed=[.8*v for v in DemoGlove.GRIP]
                transition(last_pose, [0.]*12)
                time.sleep(.7)
                transition([0.]*12, closed)
                time.sleep(.4)
                start=time.monotonic_ns()
                time.sleep(2.)
                phases['hold'] = (start,time.monotonic_ns())
                transition(closed, [0.]*12)
                time.sleep(.7)
                start=time.monotonic_ns()
                time.sleep(.8)
                phases['released']=(start,time.monotonic_ns())
                request(port,action='disable')
                stop=time.monotonic_ns()
                time.sleep(.25)
                request(port,action='stop_recording')
                state=wait_control(proc,port,lambda s:s.get('recording_file') and not s.get('recording'))
                with (temp/'superdex/recordings'/state['recording_file']).open(encoding='utf-8-sig') as f:
                    rows=list(csv.DictReader(f))
                result={'settings':settings,'profile':profile,'hand_first':hand_first,'source_cycles':len(trace),'source_seconds':trace[-1][0],
                        'dropped':state['recording_dropped'],'reconnects':state['reconnect_count'],
                        'post_disable_nonzero':sum(t>stop+30_000_000 and abs(v)>.01 for t,_,v in sim.wire),
                        'post_disable_query_axes':len({mid for t,mid in sim.queries if t>stop+30_000_000})}
                for name,(start,ends) in phases.items():
                    phase=[r for r in rows if start<=int(r['monotonic_ns'])<=ends]
                    ticks=[r for r in phase if r['id']=='1']
                    ages=[float(r['contact_ms']) for r in ticks if r['contact_ms']]
                    active=[r['feedback_active']=='1' for r in ticks]
                    streak=[bool(r['contact_ms']) and float(r['contact_ms']) >= 100. for r in ticks]
                    if not ticks:
                        raise RuntimeError(f'{name} 阶段没有控制样本')
                    result[name]=dict(cycles=len(ticks),age_ms=quantiles(ages),expired=sum(streak),
                        expiry_episodes=sum(b and (i==0 or not streak[i-1]) for i,b in enumerate(streak)),
                        unavailable=sum(not r['contact_frame_ns'] for r in ticks),active=sum(active),
                        interruptions=int(ticks[-1]['feedback_interruptions'])-int(ticks[0]['feedback_interruptions']),
                        handoffs=int(ticks[-1]['feedback_handoffs'])-int(ticks[0]['feedback_handoffs']),
                        **wire_stats(sim.wire,start,ends))
                return result
            except Exception:
                print((temp/'app.log').read_text()[-1500:],file=sys.stderr)
                raise
            finally:
                if proc.poll() is None:
                    os.killpg(proc.pid,signal.SIGINT)
                    try: proc.wait(6)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid,signal.SIGKILL)
                        proc.wait()
    finally:
        try: sim.close()
        finally:
            os.close(master)
            os.close(slave)


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('input',type=Path,help='需包含同一场景的物体、尺寸、硬度、布局、反馈档位、编码器角与采样时间戳的 CSV')
    cli.add_argument('--root',type=Path,default=ROOT,help='待测工作副本')
    cli.add_argument('--hand-first',action='store_true',help='先重放首个完整手势再放置物体，复核不同初始接触状态')
    cli.add_argument('--hardness',type=float,help='按另一硬度回放同一动作，默认使用录制硬度')
    args=cli.parse_args()
    with tempfile.TemporaryDirectory(prefix='superdex-red-pty-') as directory:
        settings, profile, trajectory = read_trajectory(args.input)
        tested = settings.copy()
        if args.hardness is not None:
            if not 0 <= args.hardness <= 1:
                cli.error('--hardness 应在 0 到 1 之间')
            tested['hardness'] = args.hardness
        result = run(args.root.resolve(),trajectory,tested,profile,Path(directory),args.hand_first)
        print(json.dumps(dict(source_settings=settings, tested_settings=tested, **result),indent=2))
