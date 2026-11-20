#!/usr/bin/env python3
"""以相同的已录物理样本回放两种反馈并绘制合成触发曲线。"""
import gzip
import json
import sys
from pathlib import Path
from unittest.mock import Mock
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from motorhand.feedback import ContactForceFeedbackLoop, MAPPING
from motorhand.mapping import GloveMapping

DATA = ROOT / 'docs/motorhand-offline'
axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}

def instance(profile):
    return ContactForceFeedbackLoop(Mock(), axes, profile=profile)

def replay(kind):
    with gzip.open(DATA / f'identity-{kind}.jsonl.gz', 'rt') as f:
        samples = [json.loads(line) for line in f]
    loops = [instance('identity'), instance('nominal')]
    rows = []
    for sample in samples:
        a = sample['control']['rows']
        keys = ['input_rad', 'motor_rad_s', 'contact_motor_nm', 'sim_rad', 'contact_input_rad', 'contact_encoder_rad', 'encoder_rad']
        if any(row.get(k) is None for row in a for k in keys):
            continue
        get = lambda key: np.array([r[key] for r in a], dtype=float)
        q, vel, reaction, sim, target, enc, live = map(get, keys)
        frame = [1, 1, True, *reaction, *np.zeros(12), *sim, *target, *enc]
        result = []
        for loop in loops:
            for _ in range(10):
                cmds, _ = loop.plan(q, vel, frame, 1., .01, 0, encoder_position=live)
            result.append([c.tau_ff for c in cmds])
        rows.append({'time': sample['monotonic_ns'] / 1e9, 'phase': sample['phase'], 'grip': sample['grip'],
                     'reaction': reaction.tolist(), 'contact_input_rad': target.tolist(), 'identity': result[0], 'nominal': result[1],
                     'links': sample['scene'].get('hand_cube_contacts', {}),
                     'vertices': sample['scene'].get('grasp_object', {}).get('vertices')})
    return rows, len(samples)

def synthetic():
    result = {}
    for event in ('zero', 'reverse', 'retreat'):
        loop = instance('nominal'); out = []
        for i in range(130):
            q = np.full(12, .8); sim = q.copy(); sim[4] = .71
            reaction = np.zeros(12); reaction[4] = -.28
            enc = q.copy()
            if i == 100:
                if event == 'zero': reaction[4] = 0
                if event == 'reverse': reaction[4] = .28
                if event == 'retreat': enc[4] -= .06
            frame = [i+1, i+1, True, *reaction, *np.zeros(12), *sim, *q, *q]
            cmds, _ = loop.plan(q, np.zeros(12), frame, 1., .01, 0, encoder_position=enc)
            out.append([i * .01, cmds[4].tau_ff, reaction[4], float(loop.frame_weight[4])])
        result[event] = out
    return result

def main():
    summary = {}
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    for kind in ('box_hard', 'ball_hard', 'ball_soft'):
        rows, total = replay(kind)
        if not rows: raise RuntimeError(f'{kind}: 无可回放的完整帧')
        start = rows[0]['time']; t = [r['time']-start for r in rows]
        if kind == 'box_hard':
            axs[0,0].plot(t, [r['identity'][4] for r in rows], label='identity')
            axs[0,0].plot(t, [r['nominal'][4] for r in rows], label='nominal')
        contact = next((r['contact_input_rad'][4] for r in rows if abs(r['reaction'][4]) >= .01), None)
        if contact is None: raise RuntimeError(f'{kind}: J5 无有效接触')
        axs[0,1].plot([max(0., r['contact_input_rad'][4]-contact) for r in rows if r['phase'] in ('close', 'hold')],
                      [abs(r['nominal'][4]) for r in rows if r['phase'] in ('close', 'hold')], label=kind)
        hold = [r for r in rows if r['phase'] == 'hold'][-10:]
        summary[kind] = {'accepted_frames': len(rows), 'total_frames': total, 'hold_frames': len(hold),
                         'links': sorted(set(link for r in hold for ls in r['links'].values() for link in ls)),
                         'axis': []}
        for i in range(12):
            result = {'axis': i+1}
            for profile in ('identity', 'nominal'):
                values = np.array([abs(r[profile][i]) for r in hold])
                result[profile] = {'median_nm': float(np.median(values)), 'p95_minus_p5_nm': float(np.percentile(values,95)-np.percentile(values,5)),
                                   'over_95_percent_limit': float(np.mean(values >= 9.5*.95)),
                                   'contact_fraction': float(np.mean([abs(r['reaction'][i]) >= .01 for r in hold]))}
            summary[kind]['axis'].append(result)
    events = synthetic(); (DATA / 'synthetic-events.json').write_text(json.dumps(events, indent=2))
    for event, data in events.items():
        a = np.array(data); axs[1,0].plot(a[:,0]-.99, a[:,1], label=event)
        axs[1,1].plot(a[:,0]-.99, a[:,3], label=event)
    axs[0,0].set(xlabel='Time (s)', ylabel='J5 nominal torque (N m)', title='Same sampled physical inputs')
    axs[0,1].set(xlabel='Extra closure after first J5 contact (rad)', ylabel='Absolute J5 torque (N m)', title='Closure and nominal torque')
    axs[1,0].set(xlabel='Time relative to event (s)', ylabel='J5 torque (N m)', title='Synthetic 100 Hz triggers')
    axs[1,1].set(xlabel='Time relative to event (s)', ylabel='Frame weight', title='Retreat weight')
    for ax in axs.flat: ax.grid(alpha=.3); ax.legend()
    fig.tight_layout(); fig.savefig(DATA / 'comparison.png', dpi=150)
    (DATA / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')

if __name__ == '__main__': main()
