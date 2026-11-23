"""用本进程创建的伪串口验证 G4 正常入口、真实 SDK 和硬方块碰撞反馈。"""
import binascii
import csv
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import tty

from motorhand_native_link_benchmark import request, wait_control, quantiles

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from motorhand.feedback import MAPPING
from motorhand.mapping import GloveMapping
from ve33_g4_view import mapping_identity
from ve33_local_grasp import DemoGlove


class SimulatedGateway:
    def __init__(self, master, motors):
        self.master = master
        self.motors = motors
        self.positions = self.packets(0.)
        self.wire = []
        self.queries = []
        self.done = threading.Event()
        self.failure = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def packets(self, grip):
        return self.pose_packets([grip * travel for travel in DemoGlove.GRIP])

    def pose_packets(self, encoder_rad):
        """编码当前限位前角度；本模拟器使用独立的全零标定。"""
        result = bytearray()
        for motor, angle in zip(self.motors, encoder_rad, strict=True):
            axis = motor.motor_id
            frame = bytearray(32)
            frame[:4] = bytes.fromhex('a55a0190')
            struct.pack_into('<BBH', frame, 6, 3 - (axis-1)//4, 6, axis)
            degrees = math.degrees(angle / (motor.sign * motor.scale))
            struct.pack_into('>BBf', frame, 16, 0xa0, 1, degrees)
            struct.pack_into('<H', frame, 30, binascii.crc_hqx(frame[:30], 65535))
            result += frame
        return bytes(result)

    def grip(self, start, end):
        for i in range(1, 17):
            self.positions = self.packets(start+(end-start)*i/16)
            time.sleep(.08)

    def run(self):
        data = bytearray()
        last = 0
        try:
            while not self.done.is_set():
                now = time.monotonic_ns()
                if now-last >= 5_000_000:
                    try:
                        os.write(self.master, self.positions)
                    except BlockingIOError:
                        pass
                    last = now
                if select.select([self.master], [], [], .001)[0]:
                    data.extend(os.read(self.master, 65536))
                    while len(data) >= 32:
                        f = data[:32]
                        del data[:32]
                        if f[:4] != bytes.fromhex('a55a0102') or binascii.crc_hqx(f[:30], 65535) != int.from_bytes(f[30:], 'little'):
                            raise ValueError('伪网关收到无效 G4 帧')
                        mid = int.from_bytes(f[8:10], 'little')
                        if f[7] == 2:
                            self.queries.append((time.monotonic_ns(), mid))
                        elif f[7] == 8:
                            bits = int.from_bytes(f[16:24], 'big')
                            if bits >> 49:
                                raise ValueError('本地抓握发送了非零位置增益')
                            kd = ((bits >> 40) & 511) * 5. / 511
                            if kd > .3:
                                raise ValueError(f'本地抓握发送了超限阻尼增益 {kd}')
                            self.wire.append((time.monotonic_ns(), mid, (bits & 4095)*40/4095-20))
        except BaseException as error:
            self.failure = error

    def close(self):
        self.done.set()
        self.thread.join()
        if self.failure:
            raise self.failure


def verify(directory):
    motors = GloveMapping.load(MAPPING).motors
    axes = {m.motor_id: (m.model_joint, m) for m in motors}
    calibration = directory / 'right-zero.json'
    calibration.write_text(json.dumps({'hand': 'right', 'axes': mapping_identity(axes),
                                       'zero_deg': {m.motor_id: 0. for m in motors}}))
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    tty.setraw(slave)
    sim = SimulatedGateway(master, motors)
    try:
        with (directory / 'app.log').open('w') as logfile:
            proc = subprocess.Popen([sys.executable, str(ROOT/'tools/ve33_g4_view.py'), '--local-grasp',
                                     '--port', os.ttyname(slave), '--preview', '--zero-file', str(calibration),
                                     '--feedback-profile', 'nominal-plus', '--quest-port', str(port)], cwd=ROOT,
                                    env={**os.environ, 'XDG_STATE_HOME': str(directory),
                                         'XDG_RUNTIME_DIR': str(directory)},
                                    stdout=logfile, stderr=logfile, start_new_session=True)
            try:
                wait_control(proc, port, lambda s: s.get('online')==12 and s.get('calibrated') and not s['demo'])
                # 先让输入角变化但不接触物体，测正常开合时的跟随误差。
                request(port, action='configure_grasp', values={'kind':'box', 'size_mm':20.,
                        'hardness':1.,'locked':True, 'layout':'desk'})
                wait_control(proc, port, lambda s: s.get('grasp',{}).get('layout')=='desk' and s.get('contact_frame_ns'))
                request(port, action='start_recording')
                wait_control(proc, port, lambda s:s.get('recording'))
                sim.grip(0., .8)
                sim.grip(.8, 0.)
                request(port, action='stop_recording')
                panel = wait_control(proc, port, lambda s:s.get('recording_file') and not s['recording'])
                with (directory/'superdex/recordings'/panel['recording_file']).open(encoding='utf-8-sig') as f:
                    motion = list(csv.DictReader(f))
                gap = [abs(float(r['sim_minus_input_rad'])) for r in motion if r['sim_minus_input_rad']]
                assert gap and len(gap) >= .95*len(motion), '自由开合时有效样本覆盖不足'
                assert not any(r['contact_gate']=='True' or abs(float(r['position_motor_nm'] or 0)) > .01
                               or abs(float(r['mapped_contact_nm'] or 0)) > .01 for r in motion), '自由开合出现虚假接触阻力'
                free_nom = [abs(float(r['nominal_nm'] or 0)) for r in motion]
                assert free_nom and max(free_nom) <= .48 + 1e-6 and max(free_nom) > .1, '自由开合摩擦补偿异常'
                # 自由运动时摩擦补偿只允许顺向助力（方向与速度一致，不得反向阻挡）。
                assert all(float(r['nominal_nm'] or 0) * float(r['motor_rad_s'] or 0) >= -1e-3
                           for r in motion if abs(float(r['motor_rad_s'] or 0)) > .1), '自由开合出现反向阻力'
                gap_by_tick = {}
                for r in motion:
                    if r['sim_minus_input_rad']:
                        t = r['monotonic_ns']
                        gap_by_tick[t] = max(gap_by_tick.get(t, 0.), abs(float(r['sim_minus_input_rad'])))
                free_motion = quantiles(list(gap_by_tick.values()))
                request(port, action='configure_grasp', values={'kind':'box','size_mm':89.,
                        'hardness':1.,'locked':True, 'layout':'palm'})
                wait_control(proc, port, lambda s: s.get('grasp',{}).get('layout')=='palm' and s.get('contact_frame_ns'))
                sim.grip(0., .8)
                wait_control(proc, port, lambda s: abs(s['rows'][4].get('nominal_nm') or 0.) > 6.)
                assert not sim.wire, '预览阶段出现 MIT 帧'
                request(port, action='start_recording')
                wait_control(proc, port, lambda s:s.get('recording'))
                enabled_at = time.monotonic_ns()
                request(port, action='enable')
                wait_control(proc, port, lambda s:s['active'] and abs(s['rows'][4].get('sent_motor_nm') or 0.)>6.)
                hold_start = time.monotonic_ns()
                time.sleep(2.)
                hold_end = time.monotonic_ns()
                sim.grip(.8, 0.)
                open_start = time.monotonic_ns()
                time.sleep(.5)
                open_end = time.monotonic_ns()
                sim.grip(0., .8)
                wait_control(proc, port, lambda s:s['active'] and abs(s['rows'][4].get('sent_motor_nm') or 0.) > 6.)
                request(port, action='disable')
                disabled_at=time.monotonic_ns()
                time.sleep(.22)
                wire_end=time.monotonic_ns()
                request(port, action='stop_recording')
                state = wait_control(proc, port, lambda s:s.get('recording_file') and not s['recording'])
                with (directory/'superdex/recordings'/state['recording_file']).open(encoding='utf-8-sig') as f:
                    all_rows = list(csv.DictReader(f))
                rows = [r for r in all_rows if r['id']=='5']
                times=[int(r['monotonic_ns']) for r in rows]
                wire=[(t,mid,v) for t,mid,v in sim.wire if enabled_at<=t<wire_end]
                assert wire and wire[0][0]-enabled_at>=180_000_000, '连续有效期不足 180 ms 已提交'
                held=[abs(v) for t,j,v in wire if j==5 and hold_start<=t<hold_end]
                released=[abs(v) for t,j,v in wire if open_start+200_000_000<=t<open_end]
                after=[abs(v) for t,j,v in wire if t>=disabled_at+30_000_000]
                assert held and min(held)>5., '稳态硬方块未维持食指反馈'
                assert released and max(released)<.01, '张手后仍输出力矩'
                assert not after or max(after)<.01, '明确关闭后仍发送非零力矩'
                grip_rows = [r for r,t in zip(rows,times) if hold_start <= t < hold_end]
                loaded = [r for r in grip_rows if r['contact_gate']=='True' and r['sim_minus_input_rad']]
                assert loaded and len(loaded) >= .95*len(grip_rows), '保持窗口无足够同帧受阻数据'
                assert all(float(r['sim_minus_input_rad']) < 0 and float(r['position_motor_nm']) < 0
                           and float(r['nominal_nm']) < 0 for r in loaded), 'J5 受阻方向与反馈不一致'
                open_rows = [r for r in all_rows if open_start+200_000_000 <= int(r['monotonic_ns']) < open_end]
                free = [r for r in open_rows if r['sim_minus_input_rad'] and r['nominal_nm']]
                assert free and len(free) >= .95*len(open_rows), '张手窗口缺少有效数据'
                free_error = max(abs(float(r['sim_minus_input_rad'])) for r in free)
                assert free_error < .01 and all(abs(float(r['nominal_nm'])) < .01 for r in free), '张手后仿真手未跟上或有残留反馈'
                # 硬方块松开期间，首次收到无反力的有效控制更新必须已经归零。
                unloaded = next(r for r,t in zip(rows,times) if hold_end <= t < open_end and r['contact_gate']=='False')
                assert float(unloaded['nominal_nm']) == 0. and float(unloaded['sent_motor_nm']) == 0.
                query_axes = {j for t,j in sim.queries if disabled_at+30_000_000 <= t < wire_end}
                assert query_axes == set(range(1,13)), '关闭出力后网关没有继续查询 12 轴'
                return {'control_samples':len(rows), 'missed_recording_samples':state['recording_dropped'],
                        'free_motion_max_axis_gap_rad':free_motion,
                        'hold_j5_control_samples':sum(hold_start<=t<hold_end for t in times),
                        'hold_j5_wire_samples':len(held), 'hold_j5_abs_nm':quantiles(held),
                        'hold_j5_abs_gap_rad':quantiles([abs(float(r['sim_minus_input_rad'])) for r in loaded]),
                        'hold_j5_abs_position_nm':quantiles([abs(float(r['position_motor_nm'])) for r in loaded]),
                        'free_hand_max_gap_rad':free_error,
                        'first_mit_ms':round((wire[0][0]-enabled_at)/1e6,3),
                        'released_wire_samples':len(released),'release_max_abs_nm':round(max(released),4),
                        'feedback_interruptions':state['feedback_interruptions'],'handoffs':state['feedback_handoffs'],
                        'post_disable_nonzero_frames':sum(v>.01 for v in after),
                        'post_disable_query_axes':len(query_axes)}
            finally:
                if proc.poll() is None:
                    os.killpg(proc.pid,signal.SIGINT)
                    try:
                        proc.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()
    finally:
        try:
            sim.close()
        finally:
            os.close(master)
            os.close(slave)


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='motorhand-pty-') as tmp:
        print(json.dumps(verify(Path(tmp)),ensure_ascii=False,indent=2))
