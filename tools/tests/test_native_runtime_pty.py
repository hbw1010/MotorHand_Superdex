"""用真正的 SDK、伪串口和独立模拟进程检验控制线程及看门狗。"""
import binascii
import ctypes
import math
import multiprocessing as mp
from multiprocessing.reduction import DupFd
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import sys
import tempfile
import time
import tty
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motorhand.native_kinematics import NativeGloveKinematics
from motorhand.native_runtime import load_native
from motorhand.native_control import make_feedback_control
from ve33_g4_view import GloveMapping, MAPPING, MODEL_DIR


def position_packet(axis, degrees):
    frame = bytearray(32)
    frame[:4] = bytes.fromhex("a55a0190")
    struct.pack_into("<BBH", frame, 6, 3 - (axis - 1)//4, 6, axis)
    struct.pack_into(">BBf", frame, 16, 0xa0, 1, degrees)
    struct.pack_into("<H", frame, 30, binascii.crc_hqx(frame[:30], 65535))
    return frame


def simulate(fd, path, pipe):
    # 与观察线程处于不同进程；观察线程持有 GIL 时，模拟输入与串口读取继续推进。
    master = fd.detach()
    link = load_native().SharedLink(path)
    frames = bytearray()
    wire = []
    mode = "hold"
    started = time.monotonic_ns()
    original_input = 0
    last_reply = 0
    replies = b"".join(position_packet(i, 0.) for i in range(1, 13))
    retreat = b"".join(position_packet(i, -5. if i == 5 else 0.) for i in range(1, 13))
    try:
        pipe.send("ready")
        while True:
            if pipe.poll():
                command = pipe.recv()
                if command == "exit":
                    break
                if command == "wire":
                    pipe.send(wire)
                    wire = []
                else:
                    mode = command
                    started = time.monotonic_ns()
                    original_input = int(link.read_target()[0])
                    pipe.send(original_input)
            now = time.monotonic_ns()
            if now - last_reply >= 4_000_000:
                if mode != "no_rx":
                    os.write(master, retreat if mode == "retreat" else replies)
                last_reply = now
                target = link.read_target()
                if target[0] and mode != "external_physics":
                    reaction = [0.] * 12
                    reaction[4] = -.8
                    reaction[6] = -.9
                    if mode == "vary":
                        reaction[4] = -.55 - .35 * math.sin((now-started)*1e-9*math.tau*4.)
                    stamp = original_input if mode == "stale_input" else target[0]
                    # 回撤模式持续给出对应旧姿态的力帧，当前限位前编码器角已退 5 度。
                    zero = [0.] * 12
                    link.publish_force([stamp, now, 1., *reaction, *zero, *zero, *zero, *zero, 0.])
            if select.select([master], [], [], .001)[0]:
                frames.extend(os.read(master, 16384))
                while len(frames) >= 32:
                    packet = bytes(frames[:32])
                    del frames[:32]
                    if packet[:4] != bytes.fromhex("a55a0102"):
                        raise ValueError("串口发送的 G4 报文失步")
                    axis = int.from_bytes(packet[8:10],"little")
                    if axis in (5,7):
                        torque = None
                        if packet[7] == 8:
                            bits = int.from_bytes(packet[16:24], "big")
                            torque = (bits & 4095)*40./4095 - 20.
                        wire.append((time.monotonic_ns(), axis, torque))
    finally:
        os.close(master)
        pipe.close()


class NativeRuntimePtyTest(unittest.TestCase):
    def setUp(self):
        native = load_native()
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        self.kin = NativeGloveKinematics(MODEL_DIR,mapping)
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        path = str(Path(folder.name) / "hand.shm")
        self.link = native.SharedLink(path,len(self.kin.joint_names))
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        tty.setraw(slave)
        ctx = mp.get_context("spawn")
        self.pipe, remote = ctx.Pipe()
        self.addCleanup(self.pipe.close)
        self.worker = ctx.Process(target=simulate,args=(DupFd(master),path,remote),daemon=True)
        self.worker.start()
        remote.close()
        self.addCleanup(self.close_worker)
        self.assertTrue(self.pipe.poll(10), "伪网关未启动")
        self.assertEqual(self.pipe.recv(),"ready")
        self.controller = make_feedback_control(axes, native)
        ratios = [m.sign * m.scale for m in mapping.motors]
        lo, hi = zip(*(m.model_rad_limits() for m in mapping.motors))
        self.loop = native.NativeLoop(self.kin.native, self.controller, self.link, os.ttyname(slave),
                                      ratios, lo, hi, [0.] * 12, 10_000_000)
        self.addCleanup(self.loop.close)
        self.loop.start([0.] * 12)

    def close_worker(self):
        if self.worker.is_alive():
            self.pipe.send("exit")
            self.worker.join(2)
        if self.worker.is_alive():
            self.worker.terminate()
            self.worker.join(2)
        self.assertEqual(self.worker.exitcode, 0)

    def command(self, command):
        self.pipe.send(command)
        self.assertTrue(self.pipe.poll(2), "伪网关应答超时")
        return self.pipe.recv()

    def await_load(self):
        until = time.monotonic()+2
        while time.monotonic()<until:
            events = self.command("wire")
            if any(j == 5 and v is not None and v < -.7 for _,j,v in events):
                return
            time.sleep(.02)
        self.fail("未收到持续抓握的非零 MIT 指令")

    def test_new_frame_before_deadline_maintains_wire_load_between_control_ticks(self):
        # 物理求解较慢，下一帧在上一批命令到期前约 4 ms 完成；
        # 将数据发布时间对齐到两个常规控制 tick 之间。电机编码器仍正常应答。
        self.link.command(True, 0, 0.)
        self.await_load()
        zero = [0.] * 12
        loads = [-.8 if i == 4 else -.9 if i == 6 else 0. for i in range(12)]

        def wait_until(stamp):
            remaining = (stamp - time.monotonic_ns()) / 1e9
            if remaining > 0:
                time.sleep(remaining)

        def publish(source):
            solved = time.monotonic_ns()
            self.link.publish_force([source, solved, True, *loads, *zero, *zero, *zero, *zero, 0.])
            return solved

        # 同一受力姿态下重复三次，避免一次碰巧在常规 tick 上发布掩盖问题。
        for _ in range(3):
            self.command("external_physics")
            old = self.loop.samples(0, True)[0]
            grid = old["now"] - round(old["cycle_ms"] * 1e6)
            source = grid + 17_000_000
            deadline = source + 100_000_000
            wait_until(grid + 21_000_000)
            publish(source)
            wait_until(deadline - 4_000_000)
            new_source = int(self.link.read_target()[0])
            solved = publish(new_source)
            self.assertLess(solved, deadline, "模拟器未能在指定时间发布新帧")
            wait_until(deadline + 20_000_000)
            states = self.loop.samples(old["seq"], False)
            accepted = [s for s in states if s["feedback"].valid_sent]
            self.assertTrue(any(s["feedback"].submitted_until == deadline for s in accepted),
                            "未建立将要到期的测试批次")
            self.assertTrue(any(s["force"][1] == solved and
                                s["feedback"].submitted_until == new_source + 100_000_000 for s in accepted),
                            "续接必须按新帧的原输入期限提交")
            self.assertTrue(all(s["feedback"].active and s["feedback"].valid_sent and
                                s["feedback"].sent[4] < -.7 for s in states),
                            "已有新帧时不应因常规 tick 迟到而卸力")
            self.assertEqual(states[-1]["feedback"].handoff_count, old["feedback"].handoff_count)
            self.assertEqual(states[-1]["feedback"].interruption_count, old["feedback"].interruption_count)
            wire = [v for t,j,v in self.command("wire") if j == 5 and deadline-10_000_000 <= t <= deadline+20_000_000]
            self.assertGreaterEqual(len(wire), 3)
            self.assertTrue(all(v is not None and v < -.7 for v in wire), "线缆上出现卸力或意外零力矩")
            self.command("hold")
            time.sleep(.04)

    def test_control_survives_stalled_observer_and_releases_stale_inputs(self):
        self.link.command(False,0,0.)
        time.sleep(.22)
        self.assertTrue(all(v is None or abs(v)<.01 for _,_,v in self.command("wire")))
        self.link.command(True,0,0.)
        self.await_load()
        self.command("vary")
        begin = time.monotonic_ns()
        ctypes.PyDLL(None).usleep(300000)
        end = time.monotonic_ns()
        wire = [(t,round(v,3)) for t,j,v in self.command("wire")
                if begin <= t <= end and j == 5 and v is not None]
        completed = [s for s in self.loop.samples(0, False) if begin <= s["now"] <= end]
        self.assertGreaterEqual(len(completed), 22, "观察线程阻塞时控制周期过少")
        self.assertTrue(all(s["feedback"].valid_sent and s["feedback"].submitted_until > s["now"]
                            for s in completed), "观察线程阻塞期间提交中断")
        self.assertGreater(len(wire), 15)
        # 不只检查 SDK 重发：持续变化的力输入在持有 GIL 期间产生了多个不同 MIT 指令。
        self.assertGreater(len({v for _,v in wire}), 9)
        self.assertGreater(max(t for t,_ in wire)-min(t for t,_ in wire),200_000_000)
        self.command("retreat")
        time.sleep(.08)
        events = self.command("wire")
        recent = [(j,v) for t,j,v in events if t >= time.monotonic_ns()-40_000_000 and v is not None]
        self.assertTrue(any(j==5 and abs(v)<.01 for j,v in recent))
        self.assertTrue(any(j==7 and v<-.7 for j,v in recent))
        self.command("hold")
        self.await_load()
        original = self.command("stale_input")
        time.sleep(.20)
        latest = self.loop.samples(0, True)[-1]["feedback"]
        self.assertFalse(latest.active)
        self.assertEqual(latest.last_interruption, "仿真使用的编码器样本过期")
        events = self.command("wire")
        # 额外 20 ms 为 SDK 卸力和发包时间；之后只能读回位置请求或零指令。
        after = [(j,v) for t,j,v in events if t>=original+120_000_000]
        self.assertTrue(after)
        self.assertTrue(all(v is None or abs(v)<.01 for j,v in after))
        self.command("hold")
        # 物理数据恢复也不能自行恢复旧出力请求，必须由用户先停再启。
        time.sleep(.40)
        since = time.monotonic_ns() - 300_000_000
        held = [(t,v) for t,j,v in self.command("wire") if t>=since and j==5]
        self.assertTrue(held)
        self.assertTrue(all(v is None or abs(v)<.01 for _,v in held))
        self.assertFalse(self.loop.samples(0, True)[0]["feedback"].active)
        self.link.command(False,0,0.)
        time.sleep(.025)
        self.command("wire")
        enabled = time.monotonic_ns()
        self.link.command(True,0,0.)
        time.sleep(.12)
        self.assertTrue(all(v is None or abs(v)<.01 for _,_,v in self.command("wire")))
        self.await_load()
        self.assertGreaterEqual(time.monotonic_ns()-enabled, 180_000_000)
        self.link.command(True,1,0.)  # 接触生成仍为旧版本，必须卸力。
        time.sleep(.10)
        events = self.command("wire")
        now = time.monotonic_ns()
        after = [v for t,j,v in events if t >= now-50_000_000]
        self.assertTrue(after)
        self.assertTrue(all(v is None or abs(v)<.01 for v in after))

        self.link.command(True,0,0.)
        self.await_load()
        self.command("no_rx")
        time.sleep(.22)
        events = self.command("wire")
        now = time.monotonic_ns()
        after = [v for t,j,v in events if t >= now-70_000_000]
        self.assertTrue(after)
        self.assertTrue(all(v is None or abs(v)<.01 for v in after))


class SdkBatchDeadlineTest(unittest.TestCase):
    def test_sdk_rejects_expired_batch_and_late_replacement_on_wire(self):
        # 原生扩展不导出 SDK 符号；从同一份源码编译，避免测到旧 ctypes 动态库。
        gateway_dir = Path(__file__).resolve().parents[2] / 'hardware/encos_g4_gateway_sdk'
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        library = Path(folder.name) / 'g4.so'
        subprocess.run(['c++', '-O2', '-std=c++17', '-shared', '-fPIC', '-pthread',
                        '-I'+str(gateway_dir/'vendor/include'), str(gateway_dir/'shim/g4_shim.cpp'),
                        '-o', str(library)], check=True, timeout=60)
        sdk = ctypes.CDLL(str(library))
        sdk.g4_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint]
        sdk.g4_open.restype = ctypes.c_void_p
        sdk.g4_renew_mit_batch.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.c_int,
                                          ctypes.c_int64, ctypes.c_int64]
        sdk.g4_close.argtypes = [ctypes.c_void_p]
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        device = sdk.g4_open(os.fsencode(os.ttyname(slave)), 921600, 200)
        self.assertTrue(device)
        self.addCleanup(sdk.g4_close, device)
        batch = (ctypes.c_float*60)(*([0., 0., 0., 0., .8]*12))
        packets = bytearray()
        def collect(seconds):
            end = time.monotonic() + seconds
            complete = []
            while time.monotonic() < end:
                if select.select([master], [], [], .002)[0]:
                    packets.extend(os.read(master, 65536))
                while len(packets) >= 32:
                    complete.append(bytes(packets[:32]))
                    del packets[:32]
            return complete
        self.assertEqual(sdk.g4_renew_mit_batch(device, batch, 12, time.monotonic_ns()-1, 0), 0)
        initial = collect(.025)
        self.assertGreaterEqual(len(initial), 12)
        self.assertEqual({p[7] for p in initial}, {2})
        lease = time.monotonic_ns()+45_000_000
        self.assertEqual(sdk.g4_renew_mit_batch(device, batch, 12, lease, 0), 1)
        first = collect(.12)
        self.assertTrue(any(p[7]==8 for p in first))
        # 新帧本身还新鲜，但旧批次的租期已经失效。续接必须被原生 SDK 拒绝。
        self.assertEqual(sdk.g4_renew_mit_batch(device, batch, 12, time.monotonic_ns()+80_000_000, lease), 0)
        after = collect(.04)
        self.assertGreaterEqual(len(after), 12)
        self.assertEqual({p[7] for p in after}, {2}, "拒绝逾期续接后应持续查询位置")


class NativeRuntimeDemoTest(unittest.TestCase):
    def test_demo_cannot_submit_when_intent_requests_output(self):
        native = load_native()
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        kin = NativeGloveKinematics(MODEL_DIR, mapping)
        lo, hi = zip(*(m.model_rad_limits() for m in mapping.motors))
        with tempfile.TemporaryDirectory() as folder:
            link = native.SharedLink(str(Path(folder) / "demo.shm"), len(kin.joint_names))
            loop = native.NativeLoop(kin.native, make_feedback_control(axes, native), link, "",
                                     [m.sign * m.scale for m in mapping.motors], lo, hi, [0.] * 12,
                                     10_000_000)
            try:
                loop.start([0.] * 12)
                link.command(True, 0, 0.)
                until = time.monotonic() + .3
                while time.monotonic() < until:
                    stamp = link.read_target()[0]
                    link.publish_force([stamp, time.monotonic_ns(), 1., *([-.8] * 12),
                                        *([0.] * 48), 0.])
                    time.sleep(.01)
                samples = loop.samples(0, False)
                self.assertTrue(any(s["feedback"].valid_nominal for s in samples))
                self.assertTrue(all(not s["requested"] and not s["feedback"].valid_sent for s in samples))
                # 网页取较早一帧时，控制线程可能已经基于随后几帧完成了新指令。
                by_frame = {int(s["force"][1]): s for s in samples if s["feedback"].valid_nominal}
                stamp, sample = list(by_frame.items())[-3]
                display = link.read_display_for(stamp, 0)
                self.assertEqual(display[28], stamp)
                self.assertEqual(display[2:14], list(sample["feedback"].nominal))
                self.assertFalse(link.read_display_for(stamp + 1, 0))
                link.command(False, 1, 0.)
                self.assertFalse(link.read_display_for(stamp, 0))
            finally:
                loop.close()
