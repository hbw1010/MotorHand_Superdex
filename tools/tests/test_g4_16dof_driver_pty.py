"""16-DOF G4 网关驱动（encos_g4_gateway_16dof / libg4_shim_16dof.so）伪串口单元测试。

验证 v0.5.0 协议与 Linux 实时优化：
1. 16 轴 ABI（g4_16dof_axis_count == 16）与 <=150 Hz 串口带宽上限钳位；
2. 全 16 轴分布在 CAN 1..3（永不发出 bus > 3）且支持任意非默认 CAN 1..3 拓扑动态发现；
3. g4_16dof_renew_mit_batch 16 轴批量下发（正常续期、过期拒绝、轴数错误拒绝）；
4. 伪串口断开时 g4_16dof_transport_lost 及时置位。
"""
from __future__ import annotations

import ctypes
import errno
import math
import os
from pathlib import Path
import pty
import select
import struct
import sys
import termios
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from motorhand.g4_types import G4Joint

LIB16_PATH = ROOT / "hardware/encos_g4_gateway_16dof/build/libg4_shim_16dof.so"


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else ((crc << 1) & 0xFFFF)
    return crc


def _build_rx_frame(bus: int, motor_id: int, angle_deg: float, cur_a: float = 0.0, ts_us: int = 1000) -> bytes:
    """构造 32 字节 GatewayWireFrame (kind=0x90, type=2 位置+电流反馈负载)。"""
    cur_i16 = max(-32768, min(32767, int(round(cur_a * 100.0))))
    payload = bytearray(8)
    payload[0] = (2 << 5) | 0  # type=2, err=0
    struct.pack_into(">f", payload, 1, float(angle_deg))
    struct.pack_into(">h", payload, 5, cur_i16)
    payload[7] = 65  # motor temp
    frame = bytearray(32)
    # header[2], version, kind, sequence(H), bus(B), len(B), can_id(H), flags(H), ts(I), data[8], aux(I), detail(H), crc(H)
    struct.pack_into(
        "<BBBBHBBHHI8sIH",
        frame,
        0,
        0xA5, 0x5A, 1, 0x90,
        0,
        bus, 8,
        motor_id, 0,
        ts_us & 0xFFFFFFFF,
        bytes(payload),
        0, 0,
    )
    struct.pack_into("<H", frame, 30, _crc16(bytes(frame[:30])))
    return bytes(frame)


class FakeGateway16:
    """在 PTY 上模拟 16 轴 GW02 网关，刻意采用与默认不同的 CAN 1..3 拓扑测试自动发现。"""

    def __init__(self, topology: dict[int, int]):
        self.topology = dict(topology)
        self.master_fd, self.slave_fd = pty.openpty()
        attrs = termios.tcgetattr(self.slave_fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = (attrs[2] & ~(termios.CSIZE | termios.PARENB)) | termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[3] = 0
        termios.tcsetattr(self.slave_fd, termios.TCSANOW, attrs)
        self.slave_path = os.ttyname(self.slave_fd)
        self.stop_event = threading.Event()
        self.seen_buses: set[int] = set()
        self.seen_poll_controls: list[int] = []
        self.mit_frames: dict[int, int] = {}
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        buf = bytearray()
        while not self.stop_event.is_set():
            r, _, _ = select.select([self.master_fd], [], [], 0.02)
            if not r:
                continue
            try:
                chunk = os.read(self.master_fd, 4096)
            except OSError as exc:
                if exc.errno in (errno.EIO, errno.EBADF):
                    break
                continue
            if not chunk:
                break
            buf.extend(chunk)
            while len(buf) >= 32:
                idx = buf.find(b"\xA5\x5A")
                if idx < 0:
                    buf.clear()
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < 32:
                    break
                pkt = bytes(buf[:32])
                del buf[:32]
                if pkt[2] != 1:
                    continue
                crc_got = struct.unpack_from("<H", pkt, 30)[0]
                if _crc16(pkt[:30]) != crc_got:
                    continue
                kind = pkt[3]
                bus = pkt[6]
                length = pkt[7]
                can_id = struct.unpack_from("<H", pkt, 8)[0]
                payload = pkt[16:24]
                aux = struct.unpack_from("<I", pkt, 24)[0]
                if kind == 8:  # GW_POLL_CONTROL
                    with self._lock:
                        self.seen_poll_controls.append(aux)
                    continue
                if kind != 2:  # GW_CAN_TX
                    continue
                with self._lock:
                    self.seen_buses.add(bus)
                    if length == 8 and (payload[0] >> 5) == 0:  # MIT frame
                        self.mit_frames[can_id] = self.mit_frames.get(can_id, 0) + 1
                expected_bus = self.topology.get(can_id)
                if expected_bus == bus:
                    angle_deg = (can_id - 8.5) * 3.0
                    ts_us = int(time.monotonic() * 1_000_000) & 0xFFFFFFFF
                    reply = _build_rx_frame(bus, can_id, angle_deg, ts_us=ts_us)
                    try:
                        os.write(self.master_fd, reply)
                    except OSError:
                        break

    def disconnect_master(self) -> None:
        self.stop_event.set()
        self._thread.join(timeout=1.5)
        for fd_attr in ("master_fd", "slave_fd"):
            fd = getattr(self, fd_attr, None)
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
                setattr(self, fd_attr, None)


class G4Gateway16DofPtyTest(unittest.TestCase):
    def setUp(self) -> None:
        self.assertTrue(LIB16_PATH.is_file(), f"缺少 16-DOF shim: {LIB16_PATH}")
        self.lib = ctypes.CDLL(str(LIB16_PATH))
        self.lib.g4_16dof_joint_size.restype = ctypes.c_size_t
        self.lib.g4_16dof_axis_count.restype = ctypes.c_int
        self.lib.g4_16dof_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint]
        self.lib.g4_16dof_open.restype = ctypes.c_void_p
        self.lib.g4_16dof_open_reader.argtypes = [ctypes.c_char_p]
        self.lib.g4_16dof_open_reader.restype = ctypes.c_void_p
        self.lib.g4_16dof_close.argtypes = [ctypes.c_void_p]
        self.lib.g4_16dof_close.restype = None
        self.lib.g4_16dof_target_hz.argtypes = [ctypes.c_void_p]
        self.lib.g4_16dof_target_hz.restype = ctypes.c_uint
        self.lib.g4_16dof_snapshots_with_current_time.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(G4Joint), ctypes.POINTER(ctypes.c_int64), ctypes.c_int,
        ]
        self.lib.g4_16dof_snapshots_with_current_time.restype = ctypes.c_int
        self.lib.g4_16dof_renew_mit_batch.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.c_int64, ctypes.c_int64,
        ]
        self.lib.g4_16dof_renew_mit_batch.restype = ctypes.c_int
        self.lib.g4_16dof_transport_lost.argtypes = [ctypes.c_void_p]
        self.lib.g4_16dof_transport_lost.restype = ctypes.c_int

    def test_16dof_cap_150hz_dynamic_can1_3_discovery_mit_batch_and_hotplug(self) -> None:
        self.assertEqual(self.lib.g4_16dof_joint_size(), ctypes.sizeof(G4Joint))
        self.assertEqual(self.lib.g4_16dof_axis_count(), 16)

        # 刻意采用非默认 CAN 1..3 拓扑（把 1..4 放到 CAN1，5..10 放到 CAN3，11..16 放到 CAN2）
        topology = {mid: (1 if mid <= 4 else (3 if mid <= 10 else 2)) for mid in range(1, 17)}
        fake = FakeGateway16(topology)
        handle = None
        try:
            # 请求 200 Hz，16-DOF v0.5.0 驱动必须自动钳位到 150 Hz
            handle = self.lib.g4_16dof_open(fake.slave_path.encode(), 921600, 200)
            self.assertTrue(handle)
            self.assertEqual(self.lib.g4_16dof_target_hz(handle), 150)

            buf = (G4Joint * 16)()
            times = (ctypes.c_int64 * 16)()
            deadline = time.monotonic() + 3.0
            online = 0
            while time.monotonic() < deadline:
                n = self.lib.g4_16dof_snapshots_with_current_time(handle, buf, times, 16)
                online = sum(1 for i in range(n) if buf[i].valid)
                if online == 16:
                    break
                time.sleep(0.02)
            self.assertEqual(online, 16, "非默认 CAN 1..3 拓扑下应自动发现全部 16 轴")

            with fake._lock:
                self.assertTrue(fake.seen_buses.issubset({1, 2, 3}), f"16-DOF 仅允许 CAN 1..3，实际 {fake.seen_buses}")
                self.assertIn(0, fake.seen_poll_controls, "应下发 PollControl(0) 停用 MCU 12 槽周期轮询")
                self.assertIn(2, fake.seen_poll_controls, "应下发 PollControl(2) 切到主机直发模式")

            for i in range(16):
                mid = buf[i].id
                self.assertEqual(buf[i].bus, topology[mid])
                expected_deg = (mid - 8.5) * 3.0
                self.assertAlmostEqual(buf[i].angle_deg, expected_deg, delta=0.15)
                self.assertGreater(times[i], 0)

            # 测试 16 轴 MIT 批量续期：正常通过、过期拒绝(0)、轴数错误拒绝(-1)
            cmds = (ctypes.c_float * (16 * 5))()
            for i in range(16):
                cmds[i * 5 + 0] = 0.0
                cmds[i * 5 + 1] = 0.0
                cmds[i * 5 + 2] = 0.0
                cmds[i * 5 + 3] = 0.0
                cmds[i * 5 + 4] = 0.05
            now_ns = time.monotonic_ns()
            first_deadline = now_ns + 150_000_000
            second_deadline = now_ns + 300_000_000
            self.assertEqual(self.lib.g4_16dof_renew_mit_batch(handle, cmds, 12, first_deadline, 0), -1)
            self.assertEqual(self.lib.g4_16dof_renew_mit_batch(handle, cmds, 16, now_ns - 1_000_000, 0), 0)
            self.assertEqual(self.lib.g4_16dof_renew_mit_batch(handle, cmds, 16, first_deadline, now_ns + 999), 0)
            self.assertEqual(self.lib.g4_16dof_renew_mit_batch(handle, cmds, 16, first_deadline, 0), 1)
            self.assertEqual(self.lib.g4_16dof_renew_mit_batch(handle, cmds, 16, second_deadline, first_deadline), 1)
            time.sleep(0.08)
            with fake._lock:
                self.assertGreaterEqual(len(fake.mit_frames), 16, f"应向全部 16 轴发出 MIT 帧，实际 {len(fake.mit_frames)}")

            # 断开伪串口，transport_lost 应置位
            self.assertEqual(self.lib.g4_16dof_transport_lost(handle), 0)
            fake.disconnect_master()
            lost_deadline = time.monotonic() + 2.0
            while time.monotonic() < lost_deadline and self.lib.g4_16dof_transport_lost(handle) == 0:
                time.sleep(0.02)
            self.assertEqual(self.lib.g4_16dof_transport_lost(handle), 1)
        finally:
            if handle:
                self.lib.g4_16dof_close(handle)
            fake.disconnect_master()


if __name__ == "__main__":
    unittest.main()
