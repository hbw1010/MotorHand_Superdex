"""用实际厂商串口和伪终端模拟真实热拔插，不需要真机。"""
import binascii
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import sys
from tempfile import TemporaryDirectory
import time
import tty
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ve33_g4_view import open_reader


class G4HotplugTest(unittest.TestCase):
    def test_transport_reopens_reenumerated_port_without_old_commands(self):
        root = Path(__file__).resolve().parents[2]
        gateway12 = root / 'hardware/encos_g4_gateway_12dof'
        gateway16 = root / 'hardware/encos_g4_gateway_16dof'
        with TemporaryDirectory() as temp:
            binary = Path(temp) / 'hotplug'
            subprocess.run(['c++', '-std=c++20', '-pthread', '-O2',
                            '-I' + str(gateway12 / 'shim'), '-I' + str(gateway12 / 'vendor/include'),
                            '-I' + str(gateway16 / 'shim'), '-I' + str(gateway16 / 'vendor/include'),
                            '-I' + str(root / 'tools/motorhand/native'),
                            str(root / 'tools/tests/cpp/g4_hotplug.cpp'), str(gateway12 / 'shim/g4_shim_12dof.cpp'),
                            '-lutil', '-o', str(binary)], check=True, timeout=60)
            subprocess.run([str(binary), str(Path(temp) / 'gateway')], check=True, timeout=10)

    def test_python_session_reconnects_after_hangup_with_query_only_frames(self):
        with TemporaryDirectory() as temp:
            link = Path(temp) / 'g4'

            def make_pty():
                master, slave = pty.openpty()
                tty.setraw(slave)
                link.unlink(missing_ok=True)
                link.symlink_to(os.ttyname(slave))
                os.close(slave)
                return master

            master = make_pty()
            replacement = None
            try:
                with open_reader(link, feedback_capable=True) as session:
                    self.assertTrue(session.online)
                    self.assertEqual(session.poll_transport(), 'none')
                    session.send([(0., 0., 0., 0., .3)] * 12, time.monotonic_ns() + 90_000_000)
                    time.sleep(.015)
                    os.close(master)
                    master = None

                    deadline = time.monotonic() + 1.0
                    while session.poll_transport() != 'lost':
                        self.assertLess(time.monotonic(), deadline, 'Python 会话未检测到串口挂断')
                        time.sleep(.005)
                    self.assertFalse(session.online)
                    self.assertEqual(session(), {})

                    replacement = make_pty()
                    deadline = time.monotonic() + 2.5
                    while session.poll_transport() != 'restored':
                        self.assertLess(time.monotonic(), deadline, 'Python 会话未重新打开串口')
                        time.sleep(.01)
                    self.assertTrue(session.online)
                    self.assertEqual(session.reconnect_count, 1)

                    # 重连后不能延续旧控制帧，且收到位置回包应能快速刷新快照。
                    data = bytearray()
                    end = time.monotonic() + .5
                    while len(data) < 32 * 12 and time.monotonic() < end:
                        if select.select([replacement], [], [], .02)[0]:
                            data.extend(os.read(replacement, 4096))
                    self.assertGreaterEqual(len(data), 32 * 12)
                    for i in range(12):
                        frame = data[i * 32:(i + 1) * 32]
                        self.assertEqual(frame[:4], bytes.fromhex('a55a0102'))
                        self.assertEqual(frame[7], 2, '重连后出现了旧控制帧')

                    packet = bytearray(32)
                    packet[:4] = bytes.fromhex('a55a0190')
                    struct.pack_into('<BBH', packet, 6, 2, 6, 5)
                    struct.pack_into('>BBf', packet, 16, 0xa0, 1, 12.5)
                    struct.pack_into('<H', packet, 30, binascii.crc_hqx(packet[:30], 65535))
                    os.write(replacement, packet)
                    deadline = time.monotonic() + .2
                    while time.monotonic() < deadline:
                        j = session().get(5)
                        if j is not None and (j.valid & 1) and abs(j.angle_deg - 12.5) < 1e-4:
                            break
                        time.sleep(.002)
                    else:
                        self.fail('重连后未收到新位置反馈')
            finally:
                if master is not None:
                    os.close(master)
                if replacement is not None:
                    os.close(replacement)


if __name__ == '__main__':
    unittest.main()
