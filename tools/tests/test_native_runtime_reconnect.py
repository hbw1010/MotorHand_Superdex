"""通过伪串口验证 C++ 周期线程在设备更换后清除先前的零位与指令。"""
import multiprocessing as mp
from multiprocessing.reduction import DupFd
import os
from pathlib import Path
import pty
import sys
import tempfile
import time
import tty
import unittest

from test_native_runtime_pty import simulate

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motorhand.feedback import MAPPING
from motorhand.mapping import GloveMapping
from motorhand.native_control import make_feedback_control
from motorhand.native_kinematics import NativeGloveKinematics
from motorhand.native_runtime import load_native
from ve33_g4_view import MODEL_DIR


class NativeReconnectTest(unittest.TestCase):
    def setUp(self):
        self.native = load_native()
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.tty_link = Path(self.folder.name) / 'g4'
        self.shared_path = str(Path(self.folder.name) / 'stream')
        mapping = GloveMapping.load(MAPPING)
        axes = {m.motor_id: (m.model_joint, m) for m in mapping.motors}
        self.kin = NativeGloveKinematics(MODEL_DIR, mapping)
        self.link = self.native.SharedLink(self.shared_path, len(self.kin.joint_names))
        self.peer = None
        self.addCleanup(self.disconnect)
        self.connect()
        lo, hi = zip(*(m.model_rad_limits() for m in mapping.motors))
        self.loop = self.native.NativeLoop(self.kin.native, make_feedback_control(axes, self.native),
                                          self.link, str(self.tty_link),
                                          [m.sign * m.scale for m in mapping.motors],
                                          lo, hi, [0.] * 12, 10_000_000)
        self.addCleanup(self.loop.close)
        self.loop.start([0.] * 12)

    def connect(self):
        master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.tty_link.symlink_to(os.ttyname(self.slave))
        ctx = mp.get_context("spawn")
        self.pipe, child = ctx.Pipe()
        self.peer = ctx.Process(target=simulate, args=(DupFd(master), self.shared_path, child), daemon=True)
        try:
            self.peer.start()
        finally:
            child.close()
            os.close(master)
        self.assertTrue(self.pipe.poll(8))
        self.assertEqual(self.pipe.recv(), 'ready')

    def disconnect(self):
        if self.peer is None:
            return
        self.pipe.send('exit')
        self.peer.join(timeout=3)
        if self.peer.is_alive():
            self.peer.terminate()
            self.peer.join()
        self.pipe.close()
        os.close(self.slave)
        self.tty_link.unlink()
        self.peer = None

    def until(self, predicate, timeout=3):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            result = self.loop.samples(0, True)
            if result and predicate(result[-1]):
                return result[-1]
            time.sleep(.01)
        self.fail('未观察到预期的连接/控制状态')

    def wire(self):
        self.pipe.send('wire')
        self.assertTrue(self.pipe.poll(2))
        return self.pipe.recv()

    def test_hotplug_returns_to_preview_after_zero_and_requires_new_enable(self):
        self.link.command(True, 0, 0.)
        self.until(lambda s: s['feedback'].valid_sent and s['feedback'].nominal[4] < -.5)
        self.disconnect()
        lost = self.until(lambda s: not s['transport_online'] and s['requires_new_zero'])
        self.assertFalse(lost['calibrated'] or lost['requested'] or lost['feedback'].valid_sent)
        self.assertFalse(any(v == 1 for v in lost['axis_status']))

        self.connect()
        seen = self.until(lambda s: s['reconnect_count'] == 1 and all(v == 1 for v in s['axis_status']))
        self.assertFalse(seen['calibrated'] or seen['requested'])
        self.link.command(True, 0, 0.)  # 接上另一台网关后，即便原启用意图被重放也不得出力。
        self.loop.stop()
        self.loop.start([0.] * 12)       # 缓存的零位无法解锁。
        time.sleep(.3)
        self.assertFalse(self.loop.samples(0, True)[-1]['calibrated'])
        self.assertTrue(all(v is None or abs(v) < .01 for _,_,v in self.wire()))

        self.loop.stop()
        with self.assertRaises(ValueError):
            self.loop.start([0.] * 12, fresh_zero=True)
        rows = self.loop.raw_for_zero()
        self.assertEqual(len(rows), 12)
        self.loop.start([r[4] for r in sorted(rows)], fresh_zero=True)
        s = self.until(lambda sample: sample['calibrated'] and sample['feedback'].valid_nominal)
        self.assertFalse(s['requires_new_zero'] or s['requested'] or s['feedback'].valid_sent)
        self.link.command(True, 0, 0.)
        self.until(lambda sample: sample['feedback'].valid_sent and sample['feedback'].nominal[4] < -.5)
        time.sleep(.03)
        self.assertTrue(any(v is not None and v < -.5 and j == 5 for _,j,v in self.wire()))

    def test_silent_open_tty_recovers_once_without_restoring_output(self):
        self.link.command(True, 0, 0.)
        self.until(lambda s: s['feedback'].valid_sent)
        self.pipe.send("no_rx")
        self.pipe.recv()
        # 保持主从 PTY 打开。先触发 100 ms 指令到期，短暂停包不会重开串口。
        stale = self.until(lambda s: not any(v == 1 for v in s['axis_status']))
        self.assertEqual(stale['reconnect_count'], 0)
        self.assertFalse(stale['feedback'].valid_sent)
        self.until(lambda s: s['reconnect_count'] == 1 and s['requires_new_zero'], timeout=5)
        time.sleep(3.3)
        self.assertEqual(self.loop.samples(0, True)[-1]['reconnect_count'], 1)
        self.pipe.send("hold")
        self.pipe.recv()
        recovered = self.until(lambda s: all(v == 1 for v in s['axis_status']))
        self.assertFalse(recovered['calibrated'] or recovered['requested'] or recovered['feedback'].valid_sent)


if __name__ == '__main__':
    unittest.main()
