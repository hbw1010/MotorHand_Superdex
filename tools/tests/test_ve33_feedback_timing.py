"""力反馈期限与输入时序的独立回归，不连接真实设备。"""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motorhand.feedback import ContactForceFeedbackLoop, MAPPING
from motorhand.mapping import GloveMapping


class Gateway:
    def __init__(self):
        self.commands = []

    def send(self, batch, deadline):
        self.commands.append((batch, deadline))

    def release(self):
        pass


class FeedbackTimingTest(unittest.TestCase):
    def test_new_encoder_samples_cannot_extend_old_physics_input_deadline(self):
        axes = {m.motor_id: (m.model_joint, m) for m in GloveMapping.load(MAPPING).motors}
        gateway = Gateway()
        loop = ContactForceFeedbackLoop(gateway, axes)
        q = np.zeros(12)
        reaction = q.copy()
        reaction[4] = -.3
        raw = dict.fromkeys(axes, 0.)
        original = 1_000_000_000
        frame = [original, original + 10_000_000, True, *reaction, *q, *q, *q]
        with patch('motorhand.feedback.time.monotonic_ns', return_value=original + 15_000_000):
            loop.start()
            loop.started -= 200_000_000
            loop.tick(q, raw, original + 15_000_000, frame)
        self.assertEqual(gateway.commands[-1][1], original + 100_000_000)
        with patch('motorhand.feedback.time.monotonic_ns', return_value=original + 90_000_000):
            loop.tick(q, raw, original + 90_000_000, frame)
        self.assertEqual(gateway.commands[-1][1], original + 100_000_000)
        count = len(gateway.commands)
        with patch('motorhand.feedback.time.monotonic_ns', return_value=original + 100_000_000):
            loop.tick(q, raw, original + 100_000_000, frame)
        self.assertFalse(loop.active)
        self.assertEqual(len(gateway.commands), count)
        self.assertEqual(loop.last_interruption, '仿真使用的编码器样本过期')


if __name__ == '__main__':
    unittest.main()
