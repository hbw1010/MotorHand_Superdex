"""诊断必须识别原接触帧过期和场景切换，不用面板发布时间续期。"""
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ve33_g4_view import capture_zero, read_feedback_display
from ve33_web_control import panel_snapshot, measure_frame_gaps


class SnapshotTest(unittest.TestCase):
    def test_tracking_error_compares_the_input_that_drove_the_same_pose(self):
        rows = [
            {"input_rad": .75, "contact_input_rad": .40, "sim_rad": .40},
            {"input_rad": .50, "contact_input_rad": .53, "sim_rad": .46},
            {"input_rad": .19, "contact_input_rad": None, "sim_rad": .11},
        ]
        self.assertAlmostEqual(measure_frame_gaps(rows), .07)
        self.assertEqual(rows[0]["sim_minus_input_rad"], 0.)
        self.assertAlmostEqual(rows[1]["sim_minus_input_rad"], -.07)
        self.assertIsNone(rows[2]["sim_minus_input_rad"])
        self.assertIsNone(measure_frame_gaps(rows[2:]))

    def test_connection_changed_at_end_of_calibration_preserves_file(self):
        axes = {i: (str(i), SimpleNamespace(sign=1, scale=1)) for i in range(1, 13)}
        def read():
            stamp = time.monotonic_ns()
            return {i: SimpleNamespace(received_ns=stamp, angle_deg=0., valid=1, error=0, bus=3-(i-1)//4)
                    for i in axes}
        def reject(_):
            raise ValueError("网关已变化")
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "zero.json"
            path.write_text("已有零位")
            with self.assertRaisesRegex(ValueError, "网关已变化"):
                capture_zero(read, axes, path, accept=reject)
            self.assertEqual(path.read_text(), "已有零位")
            self.assertEqual(list(path.parent.iterdir()), [path])

    def test_current_cycle_cannot_republish_expired_force_or_an_old_scene(self):
        now = 1_000_000_000
        data = [now, True, *[2.] * 12, *[.5] * 12, 8, now + 1]
        with patch('ve33_g4_view.time.monotonic_ns', return_value=now):
            self.assertEqual(read_feedback_display(data, True, revision=8)['nominal_nm'][0], 2.)
            self.assertIsNone(read_feedback_display(data, True, revision=9))
            data[-1] = now
            self.assertIsNone(read_feedback_display(data, True, revision=8))

    def test_http_marks_stalled_observer_and_does_not_report_old_torque(self):
        sample = dict(control_sample_ns=1_000_000_000, diagnostic_ttl_ns=100_000_000,
                      force_expiry_ns=1_060_000_000, control_revision=4, active=True,
                      rows=[{'nominal_nm': -3., 'sent_motor_nm': -3., 'encoder_rad': .2}])
        from copy import deepcopy
        for now, revision in ((1_010_000_000, 5), (1_070_000_000, 4), (1_300_000_000, 4)):
            state = panel_snapshot(deepcopy(sample), now, revision)
            if state['rows']:
                self.assertIsNone(state['rows'][0]['nominal_nm'])
                self.assertIsNone(state['rows'][0]['sent_motor_nm'])
            else:
                self.assertIsNone(state['active'])
            self.assertEqual(state['diagnostic_stale'], now >= 1_100_000_000)
        self.assertEqual(panel_snapshot(sample, 1_010_000_000, 4)['rows'][0]['nominal_nm'], -3.)
