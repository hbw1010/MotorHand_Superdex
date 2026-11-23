"""录制诊断字段与反馈曲线入口。"""
import csv
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ve33_grasp_recording import AXIS_FIELDS, HEADER, GraspRecorder


class RecordingProfileTest(unittest.TestCase):
    def test_recording_columns_include_diagnostics_and_preserve_legacy_order(self):
        for field in ("feedback_profile", "contact_frame_ns", "mapped_contact_nm", "contact_gate", "retreat_weight"):
            self.assertIn(field, HEADER)
        self.assertIn("mapped_contact_nm", AXIS_FIELDS)
        self.assertLess(HEADER.index("encoder_rad"), HEADER.index("contact_encoder_rad"))

    def test_cli_exposes_feedback_profile(self):
        entry = Path(__file__).resolve().parents[1] / "ve33_g4_view.py"
        result = subprocess.run([sys.executable, str(entry), "--help"], capture_output=True, text=True, check=True)
        self.assertIn("--feedback-profile", result.stdout)
        self.assertIn("nominal", result.stdout)

    def test_csv_contains_values_and_empty_unavailable_data(self):
        control = SimpleNamespace(record_requested=threading.Event(),
                                  record_dropped=SimpleNamespace(value=0),
                                  record_ready=SimpleNamespace(value=b""),
                                  record_error=SimpleNamespace(value=b""))
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {"XDG_STATE_HOME": directory}):
            recorder = GraspRecorder(control)
            control.record_requested.set()
            recorder.tick(123, [{"id": 5, "mapped_contact_nm": -4.2, "contact_gate": True,
                                 "retreat_weight": .5}],
                          {"kind": "ball_hard", "layout": "palm", "size_mm": 89, "hardness": 1},
                          False, True, {"feedback_gain": 1, "contact_ms": 3,
                                        "feedback_interruptions": 0, "last_feedback_interruption": None,
                                        "feedback_handoffs": 0, "last_feedback_handoff_ms": None,
                                        "feedback_profile": "nominal", "contact_frame_ns": 99,
                                        "command_deadline_ns": None, "physics_input_ns": 70,
                                        "physics_solved_ns": 99})
            recorder.tick(100_000_080, [{"id": 5}], {"kind": "box", "layout": "palm", "size_mm": 117,
                          "hardness": 1}, False, True, {"feedback_gain": 1, "contact_ms": 100.00001,
                          "feedback_interruptions": 0, "last_feedback_interruption": None,
                          "feedback_handoffs": 0, "last_feedback_handoff_ms": None,
                          "physics_input_ns": 70, "physics_solved_ns": 99})
            recorder.close()
            with open(os.fsdecode(control.record_ready.value), encoding="utf-8-sig", newline="") as stream:
                row, expired = list(csv.DictReader(stream))
            self.assertEqual(row["feedback_profile"], "nominal")
            self.assertEqual(row["contact_frame_ns"], "99")
            self.assertEqual(float(row["mapped_contact_nm"]), -4.2)
            self.assertEqual(float(row["retreat_weight"]), .5)
            self.assertEqual(row["sent_motor_nm"], "")
            self.assertEqual(row["command_deadline_ns"], "")
            self.assertEqual((expired["physics_input_ns"], expired["physics_solved_ns"]), ("70", "99"))
            self.assertEqual((expired["contact_frame_ns"], expired["nominal_nm"]), ("", ""))
