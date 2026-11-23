"""后台写入本地抓握的逐轴控制采样。"""

import csv
from datetime import datetime
import os
from pathlib import Path
from queue import Full, Queue
from tempfile import NamedTemporaryFile
from threading import Thread



AXIS_FIELDS = ("id", "joint", "raw_deg", "input_rad", "contact_input_rad", "sim_rad", "motor_rad_s", "contact_motor_nm",
               "position_motor_nm", "nominal_nm", "sent_motor_nm", "current_a", "age_ms", "state",
               "encoder_rad", "contact_encoder_rad", "mapped_contact_nm", "contact_gate", "retreat_weight")
HEADER = ("monotonic_ns", "mode", "kind", "layout", "size_mm", "hardness", "feedback_active", "feedback_gain",
          "contact_ms", "feedback_interruptions", "last_feedback_interruption",
          "feedback_handoffs", "last_feedback_handoff_ms",
          *AXIS_FIELDS, "feedback_profile", "contact_frame_ns", "command_deadline_ns",
          "control_cycle_ms", "control_missed_ticks", "transport_online", "reconnect_count", "sim_minus_input_rad",
          "physics_input_ns", "physics_solved_ns", "feedback_rearm_required")


class GraspRecorder:
    def __init__(self, control):
        self.control = control
        self.directory = Path(os.environ.get("XDG_STATE_HOME") or (Path.home() / ".local/state")) / "superdex/recordings"
        self.active = False
        self.queue = Queue(maxsize=512)
        self.thread = Thread(target=self.write, daemon=True)
        self.thread.start()

    def tick(self, stamp, rows, settings, output_active, demo, status):
        requested = self.control.record_requested.is_set()
        if requested != self.active:
            try:
                self.queue.put_nowait(("start" if requested else "stop", None))
            except Full:
                return
            self.active = requested
        if self.active:
            prefix = (stamp, "demo" if demo else "g4", settings["kind"], settings["layout"],
                      settings["size_mm"], settings["hardness"],
                      int(output_active), status["feedback_gain"],
                      status["contact_ms"], status["feedback_interruptions"], status["last_feedback_interruption"],
                      status["feedback_handoffs"], status["last_feedback_handoff_ms"])
            data = [(*prefix, *(row.get(key) for key in AXIS_FIELDS),
                     status.get("feedback_profile"), status.get("contact_frame_ns"),
                     status.get("command_deadline_ns"), status.get("control_cycle_ms"),
                     status.get("control_missed_ticks"), status.get("transport_online"),
                     status.get("reconnect_count"), row.get("sim_minus_input_rad"),
                     status.get("physics_input_ns"), status.get("physics_solved_ns"),
                     status.get("feedback_rearm_required")) for row in rows]
            try:
                self.queue.put_nowait(("sample", data))
            except Full:
                self.control.record_dropped.value += 1

    def close(self):
        self.queue.put(("exit", None))
        self.thread.join()

    def write(self):
        file = writer = None
        while True:
            action, data = self.queue.get()
            try:
                if action == "sample" and writer is not None:
                    writer.writerows(data)
                elif action in ("stop", "exit") and file is not None:
                    file.close()
                    self.control.record_ready.value = os.fsencode(file.name)
                    file = writer = None
                elif action == "start":
                    self.directory.mkdir(parents=True, exist_ok=True)
                    file = NamedTemporaryFile(mode="w", newline="", encoding="utf-8-sig", delete=False,
                        dir=self.directory, prefix=datetime.now().strftime("grasp-%Y%m%d-%H%M%S-"), suffix=".csv")
                    writer = csv.writer(file)
                    writer.writerow(HEADER)
            except OSError:
                self.control.record_error.value = "CSV 写入失败".encode()
                self.control.record_requested.clear()
                if file is not None and not file.closed:
                    try:
                        file.close()
                    except OSError:
                        pass
                file = writer = None
            if action == "exit":
                return
