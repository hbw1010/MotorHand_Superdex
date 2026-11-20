"""后台写入本地抓握的逐轴控制采样。"""

import csv
from datetime import datetime
import os
from pathlib import Path
from queue import Full, Queue
from tempfile import NamedTemporaryFile
from threading import Thread

from ve33_grasp_assets import GRASP_ASSETS


AXIS_FIELDS = ("id", "joint", "raw_deg", "input_rad", "contact_input_rad", "sim_rad", "motor_rad_s", "contact_motor_nm",
               "position_motor_nm", "nominal_nm", "sent_motor_nm", "current_a", "age_ms", "state",
               "encoder_rad", "contact_encoder_rad")
HEADER = ("monotonic_ns", "mode", "kind", "layout", "size_mm", "hardness", "feedback_active", "feedback_gain",
          "contact_ms", "feedback_interruptions", "last_feedback_interruption",
          "feedback_handoffs", "last_feedback_handoff_ms",
          *AXIS_FIELDS)


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
            asset = settings["kind"] in GRASP_ASSETS
            prefix = (stamp, "demo" if demo else "g4", settings["kind"], settings["layout"],
                      None if asset else settings["size_mm"], None if asset else settings["hardness"],
                      int(output_active), status["feedback_gain"],
                      status["contact_ms"], status["feedback_interruptions"], status["last_feedback_interruption"],
                      status["feedback_handoffs"], status["last_feedback_handoff_ms"])
            data = [(*prefix, *(row.get(key) for key in AXIS_FIELDS)) for row in rows]
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
