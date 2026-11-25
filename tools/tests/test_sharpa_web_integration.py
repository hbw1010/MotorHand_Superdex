"""验证 Sharpa 独立控制引擎、运行期零位同步、UDP 实时接管与三标签 WebControl 切换。"""
import json
import multiprocessing as mp
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ve33_sharpa_service import (
    DEFAULT_MAPPING,
    RECORDINGS_DIR,
    SharpaManager,
    build_runtime_mapping,
    resolve_sharpa_binary,
)
from ve33_web_control import WebControl


class SharpaWebIntegrationTest(unittest.TestCase):
    def test_runtime_mapping_syncs_saved_zero_and_shifts_limits(self):
        with TemporaryDirectory() as tmp:
            zero_file = Path(tmp) / "zero.json"
            base = json.loads(DEFAULT_MAPPING.read_text(encoding="utf-8"))
            old_zero = float(base["motors"]["1"]["zero_deg"])
            old_min = float(base["motors"]["1"]["min_deg"])
            old_max = float(base["motors"]["1"]["max_deg"])
            zero_file.write_text(json.dumps({"zero_deg": {"1": old_zero + 15.0}}), encoding="utf-8")
            merged_path = build_runtime_mapping(zero_file)
            merged = json.loads(merged_path.read_text(encoding="utf-8"))
            self.assertAlmostEqual(merged["motors"]["1"]["zero_deg"], old_zero + 15.0, places=4)
            self.assertAlmostEqual(merged["motors"]["1"]["min_deg"], old_min + 15.0, places=4)
            self.assertAlmostEqual(merged["motors"]["1"]["max_deg"], old_max + 15.0, places=4)

    def test_web_control_three_tab_switching(self):
        ctx = mp.get_context("spawn")
        ctrl = WebControl(
            ctx, None, default_enabled=False, local_grasp=True, demo=False,
            switchable_tasks=("local_grasp", "tabletop", "sharpa_real"),
            initial_task="local_grasp",
        )
        ctrl.command({"action": "switch_task", "task": "sharpa_real"})
        self.assertEqual(ctrl.pending_task(), "sharpa_real")
        ctrl.commit_task("sharpa_real")
        self.assertEqual(ctrl.current_task(), "sharpa_real")
        self.assertFalse(ctrl.is_local_grasp())
        ctrl.command({"action": "switch_task", "task": "tabletop"})
        self.assertEqual(ctrl.pending_task(), "tabletop")
        ctrl.commit_task("tabletop")
        self.assertEqual(ctrl.current_task(), "tabletop")

    def test_sharpa_sim_record_and_live_udp_takeover_toggle(self):
        binary = resolve_sharpa_binary(simulation=True)
        self.assertIsNotNone(binary, "sharpa_teleop 应已编译出独立的 motorhand_sharpa")

        rec_name = f"_unittest_takeover_{int(time.time() * 1000)}.jsonl"
        rec_path = RECORDINGS_DIR / rec_name
        mgr = SharpaManager()
        try:
            # 1) 启动离线仿真并录制 22 轴轨迹
            mgr.start({
                "mode": "sim",
                "hand": "right",
                "hardness": 0.3,
                "rate": 50,
                "record_output": True,
                "output_name": rec_name,
                "with_object": True,
            })
            deadline = time.monotonic() + 8.0
            frame = None
            while time.monotonic() < deadline:
                st = mgr.status()
                if st["frame"] is not None and len(st["frame"].get("joint_names", [])) == 22:
                    frame = st["frame"]
                    break
                time.sleep(0.05)
            self.assertIsNotNone(frame, f"未收到 Sharpa 仿真遥测帧，日志: {mgr.status()['logs']}")
            self.assertEqual(len(frame["position_rad"]), 22)
            self.assertEqual(len(frame["glove_motor_rad"]), 12)
            time.sleep(0.8)
            mgr.stop()
            self.assertTrue(rec_path.is_file() and rec_path.stat().st_size > 200)

            # 2) 使用刚录制的轨迹启动 takeover（离线仿真+假手套），并通过 UDP 实时切换接管↔回放
            mgr.start({
                "mode": "takeover",
                "simulation": True,
                "fake_glove": True,
                "hand": "right",
                "hardness": 0.3,
                "rate": 50,
                "trajectory": str(rec_path),
            })
            deadline = time.monotonic() + 8.0
            st = None
            while time.monotonic() < deadline:
                st = mgr.status()
                if st["running"] and st["frame"] is not None:
                    break
                time.sleep(0.05)
            self.assertIsNotNone(st)
            self.assertTrue(st["running"], f"takeover 启动失败，日志: {st['logs']}")
            self.assertTrue(st["can_takeover"])
            self.assertFalse(st["takeover"])

            # 发送接管指令 -> 切换到人手实时接管
            mgr.send_control("takeover")
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                st = mgr.status()
                if st["takeover"]:
                    break
                time.sleep(0.04)
            self.assertTrue(st["takeover"], "未能通过 UDP 实时切换至人手接管模式")

            # 发送恢复回放指令 -> 切换回轨迹回放
            mgr.send_control("replay")
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                st = mgr.status()
                if not st["takeover"]:
                    break
                time.sleep(0.04)
            self.assertFalse(st["takeover"], "未能通过 UDP 恢复轨迹回放模式")
        finally:
            mgr.stop()
            if rec_path.exists():
                rec_path.unlink()


if __name__ == "__main__":
    unittest.main()
