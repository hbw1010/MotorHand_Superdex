"""本地抓握控制台和录制。数据取自 C++ 周期结束后的有序快照。"""
import math
from pathlib import Path
import time
from types import SimpleNamespace

from .feedback import CONFIG
from ve33_web_control import measure_frame_gaps


def _finite(value):
    return value if math.isfinite(value) else None


def observe_native(sample, axes):
    """从单一控制周期解码状态，保留该周期使用的物理帧和命令期限。"""
    ctrl = sample["feedback"]
    now = sample["now"]
    frame = sample["force"]
    matches = frame[-1] == sample["revision"]
    has_timing = matches and 0 < frame[0] <= frame[1] <= now
    source = min(frame[:2]) if matches else 0
    ttl = int(CONFIG["command_ttl_s"] * 1e9)
    expiry = min(source, sample["stamp"]) + ttl if source and sample["stamp"] else 0
    fresh = (matches and frame[2] == 1 and source > 0 and source <= now < expiry
             and all(math.isfinite(x) for x in frame))
    nominal = ctrl.nominal if ctrl.valid_nominal else None
    sent = ctrl.sent if ctrl.valid_sent and ctrl.active else None
    speed, position, mapped, gate, weights = ctrl.dq, ctrl.position, ctrl.mapped_contact, ctrl.gate, ctrl.frame_weight
    rows = []
    for i, (mid, (name, _)) in enumerate(axes.items()):
        code = sample["axis_status"][i]
        stamp = sample["stamps"][i]
        state = {1: "在线", -2: "路由错误", -1: f"报警 {sample['errors'][i]}"}.get(code, "缺失或过期")
        rows.append(dict(id=mid, joint=name, state=state, raw_deg=_finite(sample["raw"][i]),
                         input_rad=_finite(sample["q"][i]), encoder_rad=_finite(sample["encoder"][i]),
                         age_ms=(now-stamp)/1e6 if stamp else None,
                         sim_rad=frame[27+i] if fresh else None, contact_motor_nm=frame[3+i] if fresh else None,
                         contact_input_rad=frame[39+i] if fresh else None, contact_encoder_rad=frame[51+i] if fresh else None,
                         motor_rad_s=speed[i] if nominal is not None else None,
                         position_motor_nm=position[i] if fresh and nominal is not None else None,
                         mapped_contact_nm=mapped[i] if fresh and nominal is not None else None,
                         contact_gate=gate[i] if fresh and nominal is not None else None,
                         retreat_weight=weights[i] if fresh and nominal is not None else None,
                         nominal_nm=nominal[i] if nominal is not None else None,
                         sent_motor_nm=sent[i] if sent is not None else None, current_a=_finite(sample["current"][i])))
    online = sample["axis_status"].count(1)
    message = ctrl.message
    if not sample["transport_online"]:
        message = "串口已断开 · 等待重新连接"
    elif not online and not any(code < 0 for code in sample["axis_status"]):
        message = "串口已打开 · 等待电机位置回包"
    elif sample["requires_new_zero"]:
        message = "网关已重连 · 请重新标零"
    return dict(rows=rows, online=online, calibrated=sample["calibrated"],
                control_sample_ns=now, control_revision=sample["revision"], diagnostic_ttl_ns=ttl,
                force_expiry_ns=expiry if fresh else 0,
                requested=sample["requested"], active=ctrl.active, output_error=ctrl.output_error,
                feedback_rearm_required=ctrl.rearm_required,
                feedback_message=message, feedback_interruptions=ctrl.interruption_count,
                last_feedback_interruption=ctrl.last_interruption or None, feedback_handoffs=ctrl.handoff_count,
                last_feedback_handoff_ms=ctrl.last_handoff_gap_ms if ctrl.handoff_count else None,
                pose_error=sample["pose_error"] or None, wrist_ms=None,
                contact_ms=(now-source)/1e6 if source else None, contact_frame_ns=int(frame[1]) if fresh else None,
                physics_input_ns=int(frame[0]) if has_timing else None,
                physics_solved_ns=int(frame[1]) if has_timing else None,
                command_deadline_ns=ctrl.submitted_until if ctrl.valid_sent and ctrl.active else None,
                motor_ms=(now-sample["stamp"])/1e6 if sample["stamp"] else None,
                applied_ms=(now-frame[0])/1e6 if fresh else None, max_angle_error_rad=measure_frame_gaps(rows),
                control_cycle_ms=sample["cycle_ms"], control_missed_ticks=sample["missed_ticks"],
                transport_online=sample["transport_online"], reconnect_count=sample["reconnect_count"],
                requires_new_zero=sample["requires_new_zero"])


def run_native_control(loop, control, axes, zero_path, wrist_state, renderer, recorder,
                       device, demo, profile, load_zero, capture_zero, mapping_identity, *, fixed_wrist=True):
    zero = dict.fromkeys(axes, 0.) if demo else load_zero(zero_path, axes)
    def start(values, *, fresh_zero=False):
        loop.start([values[mid] for mid in axes] if values else [], fresh_zero=fresh_zero)

    def read_for_zero():
        return {mid: SimpleNamespace(id=mid, bus=bus, valid=valid, error=alarm, angle_deg=angle, received_ns=stamp)
                for mid, bus, valid, alarm, angle, stamp in loop.raw_for_zero()}

    status = dict(device="演示" if demo else str(device.resolve()), zero_path=None if demo else str(zero_path.resolve()),
                  axes=mapping_identity(axes), zero_revision=0, zero_error=None,
                  can_align=control.align is not None, fixed_wrist=fixed_wrist,
                  demo=demo, can_zero=not demo, can_feedback=not demo, feedback_profile=profile,
                  feedback_source="joint_position_and_contact_force", feedback_gain=1., grasp_backend="cpp",
                  control_backend="cpp", active=False, requested=False)
    seq = 0
    waiting_for_zero = False
    settings = control.grasp_request()
    start(zero)
    try:
        while renderer.is_alive():
            if control.zero.is_set():
                loop.stop()
                status.update(calibrating=True, requested=False, active=False, feedback_message="标零中 · 真机出力关闭")
                control.publish(status)
                try:
                    zero = capture_zero(read_for_zero, axes, zero_path,
                                        accept=lambda values: start(values, fresh_zero=True))
                    status["zero_error"] = None
                except (ValueError, OSError, RuntimeError) as exc:
                    zero = None
                    status["zero_error"] = str(exc)
                    loop.stop()
                    start(None)
                status.update(calibrating=False, zero_revision=status["zero_revision"]+1)
                control.zero.clear()

            recording = recorder is not None and (recorder.active or control.record_requested.is_set())
            new_settings = control.grasp_request()
            samples = loop.samples(seq, not recording)
            for s in samples:
                if recording and seq and s["seq"] > seq+1:
                    control.record_dropped.value += s["seq"]-seq-1
                seq = s["seq"]
                if s["requires_new_zero"] and not waiting_for_zero:
                    zero = None
                    control.command({"action": "disable"})
                waiting_for_zero = s["requires_new_zero"]
                status.update(observe_native(s, axes))
                status["can_feedback"] = not demo and s["transport_online"] and s["calibrated"]
                status["can_zero"] = not demo and s["transport_online"]
                if recording:
                    cfg = new_settings if new_settings["contact_revision"] == s["revision"] else settings
                    if cfg["contact_revision"] == s["revision"]:
                        recorder.tick(s["now"], status["rows"], cfg, status["active"], demo, status)
                    else:
                        control.record_dropped.value += 1
            if samples:
                settings = new_settings
                wrist_stamp = control.wrist_stamp.value if control.wrist_stamp is not None else 0
                status.update(wrist=wrist_state.value.decode() if wrist_state is not None else "",
                              wrist_ms=(samples[-1]["now"] - wrist_stamp) / 1e6 if wrist_stamp else None,
                              demo_grip=control.demo_grip.value if demo else None)
                if control.record_requested is not None:
                    ready = control.record_ready.value
                    status.update(recording=control.record_requested.is_set(),
                                  recording_file=Path(ready.decode()).name if ready else None,
                                  recording_error=control.record_error.value.decode(),
                                  recording_dropped=control.record_dropped.value)
                control.publish(status)
            time.sleep(.025 if recording else .05)
    finally:
        loop.stop()
