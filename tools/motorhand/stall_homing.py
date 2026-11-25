"""支架小电流自动顶限位回零（Soft-Torque Stall Homing）。

仅在手套未佩戴在人手上（置于支架）时使用：
- 对每个未屏蔽轴施加极低阻尼限速与小恒定力矩（MIT 模式 kp=0, kd>0, tor=小值），缓慢轻靠伸直机械硬限位；
- 单轴一旦在停顿窗口内角速度和角度极差满足静止判据，立即撤力转为只读 Query，不持续顶撞发热；
- 任一轴发生驱动器报警、通信超时、超行程或用户触发急停时，立刻全部卸力并中止回零；
- 全部轴到位后，原子保存新的零位文件，彻底消除 122.4:1 单编码器断电丢多圈导致的零位漂移。
"""
from collections import deque
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

MOTOR_COUNT = 12
DEFAULT_HOMING = {
    "enabled": True,
    "torque_nm": [1.2, 0.9] + [1.0] * 10,
    "damping_kd": 0.12,
    "direction_sign": [-1.0] * MOTOR_COUNT,
    "stop_offset_deg": [0.0] * MOTOR_COUNT,
    "stall_velocity_deg_s": 3.0,
    "stall_span_deg": 1.2,
    "stall_dwell_s": 0.25,
    "min_run_s": 0.35,
    "timeout_s": 5.0,
    "max_travel_deg": 180.0,
    "skip_motor_ids": [],
}


def load_homing_config(raw_config, *, disabled_motor_ids=(), dof=MOTOR_COUNT):
    axis_count = int(dof)
    base_default = {
        **DEFAULT_HOMING,
        "torque_nm": [1.2, 0.9] + [1.0] * max(0, axis_count - 2),
        "direction_sign": [-1.0] * axis_count,
        "stop_offset_deg": [0.0] * axis_count,
    }
    cfg = {**base_default, **(raw_config or {})}
    def _vec(key, lo, hi):
        vals = cfg[key]
        if not isinstance(vals, (list, tuple)) or len(vals) != axis_count:
            raise ValueError(f"stall_homing.{key} 必须是长度为 {axis_count} 的列表")
        out = [float(v) for v in vals]
        if any(not math.isfinite(v) or not lo <= v <= hi for v in out):
            raise ValueError(f"stall_homing.{key} 超出安全范围 [{lo}, {hi}]")
        return out

    torque_nm = _vec("torque_nm", 0.0, 5.0)
    direction_sign = _vec("direction_sign", -1.0, 1.0)
    if any(s not in (-1.0, 1.0) for s in direction_sign):
        raise ValueError("stall_homing.direction_sign 每项必须为 -1 或 1")
    stop_offset_deg = _vec("stop_offset_deg", -90.0, 90.0)
    damping_kd = float(cfg["damping_kd"])
    if not math.isfinite(damping_kd) or not 0.0 <= damping_kd <= 0.3:
        raise ValueError("stall_homing.damping_kd 应在 [0, 0.3] 范围内")
    stall_vel = float(cfg["stall_velocity_deg_s"])
    stall_span = float(cfg["stall_span_deg"])
    stall_dwell = float(cfg["stall_dwell_s"])
    min_run = float(cfg["min_run_s"])
    timeout = float(cfg["timeout_s"])
    max_travel = float(cfg["max_travel_deg"])
    if not (0.2 <= stall_vel <= 25.0 and 0.1 <= stall_span <= 10.0 and 0.08 <= stall_dwell <= 2.0):
        raise ValueError("stall_homing 堵转判据参数超出范围")
    if not (0.05 <= min_run <= 2.0 and 0.5 <= timeout <= 12.0 and 10.0 <= max_travel <= 260.0):
        raise ValueError("stall_homing 时间或最大行程参数超出范围")
    skip = {int(mid) for mid in cfg.get("skip_motor_ids", ()) if 1 <= int(mid) <= axis_count}
    skip.update(int(mid) for mid in disabled_motor_ids if 1 <= int(mid) <= axis_count)
    return {
        "enabled": bool(cfg.get("enabled", True)),
        "torque_nm": torque_nm,
        "damping_kd": damping_kd,
        "direction_sign": direction_sign,
        "stop_offset_deg": stop_offset_deg,
        "stall_velocity_deg_s": stall_vel,
        "stall_span_deg": stall_span,
        "stall_dwell_s": stall_dwell,
        "min_run_s": min_run,
        "timeout_s": timeout,
        "max_travel_deg": max_travel,
        "skip_ids": frozenset(skip),
    }


def _read_fresh_angles(read_raw, axes, max_age_ns=100_000_000):
    joints = read_raw()
    now = time.monotonic_ns()
    angles = {}
    for mid in axes:
        j = joints.get(mid)
        if j is None or not (j.valid & 1) or not math.isfinite(j.angle_deg):
            raise ValueError(f"轴 J{mid} 缺失或无效，自动回零中止")
        if j.error:
            raise ValueError(f"轴 J{mid} 报警（err={j.error}），自动回零中止")
        if j.received_ns <= 0 or j.received_ns > now or now - j.received_ns >= max_age_ns:
            raise ValueError(f"轴 J{mid} 反馈超时，自动回零中止")
        angles[mid] = float(j.angle_deg)
    return angles


def run_stall_homing(read_raw, send_mit_batch, release_all, axes, ratios, homing_cfg, *,
                     zero_path=None, mapping_identity=None, accept=None,
                     cancel_event=None, on_progress=None, tick_s=0.01, settle_s=0.15):
    """执行支架小电流顶限位回零，保存零位文件并可选提交给控制循环。"""
    if not homing_cfg["enabled"]:
        raise ValueError("自动顶限位回零已在配置中禁用")
    skip_ids = homing_cfg["skip_ids"]
    active_ids = [mid for mid in axes if mid not in skip_ids]
    ratios_by_id = {mid: float(ratios[mid - 1]) for mid in axes}
    ttl_ns = max(60_000_000, int(tick_s * 6e9))

    try:
        start_angles = _read_fresh_angles(read_raw, axes)
        done = {}
        # 屏蔽/跳过的轴保持只读，直接记录当前静止角度为零点
        for mid in axes:
            if mid in skip_ids:
                done[mid] = start_angles[mid]

        windows = {mid: deque() for mid in active_ids}
        t_start = time.monotonic()
        last_report = 0.0

        while len(done) < len(axes):
            if cancel_event is not None and cancel_event.is_set():
                raise ValueError("自动回零已被用户取消")
            now = time.monotonic()
            elapsed = now - t_start
            if elapsed > homing_cfg["timeout_s"]:
                pending = ", ".join(f"J{m}" for m in active_ids if m not in done)
                raise ValueError(f"自动回零超时（未到位：{pending}），请检查机械限位或调大 torque_nm")

            angles = _read_fresh_angles(read_raw, axes)

            # 检查每个仍在回零中的轴是否超行程或已稳停在机械硬限位
            for mid in active_ids:
                if mid in done:
                    continue
                if abs(angles[mid] - start_angles[mid]) > homing_cfg["max_travel_deg"]:
                    raise ValueError(f"轴 J{mid} 运动超过安全行程 {homing_cfg['max_travel_deg']:.0f}°，请检查回零方向")
                win = windows[mid]
                win.append((now, angles[mid]))
                cutoff = now - homing_cfg["stall_dwell_s"]
                while len(win) > 2 and win[0][0] < cutoff:
                    win.popleft()
                if elapsed >= homing_cfg["min_run_s"] and len(win) >= 4 and (win[-1][0] - win[0][0]) >= homing_cfg["stall_dwell_s"] * 0.85:
                    vals = [v for _, v in win]
                    span = max(vals) - min(vals)
                    dt = max(1e-3, win[-1][0] - win[0][0])
                    avg_vel = abs(vals[-1] - vals[0]) / dt
                    if span <= homing_cfg["stall_span_deg"] and avg_vel <= homing_cfg["stall_velocity_deg_s"]:
                        # 用停顿窗口中位数作为硬限位角，扣除硬限位到 CAD 零位的偏置
                        stop_deg = float(np.median(vals))
                        i = mid - 1
                        kin_ratio = float(axes[mid][1].sign * axes[mid][1].scale)
                        zero_deg = stop_deg - homing_cfg["stop_offset_deg"][i] / kin_ratio
                        done[mid] = zero_deg

            # 构建仅包含尚未到位轴的 MIT 小力矩指令（已到位与屏蔽轴立刻置 0 退出）
            batch = [0.0] * (len(axes) * 5)
            active_mask = 0
            for mid in active_ids:
                if mid in done:
                    continue
                i = mid - 1
                active_mask |= 1 << i
                tor = homing_cfg["direction_sign"][i] * homing_cfg["torque_nm"][i] * ratios_by_id[mid]
                batch[i * 5 + 2] = float(homing_cfg["damping_kd"])
                batch[i * 5 + 4] = float(tor)

            if active_mask:
                ok = send_mit_batch(batch, active_mask, ttl_ns)
                if ok != 1:
                    raise ValueError("自动回零指令下发失败，串口可能已断开")

            if on_progress is not None and (now - last_report >= 0.1 or len(done) == len(axes)):
                active_done = sum(1 for m in active_ids if m in done)
                on_progress(active_done, len(active_ids))
                last_report = now

            if len(done) < len(axes):
                time.sleep(tick_s)
    finally:
        release_all()

    # 全部卸力后再做一次无力在线核验，确保 12 轴状态健康后再写入零位文件
    if settle_s > 0:
        time.sleep(settle_s)
    _read_fresh_angles(read_raw, axes)

    result = {mid: float(done[mid]) for mid in axes}
    if accept is not None:
        accept(result)
    if zero_path is not None and mapping_identity is not None:
        path = Path(zero_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "hand": "right",
            "created": datetime.now(timezone.utc).isoformat(),
            "method": "stall_homing",
            "axes": mapping_identity(axes),
            "zero_deg": {str(mid): result[mid] for mid in axes},
        }
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        tmp.replace(path)
    return result
