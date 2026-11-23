"""SuperDex 接触到原仿真驱动阻抗的适配。全部数组使用物理 ID 1..12 顺序。"""
import json
import math
from pathlib import Path
import time

import numpy as np

from .impedance import DriverImpedancePlanner, FrictionCompensator, ImpedanceCommand, VelocityEstimator
from .mapping import MOTOR_FINGER

CONFIG_DIR = Path(__file__).resolve().parents[2] / "hardware/encos_g4_gateway_sdk/config"
MAPPING = CONFIG_DIR / "right_sim_mapping.json"
CONFIG = json.loads((CONFIG_DIR / "right_sim_feedback.json").read_text())
MOTOR_COUNT = len(MOTOR_FINGER)
SURFACE_END = 3 + MOTOR_COUNT
DIRECTION_END = SURFACE_END + MOTOR_COUNT
WRIST_STAMP_INDEX = DIRECTION_END + 1
PROXY_FRAME_SIZE = WRIST_STAMP_INDEX + 1
FORCE_TORQUE_END = 3 + MOTOR_COUNT
FORCE_VELOCITY_END = FORCE_TORQUE_END + MOTOR_COUNT
FORCE_POSITION_END = FORCE_VELOCITY_END + MOTOR_COUNT
FORCE_INPUT_END = FORCE_POSITION_END + MOTOR_COUNT
FORCE_ENCODER_END = FORCE_INPUT_END + MOTOR_COUNT
FORCE_FRAME_SIZE = FORCE_ENCODER_END + 1
POSITION_DEADBAND_RAD = .002
CONTACT_THRESHOLD_NM = .01
CONTACT_BLEND_END_NM = .03
POSITION_FULL_LOAD_NM = .25
MOTOR_DAMPING_NM_S_PER_RAD = .45
RETREAT_BLEND_START_RAD = math.radians(1.)
RETREAT_BLEND_END_RAD = math.radians(3.)
PALM_CUE_MOTORS = (5, 7)
PALM_CUE_TRAVEL_RAD = .016
PALM_CUE_DURATION_NS = 120_000_000
PALM_CUE_RELEASE_NS = 50_000_000


def surface_stiffness():
    lo, hi = CONFIG["driver_impedance"]["stiffness_nm_per_rad"]
    return lo + CONFIG["hardness"] * (hi - lo)


class ContactProxy:
    def __init__(self):
        self.previous = None
        self.blocks = {}

    def update(self, target, fingers, clear, *, deepening=()):
        """闭合保持原交互规则，伸展仅在目标接触加深时受阻。"""
        margin = CONFIG["release_margin_rad"]
        stiffness = surface_stiffness()
        for i, finger in enumerate(MOTOR_FINGER):
            if i in self.blocks:
                sign, surface, _ = self.blocks[i]
                if finger in clear or sign * (target[i] - surface) < -margin:
                    del self.blocks[i]
            elif self.previous is not None:
                change = target[i] - self.previous[i]
                if change > 1e-12 and finger in fingers:
                    self.blocks[i] = (1, self.previous[i], stiffness)
                elif change < -1e-12 and finger in deepening:
                    self.blocks[i] = (-1, self.previous[i], stiffness)
        self.previous = target.copy()

    def visible(self, target):
        result = target.copy()
        for i, (sign, surface, _) in self.blocks.items():
            if sign * (target[i] - surface) > 0:
                result[i] = surface
        return result

    def hold_swept(self, target, held, fingers):
        """把连续碰撞扫描实际保留的指节位置用作阻抗接触面。"""
        stiffness = surface_stiffness()
        for i, finger in enumerate(MOTOR_FINGER):
            delta = target[i] - held[i]
            if finger in fingers and abs(delta) > 1e-12:
                self.blocks[i] = (1 if delta > 0. else -1, held[i], stiffness)


class PalmCue:
    """在再次接触掌部时，用短时的小角度阻抗提示接触。"""

    def __init__(self, axes):
        self.axes = axes
        self.touching = False
        self.started = 0
        self.cleared_at = None
        self.goals = {}

    def blocks(self, touching, q, now):
        if not touching:
            if self.touching:
                self.cleared_at = now
            self.touching = False
            self.goals.clear()
            return {}
        if not self.touching:
            self.touching = True
            if self.cleared_at is not None and now - self.cleared_at < PALM_CUE_RELEASE_NS:
                return {}
            self.started = now
            for mid in PALM_CUE_MOTORS:
                low, high = self.axes[mid][1].model_rad_limits()
                angle = q[mid - 1]
                goal = angle - PALM_CUE_TRAVEL_RAD
                if goal < low:
                    goal = min(high, angle + PALM_CUE_TRAVEL_RAD)
                self.goals[mid - 1] = (angle, goal)
        elapsed = now - self.started
        if not 0 <= elapsed < PALM_CUE_DURATION_NS:
            return {}
        gain = math.sin(math.pi * elapsed / PALM_CUE_DURATION_NS)
        result = {}
        for i, (initial, goal) in self.goals.items():
            # 达到终点或操作者主动转动远离起点时解除该轴提示。
            sign = 1 if goal < initial else -1
            depth = sign * (q[i] - goal)
            if depth <= 0 or abs(q[i] - initial) > PALM_CUE_TRAVEL_RAD:
                continue
            surface = q[i] - sign * min(depth, PALM_CUE_TRAVEL_RAD)
            result[i] = (sign, surface, surface_stiffness() * gain)
        return result


def motor_command(motor, command, q, raw):
    """与 transfer/sim/sdk/real.py 的 command_impedance 使用相同的 MIT 换算。"""
    ratio = motor.sign * motor.scale
    delta = math.degrees(command.target_rad - q) / ratio
    target = raw + delta
    kp = min(500., max(0., command.k * motor.scale**2))
    kd = min(5., max(0., command.b * motor.scale**2))
    torque = command.tau_ff * ratio * CONFIG["torque_sign"][motor.motor_id - 1]
    if abs(target) > math.degrees(5.) - 1.:
        torque += kp * math.radians(delta)
        kp, target = 0., raw
    return target, kp, kd, 0., torque


class FeedbackLoop:
    refresh_on_contact = False

    def __init__(self, gateway, axes):
        self.gateway = gateway
        self.axes = axes
        self.ttl = int(CONFIG["command_ttl_s"] * 1e9)
        self.control_period_ns = round(1e9 / CONFIG["control_hz"])
        self.friction = FrictionCompensator(CONFIG["friction_comp"])
        self.active = False
        self.rearm_required = False
        self.interruption_count = 0
        self.last_interruption = None
        self.handoff_count = 0
        self.last_handoff_gap_ms = None
        self.submitted_until = 0
        self.message = "仿真反馈预览 · 真机出力关闭"
        self.reset()

    def reset(self):
        self.nominal = None
        self.sent_nm = None
        self.ready_since = None
        self.output_error = False
        self.palm = PalmCue(self.axes)

        self.planner = DriverImpedancePlanner(CONFIG["driver_impedance"], CONFIG["torque_limit_nm"])
        self.velocity = VelocityEstimator(CONFIG["velocity_filter_hz"])
        self.started = time.monotonic_ns()
        self.last_stamp = 0
        self.last_plan_key = None
        self.last_plan_at = 0
        self.deadline = 0

    def start(self, proxy_frame=None):
        if self.rearm_required:
            return
        if proxy_frame is not None:
            sample, simulated_at, valid = proxy_frame[:3]
            now = time.monotonic_ns()
            if (not valid or not all(0 < t <= now for t in (sample, simulated_at))
                    or now >= min(sample, simulated_at) + self.ttl
                    or self.refresh_on_contact and min(sample, simulated_at) + self.ttl - now < self.control_period_ns):
                self.message = "力反馈未启用 · 等待手套采集和仿真接触帧"
                return
        self.reset()
        if proxy_frame is not None and len(proxy_frame) == PROXY_FRAME_SIZE:
            # 出力启用时已经贴住的掌面从下一次接触开始提示。
            self.palm.touching = bool(proxy_frame[DIRECTION_END])
        self.active = True
        self.message = "力反馈待下发"

    def stop(self, reason="力反馈关闭"):
        self.release_output()
        self.pause(reason)
        self.rearm_required = False

    def release_output(self):
        self.gateway.release()
        self.submitted_until = 0

    def pause(self, reason):
        self.active = False
        self.reset()
        self.message = reason

    def interrupt(self, reason):
        if self.rearm_required:
            return
        trip = self.refresh_on_contact and self.active and self.sent_nm is not None
        if self.active:
            self.interruption_count += 1
            self.last_interruption = reason
        if trip:
            self.rearm_required = True
            self.release_output()
            self.pause(f"保护已卸力 · 先停止再重新启用 · {reason}")
        else:
            self.pause(f"暂停提交力反馈 · {reason}")

    def follow_request(self, q, raw, stamp, proxy_frame, requested, *, sample_stamps=None, encoder_position=None):
        """请求启用后等待连续有效帧，并沿用驱动阻抗的渐入过程。"""
        now = time.monotonic_ns()
        sample, simulated_at, valid = proxy_frame[:3]
        if requested and (self.output_error or self.rearm_required):
            return
        ready = (valid and all(0 < t <= now for t in (sample, simulated_at, stamp))
                 and 0 <= now - min(stamp, sample, simulated_at) < self.ttl)
        if not self.active and self.refresh_on_contact:
            ready = ready and min(stamp, sample, simulated_at) + self.ttl - now >= self.control_period_ns
        if not requested:
            if self.active or self.rearm_required:
                self.stop()
            elif self.message.startswith("已请求启用"):
                self.message = "仿真反馈预览 · 真机出力关闭"
            self.ready_since = None
        elif not ready:
            self.ready_since = None
            if not self.active:
                self.message = "已请求启用 · 等待有效手套和仿真接触帧"
        elif not self.active:
            if self.ready_since is None:
                self.ready_since = now
            if now - self.ready_since >= 180_000_000:
                self.start(proxy_frame)
            else:
                self.message = "已请求启用 · 正在检查连续帧"
        self.tick(q, raw, stamp, proxy_frame, sample_stamps=sample_stamps, encoder_position=encoder_position)

    def plan(self, q, dq, proxy_frame, ramp, dt, now, *, encoder_position=None):
        surfaces = proxy_frame[3:SURFACE_END]
        directions = proxy_frame[SURFACE_END:DIRECTION_END] or [1] * MOTOR_COUNT
        palm_contact = len(proxy_frame) > DIRECTION_END and bool(proxy_frame[DIRECTION_END])
        k = surface_stiffness()
        blocks = {i: (int(sign), angle, k) for i, (angle, sign) in enumerate(zip(surfaces, directions))
                  if math.isfinite(angle) and sign in (-1, 1)}
        palm_blocks = self.palm.blocks(palm_contact, q, now)
        cmds = self.planner.plan(q, dq, {**palm_blocks, **blocks}, self.friction.torque(dq), ramp=ramp)
        count = sum(cmds[i].k > 0. for i in blocks)
        extending = sum(sign < 0 and cmds[i].k > 0. for i, (sign, _, _) in blocks.items())
        message = f"MIT 指令已提交 · {count} 轴受阻"
        if extending:
            message += f" · {extending} 轴伸展受阻"
        if any(cmds[i].k > 0. for i in palm_blocks.keys() - blocks.keys()):
            message += " · 掌部触觉提示"
        return cmds, message

    def encode(self, commands, q, raw):
        return [motor_command(self.axes[i+1][1], cmd, q[i], raw[i+1]) for i, cmd in enumerate(commands)]

    def tick(self, q, raw, stamp, proxy_frame, *, sample_stamps=None, encoder_position=None):
        # 只用真实编码器样本与当前场景生成的接触帧续期。
        if self.rearm_required or self.output_error:
            return
        sample, simulated_at, valid = proxy_frame[:3]
        now = time.monotonic_ns()
        if not all(math.isfinite(t) and t <= now for t in (sample, simulated_at, stamp)):
            self.interrupt("输入时间戳无效")
            return
        input_expiry = int(min(stamp, sample, simulated_at)) + self.ttl
        if not valid or input_expiry <= now:
            if self.active:
                if stamp + self.ttl <= now:
                    source = "编码器采集样本过期"
                elif not valid or simulated_at + self.ttl <= now:
                    source = "仿真接触帧过期或不可用"
                else:
                    source = "仿真使用的编码器样本过期"
                self.interrupt(source)
            else:
                self.nominal = None
                self.last_stamp = 0
                self.last_plan_key = None
                self.last_plan_at = 0
                self.velocity.reset()
                self.palm = PalmCue(self.axes)
            return
        if self.active and 0 < self.deadline <= now:
            self.interrupt("上一批 MIT 指令已到期")
            return
        if self.refresh_on_contact and (len(proxy_frame) < FORCE_TORQUE_END
                or not np.isfinite(q).all() or not np.isfinite(proxy_frame[3:FORCE_INPUT_END]).all()
                or (encoder_position is not None and not np.isfinite(encoder_position).all())
                or (sample_stamps is not None and not all(0 <= now - t < self.ttl for t in sample_stamps))):
            self.interrupt("编码器或仿真接触帧数据不可用")
            return
        sensor_key = stamp if sample_stamps is None else tuple(sample_stamps)
        plan_key = (sensor_key, simulated_at if self.refresh_on_contact else 0)
        if plan_key == self.last_plan_key:
            return
        dt = (stamp - self.last_stamp) / 1e9 if self.last_stamp else 0
        motion_position = q if encoder_position is None else encoder_position
        if sample_stamps is not None:
            dq = self.velocity.update_samples(motion_position, sample_stamps)
        elif stamp != self.last_stamp:
            dq = self.velocity.update(motion_position, dt)
        else:
            dq = self.velocity.dq
        if self.refresh_on_contact:
            dt = (now-self.last_plan_at) / 1e9 if self.last_plan_at else 0.
            if self.active:
                dt = min(dt, self.control_period_ns / 1e9)
        self.last_stamp = stamp
        self.last_plan_key = plan_key
        self.last_plan_at = now
        ramp = (now-self.started) / (1e9 * CONFIG["feedback_ramp_s"]) if self.active else 1.
        cmds, message = self.plan(q, dq, proxy_frame, ramp, dt, now, encoder_position=encoder_position)
        self.nominal = [cmd.torque(angle, speed) for cmd, angle, speed in zip(cmds, q, dq)]
        if not self.active:
            return
        batch = self.encode(cmds, q, raw)
        if not np.isfinite(batch).all():
            self.interrupt("力反馈输出故障 · 无效指令")
            self.output_error = True
            return
        # 命令不能活过所使用的最旧输入，重复提交同一接触帧也不会续期。
        deadline = input_expiry
        try:
            if time.monotonic_ns() >= deadline:
                raise TimeoutError("G4 指令已过期")
            if getattr(type(self.gateway), "send_before", None) is not None:
                self.gateway.send_before(batch, deadline, self.deadline)
            else:
                self.gateway.send(batch, deadline)
            self.deadline = deadline
            self.submitted_until = deadline
            self.sent_nm = [kp * math.radians(target - raw[mid]) + ff
                            for mid, (target, kp, _, _, ff) in enumerate(batch, 1)]
            self.message = message
        except TimeoutError:
            self.interrupt("G4 指令已过期")
        except Exception as exc:
            self.interrupt(f"力反馈输出故障 · {exc}")
            self.output_error = True


def _validate_anchors(cfg):
    try:
        anchors = np.asarray(cfg["anchors_nm"], dtype=float)
        max_output = min(CONFIG["torque_limit_nm"]) * CONFIG["driver_impedance"]["torque_margin"]
        valid = (anchors.ndim == 2 and anchors.shape[1] == 2 and len(anchors) >= 2
                 and np.isfinite(anchors).all() and np.array_equal(anchors[0], [0., 0.]))
        if valid:
            x, y = anchors.T
            light = x <= CONTACT_BLEND_END_NM
            valid = (np.all(np.diff(x) > 0.) and np.all(np.diff(y) >= 0.) and np.all(y <= max_output)
                     and x[-1] >= CONTACT_BLEND_END_NM and np.array_equal(x[light], y[light])
                     and math.isclose(np.interp(CONTACT_BLEND_END_NM, x, y), CONTACT_BLEND_END_NM,
                                      rel_tol=0., abs_tol=1e-12))
    except (TypeError, KeyError, ValueError, IndexError):
        valid = False
    if not valid:
        raise ValueError("名义反馈曲线无效：要求有限、单调、原点为零、轻触原尺度且不超过有效限幅")
    return anchors.tolist()


def feedback_curves(profile, profile_config=None):
    """共享曲线与可选分轴曲线，配置和原生接口都用电机编号 J1–J12。"""
    if profile == "identity":
        return [], []
    profiles = CONFIG.get("feedback_profiles", {})
    if profile not in profiles and profile != "nominal":
        raise ValueError(f"未知反馈配置：{profile}")
    cfg = profile_config if profile_config is not None else profiles.get(profile)
    base = _validate_anchors(cfg)
    groups = []
    assigned = set()
    overrides = cfg.get("axis_curves", [])
    if not isinstance(overrides, list):
        raise ValueError("axis_curves 应为列表")
    for group in overrides:
        ids = group.get("motor_ids") if isinstance(group, dict) else None
        if (not isinstance(ids, list) or not ids
                or any(type(mid) is not int or not 1 <= mid <= MOTOR_COUNT for mid in ids)
                or len(set(ids)) != len(ids) or assigned.intersection(ids)):
            raise ValueError("分轴反馈曲线的电机编号应为 1–12 且不能重复分配")
        groups.append((ids, _validate_anchors(group)))
        assigned.update(ids)
    return base, groups


def _friction_params(cfg=None):
    raw = CONFIG.get("friction_comp") if cfg is None else cfg
    comp = FrictionCompensator(raw)
    return (comp.enabled, comp.coulomb.tolist(), comp.viscous.tolist(),
            comp.v_scale, comp.v_dead, comp.gain, comp.limit.tolist())


def build_contact_kernel(native, gains, curves, friction=None):
    limit, stiffness, position_limit = gains
    base, groups = curves
    kernel = native.ContactFeedback(limit, stiffness, position_limit, POSITION_DEADBAND_RAD, CONTACT_THRESHOLD_NM,
                                    CONTACT_BLEND_END_NM, POSITION_FULL_LOAD_NM, MOTOR_DAMPING_NM_S_PER_RAD,
                                    CONFIG["driver_impedance"]["release_lookahead_s"], base)
    for ids, curve in groups:
        for mid in ids:
            kernel.set_axis_curve(mid, curve)
    kernel.set_friction(*_friction_params(friction))
    return kernel


def contact_feedback_gains(axes):
    cfg = CONFIG["driver_impedance"]
    limits = np.asarray(CONFIG["torque_limit_nm"]) * cfg["torque_margin"]
    base_stiffness = np.minimum(cfg["k_max_nm_per_rad"],
                                surface_stiffness() * cfg["gain"] * np.asarray(cfg["joint_gain"]))
    # 只缩放同向受阻角差对应的前馈位置项，独立上限由 position_limit 控制。
    gain = cfg.get("position_compensation_gain", 1.)
    if type(gain) not in (int, float) or not math.isfinite(gain) or gain < 0.:
        raise ValueError("位置补偿倍率必须是有限的非负数")
    stiffness = base_stiffness * gain
    scales = np.asarray([axes[i + 1][1].scale for i in range(MOTOR_COUNT)])
    return limits, stiffness, cfg["position_compensation_max_nm"] / np.maximum(1., scales)


class ContactForceFeedbackLoop(FeedbackLoop):
    """将真实接触反力与同向位置补偿合成 G4 前馈力矩。"""

    refresh_on_contact = True

    def __init__(self, gateway, axes, *, native=None, profile="identity", profile_config=None):
        self.native = native
        self.profile = profile
        self.curves = feedback_curves(profile, profile_config)
        self.base_curve = np.asarray(self.curves[0]).T
        self.axis_curves = [(np.asarray(ids) - 1, np.asarray(curve).T) for ids, curve in self.curves[1]]
        self.grasp_revision = None
        super().__init__(gateway, axes)

    def match_grasp_frame(self, frame, revision):
        previous = self.grasp_revision
        self.grasp_revision = revision
        if previous is not None and previous != revision:
            # 普通中断会清除控制状态，驱动器可能仍持有已接受的指令。
            if self.active or self.submitted_until > time.monotonic_ns():
                self.release_output()
            if not (self.output_error or self.rearm_required):
                self.pause("场景更新 · 等待新接触帧")
        return frame is not None and len(frame) >= FORCE_FRAME_SIZE and frame[FORCE_ENCODER_END] == revision

    def reset(self):
        super().reset()
        self.slew_torque = np.zeros(MOTOR_COUNT)
        self.filtered_torque = np.zeros(MOTOR_COUNT)
        self.position_torque = np.zeros(MOTOR_COUNT)
        self.mapped_contact = np.zeros(MOTOR_COUNT)
        self.contact_damping = np.zeros(MOTOR_COUNT)
        self.friction_torque = np.zeros(MOTOR_COUNT)
        self.friction_blend = np.ones(MOTOR_COUNT)
        self.contact_gate = np.zeros(MOTOR_COUNT, dtype=bool)
        self.frame_weight = np.ones(MOTOR_COUNT)
        self.friction = FrictionCompensator(CONFIG.get("friction_comp"))
        cfg = CONFIG["driver_impedance"]
        self.limit, self.position_stiffness, self.position_limit = contact_feedback_gains(self.axes)
        self.rebound_lookahead = cfg["release_lookahead_s"]
        if self.native is not None:
            self.native_feedback = build_contact_kernel(
                self.native, (self.limit, self.position_stiffness, self.position_limit), self.curves,
                CONFIG.get("friction_comp"))

    def plan(self, q, dq, force_frame, ramp, dt, now, *, encoder_position=None):
        reaction = np.asarray(force_frame[3:FORCE_TORQUE_END])
        # 兼容早期回放帧。当前接触帧携带实际驱动仿真的编码器角度。
        sim_position = np.asarray(force_frame[FORCE_VELOCITY_END:FORCE_POSITION_END]) if len(force_frame) >= FORCE_POSITION_END else q
        sampled_position = np.asarray(force_frame[FORCE_POSITION_END:FORCE_INPUT_END]) if len(force_frame) >= FORCE_INPUT_END else q
        movement = q - sampled_position
        if encoder_position is not None and len(force_frame) >= FORCE_ENCODER_END:
            sampled_encoder = np.asarray(force_frame[FORCE_INPUT_END:FORCE_ENCODER_END])
            if np.isfinite(sampled_encoder).all():
                movement = np.asarray(encoder_position) - sampled_encoder
        # 正值表示手套已沿该帧反力的方向退过仿真使用的位置。
        sign_r = np.where(reaction < 0., -1., 1.)
        retreat = sign_r * movement
        live_encoder = np.asarray(encoder_position) if encoder_position is not None else q
        remaining_depth = np.maximum(0., -sign_r * (live_encoder - sim_position))
        effective_retreat = retreat - np.maximum(0., remaining_depth - .03)
        progress = np.clip((effective_retreat - RETREAT_BLEND_START_RAD) /
                           (RETREAT_BLEND_END_RAD - RETREAT_BLEND_START_RAD), 0., 1.)
        frame_weight = np.where(reaction == 0., 1., 1. - progress**2 * (3. - 2. * progress))
        self.frame_weight = frame_weight.copy()
        self.contact_gate = np.abs(reaction) >= CONTACT_THRESHOLD_NM
        clipped = np.clip(reaction, -self.limit, self.limit)
        onset = np.clip((np.abs(clipped) - CONTACT_THRESHOLD_NM) / (CONTACT_BLEND_END_NM - CONTACT_THRESHOLD_NM), 0., 1.)
        contact = clipped * onset**2 * (3. - 2. * onset)
        if self.base_curve.size:
            raw_contact = contact
            contact = np.sign(raw_contact) * np.interp(np.abs(raw_contact), *self.base_curve)
            for indices, curve in self.axis_curves:
                raw = raw_contact[indices]
                contact[indices] = np.sign(raw) * np.interp(np.abs(raw), *curve)
        self.mapped_contact = contact
        dt = min(self.ttl / 1e9, dt if dt > 0. else 1. / CONFIG["control_hz"])
        if self.native is not None:
            torque, position, loaded = self.native_feedback.plan(q, dq, reaction, sim_position, sampled_position,
                                                                frame_weight, ramp, dt)
            self.position_torque = np.asarray(position)
            self.contact_damping = np.asarray(self.native_feedback.damping())
            self.friction_torque = np.asarray(self.native_feedback.friction())
        else:
            torque, loaded = self.plan_python(q, dq, reaction, sim_position, sampled_position, frame_weight, ramp, dt)
        commands = [ImpedanceCommand(target_rad=0., tau_ff=float(tau)) for tau in torque]
        count = np.count_nonzero(self.position_torque)
        return commands, f"MIT 力矩已提交 · {loaded} 轴接触 · {count} 轴叠加位置项"

    def plan_python(self, q, dq, reaction, sim_position, sampled_position, frame_weight, ramp, dt):
        limit = self.limit
        loaded = np.abs(reaction) >= CONTACT_THRESHOLD_NM
        reaction = np.clip(reaction, -limit, limit)
        onset = np.clip((np.abs(reaction) - CONTACT_THRESHOLD_NM) / (CONTACT_BLEND_END_NM - CONTACT_THRESHOLD_NM), 0., 1.)
        blend = onset**2 * (3. - 2. * onset)
        contact = self.mapped_contact
        sampled_error = sim_position - sampled_position
        direction = np.sign(sampled_error)
        speed = np.clip(dq, -2., 2.)
        # 同帧角差判定受阻，最新实测回撤时只缩小已经观测到的角差。
        live_gap = direction * (sim_position - q) - self.rebound_lookahead * np.maximum(direction * speed, 0.)
        error = direction * np.minimum(np.abs(sampled_error), np.maximum(live_gap, 0.))
        position = np.sign(error) * np.maximum(np.abs(error) - POSITION_DEADBAND_RAD, 0.) * self.position_stiffness
        load = np.clip((np.abs(reaction) - CONTACT_THRESHOLD_NM) / (POSITION_FULL_LOAD_NM - CONTACT_THRESHOLD_NM), 0., 1.)
        position *= load**2 * (3. - 2. * load)
        # 位置补偿使用独立上限，同时给物理接触反力保留合成输出空间。
        position_budget = np.minimum(self.position_limit, limit - np.abs(contact))
        self.position_torque = np.where(loaded & (position * reaction > 0.),
                                        np.sign(position) * np.minimum(np.abs(position), position_budget), 0.)
        sign_r = np.where(reaction < 0., -1., 1.)
        retreat_speed = np.maximum(0., sign_r * dq)
        press_speed = np.maximum(0., -sign_r * dq)
        damp_coeff = np.where((MOTOR_DAMPING_NM_S_PER_RAD > 0.) & (press_speed > 0.),
                              max(MOTOR_DAMPING_NM_S_PER_RAD, .90), MOTOR_DAMPING_NM_S_PER_RAD)
        damped = contact + self.position_torque - blend * damp_coeff * speed
        damped = damped / (1. + retreat_speed / .55)
        # 速度阻尼抑制真机回弹，接触力仍决定输出方向。
        target = np.where(damped * reaction > 0, np.clip(damped, -limit, limit), 0.) * np.clip(ramp, 0., 1.)
        target *= frame_weight
        self.position_torque *= frame_weight
        # 回撤时立即限制旧帧存下的力矩，接触更新后由原有斜率重新建立输出。
        cap = limit * frame_weight
        np.clip(self.slew_torque, -cap, cap, out=self.slew_torque)
        np.clip(self.filtered_torque, -cap, cap, out=self.filtered_torque)
        reset = ~loaded | (self.filtered_torque * reaction < 0.)
        self.slew_torque[reset] = self.filtered_torque[reset] = 0.
        unloading = (np.abs(target) < np.abs(self.slew_torque)) & (target * self.slew_torque >= 0.)
        press_blend = np.clip((press_speed - .08) / .35, 0., 1.)
        rate = np.where(unloading, 160., 50. + press_blend * (115. - 50.))
        self.slew_torque += np.clip(target - self.slew_torque, -rate * dt, rate * dt)
        self.filtered_torque += -math.expm1(-2. * math.pi * 15. * dt) * (self.slew_torque - self.filtered_torque)
        self.slew_torque[~loaded] = self.filtered_torque[~loaded] = 0.
        excess_retreat = np.maximum(0., retreat_speed - .6)
        kd_fade = 1. / (1. + excess_retreat / .4)
        self.contact_damping = np.where(
            loaded & (self.filtered_torque != 0.),
            .28 * np.minimum(1., np.abs(self.filtered_torque) / .8) * frame_weight * kd_fade, 0.)
        # 自由空间动摩擦力补偿（仅非接触轴，脱离接触后 80 ms 平滑渐入避免松手弹回）。
        clamped_ramp = float(np.clip(ramp, 0., 1.))
        raw_fric = self.friction.torque(dq) * clamped_ramp * self.friction_blend
        target_fric = np.where(loaded, 0., raw_fric)
        alpha_fric = -math.expm1(-2. * math.pi * 20. * dt)
        self.friction_torque = np.where(loaded, 0.,
                                        self.friction_torque + alpha_fric * (target_fric - self.friction_torque))
        self.friction_torque[(~loaded) & (target_fric == 0.) & (np.abs(self.friction_torque) < 1e-4)] = 0.
        self.friction_blend = np.where(loaded, 0., np.minimum(1., self.friction_blend + dt / .08))
        output = np.where(loaded, self.filtered_torque, self.friction_torque)
        return output, np.count_nonzero(loaded)

    def encode(self, commands, q, raw):
        # MIT 帧位置与速度目标及 KP 均为零，仅输出前馈力矩与受载接触轴的安全本地阻尼 KD（<= 0.28）。
        limit = min(20., max(CONFIG["torque_limit_nm"])) * CONFIG["driver_impedance"]["torque_margin"]
        batch = []
        for i, cmd in enumerate(commands):
            motor = self.axes[i + 1][1]
            torque = cmd.tau_ff * motor.sign * motor.scale * CONFIG["torque_sign"][i]
            kd = float(np.clip(self.contact_damping[i] * motor.scale**2, 0., .28))
            batch.append((0., 0., kd, 0., float(np.clip(torque, -limit, limit))))
        return batch
