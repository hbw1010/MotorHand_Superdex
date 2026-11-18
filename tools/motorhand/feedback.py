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
FORCE_FRAME_SIZE = FORCE_VELOCITY_END + MOTOR_COUNT
POSITION_DEADBAND_RAD = .002
CONTACT_THRESHOLD_NM = .01
POSITION_FULL_LOAD_NM = .25
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
    def __init__(self, gateway, axes):
        self.gateway = gateway
        self.axes = axes
        self.ttl = int(CONFIG["command_ttl_s"] * 1e9)
        self.friction = FrictionCompensator(CONFIG["friction_comp"])
        self.active = False
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
        self.deadline = 0

    def start(self, proxy_frame=None):
        if proxy_frame is not None:
            sample, simulated_at, valid = proxy_frame[:3]
            if not valid or time.monotonic_ns() >= min(sample, simulated_at) + self.ttl:
                self.message = "力反馈未启用 · 等待手套采集和仿真接触帧"
                return
        self.reset()
        if proxy_frame is not None and len(proxy_frame) == PROXY_FRAME_SIZE:
            # 出力启用时已经贴住的掌面从下一次接触开始提示。
            self.palm.touching = bool(proxy_frame[DIRECTION_END])
        self.active = True
        self.message = "力反馈待下发"

    def stop(self, reason="力反馈关闭"):
        self.gateway.release()
        self.pause(reason)

    def pause(self, reason):
        self.active = False
        self.reset()
        self.message = reason

    def follow_request(self, q, raw, stamp, proxy_frame, requested):
        """请求启用后等待连续有效帧，并沿用驱动阻抗的渐入过程。"""
        now = time.monotonic_ns()
        sample, simulated_at, valid = proxy_frame[:3]
        if requested and self.output_error:
            return
        ready = (valid and stamp > 0 and sample > 0 and simulated_at > 0
                 and 0 <= now - min(stamp, sample, simulated_at) < self.ttl)
        if not requested:
            if self.active:
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
        self.tick(q, raw, stamp, proxy_frame)

    def plan(self, q, dq, proxy_frame, ramp, dt, now):
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

    def tick(self, q, raw, stamp, proxy_frame):
        # 只用真实编码器样本与当前场景生成的接触帧续期。
        sample, simulated_at, valid = proxy_frame[:3]
        now = time.monotonic_ns()
        input_expiry = int(min(stamp, sample, simulated_at)) + self.ttl
        if not valid or input_expiry <= now or (self.deadline and self.deadline <= now):
            if self.active:
                if not valid or simulated_at + self.ttl <= now:
                    source = "仿真接触帧过期或不可用"
                elif min(stamp, sample) + self.ttl <= now:
                    source = "手套样本过期"
                else:
                    source = "上一批 MIT 指令已到期"
                self.pause(f"暂停提交力反馈 · {source}")
            else:
                self.nominal = None
                self.last_stamp = 0
                self.velocity.reset()
                self.palm = PalmCue(self.axes)
            return
        if stamp == self.last_stamp:
            return
        dt = (stamp - self.last_stamp) / 1e9 if self.last_stamp else 0
        dq = self.velocity.update(q, dt)
        self.last_stamp = stamp
        ramp = (now-self.started) / (1e9 * CONFIG["feedback_ramp_s"]) if self.active else 1.
        cmds, message = self.plan(q, dq, proxy_frame, ramp, dt, now)
        self.nominal = [cmd.torque(angle, speed) for cmd, angle, speed in zip(cmds, q, dq)]
        if not self.active:
            return
        batch = self.encode(cmds, q, raw)
        if not np.isfinite(batch).all():
            self.pause("力反馈输出故障 · 无效指令")
            self.output_error = True
            return
        # 命令不能活过所使用的最旧输入，重复提交同一接触帧也不会续期。
        deadline = input_expiry
        try:
            if time.monotonic_ns() >= deadline:
                raise TimeoutError("G4 指令已过期")
            self.gateway.send(batch, deadline)
            self.deadline = deadline
            self.sent_nm = [kp * math.radians(target - raw[mid]) + ff
                            for mid, (target, kp, _, _, ff) in enumerate(batch, 1)]
            self.message = message
        except TimeoutError:
            self.pause("暂停提交力反馈 · G4 指令已过期")
        except Exception as exc:
            self.pause(f"力反馈输出故障 · {exc}")
            self.output_error = True


class ContactForceFeedbackLoop(FeedbackLoop):
    """将真实接触反力与同向位置补偿合成 G4 前馈力矩。"""

    gain = 1.

    def reset(self):
        super().reset()
        self.slew_torque = np.zeros(MOTOR_COUNT)
        self.filtered_torque = np.zeros(MOTOR_COUNT)
        self.position_torque = np.zeros(MOTOR_COUNT)
        cfg = CONFIG["driver_impedance"]
        self.position_stiffness = np.minimum(cfg["k_max_nm_per_rad"],
                                              surface_stiffness() * cfg["gain"] * np.asarray(cfg["joint_gain"]))

    def plan(self, q, dq, force_frame, ramp, dt, now):
        reaction = np.asarray(force_frame[3:FORCE_TORQUE_END])
        sim_velocity = np.asarray(force_frame[FORCE_TORQUE_END:FORCE_VELOCITY_END])
        # 较短的旧回放帧只含力矩与速度，仍可用于纯接触力回放。
        sim_position = np.asarray(force_frame[FORCE_VELOCITY_END:FORCE_FRAME_SIZE]) if len(force_frame) == FORCE_FRAME_SIZE else q
        limit = np.asarray(CONFIG["torque_limit_nm"]) * CONFIG["driver_impedance"]["torque_margin"]
        loaded = np.abs(reaction) >= CONTACT_THRESHOLD_NM
        velocity_error = np.clip(dq - sim_velocity, -2., 2.)
        reaction = np.clip(reaction, -limit, limit)
        # 取 Franka 的力矩/速度 Sigmoid 阻尼和随载荷退出阻尼的权重。
        tau_sigmoid = 1. / (1. + np.exp(-.8 * np.abs(reaction)))
        vel_sigmoid = 1. / (1. + np.exp(-.5 * np.abs(velocity_error)))
        damping = .05 + .45 * (.6 * tau_sigmoid + .4 * vel_sigmoid)
        damping *= np.exp(-(reaction / .18)**2)
        contact = np.clip(self.gain * reaction, -limit, limit)
        error = sim_position - q
        position = np.sign(error) * np.maximum(np.abs(error) - POSITION_DEADBAND_RAD, 0.) * self.position_stiffness
        load = np.clip((np.abs(reaction) - CONTACT_THRESHOLD_NM) / (POSITION_FULL_LOAD_NM - CONTACT_THRESHOLD_NM), 0., 1.)
        position *= self.gain * load**2 * (3. - 2. * load)
        # 真实接触决定位置项的方向与渐入，接触和位置两项共用驱动力矩上限。
        self.position_torque = np.where(loaded & (position * reaction > 0.),
                                        np.sign(position) * np.minimum(np.abs(position), limit - np.abs(contact)), 0.)
        damped = contact + self.position_torque - damping * velocity_error
        # 速度差阻尼可以减弱反力，但不能让实际接触的反馈方向翻转。
        target = np.where(damped * reaction > 0, np.clip(damped, -limit, limit), 0.) * np.clip(ramp, 0., 1.)
        reset = ~loaded | (self.filtered_torque * reaction < 0.)
        self.slew_torque[reset] = self.filtered_torque[reset] = 0.
        dt = min(self.ttl / 1e9, dt if dt > 0. else 1. / CONFIG["control_hz"])
        self.slew_torque += np.clip(target - self.slew_torque, -60. * dt, 60. * dt)
        self.filtered_torque += (1. - math.exp(-2. * math.pi * 25. * dt)) * (self.slew_torque - self.filtered_torque)
        self.slew_torque[~loaded] = self.filtered_torque[~loaded] = 0.
        commands = [ImpedanceCommand(target_rad=0., tau_ff=float(tau)) for tau in self.filtered_torque]
        count = np.count_nonzero(self.position_torque)
        return commands, f"MIT 力矩已提交 · {np.count_nonzero(loaded)} 轴接触 · {count} 轴叠加位置项"

    def encode(self, commands, q, raw):
        # MIT 帧带位置和速度字段，KP/KD 为零，只让前馈力矩生效。
        limit = 10. * CONFIG["driver_impedance"]["torque_margin"]
        batch = []
        for i, cmd in enumerate(commands):
            motor = self.axes[i + 1][1]
            torque = cmd.tau_ff * motor.sign * motor.scale * CONFIG["torque_sign"][i]
            batch.append((0., 0., 0., 0., float(np.clip(torque, -limit, limit))))
        return batch
