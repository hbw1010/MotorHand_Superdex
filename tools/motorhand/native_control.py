"""创建实时线程独占的反馈状态机。配置仅在启动时跨语言边界。"""
from .feedback import (CONFIG, MOTOR_COUNT, RETREAT_BLEND_END_RAD, RETREAT_BLEND_START_RAD,
                       build_contact_kernel, contact_feedback_gains, feedback_curves)


def make_feedback_control(axes, native, profile="identity"):
    cfg = CONFIG["driver_impedance"]
    kernel = build_contact_kernel(native, contact_feedback_gains(axes), feedback_curves(profile))
    ratios = [axes[i+1][1].sign * axes[i+1][1].scale * CONFIG["torque_sign"][i]
              for i in range(MOTOR_COUNT)]
    motor_limit = min(20., max(CONFIG["torque_limit_nm"])) * cfg["torque_margin"]
    return native.FeedbackControl(kernel, ratios, motor_limit, int(CONFIG["command_ttl_s"] * 1e9),
                                  round(1e9 / CONFIG["control_hz"]), CONFIG["feedback_ramp_s"],
                                  CONFIG["velocity_filter_hz"], RETREAT_BLEND_START_RAD, RETREAT_BLEND_END_RAD)
