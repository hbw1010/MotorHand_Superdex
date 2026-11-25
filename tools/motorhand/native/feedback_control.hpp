#pragma once

#include "contact_feedback.hpp"

#include <cstdint>
#include <numbers>
#include <string>

namespace ve33 {

template <size_t Dof>
using StampsT = std::array<int64_t, Dof>;
template <size_t Dof>
using ForceFrameT = std::array<double, 3 + 5 * Dof + 1>;

using Stamps = StampsT<12>;
using ForceFrame = ForceFrameT<12>;
using Stamps12 = StampsT<12>;
using Stamps16 = StampsT<16>;
using ForceFrame12 = ForceFrameT<12>;
using ForceFrame16 = ForceFrameT<16>;

// 状态由一个实时循环独占。motor 在 need_send 时是候选指令，sent 只在 SDK 接受后更新。
template <size_t Dof>
struct ControlStateT {
    static constexpr size_t dof = Dof;
    using AxesType = AxesT<Dof>;
    bool active = false, valid_nominal = false, valid_sent = false, output_error = false;
    bool need_send = false, need_release = false, rearm_required = false;
    int64_t deadline = 0, submitted_until = 0;
    int interruption_count = 0, handoff_count = 0, loaded_count = 0, position_count = 0;
    double last_handoff_gap_ms = 0.;
    std::string message = "仿真反馈预览 · 真机出力关闭", last_interruption;
    AxesType nominal{}, motor{}, motor_kd{}, friction{}, sent{}, position{}, mapped_contact{}, frame_weight{}, dq{};
    std::array<bool, Dof> gate{};
};

template <size_t Dof>
class FeedbackControlT {
public:
    static constexpr size_t dof = Dof;
    using AxesType = AxesT<Dof>;
    using StampsType = StampsT<Dof>;
    using ForceFrameType = ForceFrameT<Dof>;
    using KernelType = ContactFeedbackT<Dof>;
    using StateType = ControlStateT<Dof>;

    FeedbackControlT(KernelType kernel, AxesType ratio, double motor_limit, int64_t ttl_ns,
                     int64_t period_ns, double ramp_s, double velocity_hz,
                     double retreat_start, double retreat_end)
        : kernel_(std::move(kernel)), ratio_(ratio), motor_limit_(motor_limit),
          ttl_(ttl_ns), period_(period_ns), ramp_s_(ramp_s), velocity_hz_(velocity_hz),
          retreat_start_(retreat_start), retreat_end_(retreat_end) {
        if (!finite(ratio_) || !std::isfinite(motor_limit_) || motor_limit_ <= 0. ||
            ttl_ <= 0 || period_ <= 0 || period_ > ttl_ || !std::isfinite(ramp_s_) || ramp_s_ <= 0. ||
            !std::isfinite(velocity_hz_) || velocity_hz_ <= 0. ||
            !std::isfinite(retreat_start_) || !std::isfinite(retreat_end_) ||
            retreat_start_ < 0. || retreat_end_ <= retreat_start_) {
            throw std::invalid_argument("无效的反馈周期或轴参数");
        }
        clear_planner();
    }

    const StateType& state() const { return state_; }
    int64_t ttl_ns() const { return ttl_; }

    const StateType& stop(const std::string& reason = "力反馈关闭") {
        clear_planner();
        state_.active = false;
        state_.output_error = false;
        state_.rearm_required = false;
        state_.need_release = true;
        state_.message = reason;
        return state_;
    }

    // SDK 的 release 必须真的执行后才能清除尚未到期的批次。
    void released() {
        state_.need_release = false;
        state_.submitted_until = 0;
        state_.valid_sent = false;
    }

    const StateType& missing(int64_t /*now*/, bool requested, const std::string& reason) {
        begin(requested);
        if (state_.rearm_required || state_.output_error) return state_;
        interrupt(reason);
        if (!requested && !state_.output_error) state_.message = "仿真反馈预览 · 真机出力关闭";
        return state_;
    }

    const StateType& step(int64_t now, const AxesType& q, const AxesType& encoder, const StampsType& stamps,
                          const ForceFrameType& frame, bool requested, int64_t revision) {
        begin(requested);
        if (state_.rearm_required || state_.output_error) return state_;
        if (!has_revision_ || revision_ != revision) {
            if (has_revision_) {
                const bool held = state_.active || state_.submitted_until > now;
                interrupt("场景更新 · 等待新接触帧", false);
                state_.need_release |= held;
            }
            has_revision_ = true;
            revision_ = revision;
        }
        const int64_t stamp = *std::min_element(stamps.begin(), stamps.end());
        if (!finite(q) || !finite(encoder) || !fresh(stamp, now) ||
            *std::max_element(stamps.begin(), stamps.end()) > now) {
            interrupt("编码器采集样本过期或不可用");
            return state_;
        }
        // double 时间戳来自原共享帧，只在已有可表示的单调时钟范围内转为 int64。
        if (frame[2] != 1. || frame.back() != static_cast<double>(revision) ||
            !frame_stamp_valid(frame[1], now) || !fresh(static_cast<int64_t>(frame[1]), now)) {
            interrupt("仿真接触帧过期或不可用");
            return state_;
        }
        if (!frame_stamp_valid(frame[0], now) || !fresh(static_cast<int64_t>(frame[0]), now)) {
            interrupt("仿真使用的编码器样本过期");
            return state_;
        }
        const auto simulated = static_cast<int64_t>(frame[1]);
        const int64_t oldest = std::min({stamp, static_cast<int64_t>(frame[0]), simulated});
        const int64_t remaining = ttl_ - (now - oldest);
        AxesType reaction, sim, sampled, source_encoder;
        std::copy_n(frame.begin() + 3, Dof, reaction.begin());
        std::copy_n(frame.begin() + 3 + 2 * Dof, Dof, sim.begin());
        std::copy_n(frame.begin() + 3 + 3 * Dof, Dof, sampled.begin());
        std::copy_n(frame.begin() + 3 + 4 * Dof, Dof, source_encoder.begin());
        if (!finite(reaction) || !finite(sim) || !finite(sampled) || !finite(source_encoder)) {
            interrupt("仿真接触帧数据不可用");
            return state_;
        }
        if (!requested) state_.message = "仿真反馈预览 · 真机出力关闭";
        if (requested && !state_.active) {
            if (remaining < period_) ready_since_ = 0;
            else if (!ready_since_) ready_since_ = now;
            if (ready_since_ && now - ready_since_ >= 180'000'000) {
                clear_planner();
                started_ = now;
                state_.active = true;
            } else state_.message = "已请求启用 · 正在检查连续帧";
        }
        if (state_.active && state_.submitted_until > 0 && now >= state_.submitted_until && state_.valid_sent) {
            // 看门狗可能已卸力；新物理帧不能自动重启原来的负载。
            interrupt("上一批 MIT 指令已到期");
            return state_;
        }
        if (have_sample_ && stamps == last_stamps_ && simulated == last_frame_) return state_;

        // 经过调度停顿后仍按单次更新渐入；速度和指令期限使用实际采样时间。
        const int64_t max_dt = state_.active ? period_ : ttl_;
        const auto dt_ns = last_tick_ ? std::clamp(now - last_tick_, int64_t(1), max_dt) : period_;
        update_velocity(encoder, stamps);
        last_tick_ = now;
        last_frame_ = simulated;
        for (size_t i = 0; i < Dof; ++i) {
            const double sign_r = std::copysign(1., reaction[i]);
            const double retreat = sign_r * (encoder[i] - source_encoder[i]);
            // 仍在深度按压物体（限位前编码器角距仿真表面余量充足）时避免单帧微幅让步误砍归零引发锯齿震荡；
            // 退近表面或真张手时立即按 1°–3° 衰减旧帧反力。
            const double remaining_depth = std::max(0., -sign_r * (encoder[i] - sim[i]));
            const double effective_retreat = retreat - std::max(0., remaining_depth - 0.03);
            const double t = std::clamp((effective_retreat - retreat_start_) / (retreat_end_ - retreat_start_), 0., 1.);
            state_.frame_weight[i] = reaction[i] == 0. ? 1. : 1. - t * t * (3. - 2. * t);
        }
        const double ramp = state_.active ? (now - started_) * 1e-9 / ramp_s_ : 1.;
        std::tie(state_.nominal, state_.position, state_.loaded_count) =
            kernel_.plan(q, state_.dq, reaction, sim, sampled, state_.frame_weight, ramp, dt_ns * 1e-9);
        state_.gate = kernel_.gate();
        state_.mapped_contact = kernel_.mapped();
        state_.friction = kernel_.friction();
        state_.position_count = std::count_if(state_.position.begin(), state_.position.end(),
                                               [](double v) { return v != 0.; });
        if (!finite(state_.nominal) || !finite(state_.dq)) {
            failed(false);
            return state_;
        }
        state_.valid_nominal = true;
        if (state_.active) {
            for (size_t i = 0; i < Dof; ++i) {
                state_.motor[i] = std::clamp(state_.nominal[i] * ratio_[i], -motor_limit_, motor_limit_);
                state_.motor_kd[i] = std::clamp(kernel_.damping()[i] * ratio_[i] * ratio_[i], 0., 0.28);
            }
            state_.deadline = oldest + ttl_;
            state_.need_send = true;
        }
        return state_;
    }

    void accepted() {
        if (!state_.need_send) return;
        state_.valid_sent = true;
        state_.sent = state_.motor;
        state_.submitted_until = state_.deadline;
        state_.need_send = false;
        state_.message = "MIT 力矩已提交";
    }

    void failed(bool expired) {
        interrupt(expired ? "G4 指令已过期" : "力反馈输出故障 · 无效指令");
        state_.output_error = !expired;
    }

private:
    static bool finite(const AxesType& a) {
        return std::all_of(a.begin(), a.end(), [](double v) { return std::isfinite(v); });
    }
    static bool frame_stamp_valid(double stamp, int64_t now) {
        return std::isfinite(stamp) && stamp >= 1. && stamp <= static_cast<double>(now) &&
               stamp < static_cast<double>(INT64_MAX);
    }
    bool fresh(int64_t stamp, int64_t now) const { return stamp > 0 && stamp <= now && now - stamp < ttl_; }

    void clear_planner() {
        kernel_.reset();
        state_.valid_nominal = state_.valid_sent = state_.need_send = false;
        state_.nominal = state_.motor = state_.motor_kd = state_.friction = state_.sent = state_.position = state_.mapped_contact = state_.dq = {};
        state_.gate.fill(false);
        state_.frame_weight.fill(1.);
        state_.loaded_count = state_.position_count = 0;
        state_.deadline = started_ = ready_since_ = last_tick_ = 0;
        have_sample_ = false;
    }
    void begin(bool requested) {
        state_.need_send = false;
        if (!requested && requested_) stop();
        requested_ = requested;
    }
    void interrupt(const std::string& reason, bool count = true) {
        const bool trip = count && state_.active && state_.valid_sent;
        if (count && state_.active) {
            ++state_.interruption_count;
            state_.last_interruption = reason;
        }
        if (state_.active || state_.valid_nominal || ready_since_) clear_planner();
        state_.active = false;
        if (trip) {
            state_.rearm_required = true;
            state_.need_release = true;
            state_.message = "保护已卸力 · 先停止再重新启用 · " + reason;
        } else {
            state_.message = reason;
        }
    }
    void update_velocity(const AxesType& encoder, const StampsType& stamps) {
        if (have_sample_) {
            for (size_t i = 0; i < Dof; ++i) {
                const auto delta = stamps[i] - last_stamps_[i];
                if (delta == 0) continue;
                const double dt = delta * 1e-9;
                if (delta < 0 || dt > .2) state_.dq[i] = 0.;
                else {
                    const double alpha = -std::expm1(-2. * std::numbers::pi * velocity_hz_ * dt);
                    state_.dq[i] += alpha * ((encoder[i] - last_encoder_[i]) / dt - state_.dq[i]);
                }
                last_encoder_[i] = encoder[i];
            }
        } else {
            last_encoder_ = encoder;
            have_sample_ = true;
        }
        last_stamps_ = stamps;
    }
    KernelType kernel_;
    AxesType ratio_, last_encoder_{};
    double motor_limit_;
    int64_t ttl_, period_;
    double ramp_s_, velocity_hz_, retreat_start_, retreat_end_;
    StateType state_;
    int64_t started_ = 0, ready_since_ = 0, last_tick_ = 0, last_frame_ = 0;
    int64_t revision_ = 0;
    bool requested_ = false, have_sample_ = false, has_revision_ = false;
    StampsType last_stamps_{};
};

using ControlState = ControlStateT<12>;
using ControlState12 = ControlStateT<12>;
using ControlState16 = ControlStateT<16>;
using FeedbackControl = FeedbackControlT<12>;
using FeedbackControl12 = FeedbackControlT<12>;
using FeedbackControl16 = FeedbackControlT<16>;

} // namespace ve33
