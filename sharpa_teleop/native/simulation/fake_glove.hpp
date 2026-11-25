#pragma once
#include "common/glove_device.hpp"
namespace motorhand::native::simulation {
// 20ms独立采样的无硬件手套（并列支持 12-DOF 与 16-DOF）。输出仅保存名义值，不模拟人手受力动力学。
template <std::size_t G = 12>
class FakeGloveT final : public GloveDeviceT<G> {
public:
    using VecG = GloveVec<G>;
    using Frame = GloveFrameT<G>;
    using ImpedanceCmd = DriverImpedanceCommandT<G>;
    using ImpedanceSub = DriverImpedanceSubmissionT<G>;

    explicit FakeGloveT(std::array<EncosMotorCalibration, G> calibration);
    ~FakeGloveT() override;
    FakeGloveT(const FakeGloveT&) = delete;
    FakeGloveT& operator=(const FakeGloveT&) = delete;
    void connect() override;
    Frame read() override;
    std::array<EncosMotorCalibration, G> calibration() const override;
    std::array<std::uint64_t, G> last_command_write_ns() const override;
    void home() override;
    void enable_force() override;
    VecG write_position(const VecG&, std::uint64_t original_deadline_ns) override;
    VecG write_torque(const VecG&, std::uint64_t original_deadline_ns) override;
    ImpedanceSub write_impedance(const ImpedanceCmd&, std::uint64_t original_deadline_ns) override;
    void disable() override;
    void close() override;
    void set_manual_target(const VecG& motor_rad);
    VecG last_nominal_nm() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

using FakeGlove = FakeGloveT<12>;
using FakeGlove12 = FakeGloveT<12>;
using FakeGlove16 = FakeGloveT<16>;
}
