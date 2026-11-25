#pragma once
#include "encos_driver.hpp"
#include <memory>
namespace motorhand::native {
// 共同会话和唯一输出线程所需的 G 轴接口（并列支持 12-DOF 与 16-DOF）；原始收包时间由设备拥有。
template <std::size_t G = 12>
class GloveDeviceT {
public:
    using VecG = GloveVec<G>;
    using Frame = GloveFrameT<G>;
    using ImpedanceCmd = DriverImpedanceCommandT<G>;
    using ImpedanceSub = DriverImpedanceSubmissionT<G>;

    virtual ~GloveDeviceT() = default;
    virtual void connect() = 0;
    virtual Frame read() = 0;
    virtual std::array<EncosMotorCalibration, G> calibration() const = 0;
    virtual std::array<std::uint64_t, G> last_command_write_ns() const = 0;
    virtual void home() = 0;
    virtual void mark_homed() { home(); }
    virtual void enable_force() = 0;
    virtual VecG write_position(const VecG&, std::uint64_t original_deadline_ns) = 0;
    virtual VecG write_torque(const VecG&, std::uint64_t original_deadline_ns) = 0;
    virtual ImpedanceSub write_impedance(const ImpedanceCmd&, std::uint64_t original_deadline_ns) = 0;
    virtual void disable() = 0;
    virtual void close() = 0;
};

template <std::size_t G = 12>
class EncosGloveT final : public GloveDeviceT<G> {
public:
    using VecG = GloveVec<G>;
    using Frame = GloveFrameT<G>;
    using ImpedanceCmd = DriverImpedanceCommandT<G>;
    using ImpedanceSub = DriverImpedanceSubmissionT<G>;

    explicit EncosGloveT(EncosDriverT<G>& driver) : driver_(&driver) {}
    EncosGloveT(const std::string& mapping, const std::string& limits)
        : owned_(std::make_unique<EncosDriverT<G>>(mapping, limits)), driver_(owned_.get()) {}
    void connect() override { driver_->connect(); }
    Frame read() override { return driver_->read(); }
    std::array<EncosMotorCalibration, G> calibration() const override { return driver_->calibration(); }
    std::array<std::uint64_t, G> last_command_write_ns() const override { return driver_->last_command_write_ns(); }
    void home() override { driver_->home(); }
    void mark_homed() override { driver_->mark_homed(); }
    void enable_force() override { driver_->enable_force(); }
    VecG write_position(const VecG& v, std::uint64_t until) override { return driver_->write_position(v, until); }
    VecG write_torque(const VecG& v, std::uint64_t until) override { return driver_->write_torque(v, until); }
    ImpedanceSub write_impedance(const ImpedanceCmd& v, std::uint64_t until) override { return driver_->write_impedance(v, until); }
    void disable() override { driver_->disable(); }
    void close() override { driver_->close(); }
private:
    std::unique_ptr<EncosDriverT<G>> owned_;
    EncosDriverT<G>* driver_;
};

using GloveDevice = GloveDeviceT<12>;
using EncosGlove = EncosGloveT<12>;
using GloveDevice16 = GloveDeviceT<16>;
using EncosGlove16 = EncosGloveT<16>;
}
