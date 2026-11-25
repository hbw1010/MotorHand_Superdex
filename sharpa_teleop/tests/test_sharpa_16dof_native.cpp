#include "common/driver_impedance.hpp"
#include "common/session.hpp"
#include "hands/sharpa/entry_options.hpp"
#include "hands/sharpa/feedback.hpp"
#include "hands/sharpa/model.hpp"
#include "simulation/fake_glove.hpp"
#include "simulation/sharpa_device.hpp"
#include "ve33_model.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

using namespace motorhand::native;
using namespace motorhand::native::simulation;
namespace sh = motorhand::native::sharpa;

namespace {

void check(bool cond, const char* msg) {
    if (!cond) throw std::runtime_error(msg);
}

std::array<EncosMotorCalibration, 16> make_calibration_16(double torque_limit_nm) {
    std::array<EncosMotorCalibration, 16> cal{};
    for (std::size_t i = 0; i < 16; ++i) {
        cal[i].raw_motor_id = i + 1;
        cal[i].sign = 1.0;
        cal[i].scale = 1.0;
        cal[i].model_min_rad = (i < 12) ? -1.5 : -0.6;
        cal[i].model_max_rad = (i < 12) ? 1.5 : 0.6;
        cal[i].raw_torque_limit_nm = torque_limit_nm;
        cal[i].has_torque_limit = true;
    }
    return cal;
}

void test_entry_options_16dof() {
    auto sim16 = sh::parse_options({"sim", "--glove-model", "16dof", "--duration", "0.5"});
    check(sim16.glove_model == "16dof", "16dof CLI 参数解析应设置 glove_model=16dof");
    auto sim12 = sh::parse_options({"sim"});
    check(sim12.glove_model == "12dof", "默认 CLI 参数应保持 12dof");
    bool caught = false;
    try {
        (void)sh::parse_options({"sim", "--glove-model", "18dof"});
    } catch (const std::exception&) {
        caught = true;
    }
    check(caught, "非法 glove-model 应被拒绝");
}

void test_hand16_kinematics_and_retarget() {
    const std::filesystem::path assets = std::filesystem::current_path() / "assets";
    const std::filesystem::path cfg16 = std::filesystem::current_path() / "config/retargeting/sharpa/right_16dof.yaml";
    Hand16Model model(assets, false);
    const auto zero = model.zero();
    check(zero.max_gap_m < 1e-7, "16-DOF 零位闭链残差应 < 1e-7 m");

    // 混合屈伸 + 四指侧摆测试
    GloveVec<16> m_pos{};
    m_pos[4] = 0.25;
    m_pos[5] = 0.20;
    m_pos[12] = 0.25;
    m_pos[13] = 0.20;
    m_pos[14] = 0.20;
    m_pos[15] = 0.25;
    const auto p_pos = model.evaluate_motors(m_pos, &zero);
    check(p_pos.max_gap_m < 1e-7, "16-DOF 正侧摆闭链残差应 < 1e-7 m");

    // 验证四指侧摆在 63x16 点雅可比上有明确非零灵敏度
    for (std::size_t col = 12; col < 16; ++col) {
        double col_norm = 0.0;
        for (std::size_t row = 0; row < 63; ++row) {
            const double v = p_pos.point_jacobian[row * 16 + col];
            check(std::isfinite(v), "16-DOF 点雅可比应有限");
            col_norm += v * v;
        }
        check(std::sqrt(col_norm) > 1e-3, "16-DOF 侧摆电机 J13..J16 在点雅可比上应有非零灵敏度");
    }

    // evaluate_joints 往返一致
    const auto roundtrip = model.evaluate_joints(p_pos.joint_rad, &p_pos);
    check(roundtrip.max_gap_m < 1e-7, "16-DOF evaluate_joints 闭链残差应 < 1e-7 m");
    for (std::size_t i = 0; i < 16; ++i) {
        check(std::abs(roundtrip.motor_rad[i] - m_pos[i]) < 1e-4, "16-DOF evaluate_joints/evaluate_motors 往返误差应 < 1e-4 rad");
    }

    // Sharpa 22 轴全释放重定向：验证四指 MCP_AA 跟随手套 J13..J16 张合
    GloveVec<16> m_abd_pos{}, m_abd_neg{};
    m_abd_pos[12] = 0.25; m_abd_pos[13] = 0.20; m_abd_pos[14] = 0.20; m_abd_pos[15] = 0.25;
    m_abd_neg[12] = -0.25; m_abd_neg[13] = -0.20; m_abd_neg[14] = -0.20; m_abd_neg[15] = -0.25;
    const auto g_pos = model.evaluate_motors(m_abd_pos, &zero);
    const auto g_neg = model.evaluate_motors(m_abd_neg, &zero);

    const std::filesystem::path sharpa_assets = assets / "sharpa-hand";
    sh::Model sharpa_model(sharpa_assets, false, cfg16);
    const auto q_pos = sharpa_model.solve(g_pos.points_m);
    sharpa_model.reset();
    const auto q_neg = sharpa_model.solve(g_neg.points_m);
    constexpr std::array<std::size_t, 4> aa_idx = {6, 10, 14, 19};
    double mean_span_deg = 0.0;
    for (std::size_t idx : aa_idx) {
        const double span = std::abs(q_pos[idx] - q_neg[idx]) * 180.0 / 3.14159265358979323846;
        check(span > 12.0, "16-DOF 释放全部 Sharpa 自由度后四指 MCP_AA 张合应 > 12 deg");
        mean_span_deg += span;
    }
    mean_span_deg /= 4.0;
    std::cout << "  16-DOF Sharpa 四指 MCP_AA 平均张合跨度: " << mean_span_deg << " deg\n";
}

void test_control_session_16dof() {
    const std::filesystem::path root = std::filesystem::current_path();
    const std::filesystem::path assets = root / "assets";
    const std::filesystem::path cfg16 = root / "config/retargeting/sharpa/right_16dof.yaml";

    ControlOptionsT<16> opts;
    opts.left = false;
    opts.asset_root = assets / "hand_motor_16dof";
    opts.glove_mapping = root.parent_path() / "hardware/encos_g4_gateway_sdk/config/right_sim_mapping_16dof.json";
    opts.linear_feedback = root / "config/feedback/sharpa-right-16dof.json";
    opts.driver_impedance_config = root / "config/feedback/wuji-driver-impedance-right-16dof.json";
    opts.driver_impedance_feedback = true;
    opts.feedback_hardness = 0.3;
    opts.force_feedback = true;
    opts.glove_output = true;
    opts.home_hand_on_enable = true;

    const std::filesystem::path sharpa_assets = assets / "sharpa-hand";
    auto model = std::make_unique<sh::Model>(sharpa_assets, false, cfg16);
    HandDescriptor<22> desc;
    desc.identity = {"sharpa", "SIM-SHARPA-RIGHT", "mujoco-4.0.0", false};
    desc.joint_names = model->joint_names();
    desc.lower_rad = model->lower_limits();
    desc.upper_rad = model->upper_limits();
    desc.load_unit = HandLoadUnit::newton_metre;
    desc.load_source = sharpa_contact_load_source;
    desc.online_source = HandStatusSource::complete_state_packet;
    desc.enabled_source = HandStatusSource::device_report;

    auto cal = make_calibration_16(15.0);
    auto device = std::make_unique<SharpaDevice>(sharpa_assets, desc, true);
    auto feedback = std::make_unique<sh::FeedbackT<16>>(
        root / "config/feedback/sharpa-right-16dof.json", root / "config/feedback", false, cal, desc.load_source);

    auto fake = std::make_unique<FakeGloveT<16>>(cal);
    FakeGloveT<16>* fake_ptr = fake.get();

    ControlSession<22, 16> session(opts, std::move(device), std::move(model), std::move(feedback), std::move(fake));
    session.connect();
    session.enable();
    session.request_takeover(true);

    GloveVec<16> custom_axes{};
    custom_axes[12] = 0.25;
    custom_axes[13] = -0.20;
    custom_axes[14] = 0.20;
    custom_axes[15] = -0.25;
    fake_ptr->set_manual_target(custom_axes);

    ControlTick<22, 16> last{};
    for (int i = 0; i < 6; ++i) {
        last = session.step_takeover();
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
    check(last.takeover, "16-DOF ControlSession step_takeover 应处于接管态");
    check(last.glove_pose.max_gap_m < 1e-6, "16-DOF ControlSession 闭链求解应收敛");
    for (std::size_t i = 12; i < 16; ++i) {
        check(std::abs(last.glove.motor_rad[i] - custom_axes[i]) < 1e-4,
              "16-DOF FakeGlove 交互式全轴覆盖应精确反映在遥操作帧中");
    }
    session.stop();
}

}  // namespace

int main() {
    try {
        test_entry_options_16dof();
        test_hand16_kinematics_and_retarget();
        test_control_session_16dof();
        std::cout << "Sharpa 16-DOF 并列原生回归测试全部通过\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Sharpa 16-DOF 测试失败: " << e.what() << '\n';
        return 1;
    }
}
