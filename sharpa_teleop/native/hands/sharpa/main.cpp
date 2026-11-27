#include "entry_options.hpp"
#include "motion.hpp"
#include "model.hpp"
#include "device.hpp"
#include "feedback.hpp"
#include "common/session.hpp"
#include "encos_driver.hpp"
#include "playback_control.hpp"
#include "simulation/fake_glove.hpp"
#include "simulation/sharpa_device.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <thread>
using namespace motorhand::native;
namespace sh = motorhand::native::sharpa;
namespace sim = motorhand::native::simulation;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
namespace {
std::atomic<bool> stopped{false};
static_assert(std::atomic<bool>::is_always_lock_free);
void signal_stop(int) { stopped.store(true); }
std::uint64_t now_ns() { return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count(); }
std::string failure_text(std::exception_ptr error) { try { std::rethrow_exception(error); } catch (const std::exception& e) { return e.what(); } catch (...) { return "未知错误"; } }
class Publisher {
    int fd_ = -1; sockaddr_in address_{};
public:
    explicit Publisher(int port) {
        if (!port) return;
        fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd_ < 0) throw std::runtime_error("无法创建只读遥测端口");
        address_.sin_family = AF_INET; address_.sin_port = htons(port); inet_pton(AF_INET, "127.0.0.1", &address_.sin_addr);
    }
    ~Publisher() { if (fd_ >= 0) ::close(fd_); }
    void send(const Json& j) {
        if (fd_ < 0) return; auto s = j.dump(); if (s.size() > 60000) return;
        sendto(fd_, s.data(), s.size(), MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&address_), sizeof(address_));
    }
};
class ControlListener {
    int fd_ = -1;
public:
    explicit ControlListener(int port) {
        if (!port) return;
        fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd_ < 0) throw std::runtime_error("无法创建控制命令端口");
        int reuse = 1; setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_port = htons(port); inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
        if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) { ::close(fd_); fd_ = -1; throw std::runtime_error("无法绑定Sharpa控制端口"); }
    }
    ~ControlListener() { if (fd_ >= 0) ::close(fd_); }
    std::string poll() {
        if (fd_ < 0) return {};
        char buf[512]{};
        const auto n = recv(fd_, buf, sizeof(buf) - 1, MSG_DONTWAIT);
        if (n <= 0) return {};
        std::string cmd(buf, static_cast<std::size_t>(n));
        while (!cmd.empty() && (cmd.back() == '\n' || cmd.back() == '\r' || cmd.back() == ' ')) cmd.pop_back();
        return cmd;
    }
};

template <std::size_t G>
struct FeedbackWindowT {
    std::size_t ticks{}, geometry_ticks{};
    std::array<std::size_t, G> impedance_ticks{};
    GloveVec<G> peak_nm{};
    std::array<std::optional<double>, G> peak_current_a{};

    void observe(const ControlTick<22, G>& tick, const std::array<EncosMotorCalibration, G>& calibration) {
        if (!tick.takeover) return;
        ++ticks;
        geometry_ticks += tick.force_geometry_valid;
        const auto now = tick.cycle_completed_ns;
        for (std::size_t i = 0; i < calibration.size(); ++i) {
            const auto& axis = calibration[i];
            const auto raw = axis.raw_motor_id - 1;
            if (tick.force_commanded) {
                peak_nm[raw] = std::max(peak_nm[raw], std::abs(tick.glove_target[i] * axis.sign * axis.scale));
                if (tick.glove_driver_kp[i] > 0) ++impedance_ticks[raw];
            }
            const auto received = tick.glove.current_received_ns[i];
            if (received && received <= now && now - received < hand_timeout_ns) {
                const auto current = std::abs(tick.glove.measured_current_a[i]);
                peak_current_a[raw] = std::max(peak_current_a[raw].value_or(0), current);
            }
        }
    }

    int peak_axis(int first, int end) const {
        return static_cast<int>(std::distance(peak_nm.begin(), std::max_element(peak_nm.begin() + first, peak_nm.begin() + end)));
    }

    void print_axis(std::ostream& out, const char* finger, int first, int end) const {
        const int raw = peak_axis(first, end);
        out << finger << " J" << (raw + 1) << " " << std::setprecision(2) << peak_nm[raw]
            << " Nm · " << impedance_ticks[raw] << " 帧 · ";
        if (peak_current_a[raw]) out << *peak_current_a[raw] << " A";
        else out << "无电流读数";
    }

    Json summary() const {
        Json axes = Json::object();
        for (std::size_t raw = 0; raw < G; ++raw) {
            axes["J" + std::to_string(raw + 1)] = {
                {"名义峰值Nm", peak_nm[raw]},
                {"阻抗帧", impedance_ticks[raw]},
                {"实测电流峰值A", peak_current_a[raw] ? Json(*peak_current_a[raw]) : Json(nullptr)}};
        }
        return {{"接管帧", ticks}, {"几何有效帧", geometry_ticks},
                {"全手峰值轴", peak_axis(0, static_cast<int>(G)) + 1}, {"电机", axes}};
    }
};

void start_sdk_log(const std::filesystem::path& project_root) {
    try {
        const auto path = project_root / ".cache" / "sharpa-logs" / ("sdk-" + std::to_string(getpid()) + ".log");
        std::filesystem::create_directories(path.parent_path());
        sh::configure_sdk_file_logging(path.string());
        std::cout << "Sharpa SDK 详情 " << path.string() << std::endl;
    } catch (const std::exception& error) {
        std::cerr << "Sharpa SDK 日志文件不可用，保留终端日志。" << error.what() << '\n';
    }
}
HandDescriptor<22> describe(sh::Model& model, const sh::EntryOptions& a) {
    HandDescriptor<22> d; d.identity = {"sharpa", a.serial, a.firmware, a.left};
    d.joint_names = model.joint_names(); d.lower_rad = model.lower_limits(); d.upper_rad = model.upper_limits();
    d.load_unit = HandLoadUnit::newton_metre; d.load_source = a.simulation ? sim::sharpa_contact_load_source : sh::torque_source;
    d.online_source = HandStatusSource::complete_state_packet; d.enabled_source = HandStatusSource::device_report;
    if (a.simulation) { d.identity.serial = a.left ? "SIM-SHARPA-LEFT" : "SIM-SHARPA-RIGHT"; d.identity.firmware = "mujoco-4.0.0"; }
    return d;
}
std::unique_ptr<HandDevice<22>> create_device(const sh::EntryOptions& a, const HandDescriptor<22>& d) {
    if (a.simulation) return std::make_unique<sim::SharpaDevice>(a.project_root / "assets/sharpa-hand", d, a.object);
    if (a.serial.empty()) throw std::invalid_argument("真机必须显式指定--serial；先运行sharpa-discover");
    sh::DeviceOptions o; o.serial = a.serial; o.left = a.left; o.expected_pn = a.pn; o.approved_firmware = a.firmware;
    return std::make_unique<sh::Device>(o, d);
}
Json state_json(const HandState<22>& s) {
    return Json{{"robot", "sharpa"}, {"hand", s.identity.left ? "left" : "right"}, {"serial", s.identity.serial},
        {"firmware", s.identity.firmware}, {"position_rad", s.position_rad}, {"velocity_rad_s", s.has_velocity ? Json(s.velocity_rad_s) : Json(nullptr)},
        {"load_nm", s.has_load ? Json(s.load) : Json(nullptr)}, {"load_source", s.load_source},
        {"received_ns", s.received_ns}, {"sequence", s.sequence}, {"fault", s.fault},
        {"enabled", std::all_of(s.enabled.begin(), s.enabled.end(), [](bool x) { return x; })}};
}
int inspect_or_record(const sh::EntryOptions& a) {
    sh::Model model(a.project_root / "assets/sharpa-hand", a.left, a.retarget_config);
    auto d = describe(model, a); auto device = create_device(a, d);
    try {
        device->connect(); d = device->descriptor();
        if (a.command == "inspect") { std::cout << state_json(device->read()).dump() << '\n'; device->close(); return 0; }
        auto output = a.output.empty() ? a.trajectory : a.output;
        if (output.empty()) throw std::invalid_argument("record需要--output路径");
        sh::MotionRecorder recorder(output, d); const auto start = Clock::now();
        while (!stopped.load()) {
            auto begin = Clock::now(); auto s = device->read(); validate_hand_state(s, d, now_ns(), false); recorder.append(s);
            if (a.duration_s > 0 && std::chrono::duration<double>(Clock::now() - start).count() >= a.duration_s) break;
            std::this_thread::sleep_until(begin + std::chrono::nanoseconds(1000000000 / a.rate));
        }
        device->close(); recorder.finish(); std::cout << "已保存Sharpa实测轨迹：" << output << '\n'; return 0;
    } catch (...) {
        auto original = std::current_exception();
        try { device->close(); } catch (...) { throw std::runtime_error(failure_text(original) + "；连接关闭未确认：" + failure_text(std::current_exception())); }
        std::rethrow_exception(original);
    }
}

template <std::size_t G>
int run_for_glove(const sh::EntryOptions& a) {
    constexpr bool is_16dof = (G == 16);
    const bool replay = a.command == "replay" || a.command == "takeover";
    const bool output_hardware = !a.simulation || (!a.fake_glove && (a.feedback || replay));
    if (output_hardware && !a.execute) throw std::invalid_argument("此入口会控制真实设备；需显式添加--execute");
    if (!a.simulation) start_sdk_log(a.project_root);

    std::filesystem::path retarget_cfg = a.retarget_config;
    if (retarget_cfg.empty() && is_16dof) {
        retarget_cfg = a.project_root / "config/retargeting/sharpa" / (a.left ? "left_16dof.yaml" : "right_16dof.yaml");
    }
    auto model = std::make_unique<sh::Model>(a.project_root / "assets/sharpa-hand", a.left, retarget_cfg);
    auto descriptor = describe(*model, a); auto device = create_device(a, descriptor);
    auto* simulation = dynamic_cast<sim::SharpaDevice*>(device.get());
    std::optional<sh::Motion> motion;
    if (replay) motion = sh::Motion::load(a.trajectory, descriptor);

    const std::filesystem::path mapping = !a.mapping.empty()
        ? a.mapping
        : (a.project_root.parent_path() / "hardware" /
           (is_16dof ? "encos_g4_gateway_16dof/config/right_sim_mapping_16dof.json"
                     : "encos_g4_gateway_12dof/config/right_sim_mapping_12dof.json"));
    const std::filesystem::path feedback_path = a.project_root / "config/feedback" /
        (is_16dof ? (a.left ? "sharpa-left-16dof.json" : "sharpa-right-16dof.json")
                  : (a.left ? "sharpa-left.json" : "sharpa-right.json"));
    const auto wuji_feedback = a.project_root / "config/feedback";
    const bool linear = a.command == "force-linear";
    EncosDriverT<G> calibration_only(mapping.string(), feedback_path.string());
    const auto calibration = calibration_only.calibration();
    std::unique_ptr<GloveDeviceT<G>> fake;
    sim::FakeGloveT<G>* fake_ptr = nullptr;
    if (a.fake_glove) {
        auto p = std::make_unique<sim::FakeGloveT<G>>(calibration);
        fake_ptr = p.get();
        fake = std::move(p);
    }
    ControlOptionsT<G> o;
    if constexpr (is_16dof) {
        o.asset_root = a.project_root / "assets/hand_motor_16dof";
    } else {
        const auto ve40_root = a.project_root / "assets/hand_motor_ve40";
        const auto ve40_side = ve40_root / (a.left ? "hand_motor_ve40_L" : "hand_motor_ve40_R");
        o.asset_root = std::filesystem::exists(ve40_side / "hand_tree_only.urdf")
                           ? ve40_root
                           : (a.project_root / "assets/hand_motor_ve33");
    }
    o.glove_mapping = mapping; o.left = a.left; o.linear_feedback = feedback_path; o.force_feedback = a.feedback;
    o.glove_output = a.feedback || replay; o.linear_force_feedback = linear;
    o.driver_impedance_feedback = a.feedback && !o.linear_force_feedback && a.feedback_mode == "driver-impedance";
    if (o.driver_impedance_feedback) {
        const auto imp_file = is_16dof ? "wuji-driver-impedance-right-16dof.json" : "wuji-driver-impedance-right.json";
        auto impedance = load_driver_impedance_config_for<G>((wuji_feedback / imp_file).string());
        impedance.torque_margin = .95;
        impedance.joint_gain.fill(2.7);
        impedance.surface_rate = .25;
        impedance.surface_acceleration = 5.;
        o.driver_impedance_override = impedance;
    }
    o.feedback_hardness = a.hardness; o.zero_aligned_start = false;
    if (replay) {
        o.takeover_motor_deadband_rad = 0.002;
    } else o.takeover_offset_motion_rad = 0;
    o.home_hand_on_enable = true; o.skip_glove_home = a.skip_glove_home; o.stop_requested = &stopped;
    std::unique_ptr<HandFeedback<22, G>> feedback;
    if (a.feedback) feedback = std::make_unique<sh::FeedbackT<G>>(
        feedback_path, wuji_feedback, linear, calibration, descriptor.load_source, replay ? 0 : o.transition_ns);
    ControlSession<22, G> session(o, std::move(device), std::move(model), std::move(feedback), std::move(fake));
    Publisher publisher(a.port); ControlListener control_rx(a.control_port); TerminalKeys keys;
    bool manual_demo_axes_active = false;
    GloveVec<G> manual_demo_axes{};
    try {
        const auto actual = session.connect();
        std::optional<sh::MotionRecorder> recorder;
        if (!a.output.empty()) recorder.emplace(a.output, actual, sh::MotionTargetSource::submitted);
        std::filesystem::path scene_path = a.project_root / "assets/sharpa-hand" / (a.left ? "left.xml" : "right.xml");
        if (simulation) {
            auto snapshot = simulation->snapshot();
            scene_path = a.project_root / ".cache" / ("sharpa-scene-" + std::to_string(getpid()) + ".xml");
            std::filesystem::create_directories(scene_path.parent_path());
            std::ofstream scene(scene_path); scene << snapshot.scene_xml;
            if (!scene) throw std::runtime_error("无法保存只读仿真场景");
        }
        std::cout << "启动回零：" << (a.simulation ? "虚拟Sharpa" : "真实Sharpa")
                  << " (" << (is_16dof ? "16-DOF" : "12-DOF") << ")"
                  << (o.glove_output ? (a.fake_glove ? "与假手套" : "与真实Motorhand") : "；Motorhand仅输入") << std::endl;
        session.enable(); bool takeover = !replay; session.request_takeover(takeover);
        const auto start = Clock::now(); auto last_print = start; bool inverse_ready = false, automatic_takeover = false;
        FeedbackWindowT<G> feedback_window;
        PlaybackClock playback(motion ? motion->duration_ns() : 1, now_ns());
        if (takeover) playback.pause(now_ns());
        std::cout << (a.simulation ? "Sharpa仿真" : "Sharpa真机") << "已开始；手侧=" << (a.left ? "left" : "right")
                  << "，手套=" << (is_16dof ? "16dof" : "12dof")
                  << "，反馈=" << (a.feedback ? a.feedback_mode : "关闭") << "。"
                  << (replay ? "空格切换接管/回放，" : "") << "q或Ctrl+C退出。\n";
        while (!stopped.load()) {
            const auto begin = Clock::now(); const auto ns = now_ns();
            const double elapsed = std::chrono::duration<double>(begin - start).count();
            bool toggle = false;
            for (auto key = keys.read(); key != KeyEvent::none; key = keys.read()) {
                if (key == KeyEvent::quit) stopped.store(true);
                if (key == KeyEvent::toggle) toggle = true;
            }
            for (auto cmd = control_rx.poll(); !cmd.empty(); cmd = control_rx.poll()) {
                if (cmd == "stop" || cmd == "quit") stopped.store(true);
                else if (cmd == "toggle") toggle = true;
                else if (cmd == "takeover" && !takeover) toggle = true;
                else if (cmd == "replay" && takeover) toggle = true;
                else if (cmd == "axes auto") manual_demo_axes_active = false;
                else if (cmd.rfind("axes ", 0) == 0) {
                    std::istringstream iss(cmd.substr(5));
                    GloveVec<G> parsed{};
                    std::size_t count = 0;
                    double val = 0;
                    while (count < G && (iss >> val)) {
                        if (!std::isfinite(val)) break;
                        parsed[count++] = val;
                    }
                    if (count == G) {
                        manual_demo_axes = parsed;
                        manual_demo_axes_active = true;
                    }
                }
            }
            if (stopped.load()) break;
            if (replay && a.takeover_at_s >= 0 && !automatic_takeover && elapsed >= a.takeover_at_s) {
                toggle = true; automatic_takeover = true;
            }
            if (replay && toggle) {
                takeover = !takeover;
                if (takeover) playback.pause(ns, inverse_ready);
                else { playback.resume(ns); inverse_ready = false; }
                session.request_takeover(takeover);
            }
            if (fake_ptr && takeover) {
                GloveVec<G> target{};
                if (manual_demo_axes_active) {
                    for (std::size_t i = 0; i < G; ++i) {
                        const int raw = calibration[i].raw_motor_id - 1;
                        target[i] = std::clamp(manual_demo_axes[raw], calibration[i].model_min_rad, calibration[i].model_max_rad);
                    }
                } else {
                    const double phase = elapsed * 2 * 3.141592653589793 / 6.0;
                    const double fraction = .5 - .5 * std::cos(phase);
                    const double spread = .5 + .5 * std::cos(phase);
                    GloveVec<G> raw_demo{};
                    if constexpr (is_16dof) {
                        raw_demo = {.20, .15, .5, .5, .8, 1.2, .8, 1.2, .8, 1.2, .8, 1.2, .26, .10, -.10, -.26};
                    } else {
                        raw_demo = {.20, .15, .5, .5, .8, 1.2, .8, 1.2, .8, 1.2, .8, 1.2};
                    }
                    for (std::size_t i = 0; i < G; ++i) {
                        const int raw = calibration[i].raw_motor_id - 1;
                        const double w = (raw < 12) ? fraction : spread;
                        target[i] = std::clamp(raw_demo[raw] * w, calibration[i].model_min_rad, calibration[i].model_max_rad);
                    }
                }
                fake_ptr->set_manual_target(target);
            }
            const auto position = playback.position(ns, inverse_ready);
            auto tick = takeover ? session.step_takeover() : session.step_inference(motion->at(position), ns);
            inverse_ready = tick.inverse_fresh;
            if (recorder) recorder->append(tick.hand, tick.hand_target);
            if (a.feedback) feedback_window.observe(tick, calibration);
            auto j = state_json(tick.hand);
            j["schema_version"] = 2;
            j["glove_model"] = is_16dof ? "16dof" : "12dof";
            j["glove_dof"] = G;
            j["joint_names"] = actual.joint_names;
            j["hand_target_rad"] = tick.hand_target;
            j["glove_motor_rad"] = tick.glove.motor_rad;
            j["takeover"] = tick.takeover;
            j["glove_tree_order"] = is_16dof ? "hand16_closure_independent_dependent" : "ve33_closure_independent_dependent";
            j["glove_tree_rad"] = tick.glove_pose.tree_rad;
            j["glove_received_ns"] = tick.glove.oldest_ns;
            j["glove_nominal_nm"] = tick.takeover && tick.force_commanded ? Json(tick.glove_target) : Json(nullptr);
            j["glove_driver_target_rad"] = tick.takeover && tick.force_commanded && o.driver_impedance_feedback ? Json(tick.glove_driver_target_rad) : Json(nullptr);
            j["glove_driver_kp"] = tick.takeover && tick.force_commanded && o.driver_impedance_feedback ? Json(tick.glove_driver_kp) : Json(nullptr);
            j["glove_driver_kd"] = tick.takeover && tick.force_commanded && o.driver_impedance_feedback ? Json(tick.glove_driver_kd) : Json(nullptr);
            j["glove_driver_pd_nm"] = tick.takeover && tick.force_commanded && o.driver_impedance_feedback ? Json(tick.glove_driver_pd_nm) : Json(nullptr);
            j["feedback_mode"] = a.feedback ? (o.driver_impedance_feedback ? "driver-impedance" : "torque") : "off";
            j["glove_motor_target_rad"] = !tick.takeover && tick.glove_submit_ns ? Json(tick.glove_target) : Json(nullptr);
            j["inverse_fresh"] = !tick.takeover && tick.inverse_fresh;
            j["hand_submit_ns"] = tick.hand_submit_ns; j["glove_submit_ns"] = tick.glove_submit_ns;
            j["jacobian_source_ns"] = tick.jacobian_source_ns;
            j["force_commanded"] = tick.force_commanded; j["force_geometry_valid"] = tick.force_geometry_valid;
            j["contact"] = tick.contact; j["scene_path"] = scene_path.string(); j["simulated"] = a.simulation;
            j["mode"] = replay ? (takeover ? "takeover" : "replay") : a.command; j["can_takeover"] = replay;
            j["demo_axes_active"] = manual_demo_axes_active;
            if (simulation) {
                auto s = simulation->snapshot();
                j["qpos"] = s.qpos; j["simulation_time"] = s.simulation_time; j["contact_count"] = s.contact_count;
            }
            if (std::chrono::duration<double>(begin - last_print).count() >= 1) {
                std::ostringstream status;
                status << std::fixed << std::setprecision(1)
                       << "t=" << elapsed << "s mode=" << (replay ? (takeover ? "接管" : "回放") : "遥操");
                if (feedback_window.ticks) {
                    const int peak = feedback_window.peak_axis(0, static_cast<int>(G));
                    j["feedback_window"] = feedback_window.summary();
                    status << " | 几何 " << feedback_window.geometry_ticks << '/' << feedback_window.ticks
                           << " 帧 | 名义峰值 J" << peak + 1 << " "
                           << std::setprecision(2) << feedback_window.peak_nm[peak] << " Nm\n  ";
                    feedback_window.print_axis(status, "拇指", 0, 4);
                    status << " | ";
                    feedback_window.print_axis(status, "食指", 4, 6);
                } else if (!takeover) {
                    status << " | 逆解" << (tick.inverse_fresh ? "有效" : "等待");
                }
                feedback_window = {};
                std::cout << status.str() << std::endl;
                last_print = begin;
            }
            publisher.send(j);
            if (a.duration_s > 0 && elapsed >= a.duration_s) break;
            if (replay && !takeover && inverse_ready && position >= motion->duration_ns()) break;
            std::this_thread::sleep_until(begin + std::chrono::nanoseconds(1000000000 / a.rate));
        }
        session.stop(); if (recorder) recorder->finish(); return 0;
    } catch (...) {
        auto original = std::current_exception();
        try { session.stop(); } catch (...) { throw std::runtime_error(failure_text(original) + "；停止未确认：" + failure_text(std::current_exception())); }
        std::rethrow_exception(original);
    }
}

int run(const sh::EntryOptions& a) {
    if (a.glove_model == "16dof") return run_for_glove<16>(a);
    return run_for_glove<12>(a);
}
}
int main(int argc, char** argv) {
    std::signal(SIGINT, signal_stop); std::signal(SIGTERM, signal_stop);
    try {
        auto a = sh::parse_options(std::vector<std::string>(argv + 1, argv + argc));
        if (a.help) {
            std::cout << "Sharpa: discover | inspect | teleop | force-contact | force-linear | record | replay | takeover | sim | sim-teleop\n"
                      << "--glove-model 12dof|16dof --serial SN --hand right|left --approved-firmware VERSION --execute --feedback --duration 秒 --trajectory 文件 --output 文件 --curves-port 8766\n"
                      << "仿真默认无物体；--with-object可恢复抓握球。sharpa-viewer并排显示Motorhand与Sharpa。\n"
                      << "真机输出需--execute；sim不访问硬件；sim-teleop默认仅读取真实手套。\n";
            return 0;
        }
        if (a.project_root.empty()) a.project_root = MOTORHAND_PROJECT_ROOT;
        a.project_root = std::filesystem::absolute(a.project_root);
        if (a.command == "discover") {
            auto transport = sh::make_sdk_transport(); Json devices = Json::array(); const auto until = Clock::now() + std::chrono::seconds(3);
            do {
                devices = Json::array();
                for (const auto& d : transport->discover())
                    if (d.is_hand) devices.push_back({{"serial", d.serial}, {"pn", d.pn}, {"firmware", d.firmware}, {"hand", d.left ? "left" : "right"}});
                if (!devices.empty() || stopped.load()) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            } while (Clock::now() < until);
            std::cout << devices.dump() << '\n'; return 0;
        }
        if (a.command == "clear-link-latch") {
            if (a.serial.empty()) throw std::invalid_argument("clear-link-latch 必须指定 --serial");
            auto transport = sh::make_sdk_transport();
            const auto until = Clock::now() + std::chrono::seconds(3);
            bool found = false;
            do {
                for (const auto& d : transport->discover()) if (d.serial == a.serial) { found = true; break; }
                if (found || stopped.load()) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            } while (Clock::now() < until);
            if (!found) throw std::runtime_error("Sharpa未发现指定SN: " + a.serial);
            transport->connect(a.serial, true, true);
            try {
                transport->start();
                transport->clear_network_link_down_latch();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                const bool fault = transport->has_fault();
                const auto details = transport->fault_details();
                transport->release(a.serial);
                std::cout << Json{{"serial", a.serial}, {"cleared", true}, {"has_fault", fault}, {"fault_details", details}}.dump() << '\n';
                return 0;
            } catch (...) { try { transport->release(a.serial); } catch (...) {} throw; }
        }
        if (a.command == "inspect" || a.command == "record") return inspect_or_record(a);
        return run(a);
    } catch (const std::exception& e) { std::cerr << "Sharpa失败：" << e.what() << '\n'; return 1; }
}
