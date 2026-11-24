// 单一 G4 所有者。周期线程不调用 Python；观测者只复制已完成的整周期快照。
#include "gateway_transport.hpp"
#include "feedback_control.hpp"
#include "kinematics.hpp"
#include "shared_link.hpp"
#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <chrono>
#include <mutex>
#include <optional>
#include <thread>

namespace nb = nanobind;
namespace ve33 {
using Clock = std::chrono::steady_clock;
using Ns = std::chrono::nanoseconds;
static int64_t ns_now() { return std::chrono::duration_cast<Ns>(Clock::now().time_since_epoch()).count(); }
constexpr int64_t history_size = 512;
struct Sample {
    int64_t seq{}, now{}, stamp{}, scene{}, skipped{};
    double cycle_ms{};
    bool requested{}, calibrated{}, transport_online{}, requires_new_zero{};
    unsigned reconnect_count{};
    std::array<G4Joint,12> joints{};
    Axes raw{}, current{}, q{}, encoder{};
    Stamps stamps{};
    std::array<int,12> status{};
    ForceFrame force{};
    ControlState control;
    std::string pose_error;
};

class NativeLoop {
    Kinematics& kin_;
    FeedbackControl control_;
    SharedLink& link_;
    const Axes ratio_,lo_,hi_,amplitude_;
    const int64_t period_;
    const uint32_t disabled_mask_;
    std::optional<GatewayTransport> transport_;
    bool demo_;
    Axes zero_{},last_q_{};
    bool calibrated_ = false, have_pose_ = false, requires_new_zero_ = false;
    int64_t zero_capture_ns_ = 0;
    std::vector<double> target_;
    ForceFrame last_force_{};
    Intent intent_{};
    int64_t seq_=0,skipped_=0,start_time_=0;
    std::mutex history_lock_, lifecycle_lock_;
    std::array<Sample,history_size> history_;
    std::string failure_;
    std::atomic<bool> running_{false};
    std::thread worker_;

    void release() {
        if (auto device=transport_->handle())
            for (int i=0;i<12;++i) g4_release(device,0,i+1);
        control_.released();
    }
    void disarm_feedback(const char* reason, int64_t now) {
        const bool held = control_.state().active || control_.state().submitted_until > now;
        control_.stop(reason);
        if (held) release(); else control_.released();
    }
    void clear_target() { target_[0]=0.; link_.publish_target(target_); have_pose_=false; }
    GatewayTransport::Event poll_transport(int64_t now) {
        const auto event = transport_->poll(now);
        if (event != GatewayTransport::Event::none) {
            // 串口重新枚举后，即使路径相同也可能换了手套。旧零位与旧使能不得沿用。
            requires_new_zero_=true;
            calibrated_=false;
            zero_.fill(0.);
            zero_capture_ns_=0;
            last_force_={};
            link_.disarm();
            control_.stop("串口连接变化 · 等待重新标零");
            control_.released();  // 旧句柄已由 transport 销毁，新句柄只读位置。
            clear_target();
            if (auto device=transport_->handle()) g4_set_disabled_mask(device, disabled_mask_);
        }
        return event;
    }
    struct Raw { std::array<G4Joint,16> joints{}; std::array<int64_t,16> currents{}; int size{}; };
    Raw read_raw() {
        Raw data;
        if (auto device=transport_->handle()) {
            data.size = g4_snapshots_with_current_time(device,data.joints.data(),data.currents.data(),
                                                       static_cast<int>(data.joints.size()));
        } else if (demo_) {
            const int64_t now=ns_now();
            const double grip=intent_.grip>=0. ? intent_.grip :
                        .5-.5*std::cos((now-start_time_)*1e-9*2.*std::numbers::pi/7.);
            data.size=12;
            for (unsigned i=0;i<12;++i) {
                auto& j=data.joints[i]; j.id=i+1; j.bus=3-i/4; j.valid=1; j.received_ns=now;
                j.angle_deg=static_cast<float>(grip*amplitude_[i]*180./(std::numbers::pi*ratio_[i]));
            }
        }
        return data;
    }
    void tick() {
        const int64_t began=ns_now();
        poll_transport(began);
        Intent next=intent_;
        if (link_.read_intent(next)) {
            // 即使停止和重新启用都发生在一次求解中，也必须重新建力。
            const bool can_reset=(!control_.state().output_error && !control_.state().rearm_required) ||
                                 next.disabled_generation!=intent_.disabled_generation;
            if (next.generation!=intent_.generation && can_reset)
                disarm_feedback("控制请求更新", began);
            intent_=next;
        }
        Sample s;
        s.requested=!demo_ && !requires_new_zero_ && intent_.enabled;
        s.scene=intent_.revision;
        s.calibrated=calibrated_;
        s.transport_online=demo_ || transport_->handle();
        s.requires_new_zero=requires_new_zero_;
        s.reconnect_count=transport_->reconnects();
        s.raw.fill(NAN); s.current.fill(NAN); s.encoder.fill(NAN); s.q.fill(NAN);
        const auto data=read_raw();
        const int64_t now=ns_now();
        for (int k=0;k<data.size;++k) {
            const auto& j=data.joints[k];
            if (j.id<1 || j.id>12) continue;
            const size_t i=j.id-1;
            // 错误路由/同 ID 双应答不能被后一个正确应答覆盖。
            if (s.joints[i].id) { s.status[i]=-2; continue; }
            s.joints[i]=j; s.stamps[i]=j.received_ns;
            if ((j.valid&2) && j.bus>=1 && j.bus<=3 && data.currents[k]>0 && now>=data.currents[k] &&
                now-data.currents[k]<=100'000'000 && std::isfinite(j.current_a)) s.current[i]=j.current_a;
            if (!(j.valid&1) || !std::isfinite(j.angle_deg)) continue;
            s.raw[i]=j.angle_deg;
            if (j.received_ns<=0 || j.received_ns>now || now-j.received_ns>=100'000'000) continue;
            if (j.error) { s.status[i]=-1; continue; }
            if (j.bus<1 || j.bus>3) { s.status[i]=-2; continue; }
            s.status[i]=1;
            if (calibrated_) {
                s.encoder[i]=(j.angle_deg-zero_[i])*ratio_[i]*(std::numbers::pi/180.);
                s.q[i]=std::clamp(s.encoder[i],lo_[i],hi_[i]);
            }
        }
        if (calibrated_ && std::all_of(s.status.begin(),s.status.end(),[](int v){return v==1;})) {
            try {
                if (!have_pose_ || last_q_!=s.q) {
                    auto tree=kin_.solve(s.q);
                    if (tree.size()+13!=target_.size()) throw std::runtime_error("闭链接口维度不匹配");
                    std::copy(tree.begin(),tree.end(),target_.begin()+1);
                    have_pose_=true; last_q_=s.q;
                }
                s.stamp=*std::min_element(s.stamps.begin(),s.stamps.end());
                target_[0]=static_cast<double>(s.stamp);
                std::copy(s.encoder.begin(),s.encoder.end(),target_.end()-12);
                link_.publish_target(target_);
            } catch(const std::exception& e) { s.pose_error=e.what(); clear_target(); }
        } else clear_target();

        ForcePacket frame{};
        if (link_.read_force(frame)) {
            if (frame[0] < static_cast<double>(start_time_)) frame[2]=0.;
            last_force_=frame;
        }
        s.force=last_force_;
        if (s.stamp) {
            control_.step(ns_now(),s.q,s.encoder,s.stamps,s.force,s.requested,s.scene);
        } else {
            std::string reason = calibrated_ ? "编码器采集或闭链姿态不可用" : "等待标零";
            if (calibrated_) {
                if (!s.pose_error.empty()) {
                    reason = "闭链姿态求解失败: " + s.pose_error;
                } else {
                    std::string bad;
                    for (int i = 0; i < 12; ++i) {
                        if (s.status[i] == 1) continue;
                        if (!bad.empty()) bad += ",";
                        bad += "J" + std::to_string(i + 1) + "(";
                        if (s.status[i] == -1) bad += "err=" + std::to_string(s.joints[i].error);
                        else if (s.status[i] == -2) bad += "bus=" + std::to_string(s.joints[i].bus);
                        else if (s.stamps[i] > 0) bad += "age=" + std::to_string((now - s.stamps[i]) / 1'000'000) + "ms";
                        else bad += "none";
                        bad += ")";
                    }
                    if (!bad.empty()) reason = "编码器异常: " + bad;
                }
            }
            control_.missing(ns_now(),s.requested,reason);
        }
        if (control_.state().need_release) release();
        if (control_.state().need_send) {
            std::array<float,60> batch{};
            for (int i=0;i<12;++i) {
                if (disabled_mask_ & (1u << i)) continue;
                batch[i*5+2]=static_cast<float>(control_.state().motor_kd[i]);
                batch[i*5+4]=static_cast<float>(control_.state().motor[i]);
            }
            const auto allowed=link_.if_permitted(intent_,[&]{
                const auto& state=control_.state();
                auto accepted=g4_renew_mit_batch(transport_->handle(),batch.data(),12,state.deadline,
                                                 state.valid_sent ? state.submitted_until : 0);
                if (accepted==1) control_.accepted(); else control_.failed(accepted==0);
            });
            if (allowed==CommitPermission::changed) disarm_feedback("控制请求更新", ns_now());
            if (control_.state().need_release) release();
            // 命令锁暂时忙时跳过该次提交；旧批次仍仅活到原期限。
        }
        s.control=control_.state();
        s.now=ns_now(); s.cycle_ms=(s.now-began)*1e-6; s.skipped=skipped_;
        DisplayPacket display{};
        display[0]=static_cast<double>(s.now); display[1]=s.control.active;
        for (int i=0;i<12;++i) {
            display[2+i]=s.control.valid_nominal ? s.control.nominal[i] : NAN;
            display[14+i]=s.current[i];
        }
        display[26]=static_cast<double>(s.scene);
        if(s.control.valid_nominal)
            display[27]=std::min({static_cast<double>(s.stamp),s.force[0],s.force[1]})+control_.ttl_ns();
        display[28]=s.force[1];
        display[29]=static_cast<double>(intent_.generation);
        link_.publish_display(display);
        std::lock_guard lock(history_lock_);
        s.seq=++seq_; history_[s.seq%history_size]=std::move(s);
    }
    void service_expiring_batch(Clock::time_point next) {
        const auto& state=control_.state();
        const auto next_ns=std::chrono::duration_cast<Ns>(next.time_since_epoch()).count();
        constexpr int64_t wake_margin_ns = 3'000'000;
        if (!state.active || !state.valid_sent || state.submitted_until > next_ns + wake_margin_ns) return;
        const auto expiry=state.submitted_until;
        const auto wait_until=std::min(next_ns, expiry);
        // 正常节拍来不及续接或临近到期时等待新物理帧，减少调度抖动导致的看门狗超时。
        while (running_.load()) {
            const auto observed=link_.force_notice();
            ForcePacket incoming{};
            const auto now=ns_now();
            if (now >= next_ns) return;
            if (now >= expiry) {
                tick();
                return;
            }
            if (link_.read_force(incoming) && incoming[1] != last_force_[1]) {
                tick();
                if (!state.active || !state.valid_sent || state.submitted_until != expiry) return;
                // 该物理帧可能仍使用相同的编码器输入；再有新帧时仍有机会在截止前提交。
            }
            link_.wait_for_force(observed, wait_until - ns_now());
        }
    }
    void run() noexcept {
        try {
            auto next=Clock::now();
            while(running_.load()) {
                tick(); next+=Ns(period_);
                const auto now=Clock::now();
                if (now>next) {
                    const auto missed=(now-next)/Ns(period_)+1;
                    skipped_+=missed; next+=missed*Ns(period_);
                }
                service_expiring_batch(next);
                std::this_thread::sleep_until(next);
            }
        } catch(const std::exception& e) {
            std::lock_guard lock(history_lock_); failure_=e.what();
        }
        disarm_feedback("控制线程停止", ns_now());
        clear_target();
        running_=false;
    }
public:
    NativeLoop(Kinematics& kin,const FeedbackControl& control,SharedLink& link,const std::string& port,
               Axes ratio,Axes lo,Axes hi,Axes amplitude,int64_t period_ns,uint32_t disabled_mask=0)
        : kin_(kin),control_(control),link_(link),ratio_(ratio),lo_(lo),hi_(hi),amplitude_(amplitude),
          period_(period_ns),disabled_mask_(disabled_mask),demo_(port.empty()),target_(link.target_size()) {
        if (period_<=0 || period_>100'000'000) throw std::invalid_argument("无效控制周期");
        for(int i=0;i<12;++i) if (!std::isfinite(ratio[i]) || ratio[i]==0 || !std::isfinite(lo[i]) ||
                !std::isfinite(hi[i]) || lo[i]>=hi[i] || !std::isfinite(amplitude[i]))
            throw std::invalid_argument("无效编码器映射");
        transport_.emplace(port);
        if (auto device=transport_->handle()) g4_set_disabled_mask(device, disabled_mask_);
    }
    ~NativeLoop() { close(); }
    void close() {
        std::lock_guard lock(lifecycle_lock_);
        running_=false;
        if(worker_.joinable()) worker_.join();
        transport_.reset();
    }
    void start(const std::vector<double>& zero, bool fresh_zero=false) {
        std::lock_guard lock(lifecycle_lock_);
        if (worker_.joinable() || !transport_) throw std::runtime_error("线程无法启动");
        if ((!zero.empty() && zero.size()!=12) ||
            !std::all_of(zero.begin(),zero.end(),[](double v){return std::isfinite(v);}))
            throw std::invalid_argument("无效零位");
        const int64_t now=ns_now();
        poll_transport(now);
        if (fresh_zero) {
            if (zero.empty() || !transport_->handle() || !zero_capture_ns_ ||
                now-zero_capture_ns_ >= control_.ttl_ns())
                throw std::invalid_argument("标零反馈过期或网关已变化");
            requires_new_zero_=false;
            link_.disarm();
        }
        zero_capture_ns_=0;
        calibrated_=!requires_new_zero_ && !zero.empty();
        if(calibrated_) std::copy(zero.begin(),zero.end(),zero_.begin());
        have_pose_=false; last_force_={};
        start_time_=ns_now(); running_=true;
        worker_=std::thread(&NativeLoop::run,this);
    }
    void stop() {
        std::lock_guard lock(lifecycle_lock_);
        running_=false;
        if(worker_.joinable()) worker_.join();
        zero_capture_ns_=0;
    }
    nb::list raw_for_zero() {
        std::lock_guard lock(lifecycle_lock_);
        if(worker_.joinable() || !transport_) throw std::runtime_error("标零前必须停控制线程");
        const auto now=ns_now();
        if (poll_transport(now)!=GatewayTransport::Event::none || !transport_->handle())
            throw std::invalid_argument("串口离线或已重新连接，请重新标零");
        const auto raw=read_raw(); nb::list result;
        unsigned valid_mask=0;
        for(int k=0;k<raw.size;++k) {
            const auto& j=raw.joints[k]; result.append(nb::make_tuple(j.id,j.bus,j.valid,j.error,j.angle_deg,j.received_ns));
            if(j.id>=1 && j.id<=12 && j.bus>=1 && j.bus<=3 && !j.error && (j.valid&1) &&
               std::isfinite(j.angle_deg) && j.received_ns>0 && j.received_ns<=now &&
               now-j.received_ns<control_.ttl_ns()) valid_mask |= 1u<<(j.id-1);
        }
        zero_capture_ns_=(raw.size==12 && valid_mask==0xfff) ? now : 0;
        return result;
    }
    nb::list samples(int64_t after,bool latest) {
        // 只有拷贝时持锁，构造 Python 字典以及写 CSV 不占用控制线程。
        std::vector<Sample> batch;
        {
            std::lock_guard lock(history_lock_);
            if(!failure_.empty()) throw std::runtime_error(failure_);
            const auto first=latest ? seq_ : std::max<int64_t>(after+1,seq_-history_size+1);
            if (first >= 1 && first <= seq_ && seq_ > after) batch.reserve(seq_ - first + 1);
            for(auto seq=std::max<int64_t>(1,first);seq<=seq_ && seq>after;++seq)
                batch.push_back(history_[seq%history_size]);
        }
        nb::list result;
        for(const auto& s:batch) {
            nb::dict d;
            d["seq"]=s.seq; d["now"]=s.now; d["stamp"]=s.stamp; d["revision"]=s.scene;
            d["cycle_ms"]=s.cycle_ms; d["missed_ticks"]=s.skipped; d["calibrated"]=s.calibrated;
            d["transport_online"]=s.transport_online; d["reconnect_count"]=s.reconnect_count;
            d["requires_new_zero"]=s.requires_new_zero;
            d["requested"]=s.requested; d["raw"]=s.raw; d["q"]=s.q; d["encoder"]=s.encoder;
            d["stamps"]=s.stamps; d["axis_status"]=s.status; d["current"]=s.current;
            std::array<uint32_t,12> errors{};
            for (size_t i=0;i<12;++i) errors[i]=s.joints[i].error;
            d["errors"]=errors; d["force"]=s.force; d["feedback"]=nb::cast(s.control,nb::rv_policy::copy);
            d["pose_error"]=s.pose_error;
            result.append(d);
        }
        return result;
    }
};
} // namespace ve33

void bind_native_loop(nb::module_& m) {
    using namespace ve33;
    using namespace nanobind::literals;
    nb::class_<SharedLink>(m,"SharedLink")
        .def(nb::init<const std::string&,size_t>(),"path"_a,"tree_size"_a=0)
        .def("read_target",&SharedLink::read_target)
        .def("read_display_for",&SharedLink::read_display_for)
        .def("publish_force",&SharedLink::publish_force)
        .def("invalidate_force",&SharedLink::invalidate_force)
        .def("command",&SharedLink::command);
    nb::class_<NativeLoop>(m,"NativeLoop")
        .def(nb::init<Kinematics&,const FeedbackControl&,SharedLink&,const std::string&,Axes,Axes,Axes,Axes,int64_t,uint32_t>(),
             "kin"_a,"control"_a,"link"_a,"port"_a,"ratio"_a,"lo"_a,"hi"_a,"amplitude"_a,"period_ns"_a,"disabled_mask"_a=0,
             nb::keep_alive<1,2>(),nb::keep_alive<1,4>())
        .def("start",&NativeLoop::start,"zero"_a,"fresh_zero"_a=false)
        .def("stop",&NativeLoop::stop,nb::call_guard<nb::gil_scoped_release>())
        .def("close",&NativeLoop::close,nb::call_guard<nb::gil_scoped_release>())
        .def("samples",&NativeLoop::samples).def("raw_for_zero",&NativeLoop::raw_for_zero);
}
