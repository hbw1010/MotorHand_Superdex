// 本地抓握两个进程共用的单写者邮箱。每个快照携带原始输入时戳。
#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cstdint>
#include <cstring>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <linux/futex.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace ve33 {
constexpr size_t max_tree_size = 128;
constexpr size_t force_size = 64;
using ForcePacket = std::array<double, force_size>;
using DisplayPacket = std::array<double, 30>;
static_assert(std::atomic<uint64_t>::is_always_lock_free);
static_assert(std::atomic<uint32_t>::is_always_lock_free && sizeof(std::atomic<uint32_t>) == 4);

// 跨进程使用自然对齐的无锁原子。写方被抢占时读方退避重试，避免瞬时写窗误判为无效帧。
template<size_t N> struct alignas(64) Latest {
    std::atomic<uint64_t> seq{};
    std::array<std::atomic<uint64_t>, N> words{};
    void store(const double* src, size_t count) {
        seq.fetch_add(1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        for (size_t i = 0; i < count; ++i)
            words[i].store(std::bit_cast<uint64_t>(src[i]), std::memory_order_relaxed);
        seq.fetch_add(1, std::memory_order_release);
    }
    bool load(double* dst, size_t count) const {
        for (int attempt = 0; attempt < 64; ++attempt) {
            const auto n = seq.load(std::memory_order_acquire);
            if (n & 1) {
                if ((attempt & 15) == 15) std::this_thread::yield();
#if defined(__x86_64__) || defined(__i386__)
                else __builtin_ia32_pause();
#endif
                continue;
            }
            for (size_t i = 0; i < count; ++i)
                dst[i] = std::bit_cast<double>(words[i].load(std::memory_order_relaxed));
            std::atomic_thread_fence(std::memory_order_acquire);
            if (seq.load(std::memory_order_relaxed) == n) return true;
            if ((attempt & 15) == 15) std::this_thread::yield();
#if defined(__x86_64__) || defined(__i386__)
            else __builtin_ia32_pause();
#endif
        }
        return false;
    }
};

struct Intent {
    uint64_t generation{}, disabled_generation{};
    int64_t revision{};
    bool enabled{};
    double grip{-1.};
};
enum class CommitPermission { busy, changed, allowed };

class SharedLink {
    struct Page {
        static constexpr uint32_t abi_version=5;
        uint32_t version{abi_version}, tree_size{};
        pthread_mutex_t intent_lock{};
        Intent intent;
        Latest<1+max_tree_size+12> target;
        Latest<force_size> force;
        alignas(4) std::atomic<uint32_t> force_notice{};
        // 100 Hz 控制，16 项可覆盖一条 100 ms 指令的完整有效窗口。
        std::array<Latest<30>,16> display;
        std::atomic<uint64_t> display_count{};
    };
    Page* page_{};
    void notify_force() {
        page_->force_notice.fetch_add(1, std::memory_order_release);
        syscall(SYS_futex, &page_->force_notice, FUTEX_WAKE, 1, nullptr, nullptr, 0);
    }
    // 仅限文件首次创建或同一进程内、尚未对外发布时初始化。
    void init(size_t tree_size) {
        new (page_) Page{};
        page_->tree_size = tree_size;
        pthread_mutexattr_t attr;
        if (pthread_mutexattr_init(&attr)) throw std::runtime_error("无法初始化命令锁");
        bool failed = pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED) ||
                      pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST) ||
                      pthread_mutex_init(&page_->intent_lock, &attr);
        pthread_mutexattr_destroy(&attr);
        if (failed) throw std::runtime_error("无法创建进程共享命令锁");
    }
    bool lock(bool wait) const {
        const int rc = wait ? pthread_mutex_lock(&page_->intent_lock) : pthread_mutex_trylock(&page_->intent_lock);
        if (!rc) return true;
        if (rc == EBUSY) return false;
        if (rc == EOWNERDEAD) {
            page_->intent.enabled = false;
            page_->intent.disabled_generation = ++page_->intent.generation;
            pthread_mutex_consistent(&page_->intent_lock);
            return true;
        }
        throw std::runtime_error("命令通道不可用");
    }
    struct Unlock { pthread_mutex_t* p; ~Unlock(){ pthread_mutex_unlock(p); } };
public:
    explicit SharedLink(const std::string& path, size_t tree_size=0) {
        if (tree_size > max_tree_size) throw std::invalid_argument("无效树维度");
        const bool create = tree_size != 0;
        const int fd = open(path.c_str(), O_RDWR|O_CLOEXEC|O_NOFOLLOW|(create ? O_CREAT|O_EXCL : 0), 0600);
        if (fd < 0) throw std::runtime_error("打开抓握共享通道失败");
        struct stat info{};
        bool failed = (create && ftruncate(fd, sizeof(Page))) || fstat(fd, &info) || info.st_size != sizeof(Page);
        auto* ptr = failed ? MAP_FAILED : mmap(nullptr,sizeof(Page),PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
        close(fd);
        if (ptr == MAP_FAILED) throw std::runtime_error("映射抓握共享通道失败");
        page_ = static_cast<Page*>(ptr);
        try {
            if (create) init(tree_size);
            if (page_->version != Page::abi_version || !page_->tree_size || page_->tree_size > max_tree_size)
                throw std::runtime_error("不兼容的抓握共享通道");
        } catch (...) { munmap(page_,sizeof(Page)); throw; }
    }
    SharedLink(const SharedLink&) = delete;
    ~SharedLink() { munmap(page_,sizeof(Page)); }
    size_t target_size() const { return 1 + page_->tree_size + 12; }
    bool try_read_target(std::vector<double>& result) const {
        result.resize(target_size());
        return page_->target.load(result.data(), result.size());
    }
    std::vector<double> read_target() const {
        std::vector<double> result(target_size());
        if (!page_->target.load(result.data(), result.size())) result[0] = 0.;
        return result;
    }
    void publish_target(const std::vector<double>& data) {
        if (data.size()!=target_size()) throw std::invalid_argument("目标帧长度错误");
        page_->target.store(data.data(),data.size());
    }
    void publish_force(const std::vector<double>& data) {
        if (data.size()!=force_size) throw std::invalid_argument("接触帧长度错误");
        page_->force.store(data.data(),data.size());
        notify_force();
    }
    void invalidate_force() {
        ForcePacket zero{};
        page_->force.store(zero.data(),force_size);
        notify_force();
    }
    uint32_t force_notice() const { return page_->force_notice.load(std::memory_order_acquire); }
    void wait_for_force(uint32_t observed, int64_t duration_ns) const {
        if (duration_ns <= 0) return;
        const timespec timeout{duration_ns / 1'000'000'000, duration_ns % 1'000'000'000};
        // 通知先于等待时内核返回 EAGAIN；调用方始终重新检查真实快照与期限。
        syscall(SYS_futex, &page_->force_notice, FUTEX_WAIT, observed, &timeout, nullptr, 0);
    }
    bool read_force(ForcePacket& data) const { return page_->force.load(data.data(),force_size); }
    void publish_display(const DisplayPacket& d) {
        const auto n = page_->display_count.load(std::memory_order_relaxed);
        page_->display[n % page_->display.size()].store(d.data(), d.size());
        page_->display_count.store(n + 1, std::memory_order_release);
    }
    std::vector<double> read_display_for(int64_t stamp, int64_t revision) const {
        Intent intent;
        if (stamp <= 0 || !read_intent(intent) || intent.revision != revision) return {};
        const auto n = page_->display_count.load(std::memory_order_acquire);
        const auto count = std::min<uint64_t>(n, page_->display.size());
        DisplayPacket d;
        double active = 0.;
        for (uint64_t i = 0; i < count; ++i) {
            const auto& slot = page_->display[(n - i - 1) % page_->display.size()];
            if (!slot.load(d.data(),d.size()) || d[29] != double(intent.generation) ||
                d[26] != double(revision) || d[27] <= 0.) return {};
            if (i == 0) active = d[1];
            if (d[1] != active || d[28] < double(stamp)) return {};
            if (d[28] == double(stamp)) return {d.begin(),d.end()-1};
        }
        return {};
    }
    void command(bool enabled,int64_t revision,double grip) {
        lock(true); Unlock unlock{&page_->intent_lock};
        auto& i=page_->intent;
        if (i.enabled!=enabled || i.revision!=revision) ++i.generation;
        if (i.enabled && !enabled) i.disabled_generation=i.generation;
        i.enabled=enabled; i.revision=revision; i.grip=grip;
    }
    bool read_intent(Intent& i) const {
        if (!lock(false)) return false;
        Unlock unlock{&page_->intent_lock}; i=page_->intent; return true;
    }
    void disarm() {
        lock(true); Unlock unlock{&page_->intent_lock};
        page_->intent.enabled=false;
        page_->intent.disabled_generation=++page_->intent.generation;
    }
    template<typename F> CommitPermission if_permitted(const Intent& i,F send) {
        if (!lock(false)) return CommitPermission::busy;
        Unlock unlock{&page_->intent_lock};
        if (!page_->intent.enabled || i.generation!=page_->intent.generation) return CommitPermission::changed;
        send(); return CommitPermission::allowed;
    }
};
} // namespace ve33
