#pragma once
#include "common/hand.hpp"
#include "common/inverse_retarget.hpp"

#include <array>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <mutex>
#include <optional>
#include <thread>

namespace motorhand::native {

template <std::size_t G = 12> struct InverseWorkerResultT {
    InverseResultT<G> result;
    std::uint64_t source_ns{};
    std::uint64_t request_id{};
};
using InverseWorkerResult = InverseWorkerResultT<12>;
using InverseWorkerResult16 = InverseWorkerResultT<16>;

template <std::size_t N, std::size_t G = 12> class CommonInverseWorker {
public:
    CommonInverseWorker(std::filesystem::path glove_asset_root,
                        HandModelFactory<N> hand_factory, bool left,
                        std::array<std::array<double, 2>, G> motor_bounds);
    ~CommonInverseWorker();
    CommonInverseWorker(const CommonInverseWorker&) = delete;
    CommonInverseWorker& operator=(const CommonInverseWorker&) = delete;

    std::uint64_t submit(const HandState<N>& hand, const GlovePoseT<G>& seed,
                         std::uint64_t source_ns);
    std::optional<InverseWorkerResultT<G>> poll();

private:
    struct Request {
        HandState<N> hand;
        GlovePoseT<G> seed;
        std::uint64_t source_ns;
        std::uint64_t request_id;
    };
    struct Completion {
        std::optional<InverseWorkerResultT<G>> value;
        std::exception_ptr error;
    };

    void run() noexcept;

    std::filesystem::path glove_asset_root_;
    HandModelFactory<N> hand_factory_;
    bool left_;
    std::array<std::array<double, 2>, G> motor_bounds_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::optional<Request> pending_;
    std::optional<Completion> completed_;
    std::exception_ptr initialization_error_;
    std::uint64_t last_request_id_{};
    bool stopping_{};
    std::thread thread_;
};

}  // namespace motorhand::native
