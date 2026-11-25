#include "common/inverse_worker.hpp"

#include "ve33_model.hpp"
#include "common/hand.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace motorhand::native {

template <std::size_t N, std::size_t G>
CommonInverseWorker<N, G>::CommonInverseWorker(std::filesystem::path glove_asset_root,
                                               HandModelFactory<N> hand_factory, bool left,
                                               std::array<std::array<double, 2>, G> motor_bounds)
    : glove_asset_root_(std::move(glove_asset_root)),
      hand_factory_(std::move(hand_factory)), left_(left),
      motor_bounds_(motor_bounds), thread_(&CommonInverseWorker<N, G>::run, this) {}

template <std::size_t N, std::size_t G>
CommonInverseWorker<N, G>::~CommonInverseWorker() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
        pending_.reset();
    }
    condition_.notify_one();
    if (thread_.joinable()) thread_.join();
}

template <std::size_t N, std::size_t G>
std::uint64_t CommonInverseWorker<N, G>::submit(const HandState<N>& hand, const GlovePoseT<G>& seed,
                                                std::uint64_t source_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialization_error_) std::rethrow_exception(initialization_error_);
    if (stopping_) throw std::runtime_error("inverse worker is stopping");
    if (last_request_id_ == std::numeric_limits<std::uint64_t>::max())
        throw std::overflow_error("inverse worker request ID exhausted");
    const std::uint64_t id = ++last_request_id_;
    pending_ = Request{hand, seed, source_ns, id};
    condition_.notify_one();
    return id;
}

template <std::size_t N, std::size_t G>
std::optional<InverseWorkerResultT<G>> CommonInverseWorker<N, G>::poll() {
    Completion completion;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!completed_) {
            if (initialization_error_) std::rethrow_exception(initialization_error_);
            return std::nullopt;
        }
        completion = std::move(*completed_);
        completed_.reset();
    }
    condition_.notify_one();
    if (completion.error) std::rethrow_exception(completion.error);
    return std::move(completion.value);
}

template <std::size_t N, std::size_t G>
void CommonInverseWorker<N, G>::run() noexcept {
    try {
        GloveModelT<G> glove(glove_asset_root_, left_);
        auto hand = hand_factory_();
        CommonInverseRetarget<N, G> inverse(glove, *hand, motor_bounds_);
        for (;;) {
            std::optional<Request> request;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this] {
                    const bool failure_latched = completed_ &&
                        (completed_->error || !completed_->value->result.success);
                    return stopping_ || (pending_.has_value() && !failure_latched);
                });
                if (stopping_) return;
                request = std::move(pending_);
                pending_.reset();
            }

            Completion completion;
            try {
                InverseResultT<G> result = inverse.solve(request->hand.position_rad, request->seed);
                completion.value.emplace(InverseWorkerResultT<G>{
                    std::move(result), request->source_ns, request->request_id});
            } catch (...) {
                completion.error = std::current_exception();
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (stopping_) return;
                completed_ = std::move(completion);
            }
        }
    } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        initialization_error_ = std::current_exception();
    }
}

template class CommonInverseWorker<20, 12>;
template class CommonInverseWorker<22, 12>;
template class CommonInverseWorker<22, 16>;
}  // namespace motorhand::native
