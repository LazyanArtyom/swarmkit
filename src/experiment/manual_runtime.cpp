// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/experiment/manual_runtime.h"

#include <mutex>

namespace swarmkit::experiment {

struct ManualRuntime::State {
    mutable std::mutex mutex;
    std::int64_t wall_time_ms{};
    std::int64_t monotonic_time_ns{};
    std::uint64_t next_id{};
};

ManualRuntime::ManualRuntime(std::int64_t wall_time_ms, std::int64_t monotonic_time_ns)
    : state_(std::make_shared<State>()) {
    state_->wall_time_ms = wall_time_ms;
    state_->monotonic_time_ns = monotonic_time_ns;
}

std::int64_t ManualRuntime::WallTimeMs() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->wall_time_ms;
}

std::int64_t ManualRuntime::MonotonicTimeNs() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->monotonic_time_ns;
}

std::string ManualRuntime::NewId(std::string_view prefix) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return std::string(prefix) + "-" + std::to_string(++state_->next_id);
}

void ManualRuntime::AdvanceMilliseconds(std::int64_t milliseconds) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->wall_time_ms += milliseconds;
    state_->monotonic_time_ns += milliseconds * 1'000'000;
}

void ManualRuntime::Set(std::int64_t wall_time_ms, std::int64_t monotonic_time_ns) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->wall_time_ms = wall_time_ms;
    state_->monotonic_time_ns = monotonic_time_ns;
}

}  // namespace swarmkit::experiment
