// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "runtime_providers.h"

#include <chrono>
#include <utility>

#include "env_utils.h"

namespace swarmkit::agent::internal {
namespace {

[[nodiscard]] std::int64_t SystemWallTimeMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

[[nodiscard]] std::int64_t SystemMonotonicTimeNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

RuntimeProviders RuntimeProviders::System() {
    return {
        .wall_time_ms = SystemWallTimeMs,
        .monotonic_time_ns = SystemMonotonicTimeNs,
        .new_id = [](std::string_view prefix) { return core::internal::MakeCorrelationId(prefix); },
    };
}

std::int64_t RuntimeProviders::WallTimeMs() const {
    return wall_time_ms ? wall_time_ms() : SystemWallTimeMs();
}

std::int64_t RuntimeProviders::MonotonicTimeNs() const {
    return monotonic_time_ns ? monotonic_time_ns() : SystemMonotonicTimeNs();
}

std::string RuntimeProviders::NewId(std::string_view prefix) const {
    return new_id ? new_id(prefix) : core::internal::MakeCorrelationId(prefix);
}

}  // namespace swarmkit::agent::internal
