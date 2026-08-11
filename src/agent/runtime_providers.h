// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>

namespace swarmkit::agent::internal {

/// Injectable correctness-sensitive clocks and opaque ID generation.
struct RuntimeProviders {
    std::function<std::int64_t()> wall_time_ms;
    std::function<std::int64_t()> monotonic_time_ns;
    std::function<std::string(std::string_view)> new_id;

    [[nodiscard]] static RuntimeProviders System();
    [[nodiscard]] std::int64_t WallTimeMs() const;
    [[nodiscard]] std::int64_t MonotonicTimeNs() const;
    [[nodiscard]] std::string NewId(std::string_view prefix) const;
};

}  // namespace swarmkit::agent::internal
