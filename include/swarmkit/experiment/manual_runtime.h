// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

namespace swarmkit::experiment {

/// Thread-safe deterministic clock and identifier source for experiment tests.
class ManualRuntime {
   public:
    explicit ManualRuntime(std::int64_t wall_time_ms = 1'700'000'000'000LL,
                           std::int64_t monotonic_time_ns = 0);

    [[nodiscard]] std::int64_t WallTimeMs() const;
    [[nodiscard]] std::int64_t MonotonicTimeNs() const;
    [[nodiscard]] std::string NewId(std::string_view prefix);

    void AdvanceMilliseconds(std::int64_t milliseconds);
    void Set(std::int64_t wall_time_ms, std::int64_t monotonic_time_ns);

   private:
    struct State;
    std::shared_ptr<State> state_;
};

}  // namespace swarmkit::experiment
