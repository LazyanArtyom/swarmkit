// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "swarmkit/core/result.h"

namespace swarmkit::core {

enum class BackendDispatchState : std::uint8_t {
    kUnknown,
    kAccepted,
    kRejected,
    kFailed,
};

/// One native protocol response observed during command execution.
struct BackendProtocolResponse {
    std::string protocol;
    std::string command_name;
    std::optional<std::uint32_t> native_command_id;
    bool response_expected{false};
    bool response_received{false};
    bool response_timed_out{false};
    std::optional<std::int32_t> result_code;
    std::string result_name;
    std::string status_text;
};

/// Typed backend outcome. Protocol ACK evidence never implies physical arrival.
struct BackendCommandOutcome {
    Result result{Result::Failed("backend outcome unavailable")};
    BackendDispatchState dispatch_state{BackendDispatchState::kUnknown};
    std::int64_t dispatch_monotonic_time_ns{};
    std::int64_t completion_monotonic_time_ns{};
    std::vector<BackendProtocolResponse> protocol_responses;

    [[nodiscard]] bool IsOk() const noexcept {
        return result.IsOk();
    }
};

}  // namespace swarmkit::core
