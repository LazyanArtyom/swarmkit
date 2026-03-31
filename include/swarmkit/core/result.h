// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace swarmkit::core {

/// Outcome status for operations that can fail.
enum class StatusCode : std::uint8_t {
    kOk,
    kRejected,
    kFailed,
};

/// @brief Human-readable name for a StatusCode.
[[nodiscard]] constexpr std::string_view ToString(StatusCode code) noexcept {
    switch (code) {
        case StatusCode::kOk:
            return "ok";
        case StatusCode::kRejected:
            return "rejected";
        case StatusCode::kFailed:
            return "failed";
    }
    return "unknown";
}

/// Lightweight operation result carrying a status code and optional message.
struct Result {
    StatusCode code{StatusCode::kOk};
    std::string message;

    /// Create a successful result with an optional message.
    [[nodiscard]] static Result Success(std::string msg = {}) {
        return {.code = StatusCode::kOk, .message = std::move(msg)};
    }

    /// Create a rejected result with the given reason.
    [[nodiscard]] static Result Rejected(std::string msg) {
        return {.code = StatusCode::kRejected, .message = std::move(msg)};
    }

    /// Create a failed result with the given reason.
    [[nodiscard]] static Result Failed(std::string msg) {
        return {.code = StatusCode::kFailed, .message = std::move(msg)};
    }

    /// Returns true when the operation succeeded.
    [[nodiscard]] bool IsOk() const noexcept {
        return code == StatusCode::kOk;
    }

    /// Boolean conversion -- equivalent to IsOk().
    explicit operator bool() const noexcept {
        return IsOk();
    }

    /// @brief Human-readable representation: "status_code: message" or just "ok".
    [[nodiscard]] std::string ToString() const {
        if (IsOk()) {
            return message.empty() ? std::string(core::ToString(code))
                                   : std::string(core::ToString(code)) + ": " + message;
        }
        return std::string(core::ToString(code)) + ": " + message;
    }
};

}  // namespace swarmkit::core
