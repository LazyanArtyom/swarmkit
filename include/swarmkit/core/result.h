// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <string>

namespace swarmkit::core {

/// Outcome status for operations that can fail.
enum class StatusCode : std::uint8_t {
    kOk,
    kRejected,
    kFailed,
};

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
};

}  // namespace swarmkit::core
