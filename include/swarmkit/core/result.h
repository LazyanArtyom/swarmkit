// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

namespace swarmkit::core {

/// High-level subsystem that produced an error.
enum class ErrorDomain : std::uint8_t {
    kNone,
    kValidation,
    kConfiguration,
    kSecurity,
    kAuthority,
    kCommand,
    kTransport,
    kBackend,
    kTelemetry,
    kSwarm,
    kInternal,
};

/// Stable SDK error code suitable for application branching and telemetry.
enum class ErrorCode : std::uint8_t {
    kOk,
    kInvalidArgument,
    kRejected,
    kPermissionDenied,
    kNotFound,
    kAlreadyExists,
    kFailedPrecondition,
    kUnsupported,
    kUnavailable,
    kDeadlineExceeded,
    kCancelled,
    kBackendFailure,
    kInternal,
    kUnknown,
};

/// Operational severity for logging, alerting, and UI surfacing.
enum class ErrorSeverity : std::uint8_t {
    kInfo,
    kWarning,
    kError,
    kCritical,
};

/// Whether retrying the same operation is expected to be useful.
enum class ErrorRetryability : std::uint8_t {
    kNever,
    kAfterBackoff,
    kAfterRemediation,
    kUnknown,
};

/// Optional backend/autopilot/vendor detail carried alongside SDK errors.
struct BackendErrorDetails {
    std::string name{};
    std::string protocol{};
    std::string component{};
    std::string native_code{};
    std::string native_message{};
};

/// Unified typed error used by core, client, agent, and swarm APIs.
struct SwarmError {
    ErrorDomain domain{ErrorDomain::kNone};
    ErrorCode code{ErrorCode::kOk};
    ErrorSeverity severity{ErrorSeverity::kInfo};
    ErrorRetryability retryability{ErrorRetryability::kNever};
    std::string user_message{};
    std::string debug_message{};
    std::string correlation_id{};
    std::optional<BackendErrorDetails> backend{};
    std::string remediation{};
    int attempt_count{0};
    std::unordered_map<std::string, std::string> details{};

    [[nodiscard]] static SwarmError Ok(std::string msg = {}) {
        return {
            .domain = ErrorDomain::kNone,
            .code = ErrorCode::kOk,
            .severity = ErrorSeverity::kInfo,
            .retryability = ErrorRetryability::kNever,
            .user_message = std::move(msg),
        };
    }

    [[nodiscard]] static SwarmError Make(ErrorDomain domain, ErrorCode code,
                                         std::string user_message,
                                         ErrorSeverity severity = ErrorSeverity::kError,
                                         ErrorRetryability retryability =
                                             ErrorRetryability::kNever,
                                         std::string remediation = {}) {
        return {
            .domain = domain,
            .code = code,
            .severity = severity,
            .retryability = retryability,
            .user_message = std::move(user_message),
            .remediation = std::move(remediation),
        };
    }

    [[nodiscard]] bool IsOk() const noexcept {
        return code == ErrorCode::kOk;
    }

    [[nodiscard]] bool IsRetryable() const noexcept {
        return retryability == ErrorRetryability::kAfterBackoff ||
               retryability == ErrorRetryability::kAfterRemediation;
    }

    [[nodiscard]] std::string Message() const {
        if (!user_message.empty()) {
            return user_message;
        }
        return debug_message;
    }

    [[nodiscard]] std::string ToString() const;
};

/// @brief Human-readable name for an ErrorDomain.
[[nodiscard]] constexpr std::string_view ToString(ErrorDomain domain) noexcept {
    switch (domain) {
        case ErrorDomain::kNone:
            return "none";
        case ErrorDomain::kValidation:
            return "validation";
        case ErrorDomain::kConfiguration:
            return "configuration";
        case ErrorDomain::kSecurity:
            return "security";
        case ErrorDomain::kAuthority:
            return "authority";
        case ErrorDomain::kCommand:
            return "command";
        case ErrorDomain::kTransport:
            return "transport";
        case ErrorDomain::kBackend:
            return "backend";
        case ErrorDomain::kTelemetry:
            return "telemetry";
        case ErrorDomain::kSwarm:
            return "swarm";
        case ErrorDomain::kInternal:
            return "internal";
    }
    return "unknown";
}

/// @brief Human-readable name for an ErrorCode.
[[nodiscard]] constexpr std::string_view ToString(ErrorCode code) noexcept {
    switch (code) {
        case ErrorCode::kOk:
            return "ok";
        case ErrorCode::kInvalidArgument:
            return "invalid_argument";
        case ErrorCode::kRejected:
            return "rejected";
        case ErrorCode::kPermissionDenied:
            return "permission_denied";
        case ErrorCode::kNotFound:
            return "not_found";
        case ErrorCode::kAlreadyExists:
            return "already_exists";
        case ErrorCode::kFailedPrecondition:
            return "failed_precondition";
        case ErrorCode::kUnsupported:
            return "unsupported";
        case ErrorCode::kUnavailable:
            return "unavailable";
        case ErrorCode::kDeadlineExceeded:
            return "deadline_exceeded";
        case ErrorCode::kCancelled:
            return "cancelled";
        case ErrorCode::kBackendFailure:
            return "backend_failure";
        case ErrorCode::kInternal:
            return "internal";
        case ErrorCode::kUnknown:
            return "unknown";
    }
    return "unknown";
}

/// @brief Human-readable name for an ErrorSeverity.
[[nodiscard]] constexpr std::string_view ToString(ErrorSeverity severity) noexcept {
    switch (severity) {
        case ErrorSeverity::kInfo:
            return "info";
        case ErrorSeverity::kWarning:
            return "warning";
        case ErrorSeverity::kError:
            return "error";
        case ErrorSeverity::kCritical:
            return "critical";
    }
    return "unknown";
}

/// @brief Human-readable name for an ErrorRetryability.
[[nodiscard]] constexpr std::string_view ToString(ErrorRetryability retryability) noexcept {
    switch (retryability) {
        case ErrorRetryability::kNever:
            return "never";
        case ErrorRetryability::kAfterBackoff:
            return "after_backoff";
        case ErrorRetryability::kAfterRemediation:
            return "after_remediation";
        case ErrorRetryability::kUnknown:
            return "unknown";
    }
    return "unknown";
}

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

[[nodiscard]] constexpr StatusCode ToStatusCode(ErrorCode code) noexcept {
    switch (code) {
        case ErrorCode::kOk:
            return StatusCode::kOk;
        case ErrorCode::kInvalidArgument:
        case ErrorCode::kRejected:
        case ErrorCode::kPermissionDenied:
        case ErrorCode::kNotFound:
        case ErrorCode::kAlreadyExists:
        case ErrorCode::kFailedPrecondition:
        case ErrorCode::kUnsupported:
            return StatusCode::kRejected;
        case ErrorCode::kUnavailable:
        case ErrorCode::kDeadlineExceeded:
        case ErrorCode::kCancelled:
        case ErrorCode::kBackendFailure:
        case ErrorCode::kInternal:
        case ErrorCode::kUnknown:
            return StatusCode::kFailed;
    }
    return StatusCode::kFailed;
}

inline std::string SwarmError::ToString() const {
    std::string out = std::string(core::ToString(domain)) + "/" + std::string(core::ToString(code));
    const std::string kMessage = Message();
    if (!kMessage.empty()) {
        out += ": " + kMessage;
    }
    if (!correlation_id.empty()) {
        out += " corr=" + correlation_id;
    }
    if (!remediation.empty()) {
        out += " remediation=" + remediation;
    }
    return out;
}

/// Lightweight operation result carrying a broad status plus typed error detail.
struct Result {
    StatusCode code{StatusCode::kOk};
    std::string message;
    SwarmError error;

    /// Create a successful result with an optional message.
    [[nodiscard]] static Result Success(std::string msg = {}) {
        std::string message = std::move(msg);
        return {
            .code = StatusCode::kOk,
            .message = message,
            .error = SwarmError::Ok(std::move(message)),
        };
    }

    /// Create a rejected result with the given reason.
    [[nodiscard]] static Result Rejected(std::string msg) {
        std::string message = std::move(msg);
        return {
            .code = StatusCode::kRejected,
            .message = message,
            .error = SwarmError::Make(ErrorDomain::kValidation, ErrorCode::kRejected, message,
                                      ErrorSeverity::kWarning,
                                      ErrorRetryability::kAfterRemediation),
        };
    }

    /// Create a failed result with the given reason.
    [[nodiscard]] static Result Failed(std::string msg) {
        std::string message = std::move(msg);
        return {
            .code = StatusCode::kFailed,
            .message = message,
            .error = SwarmError::Make(ErrorDomain::kInternal, ErrorCode::kInternal, message,
                                      ErrorSeverity::kError, ErrorRetryability::kUnknown),
        };
    }

    /// Create a result from a fully populated typed error.
    [[nodiscard]] static Result FromError(SwarmError err) {
        const StatusCode status = ToStatusCode(err.code);
        std::string message = err.Message();
        return {
            .code = status,
            .message = std::move(message),
            .error = std::move(err),
        };
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
        if (!error.IsOk()) {
            return error.ToString();
        }
        return std::string(core::ToString(code)) + ": " + message;
    }
};

}  // namespace swarmkit::core
