// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

/// @file env_utils.h
/// @brief Internal utilities shared by client and agent config code.
///
/// These are implementation details — not part of the public SDK.

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <expected>
#include <optional>
#include <string>
#include <string_view>

#include "swarmkit/commands.h"
#include "swarmkit/core/result.h"

namespace swarmkit::core::internal {

// ---------------------------------------------------------------------------
// String helpers
// ---------------------------------------------------------------------------

/// @brief Trim leading and trailing whitespace.
[[nodiscard]] inline std::string TrimWhitespace(std::string_view value) {
    std::size_t start = 0;
    while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start])) != 0) {
        ++start;
    }

    std::size_t end = value.size();
    while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1])) != 0) {
        --end;
    }

    return std::string(value.substr(start, end - start));
}

// ---------------------------------------------------------------------------
// Environment variable helpers
// ---------------------------------------------------------------------------

/// @brief Read an environment variable, returning nullopt if unset.
[[nodiscard]] inline std::optional<std::string> GetEnvValue(std::string_view key) {
    const char* value = std::getenv(std::string(key).c_str());
    if (value == nullptr) {
        return std::nullopt;
    }
    return std::string(value);
}

// ---------------------------------------------------------------------------
// Parsing helpers
// ---------------------------------------------------------------------------

/// @brief Parse an integer from a string, returning an error on failure.
[[nodiscard]] inline std::expected<int, Result> ParseIntValue(std::string_view value,
                                                              std::string_view field_name) {
    try {
        return std::stoi(std::string(value));
    } catch (const std::exception&) {
        return std::unexpected(
            Result::Rejected("invalid integer for '" + std::string(field_name) + "'"));
    }
}

/// @brief Parse a boolean from common string representations.
[[nodiscard]] inline std::expected<bool, Result> ParseBoolValue(std::string_view value,
                                                                std::string_view field_name) {
    const std::string kNormalized = TrimWhitespace(value);
    if (kNormalized == "1" || kNormalized == "true" || kNormalized == "TRUE" ||
        kNormalized == "yes" || kNormalized == "on") {
        return true;
    }
    if (kNormalized == "0" || kNormalized == "false" || kNormalized == "FALSE" ||
        kNormalized == "no" || kNormalized == "off") {
        return false;
    }
    return std::unexpected(
        Result::Rejected("invalid boolean for '" + std::string(field_name) + "'"));
}

/// @brief Parse a CommandPriority from a string (name or integer).
[[nodiscard]] inline std::expected<commands::CommandPriority, Result> ParsePriorityValue(
    std::string_view value, std::string_view field_name) {
    const std::string kNormalized = TrimWhitespace(value);
    if (kNormalized == "operator" || kNormalized == "OPERATOR") {
        return commands::CommandPriority::kOperator;
    }
    if (kNormalized == "supervisor" || kNormalized == "SUPERVISOR") {
        return commands::CommandPriority::kSupervisor;
    }
    if (kNormalized == "override" || kNormalized == "OVERRIDE") {
        return commands::CommandPriority::kOverride;
    }
    if (kNormalized == "emergency" || kNormalized == "EMERGENCY") {
        return commands::CommandPriority::kEmergency;
    }

    const auto kParsed = ParseIntValue(kNormalized, field_name);
    if (!kParsed.has_value()) {
        return std::unexpected(kParsed.error());
    }
    return static_cast<commands::CommandPriority>(*kParsed);
}

// ---------------------------------------------------------------------------
// Validation helpers
// ---------------------------------------------------------------------------

/// @brief Quick check whether a string looks like a host:port address.
[[nodiscard]] inline bool LooksLikeAddress(std::string_view address) {
    return !address.empty() && address.find(':') != std::string_view::npos;
}

/// @brief Check whether a CommandPriority value is one of the known levels.
[[nodiscard]] inline bool IsValidPriority(commands::CommandPriority priority) {
    switch (priority) {
        case commands::CommandPriority::kOperator:
        case commands::CommandPriority::kSupervisor:
        case commands::CommandPriority::kOverride:
        case commands::CommandPriority::kEmergency:
            return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Correlation ID generation
// ---------------------------------------------------------------------------

/// @brief Thread-safe generator for unique correlation IDs.
///
/// Produces IDs of the form "prefix-timestamp_ms-sequence".
class CorrelationIdGenerator {
   public:
    [[nodiscard]] std::string Next(std::string_view prefix) {
        const std::uint64_t kSequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
        const auto kNow = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
        return std::string(prefix) + "-" + std::to_string(kNow.count()) + "-" +
               std::to_string(kSequence);
    }

   private:
    std::atomic<std::uint64_t> next_sequence_{1};
};

/// @brief Access the process-global correlation ID generator.
[[nodiscard]] inline CorrelationIdGenerator& GetCorrelationIdGenerator() {
    static CorrelationIdGenerator generator;
    return generator;
}

/// @brief Generate a new unique correlation ID with the given prefix.
[[nodiscard]] inline std::string MakeCorrelationId(std::string_view prefix) {
    return GetCorrelationIdGenerator().Next(prefix);
}

// ---------------------------------------------------------------------------
// Environment config application helpers
// ---------------------------------------------------------------------------

/// @brief Apply an integer environment variable (prefix + suffix) to an output.
inline void ApplyIntEnv(std::string_view prefix, std::string_view suffix, int* out) {
    const auto kValue = GetEnvValue(std::string(prefix) + std::string(suffix));
    if (!kValue.has_value()) {
        return;
    }
    const auto kParsed = ParseIntValue(*kValue, suffix);
    if (kParsed.has_value()) {
        *out = *kParsed;
    }
}

/// @brief Apply a boolean environment variable (prefix + suffix) to an output.
inline void ApplyBoolEnv(std::string_view prefix, std::string_view suffix, bool* out) {
    const auto kValue = GetEnvValue(std::string(prefix) + std::string(suffix));
    if (!kValue.has_value()) {
        return;
    }
    const auto kParsed = ParseBoolValue(*kValue, suffix);
    if (kParsed.has_value()) {
        *out = *kParsed;
    }
}

/// @brief Apply a string environment variable (prefix + suffix) to an output.
inline void ApplyStringEnv(std::string_view prefix, std::string_view suffix, std::string* out) {
    const auto kValue = GetEnvValue(std::string(prefix) + std::string(suffix));
    if (kValue.has_value()) {
        *out = *kValue;
    }
}

}  // namespace swarmkit::core::internal
