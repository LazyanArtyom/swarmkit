// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <expected>
#include <fstream>
#include <iostream>
#include <iterator>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include "common/arg_utils.h"
#include "constants.h"
#include "options.h"
#include "swarmkit/client/client.h"

namespace swarmkit::apps::cli::internal::data_runtime_detail {

using swarmkit::client::Client;

[[nodiscard]] inline volatile std::sig_atomic_t& StopRequestedFlag() {
    static volatile std::sig_atomic_t stop_requested = 0;
    return stop_requested;
}

inline void OnDataSignal(int /*sig*/) {
    StopRequestedFlag() = 1;
}

[[nodiscard]] inline bool IsStopRequested() {
    return StopRequestedFlag() != 0;
}

inline void ResetStopRequested() {
    StopRequestedFlag() = 0;
}

[[nodiscard]] inline std::vector<std::string> FindActionsAfterCommand(int argc, char** argv,
                                                               std::string_view command_name) {
    int command_index = -1;
    for (int index = 1; index < argc; ++index) {
        const std::string_view current_arg = argv[index];
        if (IsOptionWithValue(current_arg)) {
            ++index;
            continue;
        }
        if (current_arg == command_name) {
            command_index = index;
            break;
        }
    }
    if (command_index < 0) {
        return {};
    }
    std::vector<std::string> actions;
    for (int index = command_index + 1; index < argc; ++index) {
        const std::string_view current_arg = argv[index];
        if (IsOptionWithValue(current_arg)) {
            ++index;
            continue;
        }
        if (current_arg.starts_with("-")) {
            continue;
        }
        actions.emplace_back(current_arg);
    }
    return actions;
}

[[nodiscard]] inline std::vector<std::string> CollectOptionValues(int argc, char** argv,
                                                           std::string_view option_name) {
    std::vector<std::string> values;
    for (int index = 1; index + 1 < argc; ++index) {
        if (std::string_view(argv[index]) == option_name) {
            values.emplace_back(argv[++index]);
        }
    }
    return values;
}

[[nodiscard]] inline std::expected<std::string, std::string> ReadSmallPayload(int argc, char** argv) {
    if (const std::string payload = common::GetOptionValue(argc, argv, "--payload");
        !payload.empty()) {
        return payload;
    }
    const std::string file_path = common::GetOptionValue(argc, argv, "--file");
    if (file_path.empty()) {
        return {};
    }
    std::ifstream input(file_path, std::ios::binary);
    if (!input.is_open()) {
        return std::unexpected("failed to open payload file '" + file_path + "'");
    }
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

[[nodiscard]] inline std::expected<std::int64_t, std::string> ParseDurationMs(int argc, char** argv) {
    const auto duration = ParseIntArg(
        common::GetOptionValue(argc, argv, "--duration-ms", kDefaultZero), "--duration-ms");
    if (!duration.has_value()) {
        return std::unexpected(duration.error());
    }
    if (*duration < 0) {
        return std::unexpected("--duration-ms must be >= 0");
    }
    return static_cast<std::int64_t>(*duration);
}

[[nodiscard]] inline std::expected<std::unordered_map<std::string, std::string>, std::string>
ParseKeyValueLabels(int argc, char** argv, std::string_view option_name) {
    std::unordered_map<std::string, std::string> labels;
    for (int index = 1; index < argc; ++index) {
        if (std::string_view(argv[index]) != option_name) {
            continue;
        }
        if (index + 1 >= argc) {
            return std::unexpected(std::string(option_name) + " requires KEY=VALUE");
        }
        const std::string entry = argv[++index];
        const std::size_t separator = entry.find('=');
        if (separator == std::string::npos || separator == 0) {
            return std::unexpected(std::string(option_name) + " requires KEY=VALUE");
        }
        labels[entry.substr(0, separator)] = entry.substr(separator + 1);
    }
    return labels;
}

}  // namespace swarmkit::apps::cli::internal::data_runtime_detail
