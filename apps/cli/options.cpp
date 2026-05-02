// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "options.h"

#include <cstdlib>
#include <exception>
#include <iostream>
#include <vector>

#include "common/arg_utils.h"
#include "constants.h"

namespace swarmkit::apps::cli::internal {

[[nodiscard]] bool IsSubcommand(std::string_view value) {
    return value == "ping" || value == "health" || value == "stats" || value == "telemetry" ||
           value == "command" || value == "lock" || value == "unlock" ||
           value == "watch-authority" || value == "swarm";
}

[[nodiscard]] bool IsOptionWithValue(std::string_view value) {
    return value == "--config" || value == "--drone" || value == "--rate" || value == "--alt" ||
           value == "--lat" || value == "--lon" || value == "--speed" || value == "--log-sink" ||
           value == "--log-file" || value == "--log-level" || value == "--ca-cert" ||
           value == "--client-cert" || value == "--client-key" || value == "--server-name" ||
           value == "--priority" || value == "--mode" || value == "--custom-mode" ||
           value == "--ground" || value == "--deg" || value == "--yaw" || value == "--vx" ||
           value == "--vy" || value == "--vz" || value == "--duration-ms" || value == "--ttl-ms" ||
           value == "--swarm-config" || value == "--address-mode" || value == "--file" ||
           value == "--seq" || value == "--first" || value == "--last" || value == "--camera" ||
           value == "--stream" || value == "--interval" || value == "--count" ||
           value == "--pitch" || value == "--roll" || value == "--gimbal" || value == "--servo" ||
           value == "--pwm" || value == "--relay" || value == "--gripper";
}

[[nodiscard]] std::expected<float, std::string> ParseFloatArg(std::string_view value,
                                                              std::string_view key) {
    try {
        return std::stof(std::string(value));
    } catch (const std::exception& exc) {
        return std::unexpected("Invalid " + std::string(key) + " value '" + std::string(value) +
                               "': " + exc.what());
    }
}

[[nodiscard]] std::expected<double, std::string> ParseDoubleArg(std::string_view value,
                                                                std::string_view key) {
    try {
        return std::stod(std::string(value));
    } catch (const std::exception& exc) {
        return std::unexpected("Invalid " + std::string(key) + " value '" + std::string(value) +
                               "': " + exc.what());
    }
}

[[nodiscard]] std::expected<int, std::string> ParseIntArg(std::string_view value,
                                                          std::string_view key) {
    try {
        return std::stoi(std::string(value));
    } catch (const std::exception& exc) {
        return std::unexpected("Invalid " + std::string(key) + " value '" + std::string(value) +
                               "': " + exc.what());
    }
}

[[nodiscard]] std::expected<commands::CommandPriority, std::string> ParseCliPriority(int argc,
                                                                                     char** argv) {
    const std::string priority_text =
        common::GetOptionValue(argc, argv, "--priority", kDefaultPriority);

    if (priority_text == "operator") {
        return commands::CommandPriority::kOperator;
    }
    if (priority_text == "supervisor") {
        return commands::CommandPriority::kSupervisor;
    }
    if (priority_text == "override") {
        return commands::CommandPriority::kOverride;
    }
    if (priority_text == "emergency") {
        return commands::CommandPriority::kEmergency;
    }
    return std::unexpected("Invalid --priority value '" + priority_text +
                           "': expected operator|supervisor|override|emergency");
}

[[nodiscard]] std::expected<CliInvocation, int> ParseInvocation(int argc, char** argv) {
    std::vector<std::string> positional_args;
    positional_args.reserve(static_cast<std::size_t>(argc));
    for (int idx = 1; idx < argc; ++idx) {
        const std::string_view kArg = argv[idx];
        if (IsOptionWithValue(kArg)) {
            ++idx;
            continue;
        }
        if (kArg.starts_with('-')) {
            continue;
        }
        positional_args.emplace_back(kArg);
    }

    CliInvocation invocation;
    if (!positional_args.empty() && !IsSubcommand(positional_args.front())) {
        invocation.has_explicit_address = true;
        invocation.address = positional_args.front();
        invocation.command =
            positional_args.size() >= 2 ? positional_args[1] : std::string(kDefaultCommand);
        return invocation;
    }

    invocation.command =
        positional_args.empty() ? std::string(kDefaultCommand) : positional_args.front();
    return invocation;
}

[[nodiscard]] std::expected<core::LoggerConfig, int> BuildCliLoggerConfig(int argc, char** argv) {
    core::LoggerConfig log_cfg;
    log_cfg.sink_type = core::LogSinkType::kStdout;
    log_cfg.level = core::LogLevel::kWarn;

    const std::string kLogSink = common::GetOptionValue(argc, argv, "--log-sink");
    if (!kLogSink.empty()) {
        const auto kParsedSink = core::ParseLogSinkType(kLogSink);
        if (!kParsedSink.has_value()) {
            std::cerr << "Invalid --log-sink value: " << kParsedSink.error().message << "\n";
            return std::unexpected(EXIT_FAILURE);
        }
        log_cfg.sink_type = *kParsedSink;
    }

    const std::string kLogFile = common::GetOptionValue(argc, argv, "--log-file");
    if (!kLogFile.empty()) {
        log_cfg.log_file_path = kLogFile;
        if (kLogSink.empty()) {
            log_cfg.sink_type = core::LogSinkType::kRotatingFile;
        }
    }

    const auto kParsedLevel =
        core::ParseLogLevel(common::GetOptionValue(argc, argv, "--log-level", kDefaultCliLogLevel));
    if (!kParsedLevel.has_value()) {
        std::cerr << "Invalid --log-level value: " << kParsedLevel.error().message << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    log_cfg.level = *kParsedLevel;

    if (const core::Result kValidation = core::ValidateLoggerConfig(log_cfg); !kValidation.IsOk()) {
        std::cerr << "Invalid logger configuration: " << kValidation.message << "\n";
        return std::unexpected(EXIT_FAILURE);
    }

    return log_cfg;
}

[[nodiscard]] std::expected<client::ClientConfig, int> BuildCliClientConfig(
    const CliInvocation& invocation, int argc, char** argv) {
    client::ClientConfig client_cfg;
    const std::string kConfigPath = common::GetOptionValue(argc, argv, "--config");
    if (!kConfigPath.empty()) {
        const auto kLoaded = client::LoadClientConfigFromFile(kConfigPath);
        if (!kLoaded.has_value()) {
            std::cerr << "Failed to load client config '" << kConfigPath
                      << "': " << kLoaded.error().message << "\n";
            return std::unexpected(EXIT_FAILURE);
        }
        client_cfg = *kLoaded;
    }

    client_cfg.ApplyEnvironment();
    client_cfg.client_id = std::string(kCliClientId);
    const auto priority = ParseCliPriority(argc, argv);
    if (!priority.has_value()) {
        std::cerr << priority.error() << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    client_cfg.priority = *priority;
    if (const std::string kRootCaCert = common::GetOptionValue(argc, argv, "--ca-cert");
        !kRootCaCert.empty()) {
        client_cfg.security.root_ca_cert_path = kRootCaCert;
    }
    if (const std::string kClientCert = common::GetOptionValue(argc, argv, "--client-cert");
        !kClientCert.empty()) {
        client_cfg.security.cert_chain_path = kClientCert;
    }
    if (const std::string kClientKey = common::GetOptionValue(argc, argv, "--client-key");
        !kClientKey.empty()) {
        client_cfg.security.private_key_path = kClientKey;
    }
    if (const std::string kServerName = common::GetOptionValue(argc, argv, "--server-name");
        !kServerName.empty()) {
        client_cfg.security.server_authority_override = kServerName;
    }
    if (invocation.has_explicit_address) {
        client_cfg.address = invocation.address;
    } else if (client_cfg.address.empty()) {
        client_cfg.address = std::string(kDefaultAddr);
    }

    if (const core::Result kValidation = client_cfg.Validate(); !kValidation.IsOk()) {
        std::cerr << "Invalid client configuration: " << kValidation.message << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    return client_cfg;
}

}  // namespace swarmkit::apps::cli::internal
