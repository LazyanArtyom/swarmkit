// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "app.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>

#include "common/arg_utils.h"
#include "swarmkit/agent/server.h"
#include "swarmkit/agent/sim_backend.h"
#include "swarmkit/core/logger.h"

namespace swarmkit::apps::agent {
namespace {

constexpr std::string_view kDefaultBindAddr = "0.0.0.0:50061";
constexpr std::string_view kDefaultAgentId = "agent-1";
constexpr std::string_view kDefaultLogLevel = "info";

void PrintUsage() {
    std::cout << "Usage: swarmkit-agent [OPTIONS]\n"
                 "\n"
                 "Options:\n"
                 "  --id        AGENT_ID      Agent identifier (default: agent-1)\n"
                 "  --bind      HOST:PORT     Bind address    (default: 0.0.0.0:50061)\n"
                 "  --config    PATH          Load agent config from YAML file\n"
                 "  --log-sink  stdout|file|both\n"
                 "                             Logger sink type (default: stdout)\n"
                 "  --log-file  PATH          Rotating log file path when file logging is used\n"
                 "  --log-level LEVEL         trace|debug|info|warn|error|critical|off\n"
                 "  --help                    Print this message\n";
}

[[nodiscard]] std::expected<swarmkit::core::LoggerConfig, int> BuildAgentLoggerConfig(int argc,
                                                                                      char** argv) {
    swarmkit::core::LoggerConfig logger_cfg;
    logger_cfg.sink_type = swarmkit::core::LogSinkType::kStdout;
    logger_cfg.level = swarmkit::core::LogLevel::kInfo;
    logger_cfg.flush_level = swarmkit::core::LogLevel::kInfo;

    const std::string kLogSink = common::GetOptionValue(argc, argv, "--log-sink");
    if (!kLogSink.empty()) {
        const auto kParsedSink = swarmkit::core::ParseLogSinkType(kLogSink);
        if (!kParsedSink.has_value()) {
            std::cerr << "Invalid --log-sink value: " << kParsedSink.error().message << "\n";
            return std::unexpected(EXIT_FAILURE);
        }
        logger_cfg.sink_type = *kParsedSink;
    }

    const std::string kLogFile = common::GetOptionValue(argc, argv, "--log-file");
    if (!kLogFile.empty()) {
        logger_cfg.log_file_path = kLogFile;
        if (kLogSink.empty()) {
            logger_cfg.sink_type = swarmkit::core::LogSinkType::kRotatingFile;
        }
    }

    const auto kParsedLevel = swarmkit::core::ParseLogLevel(
        common::GetOptionValue(argc, argv, "--log-level", kDefaultLogLevel));
    if (!kParsedLevel.has_value()) {
        std::cerr << "Invalid --log-level value: " << kParsedLevel.error().message << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    logger_cfg.level = *kParsedLevel;

    if (const swarmkit::core::Result kValidation = swarmkit::core::ValidateLoggerConfig(logger_cfg);
        !kValidation.IsOk()) {
        std::cerr << "Invalid logger configuration: " << kValidation.message << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    return logger_cfg;
}

[[nodiscard]] std::expected<swarmkit::agent::AgentConfig, int> BuildAgentConfig(int argc,
                                                                                char** argv) {
    swarmkit::agent::AgentConfig agent_cfg;
    const std::string kConfigPath = common::GetOptionValue(argc, argv, "--config");
    if (!kConfigPath.empty()) {
        const auto kLoaded = swarmkit::agent::LoadAgentConfigFromFile(kConfigPath);
        if (!kLoaded.has_value()) {
            swarmkit::core::Logger::ErrorFmt("Failed to load agent config '{}': {}", kConfigPath,
                                             kLoaded.error().message);
            return std::unexpected(EXIT_FAILURE);
        }
        agent_cfg = *kLoaded;
    }

    agent_cfg.ApplyEnvironment();

    const std::string kAgentId = common::GetOptionValue(argc, argv, "--id");
    if (!kAgentId.empty()) {
        agent_cfg.agent_id = kAgentId;
    }

    const std::string kBindAddr = common::GetOptionValue(argc, argv, "--bind");
    if (!kBindAddr.empty()) {
        agent_cfg.bind_addr = kBindAddr;
    }

    if (agent_cfg.agent_id.empty()) {
        agent_cfg.agent_id = std::string(kDefaultAgentId);
    }
    if (agent_cfg.bind_addr.empty()) {
        agent_cfg.bind_addr = std::string(kDefaultBindAddr);
    }

    if (const swarmkit::core::Result kValidation = agent_cfg.Validate(); !kValidation.IsOk()) {
        swarmkit::core::Logger::ErrorFmt("Invalid agent configuration: {}", kValidation.message);
        return std::unexpected(EXIT_FAILURE);
    }

    return agent_cfg;
}

}  // namespace

int RunAgentApp(int argc, char** argv) {
    if (common::HasFlag(argc, argv, "--help")) {
        PrintUsage();
        return EXIT_SUCCESS;
    }

    const auto kLoggerConfig = BuildAgentLoggerConfig(argc, argv);
    if (!kLoggerConfig.has_value()) {
        return kLoggerConfig.error();
    }

    swarmkit::core::Logger::Init(*kLoggerConfig);
    swarmkit::core::Logger::InfoFmt("Logger initialized: sink={} level={} file={}",
                                    swarmkit::core::ToString(kLoggerConfig->sink_type),
                                    swarmkit::core::ToString(kLoggerConfig->level),
                                    kLoggerConfig->log_file_path);

    const auto kAgentConfig = BuildAgentConfig(argc, argv);
    if (!kAgentConfig.has_value()) {
        return kAgentConfig.error();
    }

    return swarmkit::agent::RunAgentServer(*kAgentConfig, swarmkit::agent::MakeSimBackend());
}

}  // namespace swarmkit::apps::agent
