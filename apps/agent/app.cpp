// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "app.h"

#include <cstdlib>
#include <cstdint>
#include <exception>
#include <expected>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

#include <yaml-cpp/yaml.h>

#include "common/arg_utils.h"
#include "swarmkit/agent/mavlink_backend.h"
#include "swarmkit/agent/server.h"
#include "swarmkit/agent/sim_backend.h"
#include "swarmkit/core/logger.h"

namespace swarmkit::apps::agent {
namespace {

constexpr std::string_view kDefaultBindAddr = "0.0.0.0:50061";
constexpr std::string_view kDefaultAgentId = "agent-1";
constexpr std::string_view kDefaultLogLevel = "info";
constexpr std::string_view kDefaultBackend = "sim";

struct BackendSelection {
    std::string backend{std::string(kDefaultBackend)};
    swarmkit::agent::MavlinkBackendConfig mavlink;
};

void PrintUsage() {
    std::cout << "Usage: swarmkit-agent [OPTIONS]\n"
                 "\n"
                 "Options:\n"
                 "  --id        AGENT_ID      Agent identifier (default: agent-1)\n"
                 "  --bind      HOST:PORT     Bind address    (default: 0.0.0.0:50061)\n"
                 "  --config    PATH          Load agent config from YAML file\n"
                 "  --backend   sim|mavlink   Vehicle backend (default: sim)\n"
                 "  --mavlink-bind HOST:PORT  UDP MAVLink listen address\n"
                 "  --mavlink-drone ID        SwarmKit drone id for MAVLink backend\n"
                 "  --mavlink-target-system N MAVLink target system id\n"
                 "  --mavlink-target-component N\n"
                 "                             MAVLink target component id\n"
                 "  --ca-cert   PATH          mTLS CA certificate path\n"
                 "  --server-cert PATH        mTLS server certificate path\n"
                 "  --server-key PATH         mTLS server private key path\n"
                 "  --log-sink  stdout|file|both\n"
                 "                             Logger sink type (default: stdout)\n"
                 "  --log-file  PATH          Rotating log file path when file logging is used\n"
                 "  --log-level LEVEL         trace|debug|info|warn|error|critical|off\n"
                 "  --help                    Print this message\n";
}

[[nodiscard]] std::optional<YAML::Node> LoadRootYaml(int argc, char** argv) {
    const std::string kConfigPath = common::GetOptionValue(argc, argv, "--config");
    if (kConfigPath.empty()) {
        return std::nullopt;
    }
    try {
        return YAML::LoadFile(kConfigPath);
    } catch (const YAML::Exception& ex) {
        swarmkit::core::Logger::WarnFmt("Failed to parse backend config '{}': {}", kConfigPath,
                                        ex.what());
        return std::nullopt;
    }
}

[[nodiscard]] YAML::Node SelectAgentSection(const YAML::Node& root) {
    if (root && root.IsMap() && root["agent"]) {
        return root["agent"];
    }
    return root;
}

template <typename T>
void ReadOptionalYamlScalar(const YAML::Node& node, const char* key, T* out) {
    if (out == nullptr || !node || !node.IsMap() || !node[key]) {
        return;
    }
    try {
        *out = node[key].as<T>();
    } catch (const YAML::Exception& ex) {
        swarmkit::core::Logger::WarnFmt("Ignoring invalid YAML scalar '{}': {}", key, ex.what());
    }
}

void ReadOptionalByteYamlScalar(const YAML::Node& node, const char* key, std::uint8_t* out) {
    int value{};
    ReadOptionalYamlScalar(node, key, &value);
    if (value > 0 && value <= 255 && out != nullptr) {
        *out = static_cast<std::uint8_t>(value);
    }
}

[[nodiscard]] std::optional<std::uint8_t> ParseByteOption(const std::string& value,
                                                          std::string_view option_name) {
    try {
        const int id = std::stoi(value);
        if (id > 0 && id <= 255) {
            return static_cast<std::uint8_t>(id);
        }
    } catch (const std::exception&) {
    }
    swarmkit::core::Logger::WarnFmt("Ignoring invalid {} value '{}'", option_name, value);
    return std::nullopt;
}

void ApplyMavlinkCliOverrides(int argc, char** argv,
                              swarmkit::agent::MavlinkBackendConfig* mavlink) {
    if (mavlink == nullptr) {
        return;
    }

    if (const std::string kValue = common::GetOptionValue(argc, argv, "--mavlink-bind");
        !kValue.empty()) {
        mavlink->bind_addr = kValue;
    }
    if (const std::string kValue = common::GetOptionValue(argc, argv, "--mavlink-drone");
        !kValue.empty()) {
        mavlink->drone_id = kValue;
    }
    if (const std::string kValue =
            common::GetOptionValue(argc, argv, "--mavlink-target-system");
        !kValue.empty()) {
        if (const auto id = ParseByteOption(kValue, "--mavlink-target-system"); id.has_value()) {
            mavlink->target_system = *id;
        }
    }
    if (const std::string kValue =
            common::GetOptionValue(argc, argv, "--mavlink-target-component");
        !kValue.empty()) {
        if (const auto id = ParseByteOption(kValue, "--mavlink-target-component");
            id.has_value()) {
            mavlink->target_component = *id;
        }
    }
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
    if (const std::string kRootCaCert = common::GetOptionValue(argc, argv, "--ca-cert");
        !kRootCaCert.empty()) {
        agent_cfg.security.root_ca_cert_path = kRootCaCert;
    }
    if (const std::string kServerCert = common::GetOptionValue(argc, argv, "--server-cert");
        !kServerCert.empty()) {
        agent_cfg.security.cert_chain_path = kServerCert;
    }
    if (const std::string kServerKey = common::GetOptionValue(argc, argv, "--server-key");
        !kServerKey.empty()) {
        agent_cfg.security.private_key_path = kServerKey;
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

[[nodiscard]] std::expected<BackendSelection, int> BuildBackendSelection(int argc, char** argv) {
    BackendSelection selection;

    if (const auto root = LoadRootYaml(argc, argv); root.has_value()) {
        const YAML::Node agent = SelectAgentSection(*root);
        ReadOptionalYamlScalar(agent, "backend", &selection.backend);

        const YAML::Node mavlink = agent["mavlink"] ? agent["mavlink"] : (*root)["mavlink"];
        if (mavlink) {
            ReadOptionalYamlScalar(mavlink, "drone_id", &selection.mavlink.drone_id);
            ReadOptionalYamlScalar(mavlink, "bind_addr", &selection.mavlink.bind_addr);
            ReadOptionalByteYamlScalar(mavlink, "target_system",
                                       &selection.mavlink.target_system);
            ReadOptionalByteYamlScalar(mavlink, "target_component",
                                       &selection.mavlink.target_component);
            ReadOptionalByteYamlScalar(mavlink, "source_system",
                                       &selection.mavlink.source_system);
            ReadOptionalByteYamlScalar(mavlink, "source_component",
                                       &selection.mavlink.source_component);
            ReadOptionalYamlScalar(mavlink, "telemetry_rate_hz",
                                   &selection.mavlink.telemetry_rate_hz);
            ReadOptionalYamlScalar(mavlink, "peer_discovery_timeout_ms",
                                   &selection.mavlink.peer_discovery_timeout_ms);
            ReadOptionalYamlScalar(mavlink, "command_ack_timeout_ms",
                                   &selection.mavlink.command_ack_timeout_ms);
            ReadOptionalYamlScalar(mavlink, "set_guided_before_arm",
                                   &selection.mavlink.set_guided_before_arm);
            ReadOptionalYamlScalar(mavlink, "set_guided_before_takeoff",
                                   &selection.mavlink.set_guided_before_takeoff);
            ReadOptionalYamlScalar(mavlink, "guided_mode", &selection.mavlink.guided_mode);
            ReadOptionalYamlScalar(mavlink, "allow_flight_termination",
                                   &selection.mavlink.allow_flight_termination);
        }
    }

    if (const std::string kBackend = common::GetOptionValue(argc, argv, "--backend");
        !kBackend.empty()) {
        selection.backend = kBackend;
    }
    ApplyMavlinkCliOverrides(argc, argv, &selection.mavlink);

    if (selection.backend != "sim" && selection.backend != "mavlink") {
        std::cerr << "Invalid --backend value: " << selection.backend << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    if (selection.backend == "mavlink") {
        if (const swarmkit::core::Result kValidation = selection.mavlink.Validate();
            !kValidation.IsOk()) {
            std::cerr << "Invalid MAVLink backend configuration: " << kValidation.message << "\n";
            return std::unexpected(EXIT_FAILURE);
        }
    }

    return selection;
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

    const auto kBackendSelection = BuildBackendSelection(argc, argv);
    if (!kBackendSelection.has_value()) {
        return kBackendSelection.error();
    }

    swarmkit::agent::DroneBackendPtr backend;
    if (kBackendSelection->backend == "mavlink") {
        backend = swarmkit::agent::MakeMavlinkBackend(kBackendSelection->mavlink);
    } else {
        backend = swarmkit::agent::MakeSimBackend();
    }

    return swarmkit::agent::RunAgentServer(*kAgentConfig, std::move(backend));
}

}  // namespace swarmkit::apps::agent
