// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "configuration.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include "common/arg_utils.h"

namespace swarmkit::apps::agent::internal {
namespace {

constexpr std::string_view kDefaultBindAddr = "127.0.0.1:50061";
constexpr std::string_view kDefaultAgentId = "agent-1";
constexpr std::string_view kDefaultLogLevel = "info";
constexpr std::string_view kDefaultBackend = "sim";
constexpr int kMaxMavlinkId = 255;

[[nodiscard]] std::expected<std::optional<YAML::Node>, std::string> LoadRootYaml(int argc,
                                                                                 char** argv) {
    const std::string kConfigPath = common::GetOptionValue(argc, argv, "--config");
    if (kConfigPath.empty()) {
        return std::optional<YAML::Node>{};
    }
    try {
        return std::optional<YAML::Node>{YAML::LoadFile(kConfigPath)};
    } catch (const YAML::Exception& ex) {
        return std::unexpected("failed to parse backend config '" + kConfigPath +
                               "': " + ex.what());
    }
}

[[nodiscard]] YAML::Node SelectAgentSection(const YAML::Node& root) {
    if (root && root.IsMap() && root["agent"]) {
        return root["agent"];
    }
    return root;
}

template <typename T>
[[nodiscard]] std::expected<void, std::string> ReadOptionalYamlScalar(const YAML::Node& node,
                                                                      const char* key, T* out) {
    if (out == nullptr) {
        return std::unexpected("internal error while reading YAML scalar '" + std::string(key) +
                               "'");
    }
    if (!node || !node.IsMap() || !node[key]) {
        return {};
    }
    try {
        *out = node[key].as<T>();
    } catch (const YAML::Exception& ex) {
        return std::unexpected("invalid YAML scalar '" + std::string(key) + "': " + ex.what());
    }
    return {};
}

[[nodiscard]] std::expected<void, std::string> ReadOptionalByteYamlScalar(const YAML::Node& node,
                                                                          const char* key,
                                                                          std::uint8_t* out) {
    if (!node || !node.IsMap() || !node[key]) {
        return {};
    }
    int value{};
    if (const auto parsed = ReadOptionalYamlScalar(node, key, &value); !parsed.has_value()) {
        return parsed;
    }
    if (value <= 0 || value > kMaxMavlinkId || out == nullptr) {
        return std::unexpected("MAVLink YAML scalar '" + std::string(key) +
                               "' must be in the range 1..255");
    }
    *out = static_cast<std::uint8_t>(value);
    return {};
}

[[nodiscard]] std::optional<std::uint8_t> ParseByteOption(const std::string& value,
                                                          std::string_view option_name) {
    try {
        std::size_t consumed{};
        const int mavlink_id = std::stoi(value, &consumed);
        if (consumed == value.size() && mavlink_id > 0 && mavlink_id <= kMaxMavlinkId) {
            return static_cast<std::uint8_t>(mavlink_id);
        }
    } catch (const std::exception& exc) {
        swarmkit::core::Logger::ErrorFmt("Invalid {} value '{}': {}", option_name, value,
                                         exc.what());
        return std::nullopt;
    }
    swarmkit::core::Logger::ErrorFmt("Invalid {} value '{}'", option_name, value);
    return std::nullopt;
}

[[nodiscard]] std::optional<bool> ParseBoolOption(const std::string& value,
                                                  std::string_view option_name) {
    std::string lowered = value;
    std::ranges::transform(lowered, lowered.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    if (lowered == "true" || lowered == "1" || lowered == "yes" || lowered == "on") {
        return true;
    }
    if (lowered == "false" || lowered == "0" || lowered == "no" || lowered == "off") {
        return false;
    }
    swarmkit::core::Logger::ErrorFmt("Invalid {} value '{}'", option_name, value);
    return std::nullopt;
}

[[nodiscard]] std::expected<void, int> ApplyMavlinkCliOverrides(
    int argc, char** argv, swarmkit::agent::MavlinkBackendConfig* mavlink) {
    if (mavlink == nullptr) {
        return {};
    }

    if (const std::string kValue = common::GetOptionValue(argc, argv, "--mavlink-bind");
        !kValue.empty()) {
        mavlink->bind_addr = kValue;
    }
    if (const std::string kValue = common::GetOptionValue(argc, argv, "--mavlink-drone");
        !kValue.empty()) {
        mavlink->drone_id = kValue;
    }
    if (const std::string kValue = common::GetOptionValue(argc, argv, "--mavlink-autopilot");
        !kValue.empty()) {
        if (const auto parsed = swarmkit::agent::ParseMavlinkAutopilotProfile(kValue);
            parsed.has_value()) {
            mavlink->autopilot_profile = *parsed;
        } else {
            std::cerr << "Invalid --mavlink-autopilot value: " << parsed.error().message << "\n";
            return std::unexpected(EXIT_FAILURE);
        }
    }
    if (const std::string kValue = common::GetOptionValue(argc, argv, "--mavlink-target-system");
        !kValue.empty()) {
        if (const auto mavlink_id = ParseByteOption(kValue, "--mavlink-target-system");
            mavlink_id.has_value()) {
            mavlink->target_system = *mavlink_id;
        } else {
            return std::unexpected(EXIT_FAILURE);
        }
    }
    if (const std::string kValue = common::GetOptionValue(argc, argv, "--mavlink-target-component");
        !kValue.empty()) {
        if (const auto mavlink_id = ParseByteOption(kValue, "--mavlink-target-component");
            mavlink_id.has_value()) {
            mavlink->target_component = *mavlink_id;
        } else {
            return std::unexpected(EXIT_FAILURE);
        }
    }
    if (const std::string kValue =
            common::GetOptionValue(argc, argv, "--mavlink-set-guided-before-arm");
        !kValue.empty()) {
        if (const auto parsed = ParseBoolOption(kValue, "--mavlink-set-guided-before-arm");
            parsed.has_value()) {
            mavlink->set_guided_before_arm = *parsed;
        } else {
            return std::unexpected(EXIT_FAILURE);
        }
    }
    if (const std::string kValue =
            common::GetOptionValue(argc, argv, "--mavlink-set-guided-before-takeoff");
        !kValue.empty()) {
        if (const auto parsed = ParseBoolOption(kValue, "--mavlink-set-guided-before-takeoff");
            parsed.has_value()) {
            mavlink->set_guided_before_takeoff = *parsed;
        } else {
            return std::unexpected(EXIT_FAILURE);
        }
    }
    return {};
}

void PutOption(std::unordered_map<std::string, std::string>* options, std::string key,
               std::string value) {
    if (options == nullptr) {
        return;
    }
    (*options)[std::move(key)] = std::move(value);
}

void PopulateMavlinkFactoryOptions(const swarmkit::agent::MavlinkBackendConfig& mavlink,
                                   std::unordered_map<std::string, std::string>* options) {
    PutOption(options, "drone_id", mavlink.drone_id);
    PutOption(options, "bind_addr", mavlink.bind_addr);
    PutOption(options, "autopilot_profile", std::string(ToString(mavlink.autopilot_profile)));
    PutOption(options, "target_system", std::to_string(mavlink.target_system));
    PutOption(options, "target_component", std::to_string(mavlink.target_component));
    PutOption(options, "source_system", std::to_string(mavlink.source_system));
    PutOption(options, "source_component", std::to_string(mavlink.source_component));
    PutOption(options, "telemetry_rate_hz", std::to_string(mavlink.telemetry_rate_hz));
    PutOption(options, "peer_discovery_timeout_ms",
              std::to_string(mavlink.peer_discovery_timeout_ms));
    PutOption(options, "command_ack_timeout_ms", std::to_string(mavlink.command_ack_timeout_ms));
    PutOption(options, "set_guided_before_arm", mavlink.set_guided_before_arm ? "true" : "false");
    PutOption(options, "set_guided_before_takeoff",
              mavlink.set_guided_before_takeoff ? "true" : "false");
    PutOption(options, "guided_mode", std::to_string(mavlink.guided_mode));
    PutOption(options, "allow_flight_termination",
              mavlink.allow_flight_termination ? "true" : "false");
}

[[nodiscard]] std::expected<void, std::string> ReadBackendOptionsYaml(
    const YAML::Node& node, std::unordered_map<std::string, std::string>* options) {
    if (!node) {
        return {};
    }
    if (options == nullptr || !node.IsMap()) {
        return std::unexpected("agent.backend_options must be a map of scalar values");
    }
    for (const auto& entry : node) {
        try {
            (*options)[entry.first.as<std::string>()] = entry.second.as<std::string>();
        } catch (const YAML::Exception& ex) {
            return std::unexpected("invalid agent.backend_options entry: " +
                                   std::string(ex.what()));
        }
    }
    return {};
}

[[nodiscard]] std::expected<void, std::string> ReadMavlinkYaml(
    const YAML::Node& node, swarmkit::agent::MavlinkBackendConfig* config) {
    if (!node) {
        return {};
    }
    if (!node.IsMap() || config == nullptr) {
        return std::unexpected("agent.mavlink must be a map");
    }

    constexpr std::array<std::string_view, 14> kAllowedKeys{
        "drone_id",
        "bind_addr",
        "autopilot_profile",
        "target_system",
        "target_component",
        "source_system",
        "source_component",
        "telemetry_rate_hz",
        "peer_discovery_timeout_ms",
        "command_ack_timeout_ms",
        "set_guided_before_arm",
        "set_guided_before_takeoff",
        "guided_mode",
        "allow_flight_termination",
    };
    for (const auto& entry : node) {
        if (!entry.first.IsScalar()) {
            return std::unexpected("agent.mavlink keys must be scalar strings");
        }
        const std::string key = entry.first.as<std::string>();
        if (!std::ranges::contains(kAllowedKeys, key)) {
            return std::unexpected("unknown agent.mavlink option '" + key + "'");
        }
    }

    const auto read = [&](const auto& result) -> std::expected<void, std::string> {
        if (!result.has_value()) {
            return std::unexpected(result.error());
        }
        return {};
    };
    if (auto result = read(ReadOptionalYamlScalar(node, "drone_id", &config->drone_id)); !result) {
        return result;
    }
    if (auto result = read(ReadOptionalYamlScalar(node, "bind_addr", &config->bind_addr));
        !result) {
        return result;
    }
    if (std::string autopilot_profile; node["autopilot_profile"]) {
        if (auto result =
                read(ReadOptionalYamlScalar(node, "autopilot_profile", &autopilot_profile));
            !result) {
            return result;
        }
        const auto parsed = swarmkit::agent::ParseMavlinkAutopilotProfile(autopilot_profile);
        if (!parsed.has_value()) {
            return std::unexpected("invalid agent.mavlink.autopilot_profile: " +
                                   parsed.error().message);
        }
        config->autopilot_profile = *parsed;
    }
    if (auto result =
            read(ReadOptionalByteYamlScalar(node, "target_system", &config->target_system));
        !result) {
        return result;
    }
    if (auto result =
            read(ReadOptionalByteYamlScalar(node, "target_component", &config->target_component));
        !result) {
        return result;
    }
    if (auto result =
            read(ReadOptionalByteYamlScalar(node, "source_system", &config->source_system));
        !result) {
        return result;
    }
    if (auto result =
            read(ReadOptionalByteYamlScalar(node, "source_component", &config->source_component));
        !result) {
        return result;
    }
    if (auto result =
            read(ReadOptionalYamlScalar(node, "telemetry_rate_hz", &config->telemetry_rate_hz));
        !result) {
        return result;
    }
    if (auto result = read(ReadOptionalYamlScalar(node, "peer_discovery_timeout_ms",
                                                  &config->peer_discovery_timeout_ms));
        !result) {
        return result;
    }
    if (auto result = read(ReadOptionalYamlScalar(node, "command_ack_timeout_ms",
                                                  &config->command_ack_timeout_ms));
        !result) {
        return result;
    }
    if (auto result = read(
            ReadOptionalYamlScalar(node, "set_guided_before_arm", &config->set_guided_before_arm));
        !result) {
        return result;
    }
    if (auto result = read(ReadOptionalYamlScalar(node, "set_guided_before_takeoff",
                                                  &config->set_guided_before_takeoff));
        !result) {
        return result;
    }
    if (auto result = read(ReadOptionalYamlScalar(node, "guided_mode", &config->guided_mode));
        !result) {
        return result;
    }
    if (auto result = read(ReadOptionalYamlScalar(node, "allow_flight_termination",
                                                  &config->allow_flight_termination));
        !result) {
        return result;
    }
    return {};
}

}  // namespace

[[nodiscard]] std::expected<core::LoggerConfig, int> BuildAgentLoggerConfig(int argc, char** argv) {
    core::LoggerConfig logger_cfg;
    logger_cfg.sink_type = core::LogSinkType::kStdout;
    logger_cfg.level = core::LogLevel::kInfo;
    logger_cfg.flush_level = core::LogLevel::kInfo;

    const std::string kLogSink = common::GetOptionValue(argc, argv, "--log-sink");
    if (!kLogSink.empty()) {
        const auto kParsedSink = core::ParseLogSinkType(kLogSink);
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
            logger_cfg.sink_type = core::LogSinkType::kRotatingFile;
        }
    }

    const auto kParsedLevel =
        core::ParseLogLevel(common::GetOptionValue(argc, argv, "--log-level", kDefaultLogLevel));
    if (!kParsedLevel.has_value()) {
        std::cerr << "Invalid --log-level value: " << kParsedLevel.error().message << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    logger_cfg.level = *kParsedLevel;

    if (const core::Result kValidation = core::ValidateLoggerConfig(logger_cfg);
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
    if (const std::string evidence_file = common::GetOptionValue(argc, argv, "--evidence-file");
        !evidence_file.empty()) {
        agent_cfg.execution_recorder.file_path = evidence_file;
    }
    if (const std::string run_id = common::GetOptionValue(argc, argv, "--evidence-run-id");
        !run_id.empty()) {
        agent_cfg.execution_recorder.run_id = run_id;
    }
    if (const std::string scenario_id =
            common::GetOptionValue(argc, argv, "--evidence-scenario-id");
        !scenario_id.empty()) {
        agent_cfg.execution_recorder.scenario_id = scenario_id;
    }
    if (const std::string seed = common::GetOptionValue(argc, argv, "--evidence-seed");
        !seed.empty()) {
        try {
            agent_cfg.execution_recorder.random_seed = std::stoull(seed);
        } catch (const std::exception&) {
            std::cerr << "--evidence-seed must be an unsigned integer\n";
            return std::unexpected(EXIT_FAILURE);
        }
    }
    if (common::HasFlag(argc, argv, "--evidence-overwrite")) {
        agent_cfg.execution_recorder.overwrite_existing = true;
    }
    if (const std::string artifact_dir = common::GetOptionValue(argc, argv, "--artifact-dir");
        !artifact_dir.empty()) {
        agent_cfg.data.artifact_dir = artifact_dir;
    }
    if (common::HasFlag(argc, argv, "--allow-unsafe-bench-commands")) {
        agent_cfg.safety.allow_unsafe_bench_commands = true;
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
    if (common::HasFlag(argc, argv, "--insecure")) {
        agent_cfg.security.transport_security = core::TransportSecurityMode::kInsecure;
    }
    if (const std::string transport_security =
            common::GetOptionValue(argc, argv, "--transport-security");
        !transport_security.empty()) {
        const auto parsed = core::ParseTransportSecurityMode(transport_security);
        if (!parsed.has_value()) {
            std::cerr << parsed.error() << "\n";
            return std::unexpected(EXIT_FAILURE);
        }
        agent_cfg.security.transport_security = *parsed;
    }

    if (agent_cfg.agent_id.empty()) {
        agent_cfg.agent_id = std::string(kDefaultAgentId);
    }
    if (agent_cfg.bind_addr.empty()) {
        agent_cfg.bind_addr = std::string(kDefaultBindAddr);
    }

    if (const core::Result kValidation = agent_cfg.Validate(); !kValidation.IsOk()) {
        swarmkit::core::Logger::ErrorFmt("Invalid agent configuration: {}", kValidation.message);
        return std::unexpected(EXIT_FAILURE);
    }

    return agent_cfg;
}

[[nodiscard]] std::expected<BackendSelection, int> BuildBackendSelection(int argc, char** argv) {
    BackendSelection selection;
    selection.request.backend_name = std::string(kDefaultBackend);

    const auto root = LoadRootYaml(argc, argv);
    if (!root.has_value()) {
        std::cerr << "Invalid backend configuration: " << root.error() << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    if (root->has_value()) {
        const YAML::Node agent = SelectAgentSection(**root);
        if (const auto backend =
                ReadOptionalYamlScalar(agent, "backend", &selection.request.backend_name);
            !backend.has_value()) {
            std::cerr << "Invalid backend configuration: " << backend.error() << "\n";
            return std::unexpected(EXIT_FAILURE);
        }
        if (const auto options =
                ReadBackendOptionsYaml(agent["backend_options"], &selection.request.options);
            !options.has_value()) {
            std::cerr << "Invalid backend configuration: " << options.error() << "\n";
            return std::unexpected(EXIT_FAILURE);
        }

        const YAML::Node mavlink = agent["mavlink"] ? agent["mavlink"] : (**root)["mavlink"];
        if (const auto parsed = ReadMavlinkYaml(mavlink, &selection.mavlink); !parsed.has_value()) {
            std::cerr << "Invalid MAVLink backend configuration: " << parsed.error() << "\n";
            return std::unexpected(EXIT_FAILURE);
        }
    }

    if (const std::string kBackend = common::GetOptionValue(argc, argv, "--backend");
        !kBackend.empty()) {
        selection.request.backend_name = kBackend;
    }
    if (const auto cli_mavlink_overrides = ApplyMavlinkCliOverrides(argc, argv, &selection.mavlink);
        !cli_mavlink_overrides.has_value()) {
        return std::unexpected(cli_mavlink_overrides.error());
    }

    if (selection.request.backend_name.empty()) {
        std::cerr << "Invalid --backend value: backend name must not be empty\n";
        return std::unexpected(EXIT_FAILURE);
    }
    if (selection.request.backend_name == "mavlink") {
        if (const core::Result kValidation = selection.mavlink.Validate(); !kValidation.IsOk()) {
            std::cerr << "Invalid MAVLink backend configuration: " << kValidation.message << "\n";
            return std::unexpected(EXIT_FAILURE);
        }
        PopulateMavlinkFactoryOptions(selection.mavlink, &selection.request.options);
    }

    return selection;
}

}  // namespace swarmkit::apps::agent::internal
