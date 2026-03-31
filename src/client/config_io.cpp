// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <expected>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_set>

#include "config_yaml.h"
#include "env_utils.h"
#include "security_utils.h"
#include "swarmkit/client/swarm_client.h"

namespace swarmkit::client {
namespace {
using core::internal::LooksLikeAddress;
using core::internal::ParsePriorityValue;
using core::internal::ResolveConfigRelativePath;

template <typename T, typename ApplyFn>
[[nodiscard]] core::Result ApplyOptionalScalar(const YAML::Node& node, std::string_view key,
                                               ApplyFn&& apply) {
    const auto kValue = core::yaml::ReadOptionalScalar<T>(node, key);
    if (!kValue.has_value()) {
        return kValue.error();
    }
    if (!kValue->has_value()) {
        return core::Result::Success();
    }

    using ApplyResult = std::invoke_result_t<ApplyFn, const T&>;
    if constexpr (std::is_same_v<ApplyResult, core::Result>) {
        return apply(**kValue);
    }

    apply(**kValue);
    return core::Result::Success();
}

template <typename T>
[[nodiscard]] core::Result AssignOptionalScalar(const YAML::Node& node, std::string_view key,
                                                T* out) {
    if (out == nullptr) {
        return core::Result::Failed("config output field is null");
    }
    return ApplyOptionalScalar<T>(node, key, [out](const T& value) { *out = value; });
}

[[nodiscard]] core::Result RequireMapIfPresent(const YAML::Node& node,
                                               std::string_view field_name) {
    if (!node) {
        return core::Result::Success();
    }
    if (!node.IsMap()) {
        return core::Result::Rejected(std::string(field_name) + " must be a map");
    }
    return core::Result::Success();
}

[[nodiscard]] core::Result ApplyRetryPolicyNode(const YAML::Node& root, RetryPolicy* policy) {
    if (policy == nullptr) {
        return core::Result::Failed("retry policy output is null");
    }

    const YAML::Node retry = root["retry"];
    if (const core::Result kMapCheck = RequireMapIfPresent(retry, "client.retry");
        !kMapCheck.IsOk()) {
        return kMapCheck;
    }
    if (!retry) {
        return core::Result::Success();
    }

    if (const core::Result kResult =
            AssignOptionalScalar<int>(retry, "max_attempts", &policy->max_attempts);
        !kResult.IsOk()) {
        return kResult;
    }
    if (const core::Result kResult =
            AssignOptionalScalar<int>(retry, "initial_backoff_ms", &policy->initial_backoff_ms);
        !kResult.IsOk()) {
        return kResult;
    }
    return AssignOptionalScalar<int>(retry, "max_backoff_ms", &policy->max_backoff_ms);
}

[[nodiscard]] core::Result ApplyStreamReconnectPolicyNode(const YAML::Node& root,
                                                          StreamReconnectPolicy* policy) {
    if (policy == nullptr) {
        return core::Result::Failed("stream reconnect policy output is null");
    }

    const YAML::Node stream_reconnect = root["stream_reconnect"];
    if (const core::Result kMapCheck =
            RequireMapIfPresent(stream_reconnect, "client.stream_reconnect");
        !kMapCheck.IsOk()) {
        return kMapCheck;
    }
    if (!stream_reconnect) {
        return core::Result::Success();
    }

    if (const core::Result kResult =
            AssignOptionalScalar<bool>(stream_reconnect, "enabled", &policy->enabled);
        !kResult.IsOk()) {
        return kResult;
    }
    if (const core::Result kResult = AssignOptionalScalar<int>(
            stream_reconnect, "initial_backoff_ms", &policy->initial_backoff_ms);
        !kResult.IsOk()) {
        return kResult;
    }
    if (const core::Result kResult =
            AssignOptionalScalar<int>(stream_reconnect, "max_backoff_ms", &policy->max_backoff_ms);
        !kResult.IsOk()) {
        return kResult;
    }
    return AssignOptionalScalar<int>(stream_reconnect, "max_attempts", &policy->max_attempts);
}

[[nodiscard]] core::Result ApplySecurityNode(const YAML::Node& root, std::string_view config_path,
                                             ClientSecurityConfig* security) {
    if (security == nullptr) {
        return core::Result::Failed("client security config output is null");
    }

    const YAML::Node security_node = root["security"];
    if (const core::Result kMapCheck = RequireMapIfPresent(security_node, "client.security");
        !kMapCheck.IsOk()) {
        return kMapCheck;
    }
    if (!security_node) {
        return core::Result::Success();
    }

    if (const core::Result kResult = AssignOptionalScalar<std::string>(
            security_node, "root_ca_cert_path", &security->root_ca_cert_path);
        !kResult.IsOk()) {
        return kResult;
    }
    if (const core::Result kResult = AssignOptionalScalar<std::string>(
            security_node, "cert_chain_path", &security->cert_chain_path);
        !kResult.IsOk()) {
        return kResult;
    }
    if (const core::Result kResult = AssignOptionalScalar<std::string>(
            security_node, "private_key_path", &security->private_key_path);
        !kResult.IsOk()) {
        return kResult;
    }
    if (const core::Result kResult = AssignOptionalScalar<std::string>(
            security_node, "server_authority_override", &security->server_authority_override);
        !kResult.IsOk()) {
        return kResult;
    }

    security->root_ca_cert_path =
        ResolveConfigRelativePath(std::string(config_path), security->root_ca_cert_path);
    security->cert_chain_path =
        ResolveConfigRelativePath(std::string(config_path), security->cert_chain_path);
    security->private_key_path =
        ResolveConfigRelativePath(std::string(config_path), security->private_key_path);
    return core::Result::Success();
}

[[nodiscard]] core::Result ApplyClientConfigNode(const YAML::Node& document,
                                                 std::string_view config_path,
                                                 ClientConfig* config) {
    if (config == nullptr) {
        return core::Result::Failed("client config output is null");
    }

    const YAML::Node root = core::yaml::SelectSection(document, "client");
    if (!root || !root.IsMap()) {
        return core::Result::Rejected("client YAML config must be a map");
    }

    if (const core::Result kResult =
            AssignOptionalScalar<std::string>(root, "address", &config->address);
        !kResult.IsOk()) {
        return kResult;
    }
    if (const core::Result kResult =
            AssignOptionalScalar<std::string>(root, "client_id", &config->client_id);
        !kResult.IsOk()) {
        return kResult;
    }
    if (const core::Result kResult =
            AssignOptionalScalar<int>(root, "deadline_ms", &config->deadline_ms);
        !kResult.IsOk()) {
        return kResult;
    }
    if (const core::Result kResult = ApplyOptionalScalar<std::string>(
            root, "priority",
            [config](const std::string& value) {
                const auto kParsedPriority = ParsePriorityValue(value, "priority");
                if (!kParsedPriority.has_value()) {
                    return kParsedPriority.error();
                }
                config->priority = *kParsedPriority;
                return core::Result::Success();
            });
        !kResult.IsOk()) {
        return kResult;
    }
    if (const core::Result kResult = ApplyRetryPolicyNode(root, &config->retry_policy);
        !kResult.IsOk()) {
        return kResult;
    }
    if (const core::Result kResult =
            ApplyStreamReconnectPolicyNode(root, &config->stream_reconnect_policy);
        !kResult.IsOk()) {
        return kResult;
    }
    if (const core::Result kResult = ApplySecurityNode(root, config_path, &config->security);
        !kResult.IsOk()) {
        return kResult;
    }

    return config->Validate();
}

[[nodiscard]] core::Result ApplySwarmDroneNode(const YAML::Node& node,
                                               SwarmDroneConfig* drone_config) {
    if (drone_config == nullptr) {
        return core::Result::Failed("swarm drone config output is null");
    }
    if (!node.IsMap()) {
        return core::Result::Rejected("each swarm.drones entry must be a map");
    }

    if (const core::Result kResult =
            AssignOptionalScalar<std::string>(node, "drone_id", &drone_config->drone_id);
        !kResult.IsOk()) {
        return kResult;
    }
    if (const core::Result kResult =
            AssignOptionalScalar<std::string>(node, "address", &drone_config->address);
        !kResult.IsOk()) {
        return kResult;
    }
    if (const core::Result kResult =
            AssignOptionalScalar<std::string>(node, "local_address", &drone_config->local_address);
        !kResult.IsOk()) {
        return kResult;
    }

    return drone_config->Validate();
}

}  // namespace

core::Result SwarmDroneConfig::Validate() const {
    if (drone_id.empty()) {
        return core::Result::Rejected("swarm drone config drone_id must not be empty");
    }
    if (!LooksLikeAddress(address)) {
        return core::Result::Rejected("swarm drone config address must be in host:port format");
    }
    if (!local_address.empty() && !LooksLikeAddress(local_address)) {
        return core::Result::Rejected(
            "swarm drone config local_address must be in host:port format");
    }
    return core::Result::Success();
}

core::Result SwarmConfig::Validate() const {
    if (const core::Result kValidation = default_client_config.Validate(); !kValidation.IsOk()) {
        return kValidation;
    }
    if (drones.empty()) {
        return core::Result::Rejected("swarm config must contain at least one drone");
    }
    std::unordered_set<std::string> seen_drone_ids;
    for (const auto& drone : drones) {
        if (const core::Result kValidation = drone.Validate(); !kValidation.IsOk()) {
            return kValidation;
        }
        if (!seen_drone_ids.emplace(drone.drone_id).second) {
            return core::Result::Rejected("duplicate swarm drone_id '" + drone.drone_id + "'");
        }
    }
    return core::Result::Success();
}

std::expected<ClientConfig, core::Result> LoadClientConfigFromFile(const std::string& path) {
    const auto loaded_yaml = core::yaml::LoadYamlFile(path);
    if (!loaded_yaml.has_value()) {
        return std::unexpected(loaded_yaml.error());
    }

    ClientConfig config;
    if (const core::Result kResult = ApplyClientConfigNode(*loaded_yaml, path, &config);
        !kResult.IsOk()) {
        return std::unexpected(kResult);
    }
    return config;
}

std::expected<SwarmConfig, core::Result> LoadSwarmConfigFromFile(const std::string& path) {
    const auto loaded_yaml = core::yaml::LoadYamlFile(path);
    if (!loaded_yaml.has_value()) {
        return std::unexpected(loaded_yaml.error());
    }

    SwarmConfig config;
    if (const core::Result kResult =
            ApplyClientConfigNode(*loaded_yaml, path, &config.default_client_config);
        !kResult.IsOk()) {
        return std::unexpected(kResult);
    }

    const YAML::Node swarm_root = core::yaml::SelectSection(*loaded_yaml, "swarm");
    if (!swarm_root || !swarm_root.IsMap()) {
        return std::unexpected(core::Result::Rejected("swarm YAML config must be a map"));
    }

    const auto drones_node = core::yaml::ReadRequiredSequence(swarm_root, "drones");
    if (!drones_node.has_value()) {
        return std::unexpected(drones_node.error());
    }

    config.drones.reserve(drones_node->size());
    for (const auto& drone_node : *drones_node) {
        SwarmDroneConfig drone_config;
        if (const core::Result kResult = ApplySwarmDroneNode(drone_node, &drone_config);
            !kResult.IsOk()) {
            return std::unexpected(kResult);
        }
        config.drones.push_back(std::move(drone_config));
    }

    if (const core::Result kValidation = config.Validate(); !kValidation.IsOk()) {
        return std::unexpected(kValidation);
    }
    return config;
}

}  // namespace swarmkit::client
