// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/agent/backend_factory.h"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <string>
#include <utility>

#include "swarmkit/agent/mavlink_backend.h"
#include "swarmkit/agent/sim_backend.h"

namespace swarmkit::agent {
namespace {

[[nodiscard]] std::string OptionValue(const BackendFactoryRequest& request,
                                      const std::string& key,
                                      const std::string& default_value = {}) {
    const auto iter = request.options.find(key);
    return iter == request.options.end() ? default_value : iter->second;
}

[[nodiscard]] bool BoolOption(const BackendFactoryRequest& request, const std::string& key,
                              bool default_value) {
    const std::string value = OptionValue(request, key, default_value ? "true" : "false");
    return value == "1" || value == "true" || value == "yes" || value == "on";
}

[[nodiscard]] int IntOption(const BackendFactoryRequest& request, const std::string& key,
                            int default_value) {
    try {
        return std::stoi(OptionValue(request, key, std::to_string(default_value)));
    } catch (const std::exception&) {
        return default_value;
    }
}

[[nodiscard]] std::uint8_t ByteOption(const BackendFactoryRequest& request, const std::string& key,
                                      std::uint8_t default_value) {
    const int value = IntOption(request, key, static_cast<int>(default_value));
    if (value <= 0 || value > 255) {
        return default_value;
    }
    return static_cast<std::uint8_t>(value);
}

[[nodiscard]] std::expected<DroneBackendPtr, core::Result> CreateMavlinkBackend(
    const BackendFactoryRequest& request) {
    MavlinkBackendConfig config;
    config.drone_id = OptionValue(request, "drone_id", config.drone_id);
    config.bind_addr = OptionValue(request, "bind_addr", config.bind_addr);
    if (const std::string profile =
            OptionValue(request, "autopilot_profile",
                        std::string(ToString(config.autopilot_profile)));
        !profile.empty()) {
        const auto parsed = ParseMavlinkAutopilotProfile(profile);
        if (!parsed.has_value()) {
            return std::unexpected(parsed.error());
        }
        config.autopilot_profile = *parsed;
    }
    config.target_system = ByteOption(request, "target_system", config.target_system);
    config.target_component = ByteOption(request, "target_component", config.target_component);
    config.source_system = ByteOption(request, "source_system", config.source_system);
    config.source_component = ByteOption(request, "source_component", config.source_component);
    config.telemetry_rate_hz = IntOption(request, "telemetry_rate_hz", config.telemetry_rate_hz);
    config.peer_discovery_timeout_ms =
        IntOption(request, "peer_discovery_timeout_ms", config.peer_discovery_timeout_ms);
    config.command_ack_timeout_ms =
        IntOption(request, "command_ack_timeout_ms", config.command_ack_timeout_ms);
    config.set_guided_before_arm =
        BoolOption(request, "set_guided_before_arm", config.set_guided_before_arm);
    config.set_guided_before_takeoff =
        BoolOption(request, "set_guided_before_takeoff", config.set_guided_before_takeoff);
    config.guided_mode = IntOption(request, "guided_mode", config.guided_mode);
    config.allow_flight_termination =
        BoolOption(request, "allow_flight_termination", config.allow_flight_termination);

    if (const core::Result result = config.Validate(); !result.IsOk()) {
        return std::unexpected(result);
    }
    return MakeMavlinkBackend(std::move(config));
}

}  // namespace

core::Result BackendRegistry::Register(std::string name, BackendCreator creator) {
    if (name.empty()) {
        return core::Result::Rejected("backend factory name must not be empty");
    }
    if (!creator) {
        return core::Result::Rejected("backend factory creator must not be empty");
    }
    creators_[std::move(name)] = std::move(creator);
    return core::Result::Success();
}

std::expected<DroneBackendPtr, core::Result> BackendRegistry::Create(
    const BackendFactoryRequest& request) const {
    const auto iter = creators_.find(request.backend_name);
    if (iter == creators_.end()) {
        return std::unexpected(core::Result::Rejected("unknown backend '" + request.backend_name +
                                                      "'"));
    }
    return iter->second(request);
}

std::vector<std::string> BackendRegistry::Names() const {
    std::vector<std::string> names;
    names.reserve(creators_.size());
    for (const auto& creator_entry : creators_) {
        names.push_back(creator_entry.first);
    }
    std::ranges::sort(names);
    return names;
}

void RegisterBuiltinBackends(BackendRegistry* registry) {
    if (registry == nullptr) {
        return;
    }
    static_cast<void>(registry->Register(
        "sim", [](const BackendFactoryRequest&) -> std::expected<DroneBackendPtr, core::Result> {
            return MakeSimBackend();
        }));
    static_cast<void>(registry->Register("mavlink", CreateMavlinkBackend));
}

}  // namespace swarmkit::agent
