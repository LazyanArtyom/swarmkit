// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/agent/backend_factory.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "swarmkit/agent/mavlink_backend.h"
#include "swarmkit/agent/sim_backend.h"

namespace swarmkit::agent {
namespace {

[[nodiscard]] std::string OptionValue(const BackendFactoryRequest& request, const std::string& key,
                                      const std::string& default_value = {}) {
    const auto iter = request.options.find(key);
    return iter == request.options.end() ? default_value : iter->second;
}

[[nodiscard]] bool BoolOption(const BackendFactoryRequest& request, const std::string& key,
                              bool default_value) {
    const std::string value = OptionValue(request, key, default_value ? "true" : "false");
    return value == "1" || value == "true" || value == "yes" || value == "on";
}

[[nodiscard]] std::expected<double, core::Result> StrictDoubleOption(
    const BackendFactoryRequest& request, const std::string& key, double default_value) {
    const auto iter = request.options.find(key);
    if (iter == request.options.end()) {
        return default_value;
    }
    try {
        std::size_t consumed = 0;
        const double value = std::stod(iter->second, &consumed);
        if (consumed != iter->second.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return value;
    } catch (const std::exception&) {
        return std::unexpected(core::Result::Rejected("invalid sim backend option '" + key + "'"));
    }
}

[[nodiscard]] std::expected<std::int64_t, core::Result> StrictInt64Option(
    const BackendFactoryRequest& request, const std::string& key, std::int64_t default_value) {
    const auto iter = request.options.find(key);
    if (iter == request.options.end()) {
        return default_value;
    }
    try {
        std::size_t consumed = 0;
        const std::int64_t value = std::stoll(iter->second, &consumed);
        if (consumed != iter->second.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return value;
    } catch (const std::exception&) {
        return std::unexpected(core::Result::Rejected("invalid sim backend option '" + key + "'"));
    }
}

[[nodiscard]] std::expected<bool, core::Result> StrictBoolOption(
    const BackendFactoryRequest& request, const std::string& key, bool default_value) {
    const auto iter = request.options.find(key);
    if (iter == request.options.end()) {
        return default_value;
    }
    if (iter->second == "1" || iter->second == "true" || iter->second == "yes" ||
        iter->second == "on") {
        return true;
    }
    if (iter->second == "0" || iter->second == "false" || iter->second == "no" ||
        iter->second == "off") {
        return false;
    }
    return std::unexpected(core::Result::Rejected("invalid sim backend option '" + key + "'"));
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
    if (const std::string profile = OptionValue(request, "autopilot_profile",
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

[[nodiscard]] std::expected<DroneBackendPtr, core::Result> CreateSimBackend(
    const BackendFactoryRequest& request) {
    constexpr std::array<std::string_view, 13> kAllowedOptions{
        "integration_step_ms",
        "initial_source_unix_time_ms",
        "home_lat_deg",
        "home_lon_deg",
        "home_alt_m",
        "initial_battery_percent",
        "max_horizontal_speed_mps",
        "max_climb_speed_mps",
        "max_descent_speed_mps",
        "max_altitude_m",
        "default_cruise_speed_mps",
        "battery_drain_percent_per_second",
        "stop_at_target",
    };
    for (const auto& [key, value] : request.options) {
        static_cast<void>(value);
        if (!std::ranges::contains(kAllowedOptions, key)) {
            return std::unexpected(
                core::Result::Rejected("unknown sim backend option '" + key + "'"));
        }
    }
    SimBackendConfig config;
    const auto integration_step =
        StrictInt64Option(request, "integration_step_ms", config.integration_step_ms);
    const auto initial_time = StrictInt64Option(request, "initial_source_unix_time_ms",
                                                config.initial_source_unix_time_ms);
    const auto home_lat = StrictDoubleOption(request, "home_lat_deg", config.home_lat_deg);
    const auto home_lon = StrictDoubleOption(request, "home_lon_deg", config.home_lon_deg);
    const auto home_alt = StrictDoubleOption(request, "home_alt_m", config.home_alt_m);
    const auto initial_battery =
        StrictDoubleOption(request, "initial_battery_percent", config.initial_battery_percent);
    const auto max_horizontal =
        StrictDoubleOption(request, "max_horizontal_speed_mps", config.max_horizontal_speed_mps);
    const auto max_climb =
        StrictDoubleOption(request, "max_climb_speed_mps", config.max_climb_speed_mps);
    const auto max_descent =
        StrictDoubleOption(request, "max_descent_speed_mps", config.max_descent_speed_mps);
    const auto max_altitude = StrictDoubleOption(request, "max_altitude_m", config.max_altitude_m);
    const auto cruise =
        StrictDoubleOption(request, "default_cruise_speed_mps", config.default_cruise_speed_mps);
    const auto drain = StrictDoubleOption(request, "battery_drain_percent_per_second",
                                          config.battery_drain_percent_per_second);
    const auto stop_at_target = StrictBoolOption(request, "stop_at_target", config.stop_at_target);
    if (!integration_step || !initial_time || !home_lat || !home_lon || !home_alt ||
        !initial_battery || !max_horizontal || !max_climb || !max_descent || !max_altitude ||
        !cruise || !drain || !stop_at_target) {
        const auto first_error = [&](const auto& value) -> std::optional<core::Result> {
            return value.has_value() ? std::nullopt : std::optional<core::Result>{value.error()};
        };
        if (auto error = first_error(integration_step)) {
            return std::unexpected(*error);
        }
        if (auto error = first_error(initial_time)) {
            return std::unexpected(*error);
        }
        if (auto error = first_error(home_lat)) {
            return std::unexpected(*error);
        }
        if (auto error = first_error(home_lon)) {
            return std::unexpected(*error);
        }
        if (auto error = first_error(home_alt)) {
            return std::unexpected(*error);
        }
        if (auto error = first_error(initial_battery)) {
            return std::unexpected(*error);
        }
        if (auto error = first_error(max_horizontal)) {
            return std::unexpected(*error);
        }
        if (auto error = first_error(max_climb)) {
            return std::unexpected(*error);
        }
        if (auto error = first_error(max_descent)) {
            return std::unexpected(*error);
        }
        if (auto error = first_error(max_altitude)) {
            return std::unexpected(*error);
        }
        if (auto error = first_error(cruise)) {
            return std::unexpected(*error);
        }
        if (auto error = first_error(drain)) {
            return std::unexpected(*error);
        }
        return std::unexpected(stop_at_target.error());
    }
    if (*integration_step < std::numeric_limits<int>::min() ||
        *integration_step > std::numeric_limits<int>::max()) {
        return std::unexpected(
            core::Result::Rejected("sim integration_step_ms is outside the integer range"));
    }
    config.integration_step_ms = static_cast<int>(*integration_step);
    config.initial_source_unix_time_ms = *initial_time;
    config.home_lat_deg = *home_lat;
    config.home_lon_deg = *home_lon;
    config.home_alt_m = *home_alt;
    config.initial_battery_percent = static_cast<float>(*initial_battery);
    config.max_horizontal_speed_mps = static_cast<float>(*max_horizontal);
    config.max_climb_speed_mps = static_cast<float>(*max_climb);
    config.max_descent_speed_mps = static_cast<float>(*max_descent);
    config.max_altitude_m = static_cast<float>(*max_altitude);
    config.default_cruise_speed_mps = static_cast<float>(*cruise);
    config.battery_drain_percent_per_second = static_cast<float>(*drain);
    config.stop_at_target = *stop_at_target;
    auto simulator = MakeSimBackend(std::move(config));
    if (!simulator.has_value()) {
        return std::unexpected(simulator.error());
    }
    return std::move(simulator->backend);
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
        return std::unexpected(
            core::Result::Rejected("unknown backend '" + request.backend_name + "'"));
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
    static_cast<void>(registry->Register("sim", CreateSimBackend));
    static_cast<void>(registry->Register("mavlink", CreateMavlinkBackend));
}

}  // namespace swarmkit::agent
