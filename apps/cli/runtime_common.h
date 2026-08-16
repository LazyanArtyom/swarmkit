// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <expected>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include "command_builder.h"
#include "common/arg_utils.h"
#include "constants.h"
#include "options.h"
#include "output.h"
#include "swarmkit/client/swarm_client.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/telemetry.h"
#include "telemetry_sink.h"

namespace swarmkit::apps::cli::internal {

using swarmkit::client::Client;
using swarmkit::client::ClientConfig;
using swarmkit::commands::CmdLand;
using swarmkit::commands::CmdReturnHome;
using swarmkit::commands::Command;
using swarmkit::commands::CommandPriority;
using swarmkit::commands::FlightCmd;
using swarmkit::commands::NavCmd;

struct SwarmRuntime {
    std::unique_ptr<swarmkit::client::SwarmClient> client;
    std::vector<std::string> drone_ids;
    std::string client_id;
};

struct SwarmResultPolicy {
    bool continue_on_error{false};
    bool require_all{false};
};

struct SwarmResultSummary {
    int sent{0};
    int succeeded{0};
    int accepted{0};
    int already_satisfied{0};
    int failed{0};
};

constexpr std::int64_t kSwarmHealthStaleAfterMs = 5000;

struct WaitCondition {
    std::optional<float> alt_min_m;
    std::optional<float> alt_max_m;
    std::optional<double> lat_deg;
    std::optional<double> lon_deg;
    std::optional<float> target_alt_m;
    std::optional<float> battery_min_percent;
    std::optional<std::string> mode_contains;
    std::optional<bool> armed;
    bool wait_heartbeat{false};
    bool wait_landed{false};
    float position_radius_m{2.0F};
    float alt_tolerance_m{0.75F};
    float landed_alt_m{0.5F};
    int timeout_ms{30000};
};

struct SequenceStep {
    std::vector<std::string> args;
    std::vector<WaitCondition> wait_conditions;
    std::string drone_id;
    bool broadcast{false};
    bool continue_on_error{false};
    bool verify{false};
    int delay_ms{0};
    int timeout_ms{0};
    int retries{0};
    int retry_delay_ms{1000};
};

[[nodiscard]] inline volatile std::sig_atomic_t& StopRequestedFlag() {
    static volatile std::sig_atomic_t stop_requested = 0;
    return stop_requested;
}

inline void OnSignal(int /*sig*/) {
    StopRequestedFlag() = 1;
}

[[nodiscard]] inline bool IsStopRequested() {
    return StopRequestedFlag() != 0;
}

inline void ResetStopRequested() {
    StopRequestedFlag() = 0;
}

[[nodiscard]] inline std::vector<std::string> FindActionsAfterCommand(
    int argc, char** argv, std::string_view command_name) {
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

[[nodiscard]] inline std::int64_t NowUnixMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

[[nodiscard]] inline double DegToRad(double degrees) {
    return degrees * std::numbers::pi / 180.0;
}

[[nodiscard]] inline double DistanceMeters(double lat_a_deg, double lon_a_deg, double lat_b_deg,
                                           double lon_b_deg) {
    constexpr double kEarthRadiusMeters = 6371000.0;
    const double lat_a = DegToRad(lat_a_deg);
    const double lat_b = DegToRad(lat_b_deg);
    const double delta_lat = DegToRad(lat_b_deg - lat_a_deg);
    const double delta_lon = DegToRad(lon_b_deg - lon_a_deg);
    const double sin_lat = std::sin(delta_lat / 2.0);
    const double sin_lon = std::sin(delta_lon / 2.0);
    const double haversine =
        (sin_lat * sin_lat) + (std::cos(lat_a) * std::cos(lat_b) * sin_lon * sin_lon);
    return 2.0 * kEarthRadiusMeters * std::atan2(std::sqrt(haversine), std::sqrt(1.0 - haversine));
}

[[nodiscard]] inline std::string ToLowerAscii(std::string value) {
    std::ranges::transform(value, value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

[[nodiscard]] inline bool ModeContains(std::string mode, std::string expected) {
    mode = ToLowerAscii(std::move(mode));
    expected = ToLowerAscii(std::move(expected));
    return mode.find(expected) != std::string::npos;
}

[[nodiscard]] inline bool FrameMatchesWaitCondition(const swarmkit::core::TelemetryFrame& frame,
                                                    const WaitCondition& condition) {
    if (condition.wait_heartbeat && frame.agent_receive_unix_time_ms <= 0) {
        return false;
    }
    if ((condition.alt_min_m.has_value() || condition.alt_max_m.has_value() ||
         condition.target_alt_m.has_value()) &&
        !frame.HasRelativeAltitude()) {
        return false;
    }
    if (condition.alt_min_m.has_value() && frame.rel_alt_m < *condition.alt_min_m) {
        return false;
    }
    if (condition.alt_max_m.has_value() && frame.rel_alt_m > *condition.alt_max_m) {
        return false;
    }
    if (condition.target_alt_m.has_value() &&
        std::abs(frame.rel_alt_m - *condition.target_alt_m) > condition.alt_tolerance_m) {
        return false;
    }
    if (condition.lat_deg.has_value() && condition.lon_deg.has_value() && !frame.HasPosition()) {
        return false;
    }
    if (condition.lat_deg.has_value() && condition.lon_deg.has_value() &&
        DistanceMeters(frame.lat_deg, frame.lon_deg, *condition.lat_deg, *condition.lon_deg) >
            condition.position_radius_m) {
        return false;
    }
    if (condition.battery_min_percent.has_value() && !frame.HasBattery()) {
        return false;
    }
    if (condition.battery_min_percent.has_value() &&
        frame.battery_percent < *condition.battery_min_percent) {
        return false;
    }
    if (condition.mode_contains.has_value() &&
        !ModeContains(frame.mode, *condition.mode_contains)) {
        return false;
    }
    if (condition.armed.has_value()) {
        const bool frame_armed = ModeContains(frame.mode, "armed");
        const bool frame_disarmed = ModeContains(frame.mode, "disarmed");
        if (*condition.armed && (!frame_armed || frame_disarmed)) {
            return false;
        }
        if (!*condition.armed && !frame_disarmed) {
            return false;
        }
    }
    if (condition.wait_landed &&
        (!frame.HasRelativeAltitude() || std::abs(frame.rel_alt_m) > condition.landed_alt_m)) {
        return false;
    }
    return true;
}

class SequenceTelemetryMonitor {
   public:
    ~SequenceTelemetryMonitor() {
        Stop();
    }

    SequenceTelemetryMonitor(const SequenceTelemetryMonitor&) = delete;
    SequenceTelemetryMonitor& operator=(const SequenceTelemetryMonitor&) = delete;

    SequenceTelemetryMonitor() = default;

    [[nodiscard]] bool StartSingle(Client& client, const std::string& drone_id, int rate_hz) {
        Stop();
        auto subscription = client.StartTelemetry(
            {.drone_id = drone_id, .rate_hertz = rate_hz},
            [this](const swarmkit::client::TelemetryObservation& observation) {
                if (const auto* frame =
                        std::get_if<swarmkit::client::TelemetryFrameObservation>(&observation)) {
                    StoreFrame(frame->delivery.frame);
                }
            },
            [](const std::string& error_msg) {
                std::cerr << "Sequence telemetry stream error: " << error_msg << "\n";
            });
        if (!subscription.has_value()) {
            std::cerr << "Failed to start sequence telemetry: " << subscription.error().user_message
                      << "\n";
            return false;
        }
        single_subscription_.emplace(std::move(*subscription));
        return true;
    }

    [[nodiscard]] bool StartSwarm(SwarmRuntime& runtime, int rate_hz) {
        Stop();
        auto subscriptions = runtime.client->StartAllTelemetry(
            rate_hz,
            [this](const swarmkit::client::TelemetryObservation& observation) {
                if (const auto* frame =
                        std::get_if<swarmkit::client::TelemetryFrameObservation>(&observation)) {
                    StoreFrame(frame->delivery.frame);
                }
            },
            [](const std::string& error_msg) {
                std::cerr << "Sequence telemetry stream error: " << error_msg << "\n";
            });
        if (subscriptions.empty()) {
            std::cerr << "Failed to start sequence telemetry: swarm has no registered drones\n";
            return false;
        }
        bool all_started = true;
        for (auto& [drone_id, result] : subscriptions) {
            if (!result.has_value()) {
                std::cerr << "Failed to start sequence telemetry for " << drone_id << ": "
                          << result.error().user_message << "\n";
                all_started = false;
                continue;
            }
            swarm_subscriptions_.push_back(std::move(*result));
        }
        if (!all_started) {
            Stop();
            return false;
        }
        return true;
    }

    void Stop() {
        if (single_subscription_.has_value()) {
            single_subscription_->Stop();
            single_subscription_.reset();
        }
        for (auto& subscription : swarm_subscriptions_) {
            subscription.Stop();
        }
        swarm_subscriptions_.clear();
    }

    [[nodiscard]] bool WaitFor(const std::vector<std::string>& drone_ids,
                               const WaitCondition& condition, std::string* detail) {
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds{condition.timeout_ms};

        std::unique_lock<std::mutex> lock(mutex_);
        while (true) {
            bool all_match = true;
            for (const std::string& drone_id : drone_ids) {
                const auto frame = FindFrameLocked(drone_id);
                if (!frame.has_value() || !FrameMatchesWaitCondition(*frame, condition)) {
                    all_match = false;
                    break;
                }
            }
            if (all_match) {
                if (detail != nullptr) {
                    *detail = "condition satisfied";
                }
                return true;
            }
            if (cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
                if (detail != nullptr) {
                    *detail = "timed out waiting for telemetry condition";
                }
                return false;
            }
        }
    }

   private:
    void StoreFrame(const swarmkit::core::TelemetryFrame& frame) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            frames_[frame.drone_id] = frame;
            if (frames_.size() == 1) {
                only_frame_ = frame;
            }
        }
        cv_.notify_all();
    }

    [[nodiscard]] std::optional<swarmkit::core::TelemetryFrame> FindFrameLocked(
        const std::string& drone_id) const {
        if (const auto iter = frames_.find(drone_id); iter != frames_.end()) {
            return iter->second;
        }
        if (frames_.size() == 1 && (drone_id == "default" || drone_id.empty())) {
            return only_frame_;
        }
        return std::nullopt;
    }

    std::optional<swarmkit::client::Subscription> single_subscription_;
    std::vector<swarmkit::client::Subscription> swarm_subscriptions_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::unordered_map<std::string, swarmkit::core::TelemetryFrame> frames_;
    std::optional<swarmkit::core::TelemetryFrame> only_frame_;
};

[[nodiscard]] inline std::expected<std::int64_t, std::string> ParseDurationMs(int argc,
                                                                              char** argv) {
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

[[nodiscard]] inline std::expected<std::string, std::string> ParseSequenceFilePath(int argc,
                                                                                   char** argv) {
    std::string path = common::GetOptionValue(argc, argv, "--file");
    if (path.empty()) {
        return std::unexpected("sequence requires --file PATH");
    }
    return path;
}

[[nodiscard]] inline std::vector<std::string> ReadYamlStringList(const YAML::Node& node) {
    std::vector<std::string> out;
    if (!node || !node.IsSequence()) {
        return out;
    }
    out.reserve(node.size());
    for (const YAML::Node& item : node) {
        out.push_back(item.as<std::string>());
    }
    return out;
}

inline void ReadOptionalWaitFloat(const YAML::Node& node, const char* key,
                                  std::optional<float>* out) {
    if (out != nullptr && node && node[key]) {
        *out = node[key].as<float>();
    }
}

inline void ReadOptionalWaitDouble(const YAML::Node& node, const char* key,
                                   std::optional<double>* out) {
    if (out != nullptr && node && node[key]) {
        *out = node[key].as<double>();
    }
}

inline void ApplyCommonWaitFields(const YAML::Node& node, WaitCondition* condition) {
    if (condition == nullptr || !node) {
        return;
    }
    if (node["timeout_ms"]) {
        condition->timeout_ms = node["timeout_ms"].as<int>();
    }
    if (node["radius_m"]) {
        condition->position_radius_m = node["radius_m"].as<float>();
    }
    if (node["alt_tolerance_m"]) {
        condition->alt_tolerance_m = node["alt_tolerance_m"].as<float>();
    }
    if (node["landed_alt_m"]) {
        condition->landed_alt_m = node["landed_alt_m"].as<float>();
    }
}

[[nodiscard]] inline std::expected<WaitCondition, std::string> ParseWaitCondition(
    const YAML::Node& node) {
    WaitCondition condition;
    try {
        if (!node || !node.IsMap()) {
            return std::unexpected("wait condition must be a map");
        }

        ApplyCommonWaitFields(node, &condition);
        if (node["wait_heartbeat"]) {
            condition.wait_heartbeat = node["wait_heartbeat"].as<bool>();
        }
        if (node["wait_alt"]) {
            const YAML::Node wait_alt = node["wait_alt"];
            if (wait_alt.IsScalar()) {
                condition.target_alt_m = wait_alt.as<float>();
            } else {
                ReadOptionalWaitFloat(wait_alt, "min", &condition.alt_min_m);
                ReadOptionalWaitFloat(wait_alt, "max", &condition.alt_max_m);
                ReadOptionalWaitFloat(wait_alt, "target", &condition.target_alt_m);
                ApplyCommonWaitFields(wait_alt, &condition);
            }
        }
        if (node["wait_position"]) {
            const YAML::Node wait_position = node["wait_position"];
            ReadOptionalWaitDouble(wait_position, "lat", &condition.lat_deg);
            ReadOptionalWaitDouble(wait_position, "lon", &condition.lon_deg);
            ReadOptionalWaitFloat(wait_position, "alt", &condition.target_alt_m);
            ApplyCommonWaitFields(wait_position, &condition);
        }
        if (node["wait_mode"]) {
            condition.mode_contains = node["wait_mode"].as<std::string>();
        }
        if (node["wait_armed"]) {
            condition.armed = true;
        }
        if (node["wait_disarmed"]) {
            condition.armed = false;
        }
        if (node["wait_landed"]) {
            condition.wait_landed = node["wait_landed"].as<bool>();
        }
        if (node["wait_battery_min"]) {
            condition.battery_min_percent = node["wait_battery_min"].as<float>();
        }
        if (condition.timeout_ms <= 0) {
            return std::unexpected("wait timeout_ms must be > 0");
        }
        return condition;
    } catch (const YAML::Exception& exc) {
        return std::unexpected("invalid wait condition: " + std::string(exc.what()));
    }
}

[[nodiscard]] inline bool StepHasWaitCondition(const YAML::Node& node) {
    return node["wait_heartbeat"] || node["wait_alt"] || node["wait_position"] ||
           node["wait_mode"] || node["wait_armed"] || node["wait_disarmed"] ||
           node["wait_landed"] || node["wait_battery_min"];
}

[[nodiscard]] inline bool IsDisarmAction(const SequenceStep& step) {
    return step.args.size() == 1 && step.args.front() == "disarm";
}

[[nodiscard]] inline bool IsEmergencyAction(const SequenceStep& step) {
    return !step.args.empty() && step.args.front() == "emergency";
}

[[nodiscard]] inline bool IsAlreadySatisfied(const swarmkit::client::CommandResult& result) {
    std::string message = result.message;
    std::ranges::transform(message, message.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return result.ok && message.find("already satisfied") != std::string::npos;
}

[[nodiscard]] inline std::string OptionalFloatText(const std::optional<float>& value) {
    if (!value.has_value()) {
        return "unknown";
    }
    std::ostringstream out;
    out << *value;
    return out.str();
}

[[nodiscard]] inline std::string FloatText(float value, int precision = 2) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

[[nodiscard]] inline const char* BoolText(bool value) {
    return value ? "true" : "false";
}

[[nodiscard]] inline const char* BoolText(const std::optional<bool>& value) {
    if (!value.has_value()) {
        return "unknown";
    }
    return BoolText(*value);
}

[[nodiscard]] inline std::string GpsFixText(int fix_type) {
    switch (fix_type) {
        case 0:
            return "no GPS";
        case 1:
            return "no fix";
        case 2:
            return "2D fix";
        case 3:
            return "3D fix";
        case 4:
            return "DGPS";
        case 5:
            return "RTK float";
        case 6:
            return "RTK fixed";
        default:
            return "fix_type=" + std::to_string(fix_type);
    }
}

[[nodiscard]] inline std::string RelativeAltitudeText(
    const swarmkit::client::HealthStatus& status) {
    if (!status.has_relative_altitude) {
        return "unknown";
    }
    return FloatText(status.relative_alt_m) + "m";
}

[[nodiscard]] inline std::string ReadinessSeverityText(
    swarmkit::client::ReadinessCheckSeverity severity) {
    switch (severity) {
        case swarmkit::client::ReadinessCheckSeverity::kWarning:
            return "warning";
        case swarmkit::client::ReadinessCheckSeverity::kError:
            return "error";
        case swarmkit::client::ReadinessCheckSeverity::kInfo:
        default:
            return "info";
    }
}

[[nodiscard]] inline bool IsStaleTimestamp(std::int64_t timestamp_ms, std::int64_t now_ms) {
    return timestamp_ms <= 0 || now_ms - timestamp_ms > kSwarmHealthStaleAfterMs;
}

[[nodiscard]] inline std::string TimestampAgeMsText(std::int64_t timestamp_ms,
                                                    std::int64_t now_ms) {
    if (timestamp_ms <= 0) {
        return "unknown";
    }
    return std::to_string(std::max<std::int64_t>(0, now_ms - timestamp_ms));
}

[[nodiscard]] inline std::string SwarmHealthReason(const swarmkit::client::HealthStatus& status,
                                                   bool heartbeat_stale, bool telemetry_stale) {
    if (!status.ok) {
        if (!status.message.empty()) {
            return status.message;
        }
        return status.error.user_message;
    }
    if (!status.ready) {
        return status.message.empty() ? "agent not ready" : status.message;
    }
    if (status.failsafe == true) {
        return "failsafe active";
    }
    if (status.failsafe == std::nullopt) {
        return "failsafe state unknown";
    }
    if (status.ekf_ok != true) {
        return status.ekf_ok.has_value() ? "EKF unhealthy" : "EKF health unknown";
    }
    if (status.gps_ok != true) {
        return status.gps_ok.has_value() ? "GPS unhealthy" : "GPS health unknown";
    }
    if (heartbeat_stale) {
        return "heartbeat stale";
    }
    if (telemetry_stale) {
        return "telemetry stale";
    }
    return {};
}

[[nodiscard]] inline SwarmResultPolicy ParseSwarmResultPolicy(int argc, char** argv) {
    return {
        .continue_on_error = common::HasFlag(argc, argv, "--continue-on-error"),
        .require_all = common::HasFlag(argc, argv, "--require-all"),
    };
}

[[nodiscard]] inline std::expected<std::vector<SequenceStep>, std::string> LoadSequenceSteps(
    const std::string& path) {
    try {
        const YAML::Node root = YAML::LoadFile(path);
        const YAML::Node steps = root["steps"] ? root["steps"] : root;
        if (!steps || !steps.IsSequence()) {
            return std::unexpected("sequence file must contain a steps: sequence");
        }

        std::vector<SequenceStep> out;
        out.reserve(steps.size());
        for (const YAML::Node& node : steps) {
            if (!node || !node.IsMap()) {
                return std::unexpected("each sequence step must be a map");
            }

            SequenceStep step;
            step.args = ReadYamlStringList(node["args"]);
            step.drone_id = node["drone"] ? node["drone"].as<std::string>() : std::string{};
            step.broadcast = node["broadcast"] ? node["broadcast"].as<bool>() : false;
            step.continue_on_error =
                node["continue_on_error"] ? node["continue_on_error"].as<bool>() : false;
            step.verify = node["verify"] ? node["verify"].as<bool>() : false;
            step.delay_ms = node["delay_ms"] ? node["delay_ms"].as<int>() : 0;
            step.timeout_ms = node["timeout_ms"] ? node["timeout_ms"].as<int>() : 0;
            step.retries = node["retries"] ? node["retries"].as<int>() : 0;
            step.retry_delay_ms =
                node["retry_delay_ms"] ? node["retry_delay_ms"].as<int>() : step.retry_delay_ms;
            if (StepHasWaitCondition(node)) {
                const auto wait_condition = ParseWaitCondition(node);
                if (!wait_condition.has_value()) {
                    return std::unexpected(wait_condition.error());
                }
                step.wait_conditions.push_back(*wait_condition);
            }
            if (step.args.empty() && step.delay_ms <= 0 && step.wait_conditions.empty()) {
                return std::unexpected("sequence step requires args, delay_ms, or wait_*");
            }
            if (step.delay_ms < 0) {
                return std::unexpected("sequence step delay_ms must be >= 0");
            }
            if (step.retries < 0) {
                return std::unexpected("sequence step retries must be >= 0");
            }
            if (step.retry_delay_ms < 0) {
                return std::unexpected("sequence step retry_delay_ms must be >= 0");
            }
            if (step.timeout_ms < 0) {
                return std::unexpected("sequence step timeout_ms must be >= 0");
            }
            out.push_back(std::move(step));
        }
        return out;
    } catch (const YAML::Exception& exc) {
        return std::unexpected("failed to load sequence file '" + path + "': " + exc.what());
    }
}

inline void DelaySequenceStep(const SequenceStep& step) {
    if (step.delay_ms > 0) {
        std::cout << "delay " << step.delay_ms << "ms\n";
        std::this_thread::sleep_for(std::chrono::milliseconds{step.delay_ms});
    }
}

[[nodiscard]] inline std::expected<swarmkit::client::CommandWaitOptions, std::string>
ParseCommandWaitOptions(int argc, char** argv) {
    swarmkit::client::CommandWaitOptions options;
    const std::string timeout_value = common::GetOptionValue(argc, argv, "--timeout-ms");
    if (!timeout_value.empty()) {
        const auto timeout = ParseIntArg(timeout_value, "--timeout-ms");
        if (!timeout.has_value()) {
            return std::unexpected(timeout.error());
        }
        if (*timeout <= 0) {
            return std::unexpected("--timeout-ms must be > 0 when used for command verification");
        }
        options.timeout_ms = *timeout;
    }
    options.telemetry_rate_hz = kDefaultSequenceTelemetryRateHz;
    return options;
}

[[nodiscard]] inline swarmkit::client::CommandWaitOptions StepWaitOptions(
    const swarmkit::client::CommandWaitOptions& base, const SequenceStep& step) {
    swarmkit::client::CommandWaitOptions options = base;
    if (step.timeout_ms > 0) {
        options.timeout_ms = step.timeout_ms;
    }
    return options;
}

[[nodiscard]] inline bool PrintCommandResult(std::string_view label,
                                             const swarmkit::client::CommandResult& result) {
    std::cout << label << ": " << (result.ok ? "OK" : "FAILED");
    if (!result.message.empty()) {
        std::cout << " " << result.message;
    }
    if (!result.correlation_id.empty()) {
        std::cout << " [corr=" << result.correlation_id << "]";
    }
    std::cout << "\n";
    return result.ok;
}

[[nodiscard]] inline bool PrintReleaseAuthorityResult(
    std::string_view label, const swarmkit::client::ReleaseAuthorityResult& result) {
    std::cout << label << ": " << (result.ok ? "OK" : "FAILED");
    if (!result.message.empty()) {
        std::cout << " " << result.message;
    }
    if (!result.correlation_id.empty()) {
        std::cout << " [corr=" << result.correlation_id << "]";
    }
    std::cout << "\n";
    return result.ok;
}

[[nodiscard]] inline std::vector<std::string> StepTargetDrones(
    const SequenceStep& step, std::string_view default_drone_id,
    const std::vector<std::string>& swarm_ids) {
    if (step.broadcast) {
        if (swarm_ids.empty()) {
            return {std::string(default_drone_id)};
        }
        return swarm_ids;
    }
    if (!step.drone_id.empty()) {
        return {step.drone_id};
    }
    return {std::string(default_drone_id)};
}

[[nodiscard]] inline bool WaitForConditions(SequenceTelemetryMonitor& monitor,
                                            const std::vector<std::string>& drone_ids,
                                            const std::vector<WaitCondition>& conditions,
                                            std::size_t step_index) {
    for (const WaitCondition& condition : conditions) {
        std::string detail;
        if (!monitor.WaitFor(drone_ids, condition, &detail)) {
            std::cerr << "step " << step_index << " wait failed: " << detail << "\n";
            return false;
        }
        std::cout << "step " << step_index << " wait OK\n";
    }
    return true;
}

inline void WaitForStop(std::int64_t duration_ms) {
    const auto start = std::chrono::steady_clock::now();
    while (!IsStopRequested()) {
        if (duration_ms > 0) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::steady_clock::now() - start)
                                        .count();
            if (elapsed_ms >= duration_ms) {
                return;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kTelemetryPollIntervalMs));
    }
}

[[nodiscard]] inline std::expected<std::int64_t, std::string> ParseTtlMs(int argc, char** argv) {
    const auto ttl =
        ParseIntArg(common::GetOptionValue(argc, argv, "--ttl-ms", kDefaultZero), "--ttl-ms");
    if (!ttl.has_value()) {
        return std::unexpected(ttl.error());
    }
    return static_cast<std::int64_t>(*ttl);
}

}  // namespace swarmkit::apps::cli::internal
