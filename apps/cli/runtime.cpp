// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "runtime.h"

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
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
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
#include "usage.h"

namespace swarmkit::apps::cli::internal {
namespace {

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

[[nodiscard]] volatile std::sig_atomic_t& StopRequestedFlag() {
    static volatile std::sig_atomic_t stop_requested = 0;
    return stop_requested;
}

extern "C" void OnSignal(int /*sig*/) {
    StopRequestedFlag() = 1;
}

[[nodiscard]] bool IsStopRequested() {
    return StopRequestedFlag() != 0;
}

void ResetStopRequested() {
    StopRequestedFlag() = 0;
}

[[nodiscard]] std::vector<std::string> FindActionsAfterCommand(int argc, char** argv,
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

[[nodiscard]] std::int64_t NowUnixMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

int RunReports(Client& client, std::string_view drone_id, int argc, char** argv);

[[nodiscard]] std::vector<std::string> CollectOptionValues(int argc, char** argv,
                                                           std::string_view option_name) {
    std::vector<std::string> values;
    for (int index = 1; index + 1 < argc; ++index) {
        if (std::string_view(argv[index]) == option_name) {
            values.emplace_back(argv[++index]);
        }
    }
    return values;
}

[[nodiscard]] std::expected<std::string, std::string> ReadSmallPayload(int argc, char** argv) {
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

[[nodiscard]] double DegToRad(double degrees) {
    return degrees * std::numbers::pi / 180.0;
}

[[nodiscard]] double DistanceMeters(double lat_a_deg, double lon_a_deg, double lat_b_deg,
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

[[nodiscard]] std::string ToLowerAscii(std::string value) {
    std::ranges::transform(value, value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

[[nodiscard]] bool ModeContains(std::string mode, std::string expected) {
    mode = ToLowerAscii(std::move(mode));
    expected = ToLowerAscii(std::move(expected));
    return mode.find(expected) != std::string::npos;
}

[[nodiscard]] bool FrameMatchesWaitCondition(const swarmkit::core::TelemetryFrame& frame,
                                             const WaitCondition& condition) {
    if (condition.wait_heartbeat && frame.unix_time_ms <= 0) {
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
    if (condition.lat_deg.has_value() && condition.lon_deg.has_value() &&
        !frame.HasPosition()) {
        return false;
    }
    if (condition.lat_deg.has_value() && condition.lon_deg.has_value() &&
        DistanceMeters(frame.lat_deg, frame.lon_deg, *condition.lat_deg, *condition.lon_deg) >
            condition.position_radius_m) {
        return false;
    }
    if (condition.battery_min_percent.has_value() &&
        !frame.HasBattery()) {
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
            [this](const swarmkit::core::TelemetryFrame& frame) { StoreFrame(frame); },
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
            rate_hz, [this](const swarmkit::core::TelemetryFrame& frame) { StoreFrame(frame); },
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

[[nodiscard]] std::expected<std::int64_t, std::string> ParseDurationMs(int argc, char** argv) {
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

[[nodiscard]] std::expected<std::string, std::string> ParseSequenceFilePath(int argc, char** argv) {
    std::string path = common::GetOptionValue(argc, argv, "--file");
    if (path.empty()) {
        return std::unexpected("sequence requires --file PATH");
    }
    return path;
}

[[nodiscard]] std::vector<std::string> ReadYamlStringList(const YAML::Node& node) {
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

void ReadOptionalWaitFloat(const YAML::Node& node, const char* key, std::optional<float>* out) {
    if (out != nullptr && node && node[key]) {
        *out = node[key].as<float>();
    }
}

void ReadOptionalWaitDouble(const YAML::Node& node, const char* key, std::optional<double>* out) {
    if (out != nullptr && node && node[key]) {
        *out = node[key].as<double>();
    }
}

void ApplyCommonWaitFields(const YAML::Node& node, WaitCondition* condition) {
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

[[nodiscard]] std::expected<WaitCondition, std::string> ParseWaitCondition(const YAML::Node& node) {
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

[[nodiscard]] bool StepHasWaitCondition(const YAML::Node& node) {
    return node["wait_heartbeat"] || node["wait_alt"] || node["wait_position"] ||
           node["wait_mode"] || node["wait_armed"] || node["wait_disarmed"] ||
           node["wait_landed"] || node["wait_battery_min"];
}

[[nodiscard]] bool IsDisarmAction(const SequenceStep& step) {
    return step.args.size() == 1 && step.args.front() == "disarm";
}

[[nodiscard]] bool IsEmergencyAction(const SequenceStep& step) {
    return !step.args.empty() && step.args.front() == "emergency";
}

[[nodiscard]] bool IsAlreadySatisfied(const swarmkit::client::CommandResult& result) {
    std::string message = result.message;
    std::ranges::transform(message, message.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return result.ok && message.find("already satisfied") != std::string::npos;
}

[[nodiscard]] std::string OptionalFloatText(const std::optional<float>& value) {
    if (!value.has_value()) {
        return "unknown";
    }
    std::ostringstream out;
    out << *value;
    return out.str();
}

[[nodiscard]] std::string FloatText(float value, int precision = 2) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

[[nodiscard]] const char* BoolText(bool value) {
    return value ? "true" : "false";
}

[[nodiscard]] std::string GpsFixText(int fix_type) {
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

[[nodiscard]] std::string RelativeAltitudeText(const swarmkit::client::HealthStatus& status) {
    if (!status.has_relative_altitude) {
        return "unknown";
    }
    return FloatText(status.relative_alt_m) + "m";
}

[[nodiscard]] std::string ReadinessSeverityText(
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

[[nodiscard]] bool IsStaleTimestamp(std::int64_t timestamp_ms, std::int64_t now_ms) {
    return timestamp_ms <= 0 || now_ms - timestamp_ms > kSwarmHealthStaleAfterMs;
}

[[nodiscard]] std::string TimestampAgeMsText(std::int64_t timestamp_ms, std::int64_t now_ms) {
    if (timestamp_ms <= 0) {
        return "unknown";
    }
    return std::to_string(std::max<std::int64_t>(0, now_ms - timestamp_ms));
}

[[nodiscard]] std::string SwarmHealthReason(const swarmkit::client::HealthStatus& status,
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
    if (status.failsafe) {
        return "failsafe active";
    }
    if (!status.ekf_ok) {
        return "EKF unhealthy";
    }
    if (!status.gps_ok) {
        return "GPS unhealthy";
    }
    if (heartbeat_stale) {
        return "heartbeat stale";
    }
    if (telemetry_stale) {
        return "telemetry stale";
    }
    return {};
}

[[nodiscard]] SwarmResultPolicy ParseSwarmResultPolicy(int argc, char** argv) {
    return {
        .continue_on_error = common::HasFlag(argc, argv, "--continue-on-error"),
        .require_all = common::HasFlag(argc, argv, "--require-all"),
    };
}

[[nodiscard]] std::expected<std::vector<SequenceStep>, std::string> LoadSequenceSteps(
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

void DelaySequenceStep(const SequenceStep& step) {
    if (step.delay_ms > 0) {
        std::cout << "delay " << step.delay_ms << "ms\n";
        std::this_thread::sleep_for(std::chrono::milliseconds{step.delay_ms});
    }
}

[[nodiscard]] std::expected<swarmkit::client::CommandWaitOptions, std::string>
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

[[nodiscard]] swarmkit::client::CommandWaitOptions StepWaitOptions(
    const swarmkit::client::CommandWaitOptions& base, const SequenceStep& step) {
    swarmkit::client::CommandWaitOptions options = base;
    if (step.timeout_ms > 0) {
        options.timeout_ms = step.timeout_ms;
    }
    return options;
}

[[nodiscard]] bool PrintCommandResult(std::string_view label,
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

[[nodiscard]] bool PrintReleaseAuthorityResult(
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

[[nodiscard]] std::expected<std::unordered_map<std::string, std::string>, std::string>
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

[[nodiscard]] std::expected<swarmkit::client::ActiveGoal, std::string> ParseActiveGoal(
    std::string_view drone_id, int argc, char** argv) {
    const std::string goal_id = common::GetOptionValue(argc, argv, "--goal-id");
    if (goal_id.empty()) {
        return std::unexpected("goal set requires --goal-id ID");
    }
    std::string target_frame = common::GetOptionValue(argc, argv, "--frame", "global");
    target_frame = ToLowerAscii(std::move(target_frame));
    const bool use_local_target =
        target_frame == "local-ned" || !common::GetOptionValue(argc, argv, "--x").empty() ||
        !common::GetOptionValue(argc, argv, "--y").empty() ||
        !common::GetOptionValue(argc, argv, "--z").empty();
    if (use_local_target) {
        target_frame = "local-ned";
    }
    if (target_frame != "global" && target_frame != "local-ned") {
        return std::unexpected("--frame must be global or local-ned");
    }

    swarmkit::client::GeoPoint target;
    swarmkit::client::LocalPoint local_target;
    if (use_local_target) {
        const auto x = ParseDoubleArg(common::GetOptionValue(argc, argv, "--x"), "--x");
        const auto y = ParseDoubleArg(common::GetOptionValue(argc, argv, "--y"), "--y");
        const auto z = ParseDoubleArg(common::GetOptionValue(argc, argv, "--z"), "--z");
        if (!x.has_value()) {
            return std::unexpected(x.error());
        }
        if (!y.has_value()) {
            return std::unexpected(y.error());
        }
        if (!z.has_value()) {
            return std::unexpected(z.error());
        }
        local_target = swarmkit::client::LocalPoint{.x_m = *x, .y_m = *y, .z_m = *z};
    } else {
        const auto lat = ParseDoubleArg(common::GetOptionValue(argc, argv, "--lat"), "--lat");
        const auto lon = ParseDoubleArg(common::GetOptionValue(argc, argv, "--lon"), "--lon");
        const auto alt = ParseDoubleArg(common::GetOptionValue(argc, argv, "--alt"), "--alt");
        if (!lat.has_value()) {
            return std::unexpected(lat.error());
        }
        if (!lon.has_value()) {
            return std::unexpected(lon.error());
        }
        if (!alt.has_value()) {
            return std::unexpected(alt.error());
        }
        target = swarmkit::client::GeoPoint{.lat_deg = *lat, .lon_deg = *lon, .alt_m = *alt};
    }

    auto revision = static_cast<std::uint64_t>(NowUnixMs());
    if (const std::string revision_value = common::GetOptionValue(argc, argv, "--revision");
        !revision_value.empty()) {
        const auto parsed_revision = ParseIntArg(revision_value, "--revision");
        if (!parsed_revision.has_value()) {
            return std::unexpected(parsed_revision.error());
        }
        if (*parsed_revision < 0) {
            return std::unexpected("--revision must be >= 0");
        }
        revision = static_cast<std::uint64_t>(*parsed_revision);
    }

    const auto speed =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--speed", kDefaultZero), "--speed");
    const auto acceptance_radius = ParseFloatArg(
        common::GetOptionValue(argc, argv, "--accept-radius", "2"), "--accept-radius");
    const auto deviation_radius = ParseFloatArg(
        common::GetOptionValue(argc, argv, "--deviation-radius", "8"), "--deviation-radius");
    const auto timeout_ms = ParseIntArg(
        common::GetOptionValue(argc, argv, "--timeout-ms", kDefaultZero), "--timeout-ms");
    if (!speed.has_value()) {
        return std::unexpected(speed.error());
    }
    if (!acceptance_radius.has_value()) {
        return std::unexpected(acceptance_radius.error());
    }
    if (!deviation_radius.has_value()) {
        return std::unexpected(deviation_radius.error());
    }
    if (!timeout_ms.has_value()) {
        return std::unexpected(timeout_ms.error());
    }
    auto labels = ParseKeyValueLabels(argc, argv, "--label");
    if (!labels.has_value()) {
        return std::unexpected(labels.error());
    }

    return swarmkit::client::ActiveGoal{
        .drone_id = std::string(drone_id),
        .goal_id = goal_id,
        .revision = revision,
        .target = target,
        .local_target = local_target,
        .use_local_target = use_local_target,
        .target_frame = target_frame,
        .speed_mps = *speed,
        .acceptance_radius_m = *acceptance_radius,
        .deviation_radius_m = *deviation_radius,
        .timeout_ms = *timeout_ms,
        .labels = std::move(*labels),
    };
}

[[nodiscard]] std::vector<std::string> StepTargetDrones(const SequenceStep& step,
                                                        std::string_view default_drone_id,
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

[[nodiscard]] bool WaitForConditions(SequenceTelemetryMonitor& monitor,
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

void WaitForStop(std::int64_t duration_ms) {
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

int RunSequence(Client& client, std::string_view default_drone_id, std::string_view client_id,
                CommandPriority priority, int argc, char** argv) {
    const auto path = ParseSequenceFilePath(argc, argv);
    if (!path.has_value()) {
        std::cerr << path.error() << "\n";
        return EXIT_FAILURE;
    }
    const auto steps = LoadSequenceSteps(*path);
    if (!steps.has_value()) {
        std::cerr << steps.error() << "\n";
        return EXIT_FAILURE;
    }
    const bool verify_commands = common::HasFlag(argc, argv, "--verify");
    const bool continue_on_error = common::HasFlag(argc, argv, "--continue-on-error");
    const auto wait_options = ParseCommandWaitOptions(argc, argv);
    if (!wait_options.has_value()) {
        std::cerr << wait_options.error() << "\n";
        return EXIT_FAILURE;
    }

    const bool needs_telemetry = std::ranges::any_of(*steps, [](const SequenceStep& step) {
        return !step.wait_conditions.empty() || IsDisarmAction(step);
    });
    SequenceTelemetryMonitor monitor;
    if (needs_telemetry) {
        if (!monitor.StartSingle(client, std::string(default_drone_id),
                                 kDefaultSequenceTelemetryRateHz)) {
            return EXIT_FAILURE;
        }
    }

    int failed_steps = 0;
    for (std::size_t index = 0; index < steps->size(); ++index) {
        const SequenceStep& step = steps->at(index);
        DelaySequenceStep(step);
        const std::vector<std::string> target_drones = StepTargetDrones(step, default_drone_id, {});
        if (!step.wait_conditions.empty() &&
            !WaitForConditions(monitor, target_drones, step.wait_conditions, index)) {
            ++failed_steps;
            if (!step.continue_on_error && !continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }
        if (step.args.empty()) {
            continue;
        }

        const std::string drone_id =
            step.drone_id.empty() ? std::string(default_drone_id) : step.drone_id;
        const auto command = BuildCommandFromTokens(step.args);
        if (!command.has_value()) {
            std::cerr << "step " << index << " build failed: " << command.error() << "\n";
            ++failed_steps;
            if (!step.continue_on_error && !continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }

        bool step_ok = false;
        const int attempts = step.retries + 1;
        for (int attempt = 1; attempt <= attempts; ++attempt) {
            if (IsDisarmAction(step) && !IsEmergencyAction(step)) {
                WaitCondition landed_condition;
                landed_condition.wait_landed = true;
                landed_condition.timeout_ms = 30000;
                if (!WaitForConditions(monitor, target_drones, {landed_condition}, index)) {
                    step_ok = false;
                    break;
                }
            }
            const auto envelope = MakeCommandEnvelope(drone_id, client_id, *command, priority);
            const bool verify_step = verify_commands || step.verify;
            const auto result = verify_step ? client.SendCommandAndWait(
                                                  envelope, StepWaitOptions(*wait_options, step))
                                            : client.SendCommand(envelope);
            std::string label = "step " + std::to_string(index) + " drone=" + drone_id;
            if (attempts > 1) {
                label += " attempt=" + std::to_string(attempt) + "/" + std::to_string(attempts);
            }
            step_ok = PrintCommandResult(label, result);
            if (step_ok || attempt == attempts) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{step.retry_delay_ms});
        }

        if (!step_ok) {
            ++failed_steps;
            if (!step.continue_on_error && !continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }
    }

    if (failed_steps > 0 && !continue_on_error) {
        std::cerr << "sequence completed with failed_steps=" << failed_steps << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "sequence OK steps=" << steps->size() << "\n";
    return EXIT_SUCCESS;
}

int RunCommand(Client& client, std::string_view drone_id, std::string_view client_id,
               CommandPriority priority, int argc, char** argv) {
    const auto kCommand = BuildCommandFromArgs(argc, argv);
    if (!kCommand.has_value()) {
        std::cerr << kCommand.error() << "\n";
        if (kCommand.error().starts_with("Unknown action:")) {
            std::cerr << "\n";
            PrintUsage();
        }
        return EXIT_FAILURE;
    }

    const auto envelope = MakeCommandEnvelope(drone_id, client_id, *kCommand, priority);
    const bool verify = common::HasFlag(argc, argv, "--verify");
    const auto wait_options = ParseCommandWaitOptions(argc, argv);
    if (!wait_options.has_value()) {
        std::cerr << wait_options.error() << "\n";
        return EXIT_FAILURE;
    }
    const auto kResult =
        verify ? client.SendCommandAndWait(envelope, *wait_options) : client.SendCommand(envelope);
    if (!kResult.ok) {
        std::cerr << "Command FAILED: " << kResult.message;
        if (!kResult.correlation_id.empty()) {
            std::cerr << " [corr=" << kResult.correlation_id << "]";
        }
        std::cerr << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Command OK" << (kResult.message.empty() ? "" : ": " + kResult.message) << "\n";
    return EXIT_SUCCESS;
}

int RunGoal(Client& client, std::string_view drone_id, int argc, char** argv) {
    const std::vector<std::string> actions = FindActionsAfterCommand(argc, argv, "goal");
    if (actions.empty()) {
        std::cerr << "goal requires set, cancel, or get\n";
        return EXIT_FAILURE;
    }

    if (actions[0] == "set") {
        const auto goal = ParseActiveGoal(drone_id, argc, argv);
        if (!goal.has_value()) {
            std::cerr << goal.error() << "\n";
            return EXIT_FAILURE;
        }
        const auto result = client.SetActiveGoal(*goal);
        if (!result.ok) {
            std::cerr << "Goal set FAILED: " << result.message;
            if (!result.correlation_id.empty()) {
                std::cerr << " [corr=" << result.correlation_id << "]";
            }
            std::cerr << "\n";
            return EXIT_FAILURE;
        }
        std::cout << "Goal set OK"
                  << " goal_id=" << result.goal.goal_id << " revision=" << result.goal.revision
                  << " computed_timeout_ms=" << result.computed_timeout_ms;
        if (!result.goal.labels.empty()) {
            std::cout << " labels=" << result.goal.labels.size();
        }
        std::cout << "\n";
        return EXIT_SUCCESS;
    }

    if (actions[0] == "cancel") {
        const auto result = client.CancelGoal(std::string(drone_id),
                                              common::GetOptionValue(argc, argv, "--goal-id"));
        if (!result.ok) {
            std::cerr << "Goal cancel FAILED: " << result.message;
            if (!result.correlation_id.empty()) {
                std::cerr << " [corr=" << result.correlation_id << "]";
            }
            std::cerr << "\n";
            return EXIT_FAILURE;
        }
        std::cout << "Goal cancel OK" << (result.message.empty() ? "" : ": " + result.message)
                  << "\n";
        return EXIT_SUCCESS;
    }

    if (actions[0] == "get") {
        const auto status = client.GetActiveGoal(std::string(drone_id));
        if (status.error.code != swarmkit::client::RpcStatusCode::kOk) {
            std::cerr << "Goal get FAILED: " << status.message << "\n";
            return EXIT_FAILURE;
        }
        if (!status.has_goal) {
            std::cout << "No active goal for drone=" << drone_id << "\n";
            return EXIT_SUCCESS;
        }
        std::cout << "Active goal\n"
                  << "  drone_id            : " << status.goal.drone_id << "\n"
                  << "  goal_id             : " << status.goal.goal_id << "\n"
                  << "  revision            : " << status.goal.revision << "\n"
                  << "  status              : " << GoalStatusName(status.status) << "\n"
                  << "  target_frame        : " << status.goal.target_frame << "\n"
                  << "  lat                 : " << status.goal.target.lat_deg << "\n"
                  << "  lon                 : " << status.goal.target.lon_deg << "\n"
                  << "  alt_m               : " << status.goal.target.alt_m << "\n"
                  << "  local_x_m           : " << status.goal.local_target.x_m << "\n"
                  << "  local_y_m           : " << status.goal.local_target.y_m << "\n"
                  << "  local_z_m           : " << status.goal.local_target.z_m << "\n"
                  << "  speed_mps           : " << status.goal.speed_mps << "\n"
                  << "  accept_radius_m     : " << status.goal.acceptance_radius_m << "\n"
                  << "  deviation_radius_m  : " << status.goal.deviation_radius_m << "\n"
                  << "  computed_timeout_ms : " << status.computed_timeout_ms << "\n";
        if (!status.goal.labels.empty()) {
            std::cout << "  labels              :\n";
            for (const auto& [key, value] : status.goal.labels) {
                std::cout << "    " << key << "=" << value << "\n";
            }
        }
        return EXIT_SUCCESS;
    }

    std::cerr << "Unknown goal action: " << actions[0] << "\n";
    return EXIT_FAILURE;
}

int RunReports(Client& client, std::string_view drone_id, int argc, char** argv) {
    const std::string format = common::GetOptionValue(argc, argv, "--format", "text");
    if (format != "text" && format != "jsonl") {
        std::cerr << "--format must be text or jsonl\n";
        return EXIT_FAILURE;
    }
    std::unique_ptr<std::ofstream> report_file;
    std::ostream* out = &std::cout;
    if (const std::string path = common::GetOptionValue(argc, argv, "--report-file");
        !path.empty()) {
        report_file = std::make_unique<std::ofstream>(path, std::ios::app);
        if (!report_file->is_open()) {
            std::cerr << "failed to open report file: " << path << "\n";
            return EXIT_FAILURE;
        }
        out = report_file.get();
    }

    const auto after_sequence = ParseIntArg(
        common::GetOptionValue(argc, argv, "--after-sequence", kDefaultZero), "--after-sequence");
    if (!after_sequence.has_value()) {
        std::cerr << after_sequence.error() << "\n";
        return EXIT_FAILURE;
    }
    if (*after_sequence < 0) {
        std::cerr << "--after-sequence must be >= 0\n";
        return EXIT_FAILURE;
    }

    ResetStopRequested();
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    swarmkit::client::ReportSubscription subscription;
    subscription.drone_id = std::string(drone_id);
    subscription.after_sequence = static_cast<std::uint64_t>(*after_sequence);
    auto report_stream = client.StartReports(
        subscription,
        [out, &format](const swarmkit::client::AgentReport& report) {
            if (format == "jsonl") {
                PrintReportJsonl(report, *out);
            } else {
                PrintReportText(report, *out);
            }
            out->flush();
        },
        [](const std::string& error_msg) {
            std::cerr << "Report stream error: " << error_msg << "\n";
        });
    if (!report_stream.has_value()) {
        std::cerr << "Failed to start report stream: " << report_stream.error().user_message
                  << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Subscribed to reports: drone=" << drone_id << " format=" << format << "\n"
              << "Press Ctrl+C to stop.\n";
    WaitForStop(ParseDurationMs(argc, argv).value_or(0));
    report_stream->Stop();
    std::cout << "\nStopped.\n";
    return EXIT_SUCCESS;
}

int RunMessage(Client& client, int argc, char** argv) {
    const auto actions = FindActionsAfterCommand(argc, argv, "message");
    if (actions.empty()) {
        std::cerr << "message requires publish|send|subscribe|peers\n";
        return EXIT_FAILURE;
    }

    if (actions[0] == "peers") {
        const bool refresh = common::HasFlag(argc, argv, "--refresh");
        const auto result = refresh ? client.RefreshDataPeers() : client.ListDataPeers(false);
        std::cout << "Data peers: " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        const auto state_name = [](swarmkit::client::DataPeerState state) {
            switch (state) {
                case swarmkit::client::DataPeerState::kReady:
                    return "ready";
                case swarmkit::client::DataPeerState::kUnreachable:
                    return "unreachable";
                case swarmkit::client::DataPeerState::kMisconfigured:
                    return "misconfigured";
                case swarmkit::client::DataPeerState::kUnknown:
                default:
                    return "unknown";
            }
        };
        for (const auto& peer : result.peers) {
            std::cout << "peer=" << peer.drone_id << " address=" << peer.address
                      << " security=" << peer.transport_security
                      << " state=" << state_name(peer.state)
                      << " rtt_ms=" << peer.round_trip_ms;
            if (!peer.message.empty()) {
                std::cout << " message=" << peer.message;
            }
            std::cout << "\n";
        }
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "publish" || actions[0] == "send") {
        const std::string topic = common::GetOptionValue(argc, argv, "--topic");
        if (topic.empty()) {
            std::cerr << "message " << actions[0] << " requires --topic TOPIC\n";
            return EXIT_FAILURE;
        }
        auto labels = ParseKeyValueLabels(argc, argv, "--label");
        if (!labels.has_value()) {
            std::cerr << labels.error() << "\n";
            return EXIT_FAILURE;
        }
        auto payload = ReadSmallPayload(argc, argv);
        if (!payload.has_value()) {
            std::cerr << payload.error() << "\n";
            return EXIT_FAILURE;
        }
        swarmkit::client::DataMessage message;
        message.topic = topic;
        message.target_id = common::GetOptionValue(argc, argv, "--target");
        const auto ttl_ms = ParseDurationMs(argc, argv);
        if (!ttl_ms.has_value()) {
            std::cerr << ttl_ms.error() << "\n";
            return EXIT_FAILURE;
        }
        message.ttl_ms = *ttl_ms;
        message.labels = std::move(*labels);
        message.payload = std::move(*payload);

        const bool routed_send = actions[0] == "send";
        if (routed_send && message.target_id.empty()) {
            std::cerr << "message send requires --target DRONE_ID\n";
            return EXIT_FAILURE;
        }
        const auto result =
            routed_send ? client.SendMessageToDrone(std::move(message))
                        : client.PublishMessage(std::move(message));
        std::cout << (routed_send ? "Message send: " : "Message publish: ")
                  << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (result.sequence > 0) {
            std::cout << " seq=" << result.sequence;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "subscribe") {
        const auto duration_ms = ParseDurationMs(argc, argv);
        if (!duration_ms.has_value()) {
            std::cerr << duration_ms.error() << "\n";
            return EXIT_FAILURE;
        }
        const auto after_sequence =
            ParseIntArg(common::GetOptionValue(argc, argv, "--after-sequence", kDefaultZero),
                        "--after-sequence");
        if (!after_sequence.has_value() || *after_sequence < 0) {
            std::cerr << "Invalid --after-sequence\n";
            return EXIT_FAILURE;
        }

        ResetStopRequested();
        std::signal(SIGINT, OnSignal);
        std::signal(SIGTERM, OnSignal);

        swarmkit::client::MessageSubscription subscription;
        subscription.target_id = common::GetOptionValue(argc, argv, "--target");
        subscription.topics = CollectOptionValues(argc, argv, "--topic");
        subscription.after_sequence = static_cast<std::uint64_t>(*after_sequence);
        if (subscription.target_id.empty()) {
            std::cerr << "Note: no --target set; targeted messages for another id may not appear.\n";
        }

        std::mutex output_mutex;
        auto stream = client.StartMessages(
            std::move(subscription),
            [&](const swarmkit::client::DataMessage& message) {
                std::lock_guard<std::mutex> lock(output_mutex);
                std::cout << "MESSAGE seq=" << message.sequence << " topic=" << message.topic
                          << " source=" << message.source_id;
                if (!message.target_id.empty()) {
                    std::cout << " target=" << message.target_id;
                }
                if (!message.message_id.empty()) {
                    std::cout << " id=" << message.message_id;
                }
                std::cout << " payload=" << message.payload << "\n";
            },
            [](const std::string& error) { std::cerr << "message stream error: " << error << "\n"; });
        if (!stream.has_value()) {
            std::cerr << "Message subscribe failed: " << stream.error().user_message << "\n";
            return EXIT_FAILURE;
        }

        const auto start = std::chrono::steady_clock::now();
        while (!IsStopRequested()) {
            if (*duration_ms > 0 &&
                std::chrono::steady_clock::now() - start >= std::chrono::milliseconds(*duration_ms)) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
        }
        stream->Stop();
        return EXIT_SUCCESS;
    }

    std::cerr << "Unknown message action: " << actions[0] << "\n";
    return EXIT_FAILURE;
}

int RunArtifact(Client& client, int argc, char** argv) {
    const auto actions = FindActionsAfterCommand(argc, argv, "artifact");
    if (actions.empty()) {
        std::cerr << "artifact requires upload|send|start|status|cancel|list|info|download|announce\n";
        return EXIT_FAILURE;
    }

    const auto print_descriptor = [](const swarmkit::client::ArtifactDescriptor& descriptor,
                                     std::string_view indent) {
        std::cout << indent << "artifact_id : " << descriptor.artifact_id << "\n"
                  << indent << "source_id   : " << descriptor.source_id << "\n"
                  << indent << "target_id   : " << descriptor.target_id << "\n"
                  << indent << "content_type: " << descriptor.content_type << "\n"
                  << indent << "size_bytes  : " << descriptor.size_bytes << "\n"
                  << indent << "created_ms  : " << descriptor.created_unix_ms << "\n"
                  << indent << "ttl_ms      : " << descriptor.ttl_ms << "\n"
                  << indent << "sha256      : " << descriptor.sha256_hex << "\n"
                  << indent << "filename    : " << descriptor.filename << "\n";
        if (!descriptor.labels.empty()) {
            std::cout << indent << "labels:\n";
            for (const auto& [key, value] : descriptor.labels) {
                std::cout << indent << "  " << key << "=" << value << "\n";
            }
        }
    };
    const auto transfer_state_name = [](swarmkit::client::ArtifactTransferState state) {
        switch (state) {
            case swarmkit::client::ArtifactTransferState::kQueued:
                return "queued";
            case swarmkit::client::ArtifactTransferState::kRunning:
                return "running";
            case swarmkit::client::ArtifactTransferState::kCompleted:
                return "completed";
            case swarmkit::client::ArtifactTransferState::kFailed:
                return "failed";
            case swarmkit::client::ArtifactTransferState::kCancelled:
                return "cancelled";
            case swarmkit::client::ArtifactTransferState::kUnknown:
            default:
                return "unknown";
        }
    };
    const auto print_transfer = [&](std::string_view prefix,
                                    const swarmkit::client::ArtifactTransferStatusResult& result) {
        std::cout << prefix << ": " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.transfer.transfer_id.empty()) {
            std::cout << " transfer_id=" << result.transfer.transfer_id
                      << " state=" << transfer_state_name(result.transfer.state)
                      << " bytes=" << result.transfer.bytes_transferred << "/"
                      << result.transfer.bytes_total;
        }
        if (!result.transfer.descriptor.artifact_id.empty()) {
            std::cout << " artifact_id=" << result.transfer.descriptor.artifact_id;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
    };

    if (actions[0] == "upload" || actions[0] == "send") {
        const std::string file_path = common::GetOptionValue(argc, argv, "--file");
        if (file_path.empty()) {
            std::cerr << "artifact " << actions[0] << " requires --file PATH\n";
            return EXIT_FAILURE;
        }
        auto labels = ParseKeyValueLabels(argc, argv, "--label");
        if (!labels.has_value()) {
            std::cerr << labels.error() << "\n";
            return EXIT_FAILURE;
        }
        swarmkit::client::ArtifactUpload upload;
        upload.file_path = file_path;
        upload.descriptor.target_id = common::GetOptionValue(argc, argv, "--target");
        upload.descriptor.content_type =
            common::GetOptionValue(argc, argv, "--content-type", "application/octet-stream");
        const auto ttl_ms = ParseDurationMs(argc, argv);
        if (!ttl_ms.has_value()) {
            std::cerr << ttl_ms.error() << "\n";
            return EXIT_FAILURE;
        }
        upload.descriptor.ttl_ms = *ttl_ms;
        upload.descriptor.labels = std::move(*labels);
        const bool routed_send = actions[0] == "send";
        if (routed_send && upload.descriptor.target_id.empty()) {
            std::cerr << "artifact send requires --target DRONE_ID\n";
            return EXIT_FAILURE;
        }
        if (const std::string chunk_bytes = common::GetOptionValue(argc, argv, "--chunk-bytes");
            !chunk_bytes.empty()) {
            const auto parsed = ParseIntArg(chunk_bytes, "--chunk-bytes");
            if (!parsed.has_value() || *parsed <= 0) {
                std::cerr << "Invalid --chunk-bytes\n";
                return EXIT_FAILURE;
            }
            upload.chunk_bytes = static_cast<std::size_t>(*parsed);
        }

        const auto result =
            routed_send ? client.SendArtifactToDrone(upload) : client.UploadArtifact(upload);
        std::cout << (routed_send ? "Artifact send: " : "Artifact upload: ")
                  << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.descriptor.artifact_id.empty()) {
            std::cout << " artifact_id=" << result.descriptor.artifact_id;
        }
        if (!result.descriptor.sha256_hex.empty()) {
            std::cout << " sha256=" << result.descriptor.sha256_hex;
        }
        if (result.ok || result.descriptor.size_bytes > 0) {
            std::cout << " size=" << result.descriptor.size_bytes;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "start") {
        const std::string file_path = common::GetOptionValue(argc, argv, "--file");
        if (file_path.empty()) {
            std::cerr << "artifact start requires --file PATH\n";
            return EXIT_FAILURE;
        }
        auto labels = ParseKeyValueLabels(argc, argv, "--label");
        if (!labels.has_value()) {
            std::cerr << labels.error() << "\n";
            return EXIT_FAILURE;
        }
        swarmkit::client::ArtifactUpload upload;
        upload.file_path = file_path;
        upload.descriptor.target_id = common::GetOptionValue(argc, argv, "--target");
        upload.descriptor.content_type =
            common::GetOptionValue(argc, argv, "--content-type", "application/octet-stream");
        const auto ttl_ms = ParseDurationMs(argc, argv);
        if (!ttl_ms.has_value()) {
            std::cerr << ttl_ms.error() << "\n";
            return EXIT_FAILURE;
        }
        upload.descriptor.ttl_ms = *ttl_ms;
        upload.descriptor.labels = std::move(*labels);
        if (const std::string chunk_bytes = common::GetOptionValue(argc, argv, "--chunk-bytes");
            !chunk_bytes.empty()) {
            const auto parsed = ParseIntArg(chunk_bytes, "--chunk-bytes");
            if (!parsed.has_value() || *parsed <= 0) {
                std::cerr << "Invalid --chunk-bytes\n";
                return EXIT_FAILURE;
            }
            upload.chunk_bytes = static_cast<std::size_t>(*parsed);
        }
        const bool route = common::HasFlag(argc, argv, "--route") ||
                           !upload.descriptor.target_id.empty();
        if (route && upload.descriptor.target_id.empty()) {
            std::cerr << "artifact start --route requires --target DRONE_ID\n";
            return EXIT_FAILURE;
        }
        const auto result = client.StartArtifactTransfer(upload, route);
        print_transfer("Artifact transfer start", result);
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "status" || actions[0] == "cancel") {
        const std::string transfer_id = common::GetOptionValue(argc, argv, "--transfer-id");
        if (transfer_id.empty()) {
            std::cerr << "artifact " << actions[0] << " requires --transfer-id ID\n";
            return EXIT_FAILURE;
        }
        const auto result = actions[0] == "cancel" ? client.CancelArtifactTransfer(transfer_id)
                                                   : client.GetArtifactTransfer(transfer_id);
        print_transfer(actions[0] == "cancel" ? "Artifact transfer cancel"
                                              : "Artifact transfer status",
                       result);
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "list") {
        std::string source_id = common::GetOptionValue(argc, argv, "--source-id");
        if (source_id.empty()) {
            source_id = common::GetOptionValue(argc, argv, "--source");
        }
        const std::string target_id = common::GetOptionValue(argc, argv, "--target");
        const bool include_expired = common::HasFlag(argc, argv, "--include-expired");
        const auto result = client.ListArtifacts(source_id, target_id, include_expired);
        std::cout << "Artifact list: " << (result.ok ? "OK" : "FAILED")
                  << " count=" << result.artifacts.size();
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        for (const auto& descriptor : result.artifacts) {
            std::cout << "  " << descriptor.artifact_id << " source=" << descriptor.source_id
                      << " target=" << descriptor.target_id
                      << " content_type=" << descriptor.content_type
                      << " size=" << descriptor.size_bytes
                      << " sha256=" << descriptor.sha256_hex
                      << " file=" << descriptor.filename << "\n";
        }
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "info") {
        const std::string artifact_id = common::GetOptionValue(argc, argv, "--artifact-id");
        if (artifact_id.empty()) {
            std::cerr << "artifact info requires --artifact-id ID\n";
            return EXIT_FAILURE;
        }
        const auto result = client.GetArtifact(artifact_id);
        std::cout << "Artifact info: " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        if (result.ok) {
            print_descriptor(result.descriptor, "  ");
        }
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "download") {
        const std::string artifact_id = common::GetOptionValue(argc, argv, "--artifact-id");
        const std::string file_path = common::GetOptionValue(argc, argv, "--file");
        if (artifact_id.empty() || file_path.empty()) {
            std::cerr << "artifact download requires --artifact-id ID --file PATH\n";
            return EXIT_FAILURE;
        }
        const auto result = client.DownloadArtifact(artifact_id, file_path);
        std::cout << "Artifact download: " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.descriptor.artifact_id.empty()) {
            std::cout << " artifact_id=" << result.descriptor.artifact_id;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "announce") {
        const std::string artifact_id = common::GetOptionValue(argc, argv, "--artifact-id");
        if (artifact_id.empty()) {
            std::cerr << "artifact announce requires --artifact-id ID\n";
            return EXIT_FAILURE;
        }
        auto labels = ParseKeyValueLabels(argc, argv, "--label");
        if (!labels.has_value()) {
            std::cerr << labels.error() << "\n";
            return EXIT_FAILURE;
        }
        swarmkit::client::ArtifactDescriptor descriptor;
        descriptor.artifact_id = artifact_id;
        descriptor.target_id = common::GetOptionValue(argc, argv, "--target");
        descriptor.content_type =
            common::GetOptionValue(argc, argv, "--content-type", "application/octet-stream");
        descriptor.filename = common::GetOptionValue(argc, argv, "--file");
        const auto ttl_ms = ParseDurationMs(argc, argv);
        if (!ttl_ms.has_value()) {
            std::cerr << ttl_ms.error() << "\n";
            return EXIT_FAILURE;
        }
        descriptor.ttl_ms = *ttl_ms;
        descriptor.labels = std::move(*labels);
        const auto result = client.AnnounceArtifact(std::move(descriptor));
        std::cout << "Artifact announce: " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    std::cerr << "Unknown artifact action: " << actions[0] << "\n";
    return EXIT_FAILURE;
}

int RunPing(Client& client) {
    const auto kResult = client.Ping();

    if (!kResult.ok) {
        std::cerr << "Ping FAILED: " << kResult.error_message;
        if (!kResult.correlation_id.empty()) {
            std::cerr << " [corr=" << kResult.correlation_id << "]";
        }
        std::cerr << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Ping OK\n"
              << "  agent_id  : " << kResult.agent_id << "\n"
              << "  version   : " << kResult.version << "\n"
              << "  time_ms   : " << kResult.unix_time_ms << "\n";
    return EXIT_SUCCESS;
}

struct TelemetrySampleResult {
    std::optional<swarmkit::core::TelemetryFrame> frame;
    std::string error;
};

[[nodiscard]] TelemetrySampleResult CollectTelemetrySample(
    Client& client, std::string_view drone_id, std::int64_t timeout_ms) {
    std::mutex mutex;
    std::condition_variable cv;
    std::optional<swarmkit::core::TelemetryFrame> latest_frame;
    std::string stream_error;

    auto telemetry_stream = client.StartTelemetry(
        {.drone_id = std::string(drone_id), .rate_hertz = 2},
        [&](const swarmkit::core::TelemetryFrame& frame) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                latest_frame = frame;
            }
            cv.notify_all();
        },
        [&](const std::string& error_msg) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                stream_error = error_msg;
            }
            std::cerr << "Preflight telemetry stream error: " << error_msg << "\n";
            cv.notify_all();
        });
    if (!telemetry_stream.has_value()) {
        std::cerr << "Preflight telemetry FAILED: " << telemetry_stream.error().user_message
                  << "\n";
        return {.error = telemetry_stream.error().user_message};
    }

    std::unique_lock<std::mutex> lock(mutex);
    cv.wait_for(lock, std::chrono::milliseconds{timeout_ms},
                [&] { return latest_frame.has_value() || !stream_error.empty(); });
    TelemetrySampleResult result{.frame = latest_frame, .error = stream_error};
    lock.unlock();
    telemetry_stream->Stop();
    return result;
}

[[nodiscard]] std::expected<float, std::string> ParsePreflightMinBattery(int argc, char** argv) {
    const auto min_battery =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--min-battery", "20"),
                      "--min-battery");
    if (!min_battery.has_value()) {
        return std::unexpected(min_battery.error());
    }
    if (*min_battery < 0.0F || *min_battery > 100.0F) {
        return std::unexpected("--min-battery must be between 0 and 100");
    }
    return *min_battery;
}

void PrintPreflightCheck(bool ok, std::string_view name, std::string_view detail) {
    std::cout << "  [" << (ok ? "OK" : "FAIL") << "] " << name;
    if (!detail.empty()) {
        std::cout << " - " << detail;
    }
    std::cout << "\n";
}

int RunPreflight(Client& client, std::string_view drone_id, int argc, char** argv) {
    const auto timeout_ms = ParseDurationMs(argc, argv);
    if (!timeout_ms.has_value()) {
        std::cerr << timeout_ms.error() << "\n";
        return EXIT_FAILURE;
    }
    const std::int64_t sample_timeout_ms = *timeout_ms == 0 ? 3000 : *timeout_ms;
    const auto min_battery = ParsePreflightMinBattery(argc, argv);
    if (!min_battery.has_value()) {
        std::cerr << min_battery.error() << "\n";
        return EXIT_FAILURE;
    }

    const auto status = client.GetHealth();
    const std::int64_t now_ms = NowUnixMs();
    const bool heartbeat_ok =
        status.ok && !IsStaleTimestamp(status.last_heartbeat_unix_ms, now_ms);
    const bool telemetry_ok =
        status.ok && !IsStaleTimestamp(status.last_telemetry_unix_ms, now_ms);
    const auto telemetry = CollectTelemetrySample(client, drone_id, sample_timeout_ms);

    bool ready = true;
    std::cout << "Preflight " << drone_id << "\n";

    const bool agent_ok = status.ok && status.ready;
    ready = ready && agent_ok;
    PrintPreflightCheck(agent_ok, "agent",
                        status.ok ? status.message : status.error.user_message);

    ready = ready && heartbeat_ok;
    PrintPreflightCheck(heartbeat_ok, "MAVLink heartbeat",
                        "age_ms=" + TimestampAgeMsText(status.last_heartbeat_unix_ms, now_ms));

    ready = ready && telemetry_ok && telemetry.frame.has_value();
    PrintPreflightCheck(telemetry_ok && telemetry.frame.has_value(), "telemetry",
                        telemetry.error.empty()
                            ? "age_ms=" +
                                  TimestampAgeMsText(status.last_telemetry_unix_ms, now_ms)
                            : telemetry.error);

    ready = ready && !status.armed;
    PrintPreflightCheck(!status.armed, "armed state",
                        status.armed ? "vehicle is already armed" : "disarmed");

    ready = ready && status.landed;
    PrintPreflightCheck(status.landed, "landed state",
                        std::string{"landed="} + BoolText(status.landed) +
                            " rel_alt=" + RelativeAltitudeText(status));

    ready = ready && !status.failsafe;
    PrintPreflightCheck(!status.failsafe, "failsafe", BoolText(status.failsafe));

    ready = ready && status.ekf_ok;
    PrintPreflightCheck(status.ekf_ok, "EKF", status.ekf_ok ? "healthy" : "unhealthy");

    const std::string gps_detail = "fix=" + std::to_string(status.gps_fix_type) + " (" +
                                   GpsFixText(status.gps_fix_type) + ") sats=" +
                                   std::to_string(status.satellites_visible) +
                                   " hdop=" + FloatText(status.gps_hdop);
    ready = ready && status.gps_ok;
    PrintPreflightCheck(status.gps_ok, "GPS", gps_detail);

    bool battery_ok = false;
    std::string battery_detail = "unknown";
    if (telemetry.frame.has_value() && telemetry.frame->HasBattery()) {
        battery_ok = telemetry.frame->battery_percent >= *min_battery;
        battery_detail = FloatText(telemetry.frame->battery_percent, 1) + "% minimum=" +
                         FloatText(*min_battery, 1) + "%";
    }
    ready = ready && battery_ok;
    PrintPreflightCheck(battery_ok, "battery", battery_detail);

    const std::string mode_text = status.mode.empty() ? "unknown" : status.mode;
    PrintPreflightCheck(true, "mode", mode_text + " custom_mode=" +
                                          std::to_string(status.custom_mode));

    if (!status.gps_ok) {
        std::cout << "  guidance: GPS is not ready. Normal GPS modes like GUIDED/takeoff may "
                     "reject arming or takeoff. Use emergency force-arm only for bench/no-prop "
                     "tests.\n";
    }
    if (ready) {
        std::cout << "Preflight OK: drone appears ready for normal flight commands.\n";
        return EXIT_SUCCESS;
    }
    std::cout << "Preflight FAILED: fix the failed checks before flight.\n";
    return EXIT_FAILURE;
}

int RunHealth(Client& client) {
    const auto kStatus = client.GetHealth();
    if (!kStatus.ok) {
        std::cerr << "Health FAILED: " << kStatus.message;
        if (!kStatus.correlation_id.empty()) {
            std::cerr << " [corr=" << kStatus.correlation_id << "]";
        }
        std::cerr << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Health OK\n"
              << "  ready                  : " << (kStatus.ready ? "true" : "false") << "\n"
              << "  agent_id               : " << kStatus.agent_id << "\n"
              << "  version                : " << kStatus.version << "\n"
              << "  time_ms                : " << kStatus.unix_time_ms << "\n"
              << "  backend_name           : " << kStatus.backend_name << "\n"
              << "  protocol               : " << kStatus.protocol << "\n"
              << "  last_heartbeat_unix_ms : " << kStatus.last_heartbeat_unix_ms << "\n"
              << "  last_telemetry_unix_ms : " << kStatus.last_telemetry_unix_ms << "\n"
              << "  armed                  : " << (kStatus.armed ? "true" : "false") << "\n"
              << "  landed                 : " << (kStatus.landed ? "true" : "false") << "\n"
              << "  mode                   : "
              << (kStatus.mode.empty() ? "unknown" : kStatus.mode) << "\n"
              << "  custom_mode            : " << kStatus.custom_mode << "\n"
              << "  failsafe               : " << (kStatus.failsafe ? "true" : "false") << "\n"
              << "  gps_ok                 : " << (kStatus.gps_ok ? "true" : "false") << "\n"
              << "  gps_fix                : " << kStatus.gps_fix_type << " ("
              << GpsFixText(kStatus.gps_fix_type) << ")\n"
              << "  satellites_visible     : " << kStatus.satellites_visible << "\n"
              << "  gps_hdop               : " << FloatText(kStatus.gps_hdop) << "\n"
              << "  ekf_ok                 : " << (kStatus.ekf_ok ? "true" : "false") << "\n"
              << "  relative_altitude      : " << RelativeAltitudeText(kStatus) << "\n"
              << "  autonomous_ready       : "
              << (kStatus.autonomous_ready ? "true" : "false") << "\n"
              << "  link_quality_percent   : "
              << OptionalFloatText(kStatus.link_quality_percent) << "\n"
              << "  message                : " << kStatus.message << "\n";
    if (!kStatus.arming_blockers.empty()) {
        std::cout << "  arming_blockers        : ";
        for (std::size_t index = 0; index < kStatus.arming_blockers.size(); ++index) {
            if (index > 0) {
                std::cout << ", ";
            }
            std::cout << kStatus.arming_blockers[index];
        }
        std::cout << "\n";
    }
    if (!kStatus.readiness_checks.empty()) {
        std::cout << "  readiness_checks       :\n";
        for (const auto& check : kStatus.readiness_checks) {
            std::cout << "    [" << (check.ok ? "OK" : "FAIL") << "] " << check.name
                      << " severity=" << ReadinessSeverityText(check.severity);
            if (!check.detail.empty()) {
                std::cout << " detail=" << check.detail;
            }
            std::cout << "\n";
        }
    }
    return EXIT_SUCCESS;
}

int RunStats(Client& client) {
    const auto kStats = client.GetRuntimeStats();
    if (!kStats.ok) {
        std::cerr << "Stats FAILED: " << kStats.error.user_message;
        if (!kStats.correlation_id.empty()) {
            std::cerr << " [corr=" << kStats.correlation_id << "]";
        }
        std::cerr << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Runtime Stats\n"
              << "  agent_id                    : " << kStats.agent_id << "\n"
              << "  ready                       : " << (kStats.ready ? "true" : "false") << "\n"
              << "  ping_requests_total         : " << kStats.ping_requests_total << "\n"
              << "  health_requests_total       : " << kStats.health_requests_total << "\n"
              << "  stats_requests_total        : " << kStats.runtime_stats_requests_total << "\n"
              << "  command_requests_total      : " << kStats.command_requests_total << "\n"
              << "  command_rejected_total      : " << kStats.command_rejected_total << "\n"
              << "  command_failed_total        : " << kStats.command_failed_total << "\n"
              << "  lock_requests_total         : " << kStats.lock_requests_total << "\n"
              << "  watch_requests_total        : " << kStats.watch_requests_total << "\n"
              << "  current_authority_watchers  : " << kStats.current_authority_watchers << "\n"
              << "  total_telemetry_subs        : " << kStats.total_telemetry_subscriptions << "\n"
              << "  current_telemetry_streams   : " << kStats.current_telemetry_streams << "\n"
              << "  telemetry_frames_sent_total : " << kStats.telemetry_frames_sent_total << "\n"
              << "  backend_failures_total      : " << kStats.backend_failures_total << "\n"
              << "  data_messages_published     : " << kStats.data_messages_published_total
              << "\n"
              << "  data_messages_rejected      : " << kStats.data_messages_rejected_total << "\n"
              << "  current_message_subscribers : " << kStats.current_message_subscribers << "\n"
              << "  artifact_uploads_total      : " << kStats.artifact_uploads_total << "\n"
              << "  artifact_downloads_total    : " << kStats.artifact_downloads_total << "\n"
              << "  artifact_bytes_received     : " << kStats.artifact_bytes_received_total << "\n"
              << "  artifact_bytes_sent         : " << kStats.artifact_bytes_sent_total << "\n"
              << "  artifact_failures_total     : " << kStats.artifact_failures_total << "\n";
    return EXIT_SUCCESS;
}

int RunCapabilities(Client& client) {
    const auto capabilities = client.GetCapabilities();
    if (!capabilities.ok) {
        std::cerr << "Capabilities FAILED: " << capabilities.error.user_message;
        if (!capabilities.correlation_id.empty()) {
            std::cerr << " [corr=" << capabilities.correlation_id << "]";
        }
        std::cerr << "\n";
        return EXIT_FAILURE;
    }

    const auto print_list = [](std::string_view label, const std::vector<std::string>& values) {
        std::cout << "  " << std::left << std::setw(29) << label << ": ";
        for (std::size_t idx = 0; idx < values.size(); ++idx) {
            if (idx != 0) {
                std::cout << ", ";
            }
            std::cout << values[idx];
        }
        std::cout << "\n";
    };

    std::cout << "Backend Capabilities\n"
              << "  agent_id                    : " << capabilities.agent_id << "\n"
              << "  backend_name                : " << capabilities.backend_name << "\n"
              << "  protocol                    : " << capabilities.protocol << "\n"
              << "  vehicle_class               : " << capabilities.vehicle_class << "\n"
              << "  autopilot_type              : " << capabilities.autopilot_type << "\n"
              << "  supports_payload_control    : "
              << (capabilities.supports_payload_control ? "true" : "false") << "\n"
              << "  supports_velocity_control   : "
              << (capabilities.supports_velocity_control ? "true" : "false") << "\n"
              << "  supports_flight_termination : "
              << (capabilities.supports_flight_termination ? "true" : "false") << "\n"
              << "  supports_backend_commands   : "
              << (capabilities.supports_backend_commands ? "true" : "false") << "\n"
              << "  max_horizontal_speed_mps    : "
              << OptionalFloatText(capabilities.max_horizontal_speed_mps) << "\n"
              << "  max_climb_speed_mps         : "
              << OptionalFloatText(capabilities.max_climb_speed_mps) << "\n"
              << "  max_descent_speed_mps       : "
              << OptionalFloatText(capabilities.max_descent_speed_mps) << "\n"
              << "  max_altitude_m              : "
              << OptionalFloatText(capabilities.max_altitude_m) << "\n";
    print_list("supported_modes", capabilities.supported_modes);
    print_list("supported_commands", capabilities.supported_commands);
    print_list("supported_payloads", capabilities.supported_payloads);
    print_list("supported_telemetry_fields", capabilities.supported_telemetry_fields);
    print_list("backend_command_names", capabilities.backend_command_names);
    return EXIT_SUCCESS;
}

[[nodiscard]] std::expected<int, std::string> ParseTelemetryRate(int argc, char** argv) {
    try {
        return std::stoi(common::GetOptionValue(argc, argv, "--rate", kDefaultTelemetryRate));
    } catch (const std::exception& exc) {
        return std::unexpected("Invalid --rate value '" +
                               common::GetOptionValue(argc, argv, "--rate", kDefaultTelemetryRate) +
                               "': " + exc.what());
    }
}

int RunTelemetry(Client& client, std::string_view drone_id, int rate_hz, int argc, char** argv) {
    auto sink = TelemetrySink::FromArgs(argc, argv, false);
    if (!sink.has_value()) {
        std::cerr << sink.error() << "\n";
        return EXIT_FAILURE;
    }
    auto telemetry_sink = std::move(*sink);
    const auto duration_ms = ParseDurationMs(argc, argv);
    if (!duration_ms.has_value()) {
        std::cerr << duration_ms.error() << "\n";
        return EXIT_FAILURE;
    }

    ResetStopRequested();
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    std::cout << "Subscribing to telemetry: drone=" << drone_id << " rate=" << rate_hz << " Hz\n"
              << "Press Ctrl+C to stop.\n\n";

    swarmkit::client::TelemetrySubscription subscription;
    subscription.drone_id = std::string(drone_id);
    subscription.rate_hertz = rate_hz;

    if (telemetry_sink->WritesFiles()) {
        std::cout << "Telemetry CSV logging enabled.\n";
    }

    std::atomic<bool> stream_failed{false};
    std::string stream_error;
    std::mutex stream_error_mutex;
    auto telemetry_stream = client.StartTelemetry(
        subscription,
        [&telemetry_sink](const swarmkit::core::TelemetryFrame& frame) {
            telemetry_sink->Write(frame);
        },
        [&](const std::string& error_msg) {
            {
                std::lock_guard<std::mutex> lock(stream_error_mutex);
                stream_error = error_msg;
            }
            stream_failed.store(true, std::memory_order_relaxed);
            std::cerr << "Telemetry stream error: " << error_msg << "\n";
        });
    if (!telemetry_stream.has_value()) {
        std::cerr << "Failed to start telemetry stream: " << telemetry_stream.error().user_message
                  << "\n";
        return EXIT_FAILURE;
    }

    WaitForStop(*duration_ms);

    telemetry_stream->Stop();
    std::cout << "\nStopped.\n";
    if (stream_failed.load(std::memory_order_relaxed)) {
        std::lock_guard<std::mutex> lock(stream_error_mutex);
        std::cerr << "Telemetry FAILED: " << stream_error << "\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

[[nodiscard]] std::expected<std::int64_t, std::string> ParseTtlMs(int argc, char** argv) {
    const auto ttl =
        ParseIntArg(common::GetOptionValue(argc, argv, "--ttl-ms", kDefaultZero), "--ttl-ms");
    if (!ttl.has_value()) {
        return std::unexpected(ttl.error());
    }
    return static_cast<std::int64_t>(*ttl);
}

int RunLock(Client& client, std::string_view drone_id, int argc, char** argv) {
    const auto ttl_ms = ParseTtlMs(argc, argv);
    if (!ttl_ms.has_value()) {
        std::cerr << ttl_ms.error() << "\n";
        return EXIT_FAILURE;
    }
    const auto result = client.LockAuthority(std::string(drone_id), *ttl_ms);
    if (!result.ok) {
        std::cerr << "Lock FAILED: " << result.message << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "Lock OK" << (result.message.empty() ? "" : ": " + result.message) << "\n";
    return EXIT_SUCCESS;
}

int RunUnlock(Client& client, std::string_view drone_id) {
    const auto result = client.ReleaseAuthority(std::string(drone_id));
    return PrintReleaseAuthorityResult("Unlock", result) ? EXIT_SUCCESS : EXIT_FAILURE;
}

int RunWatchAuthority(Client& client, std::string_view drone_id, CommandPriority priority) {
    ResetStopRequested();
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    swarmkit::client::AuthoritySubscription subscription;
    subscription.drone_id = std::string(drone_id);
    subscription.priority = priority;

    auto authority_watch = client.StartAuthorityWatch(
        subscription,
        [](const swarmkit::client::AuthorityEventInfo& event) {
            std::cout << "authority event drone=" << event.drone_id
                      << " holder=" << event.holder_client_id
                      << " priority=" << static_cast<int>(event.holder_priority)
                      << " kind=" << static_cast<int>(event.kind) << "\n";
        },
        [](const std::string& error_msg) {
            std::cerr << "Authority stream error: " << error_msg << "\n";
        });
    if (!authority_watch.has_value()) {
        std::cerr << "Failed to start authority watch: " << authority_watch.error().user_message
                  << "\n";
        return EXIT_FAILURE;
    }

    while (!IsStopRequested()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kTelemetryPollIntervalMs));
    }
    authority_watch->Stop();
    std::cout << "\nStopped.\n";
    return EXIT_SUCCESS;
}

[[nodiscard]] std::expected<SwarmRuntime, int> BuildSwarmRuntime(const ClientConfig& client_cfg,
                                                                 int argc, char** argv) {
    const std::string config_path = common::GetOptionValue(argc, argv, "--swarm-config");
    if (config_path.empty()) {
        std::cerr << "swarm requires --swarm-config PATH\n";
        return std::unexpected(EXIT_FAILURE);
    }
    auto loaded = swarmkit::client::LoadSwarmConfigFromFile(config_path);
    if (!loaded.has_value()) {
        std::cerr << "Failed to load swarm config '" << config_path
                  << "': " << loaded.error().message << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    loaded->default_client_config = client_cfg;

    SwarmRuntime runtime;
    runtime.drone_ids.reserve(loaded->drones.size());
    for (const auto& drone : loaded->drones) {
        runtime.drone_ids.push_back(drone.drone_id);
    }
    runtime.client = std::make_unique<swarmkit::client::SwarmClient>(client_cfg);
    runtime.client_id = client_cfg.client_id;
    const std::string address_mode =
        common::GetOptionValue(argc, argv, "--address-mode", kDefaultAddressMode);
    const auto preference = address_mode == "local"
                                ? swarmkit::client::SwarmAddressPreference::kPreferLocal
                                : swarmkit::client::SwarmAddressPreference::kPrimary;
    if (const auto result = runtime.client->ApplyConfig(*loaded, preference); !result.IsOk()) {
        std::cerr << "Invalid swarm config: " << result.message << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    return runtime;
}

[[nodiscard]] SwarmResultSummary SummarizeSwarmResults(
    const std::unordered_map<std::string, swarmkit::client::CommandResult>& results) {
    SwarmResultSummary summary;
    summary.sent = static_cast<int>(results.size());
    for (const auto& [drone_id, result] : results) {
        static_cast<void>(drone_id);
        if (IsAlreadySatisfied(result)) {
            ++summary.already_satisfied;
            ++summary.succeeded;
        } else if (result.ok) {
            ++summary.accepted;
            ++summary.succeeded;
        } else {
            ++summary.failed;
        }
    }
    return summary;
}

[[nodiscard]] bool SwarmResultAccepted(const SwarmResultSummary& summary,
                                       const SwarmResultPolicy& policy) {
    if (summary.sent == 0) {
        return false;
    }
    if (policy.require_all) {
        return summary.failed == 0 && summary.succeeded == summary.sent;
    }
    if (policy.continue_on_error) {
        return summary.succeeded > 0;
    }
    return summary.failed == 0 && summary.succeeded == summary.sent;
}

[[nodiscard]] bool PrintSwarmResults(
    const std::unordered_map<std::string, swarmkit::client::CommandResult>& results,
    const SwarmResultPolicy& policy) {
    for (const auto& [drone_id, result] : results) {
        std::cout << drone_id << ": ";
        if (IsAlreadySatisfied(result)) {
            std::cout << "ALREADY_SATISFIED";
        } else {
            std::cout << (result.ok ? "OK" : "FAILED");
        }
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
    }
    const SwarmResultSummary summary = SummarizeSwarmResults(results);
    std::cout << "summary: sent=" << summary.sent << " succeeded=" << summary.succeeded
              << " accepted=" << summary.accepted
              << " already_satisfied=" << summary.already_satisfied << " failed=" << summary.failed
              << "\n";
    return SwarmResultAccepted(summary, policy);
}

[[nodiscard]] bool PrintSwarmReleaseResults(
    const std::unordered_map<std::string, swarmkit::client::ReleaseAuthorityResult>& results,
    const SwarmResultPolicy& policy) {
    SwarmResultSummary summary;
    summary.sent = static_cast<int>(results.size());
    for (const auto& [drone_id, result] : results) {
        std::cout << drone_id << ": " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        if (result.ok) {
            ++summary.accepted;
            ++summary.succeeded;
        } else {
            ++summary.failed;
        }
    }
    std::cout << "summary: sent=" << summary.sent << " succeeded=" << summary.succeeded
              << " released=" << summary.accepted << " failed=" << summary.failed << "\n";
    return SwarmResultAccepted(summary, policy);
}

[[nodiscard]] std::vector<std::string> FindSwarmActions(int argc, char** argv) {
    int swarm_index = -1;
    for (int index = 1; index < argc; ++index) {
        const std::string_view arg = argv[index];
        if (IsOptionWithValue(arg)) {
            ++index;
            continue;
        }
        if (arg == "swarm") {
            swarm_index = index;
            break;
        }
    }
    if (swarm_index < 0) {
        return {};
    }
    std::vector<std::string> actions;
    for (int index = swarm_index + 1; index < argc; ++index) {
        const std::string_view arg = argv[index];
        if (IsOptionWithValue(arg)) {
            ++index;
            continue;
        }
        if (!arg.starts_with("-")) {
            actions.emplace_back(arg);
        }
    }
    return actions;
}

[[nodiscard]] bool ContainsDrone(const std::vector<std::string>& drone_ids,
                                 std::string_view drone_id) {
    return std::ranges::find(drone_ids, drone_id) != drone_ids.end();
}

int RunSwarmHealth(const SwarmRuntime& runtime) {
    bool all_ok = true;
    const std::int64_t now_ms = NowUnixMs();
    for (const auto& drone_id : runtime.drone_ids) {
        const auto status = runtime.client->GetHealth(drone_id);
        const bool heartbeat_stale =
            status.ok && IsStaleTimestamp(status.last_heartbeat_unix_ms, now_ms);
        const bool telemetry_stale =
            status.ok && IsStaleTimestamp(status.last_telemetry_unix_ms, now_ms);
        const std::string reason = SwarmHealthReason(status, heartbeat_stale, telemetry_stale);
        const bool healthy = status.ok && status.ready && !status.failsafe && status.gps_ok &&
                             status.ekf_ok && !heartbeat_stale && !telemetry_stale;
        all_ok = all_ok && healthy;
        std::cout << drone_id << ": " << (status.ok ? "OK" : "FAILED");
        if (status.ok) {
            std::cout << " ready=" << BoolText(status.ready) << " backend=" << status.backend_name
                      << " protocol=" << status.protocol << " armed=" << BoolText(status.armed)
                      << " landed=" << BoolText(status.landed)
                      << " failsafe=" << BoolText(status.failsafe)
                      << " gps=" << BoolText(status.gps_ok) << " ekf=" << BoolText(status.ekf_ok)
                      << " link_quality=" << OptionalFloatText(status.link_quality_percent)
                      << " heartbeat_age_ms="
                      << TimestampAgeMsText(status.last_heartbeat_unix_ms, now_ms)
                      << " heartbeat_stale=" << BoolText(heartbeat_stale)
                      << " telemetry_age_ms="
                      << TimestampAgeMsText(status.last_telemetry_unix_ms, now_ms)
                      << " telemetry_stale=" << BoolText(telemetry_stale)
                      << " agent_id=" << status.agent_id << " version=" << status.version;
        }
        if (!reason.empty()) {
            std::cout << " reason=" << reason;
        }
        std::cout << "\n";
    }
    return all_ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

int RunSwarmStats(const SwarmRuntime& runtime) {
    bool all_ok = true;
    for (const auto& drone_id : runtime.drone_ids) {
        const auto stats = runtime.client->GetRuntimeStats(drone_id);
        all_ok = all_ok && stats.ok;
        std::cout << drone_id << ": " << (stats.ok ? "OK" : "FAILED");
        if (stats.ok) {
            std::cout << " ready=" << (stats.ready ? "true" : "false")
                      << " commands=" << stats.command_requests_total
                      << " rejected=" << stats.command_rejected_total
                      << " failed=" << stats.command_failed_total
                      << " telemetry_streams=" << stats.current_telemetry_streams
                      << " frames=" << stats.telemetry_frames_sent_total
                      << " backend_failures=" << stats.backend_failures_total
                      << " data_messages=" << stats.data_messages_published_total
                      << " data_rejected=" << stats.data_messages_rejected_total
                      << " artifacts_up=" << stats.artifact_uploads_total
                      << " artifacts_down=" << stats.artifact_downloads_total
                      << " artifact_rx_bytes=" << stats.artifact_bytes_received_total
                      << " artifact_tx_bytes=" << stats.artifact_bytes_sent_total
                      << " artifact_failures=" << stats.artifact_failures_total;
        } else if (!stats.error.user_message.empty()) {
            std::cout << " " << stats.error.user_message;
        }
        std::cout << "\n";
    }
    return all_ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

int RunSwarmTelemetry(SwarmRuntime& runtime, int argc, char** argv) {
    const auto rate_hz = ParseTelemetryRate(argc, argv);
    if (!rate_hz.has_value()) {
        std::cerr << rate_hz.error() << "\n";
        return EXIT_FAILURE;
    }
    auto sink = TelemetrySink::FromArgs(argc, argv, true);
    if (!sink.has_value()) {
        std::cerr << sink.error() << "\n";
        return EXIT_FAILURE;
    }
    auto telemetry_sink = std::move(*sink);
    const auto duration_ms = ParseDurationMs(argc, argv);
    if (!duration_ms.has_value()) {
        std::cerr << duration_ms.error() << "\n";
        return EXIT_FAILURE;
    }

    const std::string drone_id = common::GetOptionValue(argc, argv, "--drone");
    if (!drone_id.empty() && !ContainsDrone(runtime.drone_ids, drone_id)) {
        std::cerr << "drone '" << drone_id << "' is not present in swarm config\n";
        return EXIT_FAILURE;
    }

    ResetStopRequested();
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    if (drone_id.empty()) {
        std::cout << "Subscribing to swarm telemetry: drones=" << runtime.drone_ids.size()
                  << " rate=" << *rate_hz << " Hz\n";
        auto telemetry_streams = runtime.client->StartAllTelemetry(
            *rate_hz,
            [&telemetry_sink](const swarmkit::core::TelemetryFrame& frame) {
                telemetry_sink->Write(frame);
            },
            [](const std::string& error_msg) {
                std::cerr << "Telemetry stream error: " << error_msg << "\n";
            });
        bool started = true;
        for (const auto& [stream_drone_id, result] : telemetry_streams) {
            if (!result.has_value()) {
                std::cerr << "Failed to start telemetry stream for " << stream_drone_id << ": "
                          << result.error().user_message << "\n";
                started = false;
            }
        }
        if (!started) {
            for (auto& entry : telemetry_streams) {
                auto& result = entry.second;
                if (result.has_value()) {
                    result->Stop();
                }
            }
            return EXIT_FAILURE;
        }
        std::cout << "Press Ctrl+C to stop.\n\n";
        if (telemetry_sink->WritesFiles()) {
            std::cout << "Telemetry CSV logging enabled.\n";
        }
        WaitForStop(*duration_ms);
        for (auto& entry : telemetry_streams) {
            auto& result = entry.second;
            if (result.has_value()) {
                result->Stop();
            }
        }
    } else {
        std::cout << "Subscribing to swarm telemetry: drone=" << drone_id << " rate=" << *rate_hz
                  << " Hz\n";
        auto telemetry_stream = runtime.client->StartTelemetry(
            {.drone_id = drone_id, .rate_hertz = *rate_hz},
            [&telemetry_sink](const swarmkit::core::TelemetryFrame& frame) {
                telemetry_sink->Write(frame);
            },
            [](const std::string& error_msg) {
                std::cerr << "Telemetry stream error: " << error_msg << "\n";
            });
        if (!telemetry_stream.has_value()) {
            std::cerr << "Failed to start telemetry stream: "
                      << telemetry_stream.error().user_message << "\n";
            return EXIT_FAILURE;
        }
        std::cout << "Press Ctrl+C to stop.\n\n";
        if (telemetry_sink->WritesFiles()) {
            std::cout << "Telemetry CSV logging enabled.\n";
        }
        WaitForStop(*duration_ms);
        telemetry_stream->Stop();
    }
    std::cout << "\nStopped.\n";
    return EXIT_SUCCESS;
}

int RunSwarmSequence(SwarmRuntime& runtime, CommandPriority priority, int argc, char** argv) {
    const auto path = ParseSequenceFilePath(argc, argv);
    if (!path.has_value()) {
        std::cerr << path.error() << "\n";
        return EXIT_FAILURE;
    }
    const auto steps = LoadSequenceSteps(*path);
    if (!steps.has_value()) {
        std::cerr << steps.error() << "\n";
        return EXIT_FAILURE;
    }
    const bool verify_commands = common::HasFlag(argc, argv, "--verify");
    SwarmResultPolicy result_policy = ParseSwarmResultPolicy(argc, argv);
    if (!result_policy.require_all && !result_policy.continue_on_error) {
        result_policy.require_all = true;
    }
    const auto wait_options = ParseCommandWaitOptions(argc, argv);
    if (!wait_options.has_value()) {
        std::cerr << wait_options.error() << "\n";
        return EXIT_FAILURE;
    }

    const std::string default_drone_id = common::GetOptionValue(argc, argv, "--drone");
    if (!default_drone_id.empty() && !ContainsDrone(runtime.drone_ids, default_drone_id)) {
        std::cerr << "drone '" << default_drone_id << "' is not present in swarm config\n";
        return EXIT_FAILURE;
    }

    const bool needs_telemetry = std::ranges::any_of(*steps, [](const SequenceStep& step) {
        return !step.wait_conditions.empty() || IsDisarmAction(step);
    });
    SequenceTelemetryMonitor monitor;
    if (needs_telemetry) {
        if (!monitor.StartSwarm(runtime, kDefaultSequenceTelemetryRateHz)) {
            return EXIT_FAILURE;
        }
    }

    int failed_steps = 0;
    for (std::size_t index = 0; index < steps->size(); ++index) {
        const SequenceStep& step = steps->at(index);
        DelaySequenceStep(step);
        const std::vector<std::string> target_drones =
            StepTargetDrones(step, default_drone_id, runtime.drone_ids);
        if (!step.wait_conditions.empty() && target_drones.size() == 1 &&
            target_drones.front().empty()) {
            std::cerr << "step " << index
                      << " wait requires drone: DRONE_ID, broadcast: true, or global --drone\n";
            ++failed_steps;
            if (!step.continue_on_error && !result_policy.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }
        if (!step.wait_conditions.empty() &&
            !WaitForConditions(monitor, target_drones, step.wait_conditions, index)) {
            ++failed_steps;
            if (!step.continue_on_error && !result_policy.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }
        if (step.args.empty()) {
            continue;
        }

        const auto command = BuildCommandFromTokens(step.args);
        if (!command.has_value()) {
            std::cerr << "step " << index << " build failed: " << command.error() << "\n";
            ++failed_steps;
            if (!step.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }

        const std::string drone_id = step.drone_id.empty() ? default_drone_id : step.drone_id;
        if (!step.broadcast && drone_id.empty()) {
            std::cerr << "step " << index
                      << " requires drone: DRONE_ID, broadcast: true, or global --drone\n";
            ++failed_steps;
            if (!step.continue_on_error && !result_policy.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }
        if (!step.broadcast && !ContainsDrone(runtime.drone_ids, drone_id)) {
            std::cerr << "step " << index << " drone '" << drone_id
                      << "' is not present in swarm config\n";
            ++failed_steps;
            if (!step.continue_on_error && !result_policy.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }

        const int attempts = step.retries + 1;
        bool step_ok = false;
        for (int attempt = 1; attempt <= attempts; ++attempt) {
            if (IsDisarmAction(step) && !IsEmergencyAction(step)) {
                WaitCondition landed_condition;
                landed_condition.wait_landed = true;
                landed_condition.timeout_ms = 30000;
                if (!WaitForConditions(monitor, target_drones, {landed_condition}, index)) {
                    step_ok = false;
                    break;
                }
            }
            if (step.broadcast) {
                if (attempts > 1) {
                    std::cout << "step " << index << " broadcast attempt=" << attempt << "/"
                              << attempts << "\n";
                }
                swarmkit::commands::CommandContext context;
                context.client_id = runtime.client_id;
                context.priority = priority;
                const bool verify_step = verify_commands || step.verify;
                step_ok = PrintSwarmResults(
                    verify_step ? runtime.client->BroadcastCommandAndWait(
                                      *command, context, StepWaitOptions(*wait_options, step))
                                : runtime.client->BroadcastCommand(*command, context),
                    result_policy);
            } else {
                const auto envelope =
                    MakeCommandEnvelope(drone_id, runtime.client_id, *command, priority);
                const bool verify_step = verify_commands || step.verify;
                const auto result = verify_step
                                        ? runtime.client->SendCommandAndWait(
                                              envelope, StepWaitOptions(*wait_options, step))
                                        : runtime.client->SendCommand(envelope);
                std::string label = "step " + std::to_string(index) + " drone=" + drone_id;
                if (attempts > 1) {
                    label += " attempt=" + std::to_string(attempt) + "/" + std::to_string(attempts);
                }
                step_ok = PrintCommandResult(label, result);
            }

            if (step_ok || attempt == attempts) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{step.retry_delay_ms});
        }

        if (!step_ok) {
            ++failed_steps;
            if (!step.continue_on_error && !result_policy.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }
    }

    if (failed_steps > 0 && !result_policy.continue_on_error) {
        std::cerr << "swarm sequence completed with failed_steps=" << failed_steps << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "swarm sequence OK steps=" << steps->size() << "\n";
    return EXIT_SUCCESS;
}

int RunSwarm(const ClientConfig& client_cfg, int argc, char** argv) {
    auto runtime = BuildSwarmRuntime(client_cfg, argc, argv);
    if (!runtime.has_value()) {
        return runtime.error();
    }
    const auto actions = FindSwarmActions(argc, argv);
    if (actions.empty()) {
        std::cerr << "swarm requires health, stats, telemetry, sequence, command, lock-all, "
                     "unlock-all, broadcast, land-all, or rtl-all\n";
        return EXIT_FAILURE;
    }

    Command command;
    SwarmResultPolicy result_policy = ParseSwarmResultPolicy(argc, argv);
    if (!result_policy.require_all && !result_policy.continue_on_error) {
        result_policy.require_all = true;
    }
    if (actions[0] == "health") {
        return RunSwarmHealth(*runtime);
    }
    if (actions[0] == "stats") {
        return RunSwarmStats(*runtime);
    }
    if (actions[0] == "telemetry") {
        return RunSwarmTelemetry(*runtime, argc, argv);
    }
    if (actions[0] == "sequence") {
        return RunSwarmSequence(*runtime, client_cfg.priority, argc, argv);
    }
    if (actions[0] == "lock-all") {
        const auto ttl_ms = ParseTtlMs(argc, argv);
        if (!ttl_ms.has_value()) {
            std::cerr << ttl_ms.error() << "\n";
            return EXIT_FAILURE;
        }
        return PrintSwarmResults(runtime->client->LockAll(*ttl_ms), result_policy) ? EXIT_SUCCESS
                                                                                   : EXIT_FAILURE;
    }
    if (actions[0] == "unlock-all") {
        return PrintSwarmReleaseResults(runtime->client->UnlockAll(), result_policy) ? EXIT_SUCCESS
                                                                                     : EXIT_FAILURE;
    }
    if (actions[0] == "command") {
        const std::string drone_id = common::GetOptionValue(argc, argv, "--drone");
        if (drone_id.empty()) {
            std::cerr << "swarm command requires --drone DRONE_ID\n";
            return EXIT_FAILURE;
        }
        if (!ContainsDrone(runtime->drone_ids, drone_id)) {
            std::cerr << "drone '" << drone_id << "' is not present in swarm config\n";
            return EXIT_FAILURE;
        }
        std::vector<std::string> command_actions;
        if (actions.size() > 1) {
            command_actions.assign(actions.begin() + 1, actions.end());
        }
        const auto built = BuildCommandFromActions(command_actions, argc, argv);
        if (!built.has_value()) {
            std::cerr << built.error() << "\n";
            return EXIT_FAILURE;
        }
        swarmkit::commands::CommandEnvelope envelope =
            MakeCommandEnvelope(drone_id, client_cfg.client_id, *built, client_cfg.priority);
        const bool verify = common::HasFlag(argc, argv, "--verify");
        const auto wait_options = ParseCommandWaitOptions(argc, argv);
        if (!wait_options.has_value()) {
            std::cerr << wait_options.error() << "\n";
            return EXIT_FAILURE;
        }
        const auto result = verify ? runtime->client->SendCommandAndWait(envelope, *wait_options)
                                   : runtime->client->SendCommand(envelope);
        return PrintSwarmResults({{drone_id, result}}, result_policy) ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    if (actions[0] == "land-all") {
        command = FlightCmd{CmdLand{}};
    } else if (actions[0] == "rtl-all") {
        command = NavCmd{CmdReturnHome{}};
    } else if (actions[0] == "broadcast") {
        std::vector<std::string> command_actions;
        if (actions.size() > 1) {
            command_actions.assign(actions.begin() + 1, actions.end());
        }
        const auto built = BuildCommandFromActions(command_actions, argc, argv);
        if (!built.has_value()) {
            std::cerr << built.error() << "\n";
            return EXIT_FAILURE;
        }
        command = *built;
    } else {
        std::cerr << "Unknown swarm action: " << actions[0] << "\n";
        return EXIT_FAILURE;
    }

    swarmkit::commands::CommandContext context;
    context.client_id = client_cfg.client_id;
    context.priority = client_cfg.priority;
    const bool verify = common::HasFlag(argc, argv, "--verify");
    const auto wait_options = ParseCommandWaitOptions(argc, argv);
    if (!wait_options.has_value()) {
        std::cerr << wait_options.error() << "\n";
        return EXIT_FAILURE;
    }
    const auto results =
        verify ? runtime->client->BroadcastCommandAndWait(command, context, *wait_options)
               : runtime->client->BroadcastCommand(command, context);
    return PrintSwarmResults(results, result_policy) ? EXIT_SUCCESS : EXIT_FAILURE;
}

}  // namespace

[[nodiscard]] int DispatchCommand(const CliInvocation& invocation, const ClientConfig& client_cfg,
                                  Client& client, int argc, char** argv) {
    if (invocation.command == "ping") {
        return RunPing(client);
    }
    if (invocation.command == "health") {
        return RunHealth(client);
    }
    if (invocation.command == "preflight") {
        return RunPreflight(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId),
                            argc, argv);
    }
    if (invocation.command == "stats") {
        return RunStats(client);
    }
    if (invocation.command == "capabilities") {
        return RunCapabilities(client);
    }
    if (invocation.command == "telemetry") {
        const auto kRateHz = ParseTelemetryRate(argc, argv);
        if (!kRateHz.has_value()) {
            std::cerr << kRateHz.error() << "\n";
            return EXIT_FAILURE;
        }
        return RunTelemetry(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId),
                            *kRateHz, argc, argv);
    }
    if (invocation.command == "command") {
        return RunCommand(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId),
                          client_cfg.client_id, client_cfg.priority, argc, argv);
    }
    if (invocation.command == "sequence") {
        return RunSequence(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId),
                           client_cfg.client_id, client_cfg.priority, argc, argv);
    }
    if (invocation.command == "goal") {
        return RunGoal(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId), argc,
                       argv);
    }
    if (invocation.command == "reports") {
        return RunReports(client, common::GetOptionValue(argc, argv, "--drone", "all"), argc, argv);
    }
    if (invocation.command == "message") {
        return RunMessage(client, argc, argv);
    }
    if (invocation.command == "artifact") {
        return RunArtifact(client, argc, argv);
    }
    if (invocation.command == "lock") {
        return RunLock(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId), argc,
                       argv);
    }
    if (invocation.command == "unlock") {
        return RunUnlock(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId));
    }
    if (invocation.command == "watch-authority") {
        return RunWatchAuthority(client,
                                 common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId),
                                 client_cfg.priority);
    }
    if (invocation.command == "swarm") {
        return RunSwarm(client_cfg, argc, argv);
    }

    std::cerr << "Unknown command: " << invocation.command << "\n\n";
    PrintUsage();
    return EXIT_FAILURE;
}

}  // namespace swarmkit::apps::cli::internal
