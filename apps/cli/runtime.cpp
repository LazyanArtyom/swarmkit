// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "runtime.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
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
#include "swarmkit/client/swarm_client.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/telemetry.h"
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
};

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
        if (current_arg.starts_with("-") || IsSubcommand(current_arg)) {
            continue;
        }
        actions.emplace_back(current_arg);
    }
    return actions;
}

[[nodiscard]] std::string CsvField(std::string_view value) {
    std::string out;
    out.reserve(value.size() + 2);
    out.push_back('"');
    for (const char character : value) {
        if (character == '"') {
            out.push_back('"');
        }
        out.push_back(character);
    }
    out.push_back('"');
    return out;
}

[[nodiscard]] std::int64_t NowUnixMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

[[nodiscard]] std::string JsonEscape(std::string_view value) {
    std::string out;
    out.reserve(value.size() + 2);
    for (const char character : value) {
        switch (character) {
            case '\\':
                out += "\\\\";
                break;
            case '"':
                out += "\\\"";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out.push_back(character);
                break;
        }
    }
    return out;
}

[[nodiscard]] std::string_view GoalStatusName(swarmkit::client::GoalStatus status) {
    using swarmkit::client::GoalStatus;
    switch (status) {
        case GoalStatus::kActive:
            return "active";
        case GoalStatus::kReached:
            return "reached";
        case GoalStatus::kDeviating:
            return "deviating";
        case GoalStatus::kTimeout:
            return "timeout";
        case GoalStatus::kCancelled:
            return "cancelled";
        case GoalStatus::kSuperseded:
            return "superseded";
        case GoalStatus::kFailed:
            return "failed";
        case GoalStatus::kUnspecified:
            return "unspecified";
    }
    return "unspecified";
}

[[nodiscard]] std::string_view ReportTypeName(swarmkit::client::AgentReportType type) {
    using swarmkit::client::AgentReportType;
    switch (type) {
        case AgentReportType::kCommandAccepted:
            return "command_accepted";
        case AgentReportType::kCommandRejected:
            return "command_rejected";
        case AgentReportType::kCommandAcked:
            return "command_acked";
        case AgentReportType::kCommandFailed:
            return "command_failed";
        case AgentReportType::kGoalReport:
            return "goal_report";
        case AgentReportType::kTelemetryStale:
            return "telemetry_stale";
        case AgentReportType::kHeartbeatLost:
            return "heartbeat_lost";
        case AgentReportType::kHealthChanged:
            return "health_changed";
        case AgentReportType::kAuthorityLocked:
            return "authority_locked";
        case AgentReportType::kAuthorityRejected:
            return "authority_rejected";
        case AgentReportType::kAuthorityReleased:
            return "authority_released";
        case AgentReportType::kUnspecified:
            return "unspecified";
    }
    return "unspecified";
}

[[nodiscard]] std::string_view SeverityName(swarmkit::client::ReportSeverity severity) {
    using swarmkit::client::ReportSeverity;
    switch (severity) {
        case ReportSeverity::kWarning:
            return "warning";
        case ReportSeverity::kError:
            return "error";
        case ReportSeverity::kCritical:
            return "critical";
        case ReportSeverity::kInfo:
            return "info";
    }
    return "info";
}

[[nodiscard]] std::string TelemetryCsvLine(const swarmkit::core::TelemetryFrame& frame) {
    std::ostringstream line;
    line << frame.unix_time_ms << "," << CsvField(frame.drone_id) << std::setprecision(10) << ","
         << frame.lat_deg << "," << frame.lon_deg << "," << std::setprecision(5) << frame.rel_alt_m
         << "," << frame.battery_percent << "," << CsvField(frame.mode) << "\n";
    return line.str();
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
        DistanceMeters(frame.lat_deg, frame.lon_deg, *condition.lat_deg, *condition.lon_deg) >
            condition.position_radius_m) {
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
    if (condition.wait_landed && std::abs(frame.rel_alt_m) > condition.landed_alt_m) {
        return false;
    }
    return true;
}

[[nodiscard]] bool FileNeedsHeader(const std::filesystem::path& path) {
    std::error_code error;
    return !std::filesystem::exists(path, error) || std::filesystem::file_size(path, error) == 0;
}

[[nodiscard]] std::string SanitizeFileStem(std::string_view value) {
    std::string out;
    out.reserve(value.size());
    for (const char character : value) {
        const bool is_safe =
            (character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z') ||
            (character >= '0' && character <= '9') || character == '-' || character == '_';
        out.push_back(is_safe ? character : '_');
    }
    return out.empty() ? "default" : out;
}

class TelemetrySink {
   public:
    [[nodiscard]] static std::expected<std::unique_ptr<TelemetrySink>, std::string> FromArgs(
        int argc, char** argv, bool allow_split) {
        auto sink = std::make_unique<TelemetrySink>();
        sink->console_enabled_ = !common::HasFlag(argc, argv, "--no-console");

        const std::string combined_path = common::GetOptionValue(argc, argv, "--telemetry-file");
        if (!combined_path.empty()) {
            if (auto opened = OpenCsvFile(combined_path); opened.has_value()) {
                sink->combined_file_ = std::make_unique<std::ofstream>(std::move(*opened));
            } else {
                return std::unexpected(opened.error());
            }
        }

        const std::string dir_path = common::GetOptionValue(argc, argv, "--telemetry-dir");
        if (!dir_path.empty()) {
            if (!allow_split) {
                return std::unexpected("--telemetry-dir is only valid for swarm telemetry");
            }
            std::error_code error;
            std::filesystem::create_directories(dir_path, error);
            if (error) {
                return std::unexpected("failed to create telemetry directory '" + dir_path +
                                       "': " + error.message());
            }
            sink->per_drone_dir_ = dir_path;
        }
        return sink;
    }

    void Write(const swarmkit::core::TelemetryFrame& frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (console_enabled_) {
            std::cout << std::fixed << std::setprecision(kTelemetryCoordPrecision) << "["
                      << frame.unix_time_ms << "]"
                      << " drone=" << frame.drone_id << " lat=" << frame.lat_deg
                      << " lon=" << frame.lon_deg << std::setprecision(kTelemetryValuePrecision)
                      << " alt=" << frame.rel_alt_m << "m"
                      << " bat=" << frame.battery_percent << "%"
                      << " mode=" << frame.mode << "\n";
        }

        const std::string csv_line = TelemetryCsvLine(frame);
        if (combined_file_) {
            *combined_file_ << csv_line;
            combined_file_->flush();
        }
        if (!per_drone_dir_.empty()) {
            std::ofstream* file = GetDroneFile(frame.drone_id);
            if (file != nullptr) {
                *file << csv_line;
                file->flush();
            }
        }
    }

    [[nodiscard]] bool WritesFiles() const {
        return combined_file_ != nullptr || !per_drone_dir_.empty();
    }

   private:
    [[nodiscard]] static std::expected<std::ofstream, std::string> OpenCsvFile(
        const std::filesystem::path& path) {
        if (path.has_parent_path()) {
            std::error_code error;
            std::filesystem::create_directories(path.parent_path(), error);
            if (error) {
                return std::unexpected("failed to create telemetry directory '" +
                                       path.parent_path().string() + "': " + error.message());
            }
        }

        const bool needs_header = FileNeedsHeader(path);
        std::ofstream file(path, std::ios::app);
        if (!file.is_open()) {
            return std::unexpected("failed to open telemetry file '" + path.string() + "'");
        }
        if (needs_header) {
            file << "unix_time_ms,drone_id,lat_deg,lon_deg,rel_alt_m,battery_percent,mode\n";
        }
        return file;
    }

    std::ofstream* GetDroneFile(const std::string& drone_id) {
        if (const auto iter = per_drone_files_.find(drone_id); iter != per_drone_files_.end()) {
            return iter->second.get();
        }

        const std::filesystem::path path =
            std::filesystem::path(per_drone_dir_) / (SanitizeFileStem(drone_id) + ".csv");
        auto opened = OpenCsvFile(path);
        if (!opened.has_value()) {
            std::cerr << opened.error() << "\n";
            return nullptr;
        }
        auto file = std::make_unique<std::ofstream>(std::move(*opened));
        std::ofstream* raw = file.get();
        per_drone_files_.emplace(drone_id, std::move(file));
        return raw;
    }

    bool console_enabled_{true};
    std::unique_ptr<std::ofstream> combined_file_;
    std::filesystem::path per_drone_dir_;
    std::unordered_map<std::string, std::unique_ptr<std::ofstream>> per_drone_files_;
    std::mutex mutex_;
};

class SequenceTelemetryMonitor {
   public:
    ~SequenceTelemetryMonitor() {
        Stop();
    }

    SequenceTelemetryMonitor(const SequenceTelemetryMonitor&) = delete;
    SequenceTelemetryMonitor& operator=(const SequenceTelemetryMonitor&) = delete;

    SequenceTelemetryMonitor() = default;

    void StartSingle(Client& client, const std::string& drone_id, int rate_hz) {
        Stop();
        single_client_ = &client;
        single_drone_id_ = drone_id;
        client.SubscribeTelemetry(
            {.drone_id = drone_id, .rate_hertz = rate_hz},
            [this](const swarmkit::core::TelemetryFrame& frame) { StoreFrame(frame); },
            [](const std::string& error_msg) {
                std::cerr << "Sequence telemetry stream error: " << error_msg << "\n";
            });
        running_ = true;
    }

    void StartSwarm(SwarmRuntime& runtime, int rate_hz) {
        Stop();
        swarm_client_ = runtime.client.get();
        runtime.client->SubscribeAllTelemetry(
            rate_hz, [this](const swarmkit::core::TelemetryFrame& frame) { StoreFrame(frame); },
            [](const std::string& error_msg) {
                std::cerr << "Sequence telemetry stream error: " << error_msg << "\n";
            });
        running_ = true;
    }

    void Stop() {
        if (!running_) {
            return;
        }
        if (single_client_ != nullptr) {
            single_client_->StopTelemetry();
        }
        if (swarm_client_ != nullptr) {
            swarm_client_->StopAllTelemetry();
        }
        single_client_ = nullptr;
        swarm_client_ = nullptr;
        running_ = false;
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

    Client* single_client_{nullptr};
    swarmkit::client::SwarmClient* swarm_client_{nullptr};
    std::string single_drone_id_;
    bool running_{false};
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

[[nodiscard]] std::optional<WaitCondition> MakeVerificationCondition(const SequenceStep& step) {
    if (!step.verify || step.args.empty()) {
        return std::nullopt;
    }
    const std::string& action = step.args.front();
    WaitCondition condition;
    if (action == "takeoff") {
        const auto alt_iter = std::ranges::find(step.args, "--alt");
        condition.target_alt_m =
            alt_iter != step.args.end() && std::next(alt_iter) != step.args.end()
                ? std::stof(*std::next(alt_iter))
                : std::stof(std::string(kDefaultTakeoffAlt));
        condition.timeout_ms = 45000;
        return condition;
    }
    if (action == "goto" || action == "waypoint") {
        const auto read_value = [&](std::string_view key) -> std::optional<std::string> {
            const auto iter = std::ranges::find(step.args, std::string(key));
            if (iter == step.args.end() || std::next(iter) == step.args.end()) {
                return std::nullopt;
            }
            return *std::next(iter);
        };
        const auto lat = read_value("--lat");
        const auto lon = read_value("--lon");
        if (!lat.has_value() || !lon.has_value()) {
            return std::nullopt;
        }
        condition.lat_deg = std::stod(*lat);
        condition.lon_deg = std::stod(*lon);
        if (const auto alt = read_value("--alt"); alt.has_value()) {
            condition.target_alt_m = std::stof(*alt);
        }
        condition.timeout_ms = 60000;
        return condition;
    }
    if (action == "land") {
        condition.wait_landed = true;
        condition.timeout_ms = 60000;
        return condition;
    }
    if (action == "disarm") {
        condition.armed = false;
        condition.timeout_ms = 30000;
        return condition;
    }
    return std::nullopt;
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

[[nodiscard]] std::expected<swarmkit::client::ActiveGoal, std::string> ParseActiveGoal(
    std::string_view drone_id, int argc, char** argv) {
    const std::string goal_id = common::GetOptionValue(argc, argv, "--goal-id");
    if (goal_id.empty()) {
        return std::unexpected("goal set requires --goal-id ID");
    }
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

    const auto speed = ParseFloatArg(common::GetOptionValue(argc, argv, "--speed", kDefaultZero),
                                     "--speed");
    const auto acceptance_radius =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--accept-radius", "2"),
                      "--accept-radius");
    const auto deviation_radius =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--deviation-radius", "8"),
                      "--deviation-radius");
    const auto timeout_ms =
        ParseIntArg(common::GetOptionValue(argc, argv, "--timeout-ms", kDefaultZero),
                    "--timeout-ms");
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

    return swarmkit::client::ActiveGoal{
        .drone_id = std::string(drone_id),
        .goal_id = goal_id,
        .revision = revision,
        .target = swarmkit::client::GeoPoint{
            .lat_deg = *lat,
            .lon_deg = *lon,
            .alt_m = *alt,
        },
        .speed_mps = *speed,
        .acceptance_radius_m = *acceptance_radius,
        .deviation_radius_m = *deviation_radius,
        .timeout_ms = *timeout_ms,
        .role = common::GetOptionValue(argc, argv, "--role"),
    };
}

void PrintReportText(const swarmkit::client::AgentReport& report, std::ostream& out) {
    out << "[" << report.sequence << "] drone=" << report.drone_id
        << " type=" << ReportTypeName(report.type) << " severity=" << SeverityName(report.severity);
    if (report.goal.has_value()) {
        out << " goal=" << report.goal->goal_id << " rev=" << report.goal->revision
            << " status=" << GoalStatusName(report.goal->status)
            << " dist=" << std::fixed << std::setprecision(1) << report.goal->distance_to_goal_m
            << "m dev=" << report.goal->deviation_m
            << "m alt_err=" << report.goal->altitude_error_m << "m";
    }
    if (!report.message.empty()) {
        out << " msg=" << report.message;
    }
    out << "\n";
}

void PrintReportJsonl(const swarmkit::client::AgentReport& report, std::ostream& out) {
    out << R"({"sequence":)" << report.sequence << R"(,"unix_time_ms":)" << report.unix_time_ms
        << R"(,"drone_id":")" << JsonEscape(report.drone_id) << R"(","type":")"
        << ReportTypeName(report.type) << R"(","severity":")" << SeverityName(report.severity)
        << R"(","correlation_id":")" << JsonEscape(report.correlation_id) << R"(","message":")"
        << JsonEscape(report.message) << "\"";
    if (report.goal.has_value()) {
        out << R"(,"goal":{"goal_id":")" << JsonEscape(report.goal->goal_id)
            << R"(","revision":)" << report.goal->revision << R"(,"status":")"
            << GoalStatusName(report.goal->status)
            << R"(","distance_to_goal_m":)" << report.goal->distance_to_goal_m
            << ",\"deviation_m\":" << report.goal->deviation_m
            << ",\"altitude_error_m\":" << report.goal->altitude_error_m
            << ",\"acceptance_radius_m\":" << report.goal->acceptance_radius_m
            << ",\"deviation_radius_m\":" << report.goal->deviation_radius_m
            << ",\"elapsed_ms\":" << report.goal->elapsed_ms
            << ",\"timeout_ms\":" << report.goal->timeout_ms << R"(,"message":")"
            << JsonEscape(report.goal->message) << "\"}";
    }
    out << "}\n";
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

int RunSequence(Client& client, std::string_view default_drone_id, CommandPriority priority,
                int argc, char** argv) {
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

    const bool needs_telemetry = std::ranges::any_of(*steps, [](const SequenceStep& step) {
        return !step.wait_conditions.empty() || step.verify || IsDisarmAction(step);
    });
    SequenceTelemetryMonitor monitor;
    if (needs_telemetry) {
        monitor.StartSingle(client, std::string(default_drone_id), kDefaultSequenceTelemetryRateHz);
    }

    int failed_steps = 0;
    for (std::size_t index = 0; index < steps->size(); ++index) {
        const SequenceStep& step = steps->at(index);
        DelaySequenceStep(step);
        const std::vector<std::string> target_drones = StepTargetDrones(step, default_drone_id, {});
        if (!step.wait_conditions.empty() &&
            !WaitForConditions(monitor, target_drones, step.wait_conditions, index)) {
            ++failed_steps;
            if (!step.continue_on_error) {
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
            if (!step.continue_on_error) {
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
            const auto result =
                client.SendCommand(MakeCommandEnvelope(drone_id, *command, priority));
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
            if (!step.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }

        if (const auto verification = MakeVerificationCondition(step); verification.has_value()) {
            if (!WaitForConditions(monitor, target_drones, {*verification}, index)) {
                ++failed_steps;
                if (!step.continue_on_error) {
                    return EXIT_FAILURE;
                }
            }
        }
    }

    if (failed_steps > 0) {
        std::cerr << "sequence completed with failed_steps=" << failed_steps << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "sequence OK steps=" << steps->size() << "\n";
    return EXIT_SUCCESS;
}

int RunCommand(Client& client, std::string_view drone_id, CommandPriority priority, int argc,
               char** argv) {
    const auto kCommand = BuildCommandFromArgs(argc, argv);
    if (!kCommand.has_value()) {
        std::cerr << kCommand.error() << "\n";
        if (kCommand.error().starts_with("Unknown action:")) {
            std::cerr << "\n";
            PrintUsage();
        }
        return EXIT_FAILURE;
    }

    const auto kResult = client.SendCommand(MakeCommandEnvelope(drone_id, *kCommand, priority));
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
                  << " computed_timeout_ms=" << result.computed_timeout_ms << "\n";
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
                  << "  lat                 : " << status.goal.target.lat_deg << "\n"
                  << "  lon                 : " << status.goal.target.lon_deg << "\n"
                  << "  alt_m               : " << status.goal.target.alt_m << "\n"
                  << "  speed_mps           : " << status.goal.speed_mps << "\n"
                  << "  accept_radius_m     : " << status.goal.acceptance_radius_m << "\n"
                  << "  deviation_radius_m  : " << status.goal.deviation_radius_m << "\n"
                  << "  computed_timeout_ms : " << status.computed_timeout_ms << "\n";
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
    if (const std::string path = common::GetOptionValue(argc, argv, "--report-file"); !path.empty()) {
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
    client.SubscribeReports(
        subscription,
        [out, &format](const swarmkit::client::AgentReport& report) {
            if (format == "jsonl") {
                PrintReportJsonl(report, *out);
            } else {
                PrintReportText(report, *out);
            }
            out->flush();
        },
        [](const std::string& error_msg) { std::cerr << "Report stream error: " << error_msg << "\n"; });

    std::cout << "Subscribed to reports: drone=" << drone_id << " format=" << format << "\n"
              << "Press Ctrl+C to stop.\n";
    WaitForStop(ParseDurationMs(argc, argv).value_or(0));
    client.StopReports();
    std::cout << "\nStopped.\n";
    return EXIT_SUCCESS;
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
              << "  ready     : " << (kStatus.ready ? "true" : "false") << "\n"
              << "  agent_id  : " << kStatus.agent_id << "\n"
              << "  version   : " << kStatus.version << "\n"
              << "  time_ms   : " << kStatus.unix_time_ms << "\n";
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
              << "  backend_failures_total      : " << kStats.backend_failures_total << "\n";
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

    client.SubscribeTelemetry(
        subscription,
        [&telemetry_sink](const swarmkit::core::TelemetryFrame& frame) {
            telemetry_sink->Write(frame);
        },
        [](const std::string& error_msg) {
            std::cerr << "Telemetry stream error: " << error_msg << "\n";
        });

    WaitForStop(*duration_ms);

    client.StopTelemetry();
    std::cout << "\nStopped.\n";
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
    client.ReleaseAuthority(std::string(drone_id));
    std::cout << "Unlock OK\n";
    return EXIT_SUCCESS;
}

int RunWatchAuthority(Client& client, std::string_view drone_id, CommandPriority priority) {
    ResetStopRequested();
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    swarmkit::client::AuthoritySubscription subscription;
    subscription.drone_id = std::string(drone_id);
    subscription.priority = priority;

    client.WatchAuthority(
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

    while (!IsStopRequested()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kTelemetryPollIntervalMs));
    }
    client.StopAuthorityWatch();
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

[[nodiscard]] bool PrintSwarmResults(
    const std::unordered_map<std::string, swarmkit::client::CommandResult>& results) {
    bool all_ok = true;
    for (const auto& [drone_id, result] : results) {
        std::cout << drone_id << ": " << (result.ok ? "OK" : "FAILED");
        all_ok = all_ok && result.ok;
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        std::cout << "\n";
    }
    return all_ok;
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
    for (const auto& drone_id : runtime.drone_ids) {
        const auto status = runtime.client->GetHealth(drone_id);
        all_ok = all_ok && status.ok;
        std::cout << drone_id << ": " << (status.ok ? "OK" : "FAILED");
        if (status.ok) {
            std::cout << " ready=" << (status.ready ? "true" : "false")
                      << " agent_id=" << status.agent_id << " version=" << status.version;
        } else if (!status.message.empty()) {
            std::cout << " " << status.message;
        } else if (!status.error.user_message.empty()) {
            std::cout << " " << status.error.user_message;
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
                      << " backend_failures=" << stats.backend_failures_total;
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
        runtime.client->SubscribeAllTelemetry(
            *rate_hz,
            [&telemetry_sink](const swarmkit::core::TelemetryFrame& frame) {
                telemetry_sink->Write(frame);
            },
            [](const std::string& error_msg) {
                std::cerr << "Telemetry stream error: " << error_msg << "\n";
            });
    } else {
        std::cout << "Subscribing to swarm telemetry: drone=" << drone_id << " rate=" << *rate_hz
                  << " Hz\n";
        runtime.client->SubscribeTelemetry(
            {.drone_id = drone_id, .rate_hertz = *rate_hz},
            [&telemetry_sink](const swarmkit::core::TelemetryFrame& frame) {
                telemetry_sink->Write(frame);
            },
            [](const std::string& error_msg) {
                std::cerr << "Telemetry stream error: " << error_msg << "\n";
            });
    }
    std::cout << "Press Ctrl+C to stop.\n\n";
    if (telemetry_sink->WritesFiles()) {
        std::cout << "Telemetry CSV logging enabled.\n";
    }

    WaitForStop(*duration_ms);

    if (drone_id.empty()) {
        runtime.client->StopAllTelemetry();
    } else {
        runtime.client->StopTelemetry(drone_id);
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

    const std::string default_drone_id = common::GetOptionValue(argc, argv, "--drone");
    if (!default_drone_id.empty() && !ContainsDrone(runtime.drone_ids, default_drone_id)) {
        std::cerr << "drone '" << default_drone_id << "' is not present in swarm config\n";
        return EXIT_FAILURE;
    }

    const bool needs_telemetry = std::ranges::any_of(*steps, [](const SequenceStep& step) {
        return !step.wait_conditions.empty() || step.verify || IsDisarmAction(step);
    });
    SequenceTelemetryMonitor monitor;
    if (needs_telemetry) {
        monitor.StartSwarm(runtime, kDefaultSequenceTelemetryRateHz);
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
            if (!step.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }
        if (!step.wait_conditions.empty() &&
            !WaitForConditions(monitor, target_drones, step.wait_conditions, index)) {
            ++failed_steps;
            if (!step.continue_on_error) {
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
            if (!step.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }
        if (!step.broadcast && !ContainsDrone(runtime.drone_ids, drone_id)) {
            std::cerr << "step " << index << " drone '" << drone_id
                      << "' is not present in swarm config\n";
            ++failed_steps;
            if (!step.continue_on_error) {
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
                context.client_id = std::string(kCliClientId);
                context.priority = priority;
                step_ok = PrintSwarmResults(runtime.client->BroadcastCommand(*command, context));
            } else {
                const auto result =
                    runtime.client->SendCommand(MakeCommandEnvelope(drone_id, *command, priority));
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
            if (!step.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }

        if (const auto verification = MakeVerificationCondition(step); verification.has_value()) {
            if (!WaitForConditions(monitor, target_drones, {*verification}, index)) {
                ++failed_steps;
                if (!step.continue_on_error) {
                    return EXIT_FAILURE;
                }
            }
        }
    }

    if (failed_steps > 0) {
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
        return PrintSwarmResults(runtime->client->LockAll(*ttl_ms)) ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    if (actions[0] == "unlock-all") {
        runtime->client->UnlockAll();
        std::cout << "Unlock all OK\n";
        return EXIT_SUCCESS;
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
            MakeCommandEnvelope(drone_id, *built, client_cfg.priority);
        const auto result = runtime->client->SendCommand(envelope);
        return PrintSwarmResults({{drone_id, result}}) ? EXIT_SUCCESS : EXIT_FAILURE;
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
    context.client_id = std::string(kCliClientId);
    context.priority = client_cfg.priority;
    return PrintSwarmResults(runtime->client->BroadcastCommand(command, context)) ? EXIT_SUCCESS
                                                                                  : EXIT_FAILURE;
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
    if (invocation.command == "stats") {
        return RunStats(client);
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
                          client_cfg.priority, argc, argv);
    }
    if (invocation.command == "sequence") {
        return RunSequence(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId),
                           client_cfg.priority, argc, argv);
    }
    if (invocation.command == "goal") {
        return RunGoal(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId), argc,
                       argv);
    }
    if (invocation.command == "reports") {
        return RunReports(client, common::GetOptionValue(argc, argv, "--drone", "all"), argc, argv);
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
