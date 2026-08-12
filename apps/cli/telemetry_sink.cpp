// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "telemetry_sink.h"

#include <expected>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "common/arg_utils.h"
#include "constants.h"

namespace swarmkit::apps::cli::internal {
namespace {

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

template <typename T>
[[nodiscard]] std::string CsvScalar(bool valid, const T& value) {
    if (!valid) {
        return {};
    }
    std::ostringstream out;
    out << value;
    return out.str();
}

template <typename T>
[[nodiscard]] std::string ConsoleScalar(bool valid, const T& value, std::string_view suffix = {}) {
    if (!valid) {
        return "unknown";
    }
    std::ostringstream out;
    out << value << suffix;
    return out.str();
}

[[nodiscard]] const char* ConsoleBool(bool valid, bool value) {
    if (!valid) {
        return "unknown";
    }
    return value ? "true" : "false";
}

}  // namespace

std::string TelemetryCsvLine(const swarmkit::client::TelemetryDelivery& delivery) {
    const auto& frame = delivery.frame;
    const auto* execution = frame.execution_handle ? &*frame.execution_handle : nullptr;
    const auto* context =
        execution != nullptr && execution->context ? &*execution->context : nullptr;
    std::ostringstream line;
    line << frame.agent_receive_unix_time_ms << "," << frame.agent_receive_monotonic_time_ns << ","
         << delivery.sdk_receive_unix_time_ms.value_or(0) << "," << CsvField(frame.agent_session_id)
         << "," << frame.telemetry_sequence << "," << CsvField(delivery.transport_stream_id) << ","
         << CsvField(frame.drone_id) << std::setprecision(10) << ","
         << CsvScalar(frame.validity.position, frame.lat_deg) << ","
         << CsvScalar(frame.validity.position, frame.lon_deg) << "," << std::setprecision(5)
         << CsvScalar(frame.validity.relative_altitude, frame.rel_alt_m) << ","
         << CsvScalar(frame.validity.battery, frame.battery_percent) << ","
         << CsvField(frame.validity.mode ? frame.mode : "") << ","
         << CsvScalar(frame.validity.gps, frame.gps_fix_type) << ","
         << CsvScalar(frame.validity.gps_hdop, frame.gps_hdop) << ","
         << CsvScalar(frame.accuracy.horizontal_position.has_value(),
                      frame.accuracy.horizontal_position ? frame.accuracy.horizontal_position->value
                                                         : 0.0F)
         << "," << CsvScalar(frame.validity.estimator, static_cast<int>(frame.estimator_state))
         << "," << CsvField(execution != nullptr ? execution->goal_id : "") << ","
         << (execution != nullptr ? execution->goal_revision : 0) << ","
         << CsvField(execution != nullptr ? execution->physical_attempt_id : "") << ","
         << (execution != nullptr ? execution->physical_attempt_revision : 0) << ","
         << CsvField(context != nullptr ? context->operation_id : "") << "\n";
    return line.str();
}

std::expected<std::unique_ptr<TelemetrySink>, std::string> TelemetrySink::FromArgs(
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

void TelemetrySink::Write(const swarmkit::client::TelemetryDelivery& delivery) {
    const auto& frame = delivery.frame;
    std::lock_guard<std::mutex> lock(mutex_);
    if (console_enabled_) {
        std::string_view estimator_text = "unknown";
        if (frame.validity.estimator) {
            estimator_text = frame.estimator_state == swarmkit::core::EstimatorState::kHealthy
                                 ? "healthy"
                                 : "unhealthy";
        }
        std::cout << std::fixed << std::setprecision(kTelemetryCoordPrecision) << "["
                  << frame.agent_receive_unix_time_ms << "]"
                  << " drone=" << frame.drone_id
                  << " lat=" << ConsoleScalar(frame.validity.position, frame.lat_deg)
                  << " lon=" << ConsoleScalar(frame.validity.position, frame.lon_deg)
                  << std::setprecision(kTelemetryValuePrecision) << " alt="
                  << ConsoleScalar(frame.validity.relative_altitude, frame.rel_alt_m, "m")
                  << " bat=" << ConsoleScalar(frame.validity.battery, frame.battery_percent, "%")
                  << " gps_fix=" << ConsoleScalar(frame.validity.gps, frame.gps_fix_type)
                  << " sats=" << ConsoleScalar(frame.validity.gps, frame.satellites_visible)
                  << " hdop=" << ConsoleScalar(frame.validity.gps_hdop, frame.gps_hdop)
                  << " ekf=" << estimator_text
                  << " armed=" << ConsoleBool(frame.validity.armed, frame.armed)
                  << " landed=" << ConsoleBool(frame.validity.landed, frame.landed)
                  << " failsafe=" << ConsoleBool(frame.validity.failsafe, frame.failsafe)
                  << " mode=" << (frame.validity.mode ? frame.mode : "unknown") << "\n";
    }

    const std::string csv_line = TelemetryCsvLine(delivery);
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

bool TelemetrySink::WritesFiles() const {
    return combined_file_ != nullptr || !per_drone_dir_.empty();
}

std::expected<std::ofstream, std::string> TelemetrySink::OpenCsvFile(
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
        file << "agent_receive_unix_time_ms,agent_receive_monotonic_time_ns,"
                "sdk_receive_unix_time_ms,agent_session_id,telemetry_sequence,"
                "transport_stream_id,drone_id,lat_deg,lon_deg,"
                "rel_alt_m,battery_percent,mode,gps_fix_type,gps_hdop,horizontal_accuracy_m,"
                "estimator_state,goal_id,goal_revision,physical_attempt_id,"
                "physical_attempt_revision,operation_id\n";
    }
    return file;
}

std::ofstream* TelemetrySink::GetDroneFile(const std::string& drone_id) {
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

}  // namespace swarmkit::apps::cli::internal
