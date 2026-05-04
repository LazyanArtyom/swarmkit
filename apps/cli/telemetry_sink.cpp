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

}  // namespace

std::string TelemetryCsvLine(const swarmkit::core::TelemetryFrame& frame) {
    std::ostringstream line;
    line << frame.unix_time_ms << "," << CsvField(frame.drone_id) << std::setprecision(10) << ","
         << frame.lat_deg << "," << frame.lon_deg << "," << std::setprecision(5) << frame.rel_alt_m
         << "," << frame.battery_percent << "," << CsvField(frame.mode) << "\n";
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

void TelemetrySink::Write(const swarmkit::core::TelemetryFrame& frame) {
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
        file << "unix_time_ms,drone_id,lat_deg,lon_deg,rel_alt_m,battery_percent,mode\n";
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
