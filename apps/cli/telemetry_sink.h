// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <expected>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>

#include "swarmkit/core/telemetry.h"

namespace swarmkit::apps::cli::internal {

[[nodiscard]] std::string TelemetryCsvLine(const swarmkit::core::TelemetryFrame& frame);

class TelemetrySink {
   public:
    [[nodiscard]] static std::expected<std::unique_ptr<TelemetrySink>, std::string> FromArgs(
        int argc, char** argv, bool allow_split);

    void Write(const swarmkit::core::TelemetryFrame& frame);

    [[nodiscard]] bool WritesFiles() const;

   private:
    [[nodiscard]] static std::expected<std::ofstream, std::string> OpenCsvFile(
        const std::filesystem::path& path);
    [[nodiscard]] std::ofstream* GetDroneFile(const std::string& drone_id);

    bool console_enabled_{true};
    std::unique_ptr<std::ofstream> combined_file_;
    std::filesystem::path per_drone_dir_;
    std::unordered_map<std::string, std::unique_ptr<std::ofstream>> per_drone_files_;
    std::mutex mutex_;
};

}  // namespace swarmkit::apps::cli::internal
