// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <expected>
#include <iosfwd>
#include <string>
#include <string_view>
#include <unordered_map>

#include "swarmkit/client/client.h"

namespace swarmkit::client {

enum class TrajectoryFileFormat : std::uint8_t {
    kAuto,
    kYaml,
    kJsonLines,
    kCsv,
};

struct TrajectoryLoadOptions {
    std::string fallback_drone_id{"default"};
    std::string default_execution_id;
    std::string default_frame{"global"};
    std::uint64_t default_revision{};
    TrajectoryValidationPolicy default_validation;
    std::unordered_map<std::string, std::string> default_labels;
};

[[nodiscard]] std::expected<TrajectoryFileFormat, std::string> ParseTrajectoryFileFormat(
    std::string_view format);

[[nodiscard]] TrajectoryFileFormat DetectTrajectoryFileFormat(std::string_view path);

[[nodiscard]] std::expected<TrajectoryPlan, std::string> LoadTrajectoryPlan(
    std::istream& input, TrajectoryFileFormat format,
    const TrajectoryLoadOptions& options = {});

[[nodiscard]] std::expected<TrajectoryPlan, std::string> LoadTrajectoryPlanFile(
    std::string_view path, TrajectoryFileFormat format = TrajectoryFileFormat::kAuto,
    const TrajectoryLoadOptions& options = {});

}  // namespace swarmkit::client
