// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <string_view>

namespace swarmkit::apps::cli::internal {

inline constexpr std::string_view kDefaultAddr = "127.0.0.1:50061";
inline constexpr std::string_view kDefaultCommand = "ping";
inline constexpr std::string_view kDefaultDroneId = "default";
inline constexpr std::string_view kDefaultTakeoffAlt = "10";
inline constexpr std::string_view kDefaultWaypointCoord = "0";
inline constexpr std::string_view kDefaultWaypointSpeed = "0";
inline constexpr std::string_view kDefaultTelemetryRate = "1";
inline constexpr std::string_view kDefaultCliLogLevel = "warn";
inline constexpr std::string_view kCliClientId = "swarmkit-cli";
inline constexpr std::string_view kDefaultAddressMode = "primary";
inline constexpr std::string_view kDefaultDurationMs = "0";
inline constexpr std::string_view kDefaultZero = "0";
inline constexpr std::string_view kDefaultCustomMode = "-1";
inline constexpr std::string_view kDefaultPriority = "supervisor";

inline constexpr std::uint16_t kMavCmdNavWaypoint = 16;
inline constexpr std::uint16_t kMavCmdNavTakeoff = 22;
inline constexpr std::uint16_t kMavCmdNavLand = 21;
inline constexpr std::uint16_t kMavCmdNavReturnToLaunch = 20;

inline constexpr int kTelemetryPollIntervalMs = 100;
inline constexpr int kDefaultSequenceTelemetryRateHz = 5;
inline constexpr int kTelemetryCoordPrecision = 5;
inline constexpr int kTelemetryValuePrecision = 1;

}  // namespace swarmkit::apps::cli::internal
