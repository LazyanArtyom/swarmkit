// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <variant>
#include <vector>

namespace swarmkit::commands {

/// ---------------------------------------------------------------------------
/// Mission commands -- upload, start, pause/resume, and mission recovery.
/// ---------------------------------------------------------------------------

struct MissionItem {
    std::uint16_t command{};  ///< MAV_CMD_* value.
    double lat_deg{};
    double lon_deg{};
    double alt_m{};
    float param1{};
    float param2{};
    float param3{};
    float param4{};
    bool current{false};
    bool autocontinue{true};
};

struct CmdUploadMission {
    std::vector<MissionItem> items;
};

struct CmdClearMission {};

struct CmdStartMission {
    int first_item{0};
    int last_item{0};  ///< 0 means autopilot default / all remaining items.
};

struct CmdPauseMission {};
struct CmdResumeMission {};

struct CmdSetCurrentMissionItem {
    int seq{};
};

using MissionCmd = std::variant<CmdUploadMission, CmdClearMission, CmdStartMission, CmdPauseMission,
                                CmdResumeMission, CmdSetCurrentMissionItem>;

}  // namespace swarmkit::commands
