// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>

#include "mavlink_common.h"
#include "swarmkit/commands/mission.h"
#include "swarmkit/core/result.h"

namespace swarmkit::agent::mavlink {

class MavlinkMissionProtocol {
   public:
    using Sender = std::function<core::Result(const mavlink_message_t&)>;

    void RecordMissionRequest(const MissionRequest& request);
    void RecordMissionAck(const MissionAck& ack);

    [[nodiscard]] core::Result UploadMission(const commands::CmdUploadMission& upload,
                                             const MavlinkBackendConfig& config,
                                             const Sender& send_message);
    [[nodiscard]] core::Result ClearMission(const MavlinkBackendConfig& config,
                                            const Sender& send_message);
    [[nodiscard]] static core::Result SetCurrentMissionItem(
        const commands::CmdSetCurrentMissionItem& current, const MavlinkBackendConfig& config,
        const Sender& send_message);

   private:
    [[nodiscard]] std::uint64_t CaptureMissionRequestSequence();
    [[nodiscard]] std::uint64_t CaptureMissionAckSequence();
    [[nodiscard]] std::optional<std::pair<MissionRequest, std::uint64_t>> WaitForMissionRequest(
        std::uint64_t start_sequence, int timeout_ms);
    [[nodiscard]] core::Result WaitForMissionAck(std::uint64_t start_sequence, int timeout_ms);
    [[nodiscard]] static core::Result SendMissionItem(const commands::MissionItem& item,
                                                      std::uint16_t seq,
                                                      const MavlinkBackendConfig& config,
                                                      const Sender& send_message);

    std::mutex mutex_;
    std::condition_variable cv_;
    std::optional<MissionRequest> last_mission_request_;
    std::optional<MissionAck> last_mission_ack_;
    std::uint64_t mission_request_sequence_{0};
    std::uint64_t mission_ack_sequence_{0};
};

}  // namespace swarmkit::agent::mavlink
