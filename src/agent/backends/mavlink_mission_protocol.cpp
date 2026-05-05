// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "mavlink_mission_protocol.h"

#include <algorithm>
#include <cmath>
#include <expected>
#include <string>

#include "swarmkit/core/logger.h"

namespace swarmkit::agent::mavlink {
namespace {

[[nodiscard]] std::expected<std::uint16_t, core::Result> MissionCommandForItem(
    const commands::MissionItem& item) {
    if (item.backend_command != 0U) {
        return item.backend_command;
    }

    switch (item.type) {
        case commands::MissionItemType::kWaypoint:
            return MAV_CMD_NAV_WAYPOINT;
        case commands::MissionItemType::kTakeoff:
            return MAV_CMD_NAV_TAKEOFF;
        case commands::MissionItemType::kLand:
            return MAV_CMD_NAV_LAND;
        case commands::MissionItemType::kLoiter:
            return MAV_CMD_NAV_LOITER_TIME;
        case commands::MissionItemType::kDelay:
            return MAV_CMD_NAV_DELAY;
        case commands::MissionItemType::kAction:
        case commands::MissionItemType::kPayloadAction:
            return std::unexpected(core::Result::Rejected(
                "mission action items require a backend-native command id for MAVLink"));
    }
    return std::unexpected(core::Result::Rejected("unknown mission item type"));
}

[[nodiscard]] float ParamOr(float value, float fallback) {
    return value == 0.0F ? fallback : value;
}

}  // namespace

void MavlinkMissionProtocol::RecordMissionRequest(const MissionRequest& request) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_mission_request_ = request;
        ++mission_request_sequence_;
    }
    cv_.notify_all();
    core::Logger::DebugFmt("MavlinkBackend: MISSION_REQUEST seq={} type={}", request.seq,
                           static_cast<int>(request.mission_type));
}

void MavlinkMissionProtocol::RecordMissionAck(const MissionAck& ack) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_mission_ack_ = ack;
        ++mission_ack_sequence_;
    }
    cv_.notify_all();
    core::Logger::DebugFmt("MavlinkBackend: MISSION_ACK result={} type={}",
                           MissionResultName(ack.type), static_cast<int>(ack.mission_type));
}

core::Result MavlinkMissionProtocol::UploadMission(const commands::CmdUploadMission& upload,
                                                   const MavlinkBackendConfig& config,
                                                   const Sender& send_message) {
    if (!send_message) {
        return core::Result::Failed("mission protocol sender is not configured");
    }
    if (upload.items.empty()) {
        return core::Result::Rejected("mission upload requires at least one item");
    }
    if (upload.items.size() > kMissionUploadMaxItems) {
        return core::Result::Rejected("mission upload item count exceeds limit");
    }

    const std::uint64_t request_start_sequence = CaptureMissionRequestSequence();
    const std::uint64_t ack_start_sequence = CaptureMissionAckSequence();

    mavlink_message_t count_message{};
    mavlink_msg_mission_count_pack(
        config.source_system, config.source_component, &count_message, config.target_system,
        config.target_component, static_cast<std::uint16_t>(upload.items.size()),
        MAV_MISSION_TYPE_MISSION, 0);
    if (const core::Result send_result = send_message(count_message); !send_result.IsOk()) {
        return send_result;
    }

    std::uint64_t request_sequence = request_start_sequence;
    for (std::size_t sent_items = 0; sent_items < upload.items.size(); ++sent_items) {
        const auto request = WaitForMissionRequest(request_sequence, config.command_ack_timeout_ms);
        if (!request.has_value()) {
            return core::Result::Failed("timed out waiting for MISSION_REQUEST");
        }
        request_sequence = request->second;
        const std::uint16_t seq = request->first.seq;
        if (seq >= upload.items.size()) {
            return core::Result::Failed("autopilot requested invalid mission seq=" +
                                        std::to_string(seq));
        }
        if (const core::Result item_result =
                SendMissionItem(upload.items[seq], seq, config, send_message);
            !item_result.IsOk()) {
            return item_result;
        }
    }

    return WaitForMissionAck(ack_start_sequence, config.command_ack_timeout_ms);
}

core::Result MavlinkMissionProtocol::ClearMission(const MavlinkBackendConfig& config,
                                                  const Sender& send_message) {
    if (!send_message) {
        return core::Result::Failed("mission protocol sender is not configured");
    }

    mavlink_message_t message{};
    mavlink_msg_mission_clear_all_pack(config.source_system, config.source_component, &message,
                                       config.target_system, config.target_component,
                                       MAV_MISSION_TYPE_MISSION);
    const std::uint64_t start_sequence = CaptureMissionAckSequence();
    if (const core::Result send_result = send_message(message); !send_result.IsOk()) {
        return send_result;
    }
    return WaitForMissionAck(start_sequence, config.command_ack_timeout_ms);
}

core::Result MavlinkMissionProtocol::SetCurrentMissionItem(
    const commands::CmdSetCurrentMissionItem& current, const MavlinkBackendConfig& config,
    const Sender& send_message) {
    if (!send_message) {
        return core::Result::Failed("mission protocol sender is not configured");
    }

    mavlink_message_t message{};
    mavlink_msg_mission_set_current_pack(
        config.source_system, config.source_component, &message, config.target_system,
        config.target_component, static_cast<std::uint16_t>(std::max(0, current.seq)));
    return send_message(message);
}

std::uint64_t MavlinkMissionProtocol::CaptureMissionRequestSequence() {
    std::lock_guard<std::mutex> lock(mutex_);
    return mission_request_sequence_;
}

std::uint64_t MavlinkMissionProtocol::CaptureMissionAckSequence() {
    std::lock_guard<std::mutex> lock(mutex_);
    return mission_ack_sequence_;
}

std::optional<std::pair<MissionRequest, std::uint64_t>>
MavlinkMissionProtocol::WaitForMissionRequest(std::uint64_t start_sequence, int timeout_ms) {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool got_request = cv_.wait_for(lock, std::chrono::milliseconds{timeout_ms}, [&] {
        return mission_request_sequence_ != start_sequence && last_mission_request_.has_value();
    });
    if (!got_request) {
        return std::nullopt;
    }
    return std::make_pair(last_mission_request_.value_or(MissionRequest{}),
                          mission_request_sequence_);
}

core::Result MavlinkMissionProtocol::WaitForMissionAck(std::uint64_t start_sequence,
                                                       int timeout_ms) {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool got_ack = cv_.wait_for(lock, std::chrono::milliseconds{timeout_ms}, [&] {
        return mission_ack_sequence_ != start_sequence && last_mission_ack_.has_value();
    });
    if (!got_ack) {
        return core::Result::Failed("timed out waiting for MISSION_ACK");
    }

    const MissionAck ack = last_mission_ack_.value_or(MissionAck{});
    const std::string detail = "MISSION_ACK result=" + MissionResultName(ack.type) +
                               " type=" + std::to_string(ack.mission_type);
    if (ack.type == MAV_MISSION_ACCEPTED) {
        return core::Result::Success(detail);
    }
    return core::Result::Failed(detail);
}

core::Result MavlinkMissionProtocol::SendMissionItem(const commands::MissionItem& item,
                                                     std::uint16_t seq,
                                                     const MavlinkBackendConfig& config,
                                                     const Sender& send_message) {
    const auto command = MissionCommandForItem(item);
    if (!command.has_value()) {
        return command.error();
    }

    const float param1 = ParamOr(item.param1, item.hold_s);
    const float param2 = ParamOr(item.param2, item.acceptance_radius_m);
    const float param4 = ParamOr(item.param4, item.yaw_deg);

    mavlink_message_t message{};
    mavlink_msg_mission_item_int_pack(
        config.source_system, config.source_component, &message, config.target_system,
        config.target_component, seq, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, *command,
        item.current ? 1 : 0, item.autocontinue ? 1 : 0, param1, param2, item.param3,
        param4, static_cast<std::int32_t>(std::llround(item.lat_deg * kDegE7)),
        static_cast<std::int32_t>(std::llround(item.lon_deg * kDegE7)),
        static_cast<float>(item.alt_m), MAV_MISSION_TYPE_MISSION);
    return send_message(message);
}

}  // namespace swarmkit::agent::mavlink
