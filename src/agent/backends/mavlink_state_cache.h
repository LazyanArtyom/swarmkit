// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <mutex>

#include "mavlink_common.h"
#include "swarmkit/agent/backend.h"

namespace swarmkit::agent::mavlink {

class MavlinkStateCache {
   public:
    void UpdateHeartbeat(const mavlink_message_t& message, const mavlink_heartbeat_t& heartbeat);
    void UpdateTelemetry(const mavlink_message_t& message);
    void UpdateGlobalPosition(const mavlink_message_t& message,
                              const mavlink_global_position_int_t& position);
    void UpdateGps(const mavlink_message_t& message, const mavlink_gps_raw_int_t& gps);
    void UpdateSysStatus(const mavlink_message_t& message, const mavlink_sys_status_t& sys_status);
    void UpdateExtendedSysState(const mavlink_message_t& message,
                                const mavlink_extended_sys_state_t& state);
    void UpdateEstimatorStatus(const mavlink_message_t& message,
                               const mavlink_estimator_status_t& estimator);
    void UpdateEkfStatus(const mavlink_message_t& message,
                         const mavlink_ekf_status_report_t& ekf);

    [[nodiscard]] MavlinkVehicleState Snapshot() const;
    [[nodiscard]] BackendHealth Health() const;

   private:
    mutable std::mutex mutex_;
    MavlinkVehicleState state_;
};

}  // namespace swarmkit::agent::mavlink
