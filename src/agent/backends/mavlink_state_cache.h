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

    [[nodiscard]] MavlinkVehicleState Snapshot() const;
    [[nodiscard]] BackendHealth Health() const;

   private:
    mutable std::mutex mutex_;
    MavlinkVehicleState state_;
};

}  // namespace swarmkit::agent::mavlink
