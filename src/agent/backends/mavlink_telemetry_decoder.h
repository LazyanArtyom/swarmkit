// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <chrono>
#include <optional>

#include "mavlink_common.h"
#include "mavlink_state_cache.h"

namespace swarmkit::agent::mavlink {

struct MavlinkTelemetryDecodeResult {
    bool should_publish{false};
    bool should_request_intervals{false};
    core::TelemetryProvenance provenance;
};

/// Coalesces independently arriving MAVLink measurements into one normalized
/// producer frame cadence. The latest provenance for every measurement group
/// is retained until the next frame is emitted.
class MavlinkTelemetryCoalescer {
   public:
    explicit MavlinkTelemetryCoalescer(int rate_hz);

    void Reset(int rate_hz);

    [[nodiscard]] std::optional<core::TelemetryProvenance> Push(
        const core::TelemetryProvenance& update, std::chrono::steady_clock::time_point now);

   private:
    int rate_hz_{1};
    std::optional<std::chrono::steady_clock::time_point> last_publish_time_;
    core::TelemetryProvenance pending_;
};

class MavlinkTelemetryDecoder {
   public:
    [[nodiscard]] MavlinkTelemetryDecodeResult Decode(const mavlink_message_t& message,
                                                      TelemetryCache* telemetry_cache,
                                                      MavlinkStateCache* state_cache,
                                                      MavlinkAutopilotProfile profile);

   private:
    bool message_intervals_requested_{false};
};

}  // namespace swarmkit::agent::mavlink
