// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include "mavlink_common.h"
#include "mavlink_state_cache.h"

namespace swarmkit::agent::mavlink {

struct MavlinkTelemetryDecodeResult {
    bool should_publish{false};
    bool should_request_intervals{false};
};

class MavlinkTelemetryDecoder {
   public:
    [[nodiscard]] MavlinkTelemetryDecodeResult Decode(const mavlink_message_t& message,
                                                      TelemetryCache* telemetry_cache,
                                                      MavlinkStateCache* state_cache);

   private:
    bool message_intervals_requested_{false};
};

}  // namespace swarmkit::agent::mavlink
