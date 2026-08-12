// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#pragma once

#include <string>

#include "swarmkit/client/client.h"
#include "swarmkit/evidence/execution_log.h"

namespace swarmkit::experiment {

struct TelemetryReplayOptions {
    /// Empty replays every drone in the evidence log.
    std::string drone_id;
    bool require_at_least_one_frame{true};
};

/// Replay already-normalized telemetry through the SDK's canonical observation
/// interface. Recorded producer sequence, session, receive time, lineage, and
/// provenance are preserved exactly; SDK receive time remains unavailable.
[[nodiscard]] core::Result ReplayNormalizedTelemetry(
    const evidence::ExecutionLog& log, const client::TelemetryObservationHandler& handler,
    const TelemetryReplayOptions& options = {});

}  // namespace swarmkit::experiment
