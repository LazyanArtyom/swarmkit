// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/experiment/telemetry_replay.h"

#include <exception>
#include <unordered_map>
#include <utility>

#include "swarmkit/client/telemetry_codec.h"

namespace swarmkit::experiment {

core::Result ReplayNormalizedTelemetry(const evidence::ExecutionLog& log,
                                       const client::TelemetryObservationHandler& handler,
                                       const TelemetryReplayOptions& options) {
    if (!handler) {
        return core::Result::Rejected("telemetry replay handler must not be empty");
    }
    std::unordered_map<std::string, client::TelemetrySequenceTracker> trackers;
    std::size_t frame_count = 0;
    try {
        for (const auto& event : log.events) {
            if (!event.has_telemetry()) {
                continue;
            }
            const auto& recorded = event.telemetry();
            if (!options.drone_id.empty() && recorded.drone_id() != options.drone_id) {
                continue;
            }
            client::TelemetryDelivery delivery = client::DecodeTelemetryFrame(recorded);
            client::TelemetryFrameObservation frame =
                trackers[recorded.drone_id()].Observe(std::move(delivery), true);
            client::TelemetryObservation observation{std::move(frame)};
            handler(observation);
            ++frame_count;
        }
    } catch (const std::exception& exception) {
        return core::Result::Failed(std::string("telemetry replay callback failed: ") +
                                    exception.what());
    } catch (...) {
        return core::Result::Failed("telemetry replay callback failed");
    }
    if (options.require_at_least_one_frame && frame_count == 0) {
        return core::Result::Rejected("execution log contains no matching telemetry frames");
    }
    return core::Result::Success();
}

}  // namespace swarmkit::experiment
