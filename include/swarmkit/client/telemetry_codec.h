// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#pragma once

#include <cstdint>
#include <optional>

#include "swarmkit/client/client.h"
#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::client {

/// Convert one canonical protobuf frame to the public domain type. The
/// optional consumer receive time is transport-local and never inferred for
/// offline replay.
[[nodiscard]] TelemetryDelivery DecodeTelemetryFrame(
    const swarmkit::v1::TelemetryFrame& frame,
    std::optional<std::int64_t> consumer_receive_unix_time_ms = std::nullopt);

/// Stateful sequence classifier shared by live SDK streaming and log replay.
class TelemetrySequenceTracker {
   public:
    [[nodiscard]] const std::string& AgentSessionId() const noexcept;
    [[nodiscard]] std::uint64_t LastAcceptedSequence() const noexcept;

    void AdoptSession(std::string agent_session_id, std::uint64_t last_accepted_sequence = 0,
                      bool mark_next_frame_as_new_session = false);
    void SetLastAcceptedSequence(std::uint64_t sequence) noexcept;

    [[nodiscard]] TelemetryFrameObservation Observe(TelemetryDelivery delivery,
                                                    bool replayed = false);

   private:
    std::string agent_session_id_;
    std::uint64_t last_accepted_sequence_{};
    bool next_frame_is_new_session_{false};
};

}  // namespace swarmkit::client
