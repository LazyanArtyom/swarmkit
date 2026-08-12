// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/client/telemetry_codec.h"

#include <utility>

namespace swarmkit::client {

const std::string& TelemetrySequenceTracker::AgentSessionId() const noexcept {
    return agent_session_id_;
}

std::uint64_t TelemetrySequenceTracker::LastAcceptedSequence() const noexcept {
    return last_accepted_sequence_;
}

void TelemetrySequenceTracker::AdoptSession(std::string agent_session_id,
                                            std::uint64_t last_accepted_sequence,
                                            bool mark_next_frame_as_new_session) {
    agent_session_id_ = std::move(agent_session_id);
    last_accepted_sequence_ = last_accepted_sequence;
    next_frame_is_new_session_ = mark_next_frame_as_new_session;
}

void TelemetrySequenceTracker::SetLastAcceptedSequence(std::uint64_t sequence) noexcept {
    last_accepted_sequence_ = sequence;
}

TelemetryFrameObservation TelemetrySequenceTracker::Observe(TelemetryDelivery delivery,
                                                            bool replayed) {
    const std::string& frame_session = delivery.frame.agent_session_id;
    const std::uint64_t frame_sequence = delivery.frame.telemetry_sequence;
    const std::uint64_t previous_sequence = last_accepted_sequence_;
    TelemetrySequenceRelation relation = TelemetrySequenceRelation::kFirst;
    std::uint64_t missing_first = 0;
    std::uint64_t missing_last = 0;

    if (next_frame_is_new_session_) {
        relation = TelemetrySequenceRelation::kNewSession;
        agent_session_id_ = frame_session;
        last_accepted_sequence_ = frame_sequence;
        next_frame_is_new_session_ = false;
    } else if (!agent_session_id_.empty() && frame_session != agent_session_id_) {
        relation = TelemetrySequenceRelation::kNewSession;
        agent_session_id_ = frame_session;
        last_accepted_sequence_ = frame_sequence;
    } else {
        if (agent_session_id_.empty()) {
            agent_session_id_ = frame_session;
        }
        if (previous_sequence == 0) {
            relation = TelemetrySequenceRelation::kFirst;
            last_accepted_sequence_ = frame_sequence;
        } else if (frame_sequence == previous_sequence + 1) {
            relation = TelemetrySequenceRelation::kNext;
            last_accepted_sequence_ = frame_sequence;
        } else if (frame_sequence > previous_sequence + 1) {
            relation = TelemetrySequenceRelation::kGap;
            missing_first = previous_sequence + 1;
            missing_last = frame_sequence - 1;
            last_accepted_sequence_ = frame_sequence;
        } else if (frame_sequence == previous_sequence) {
            relation = TelemetrySequenceRelation::kDuplicate;
        } else {
            relation = TelemetrySequenceRelation::kReordered;
        }
    }

    return {
        .delivery = std::move(delivery),
        .sequence_relation = relation,
        .previous_accepted_sequence = previous_sequence,
        .missing_first_sequence = missing_first,
        .missing_last_sequence = missing_last,
        .replayed = replayed,
    };
}

}  // namespace swarmkit::client
