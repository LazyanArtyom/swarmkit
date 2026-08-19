// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "swarmkit/core/clock_quality.h"
#include "swarmkit/core/evidence_record.h"
#include "swarmkit/core/state_acceptance_certificate.h"
#include "swarmkit/core/state_acceptance_engine.h"
#include "swarmkit/core/state_quality_contract.h"

namespace swarmkit::core {

/// Event: An evidence record was received by the runtime (§8).
struct EvidenceReceivedEvent {
    std::int64_t receive_time_ms{};
    std::string agent_id;
    EvidenceRecord record;

    bool operator==(const EvidenceReceivedEvent&) const = default;
};

/// Event: Authoritative agent session transition (§6).
struct SessionTransitionEvent {
    std::int64_t timestamp_ms{};
    std::string agent_id;
    std::string new_session_id;

    bool operator==(const SessionTransitionEvent&) const = default;
};

/// Event: Clock model update for an agent (§7).
struct ClockModelUpdateEvent {
    std::int64_t timestamp_ms{};
    std::string agent_id;
    ClockQualityState clock_state;

    bool operator==(const ClockModelUpdateEvent&) const = default;
};

/// Event: Swarm membership change (§8).
struct MembershipChangeEvent {
    std::int64_t timestamp_ms{};
    ParticipantSnapshot participants;

    bool operator==(const MembershipChangeEvent&) const = default;
};

/// Event: A state acceptance request was evaluated (§8).
struct SnapshotRequestEvent {
    std::string request_id;
    double evaluation_time_ms{};      // t*
    std::int64_t evidence_freeze_ms{}; // r*
    std::string contract_hash;
    ParticipantSnapshot participants;
    std::optional<StateAcceptanceCertificate> certificate;

    bool operator==(const SnapshotRequestEvent&) const = default;
};

/// Discriminated union of all replay trace events.
using ReplayEvent = std::variant<
    EvidenceReceivedEvent,
    SessionTransitionEvent,
    ClockModelUpdateEvent,
    MembershipChangeEvent,
    SnapshotRequestEvent
>;

/// Replay trace container for persisted offline replay (§16).
struct ReplayTrace {
    std::string trace_id;
    std::string version{"1.0"};
    std::vector<ReplayEvent> events;

    /// Serialize trace to JSON-lines formatted string.
    [[nodiscard]] std::string ToJsonLines() const;

    /// Deserialize trace from JSON-lines formatted string.
    [[nodiscard]] static std::optional<ReplayTrace> FromJsonLines(std::string_view json_lines);

    /// Save trace to a file.
    [[nodiscard]] bool SaveToFile(const std::string& path) const;

    /// Load trace from a file.
    [[nodiscard]] static std::optional<ReplayTrace> LoadFromFile(const std::string& path);
};

}  // namespace swarmkit::core
