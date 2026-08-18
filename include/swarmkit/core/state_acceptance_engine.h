// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include "swarmkit/core/clock_quality.h"
#include "swarmkit/core/evidence_record.h"
#include "swarmkit/core/evidence_store.h"
#include "swarmkit/core/state_quality_contract.h"

namespace swarmkit::core {

/// Rejection reason for a single predicate failure (§47).
enum class RejectionReason : std::uint8_t {
    kMissingRequiredEvidence,
    kCausalSampleUnavailable,
    kAgeExceeded,
    kClockUncertaintyExceeded,
    kStateUncertaintyExceeded,
    kEstimatorUnhealthy,
    kFrameMismatch,
    kStaleAgentEpoch,
    kMissionRevisionMismatch,
    kProvenanceMismatch,
    kIncompleteAgentSet,
    kUnsupportedUncertaintySemantics,
    kMissingClockQuality,
    kMissingSourceTimestamp,
};

/// One predicate failure with context.
struct PredicateFailure {
    RejectionReason reason{};
    std::string agent_id;
    EvidenceFieldId field{};
    std::string detail;
};

/// Accepted evidence state for one agent and one field.
struct AcceptedFieldState {
    /// The evidence record that was accepted.
    EvidenceRecord evidence;

    /// Generation-time interval in reference domain [g⁻, g⁺].
    GenerationTimeInterval generation_interval;

    /// Conservative elapsed time Δ⁺ = t* - g⁻ in milliseconds.
    double conservative_elapsed_ms{};

    /// Observation-time uncertainty e_p (from the estimator).
    double observation_uncertainty{};

    /// Propagated uncertainty ε(t*) = e_p + V_max · Δ⁺ (§9 Eq.10).
    double propagated_uncertainty{};

    /// Clock uncertainty ρ used for this evidence item.
    double clock_uncertainty_ms{};
};

/// Complete accepted multi-UAV snapshot at a common evaluation time.
struct AcceptedSnapshot {
    /// Unique snapshot identifier.
    std::string snapshot_id;

    /// Common evaluation time t* (§8).
    double evaluation_time_ms{};

    /// Contract that was satisfied.
    std::string contract_id;
    std::uint32_t contract_version{};
    std::string contract_hash;

    /// Propagation model binding (§14 item M).
    std::string model_id;
    std::string model_version;

    /// Per-agent, per-field accepted state.
    /// Key: agent_id → field_id → AcceptedFieldState
    std::unordered_map<std::string,
                       std::unordered_map<std::uint8_t, AcceptedFieldState>>
        agent_states;

    /// Agents that contributed to this snapshot.
    std::vector<std::string> accepted_agents;

    /// Timestamp when the snapshot was produced (reference clock, Unix ms).
    std::int64_t produced_at_ms{};
};

/// Structured rejection result with per-predicate failure details.
struct StructuredRejection {
    /// Evaluation time that was requested.
    double evaluation_time_ms{};

    /// Contract that could not be satisfied.
    std::string contract_id;

    /// All predicate failures that prevented acceptance.
    std::vector<PredicateFailure> failures;
};

/// Result of a state acceptance request: either an accepted snapshot
/// or a structured rejection.  There is no third state — the runtime
/// never returns "accepted with warnings" (§11: no silent downgrade).
using AcceptanceResult = std::variant<AcceptedSnapshot, StructuredRejection>;

/// Configuration for the state acceptance engine.
struct AcceptanceEngineConfig {
    /// When true, generate a unique snapshot ID for each acceptance.
    bool generate_snapshot_ids{true};
};

/// State Acceptance Engine (§8–§12, §19.3).
///
/// The engine receives a contract C and evaluation time t*, selects
/// causally eligible evidence, propagates uncertainty to t*, evaluates
/// all mandatory contract predicates, and returns ACCEPTED or REJECTED.
///
/// This is the core dissertation mechanism: it determines whether
/// physical observations are sufficient to justify an application-visible
/// state at the requested time.
class StateAcceptanceEngine {
   public:
    explicit StateAcceptanceEngine(AcceptanceEngineConfig config = {});

    /// Request a multi-UAV state snapshot satisfying contract C at time t*.
    ///
    /// @param contract        State-Quality Contract defining requirements.
    /// @param evaluation_time Requested common evaluation time t* (Unix ms).
    /// @param evidence        Evidence store containing per-agent observations.
    /// @param clock_states    Per-agent clock quality state (agent_id → state).
    /// @return AcceptedSnapshot on success, StructuredRejection on failure.
    [[nodiscard]] AcceptanceResult RequestSnapshot(
        const StateQualityContract& contract,
        double evaluation_time_ms,
        const EvidenceStore& evidence,
        const std::unordered_map<std::string, ClockQualityState>& clock_states)
        const;

   private:
    AcceptanceEngineConfig config_;
    std::uint64_t next_snapshot_id_{1};

    /// Select the best causal evidence for one agent and one field.
    struct EvidenceSelection {
        EvidenceRecord record;
        GenerationTimeInterval generation_interval;
        double clock_uncertainty_ms{};
    };

    [[nodiscard]] std::optional<EvidenceSelection> SelectCausalEvidence(
        const EvidenceStore& evidence,
        const std::string& agent_id,
        EvidenceFieldId field,
        double evaluation_time_ms,
        const std::unordered_map<std::string, ClockQualityState>& clock_states)
        const;

    /// Evaluate a single field's evidence against the contract.
    [[nodiscard]] std::optional<PredicateFailure> EvaluateFieldPredicates(
        const StateQualityContract& contract,
        const std::string& agent_id,
        EvidenceFieldId field,
        const EvidenceSelection& selection,
        double evaluation_time_ms,
        double propagated_uncertainty,
        const std::string& current_session_id) const;
};

}  // namespace swarmkit::core
