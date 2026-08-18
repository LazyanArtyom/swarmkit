// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/core/state_acceptance_engine.h"

#include <chrono>
#include <cmath>
#include <sstream>

namespace swarmkit::core {

StateAcceptanceEngine::StateAcceptanceEngine(AcceptanceEngineConfig config)
    : config_(std::move(config)) {}

// ---------------------------------------------------------------------------
// Evidence selection: find the most recent causal evidence (§8)
// ---------------------------------------------------------------------------

std::optional<StateAcceptanceEngine::EvidenceSelection>
StateAcceptanceEngine::SelectCausalEvidence(
    const EvidenceStore& evidence,
    const std::string& agent_id,
    EvidenceFieldId field,
    double evaluation_time_ms,
    const std::unordered_map<std::string, ClockQualityState>& clock_states)
    const {

    // Retrieve the most recent evidence records for this agent/field.
    // We check up to 16 candidates to find the newest causal one.
    auto candidates = evidence.Recent(agent_id, field, 16);

    for (const auto& record : candidates) {
        // Compute generation-time interval.
        std::optional<GenerationTimeInterval> interval;

        auto clock_it = clock_states.find(agent_id);
        if (clock_it != clock_states.end() && clock_it->second.IsValid()) {
            // Use persistent clock quality state.
            interval = ComputeGenerationInterval(
                record.source_time, clock_it->second);
        } else {
            // Fall back to per-sample clock evidence.
            interval = ComputeGenerationIntervalFromSample(record.source_time);
        }

        if (!interval.has_value()) continue;

        // Causal check: g⁺ ≤ t* (§8 Eq.7).
        if (!interval->IsCausal(evaluation_time_ms)) continue;

        const double clock_unc =
            (clock_it != clock_states.end() && clock_it->second.IsValid())
                ? clock_it->second.uncertainty_radius_ms
                : record.source_time.clock_uncertainty_ms.value_or(0.0);

        return EvidenceSelection{
            .record = record,
            .generation_interval = *interval,
            .clock_uncertainty_ms = clock_unc,
        };
    }

    return std::nullopt;
}

// ---------------------------------------------------------------------------
// Per-field predicate evaluation
// ---------------------------------------------------------------------------

std::optional<PredicateFailure>
StateAcceptanceEngine::EvaluateFieldPredicates(
    const StateQualityContract& contract,
    const std::string& agent_id,
    EvidenceFieldId field,
    const EvidenceSelection& selection,
    double evaluation_time_ms,
    double propagated_uncertainty,
    const std::string& current_session_id) const {

    const auto& record = selection.record;
    const auto& interval = selection.generation_interval;

    // Age check (A): Δ⁺ ≤ max_evidence_age.
    if (contract.max_evidence_age_ms.has_value()) {
        const double elapsed = interval.ConservativeElapsed(evaluation_time_ms);
        if (elapsed > *contract.max_evidence_age_ms) {
            return PredicateFailure{
                .reason = RejectionReason::kAgeExceeded,
                .agent_id = agent_id,
                .field = field,
                .detail = "Δ⁺=" + std::to_string(elapsed) +
                          "ms > max=" + std::to_string(*contract.max_evidence_age_ms) + "ms",
            };
        }
    }

    // Clock uncertainty check (R): ρ ≤ max_clock_uncertainty.
    if (contract.max_clock_uncertainty_ms.has_value()) {
        if (selection.clock_uncertainty_ms > *contract.max_clock_uncertainty_ms) {
            return PredicateFailure{
                .reason = RejectionReason::kClockUncertaintyExceeded,
                .agent_id = agent_id,
                .field = field,
                .detail = "ρ=" + std::to_string(selection.clock_uncertainty_ms) +
                          "ms > max=" + std::to_string(*contract.max_clock_uncertainty_ms) + "ms",
            };
        }
    }

    // Propagated uncertainty check (U): ε(t*) ≤ max threshold.
    if (field == EvidenceFieldId::kPosition &&
        contract.max_position_uncertainty_m.has_value()) {
        if (propagated_uncertainty > *contract.max_position_uncertainty_m) {
            return PredicateFailure{
                .reason = RejectionReason::kStateUncertaintyExceeded,
                .agent_id = agent_id,
                .field = field,
                .detail = "ε=" + std::to_string(propagated_uncertainty) +
                          "m > max=" + std::to_string(*contract.max_position_uncertainty_m) + "m",
            };
        }
    }
    if (field == EvidenceFieldId::kVelocity &&
        contract.max_velocity_uncertainty_mps.has_value()) {
        if (propagated_uncertainty > *contract.max_velocity_uncertainty_mps) {
            return PredicateFailure{
                .reason = RejectionReason::kStateUncertaintyExceeded,
                .agent_id = agent_id,
                .field = field,
                .detail = "ε=" + std::to_string(propagated_uncertainty) +
                          "m/s > max=" + std::to_string(*contract.max_velocity_uncertainty_mps) +
                          "m/s",
            };
        }
    }

    // Health predicates (H).
    if (contract.require_estimator_healthy &&
        (field == EvidenceFieldId::kPosition || field == EvidenceFieldId::kVelocity)) {
        if (!record.quality.estimator_healthy) {
            return PredicateFailure{
                .reason = RejectionReason::kEstimatorUnhealthy,
                .agent_id = agent_id,
                .field = field,
                .detail = "estimator not healthy",
            };
        }
    }
    if (contract.require_estimator_position_ok && field == EvidenceFieldId::kPosition) {
        if (!record.quality.estimator_position_ok) {
            return PredicateFailure{
                .reason = RejectionReason::kEstimatorUnhealthy,
                .agent_id = agent_id,
                .field = field,
                .detail = "estimator position not ok",
            };
        }
    }
    if (contract.require_estimator_velocity_ok && field == EvidenceFieldId::kVelocity) {
        if (!record.quality.estimator_velocity_ok) {
            return PredicateFailure{
                .reason = RejectionReason::kEstimatorUnhealthy,
                .agent_id = agent_id,
                .field = field,
                .detail = "estimator velocity not ok",
            };
        }
    }

    // Frame predicates (P).
    if (field == EvidenceFieldId::kPosition &&
        contract.required_position_frame != CoordinateFrame::kUnknown) {
        if (record.identity.coordinate_frame != contract.required_position_frame) {
            return PredicateFailure{
                .reason = RejectionReason::kFrameMismatch,
                .agent_id = agent_id,
                .field = field,
                .detail = "position frame mismatch",
            };
        }
    }
    if (field == EvidenceFieldId::kVelocity &&
        contract.required_velocity_frame != CoordinateFrame::kUnknown) {
        if (record.identity.coordinate_frame != contract.required_velocity_frame) {
            return PredicateFailure{
                .reason = RejectionReason::kFrameMismatch,
                .agent_id = agent_id,
                .field = field,
                .detail = "velocity frame mismatch",
            };
        }
    }

    // Agent epoch predicate (P/§6): E_msg == E_cur.
    if (contract.require_current_epoch) {
        if (record.identity.agent_session_id != current_session_id) {
            return PredicateFailure{
                .reason = RejectionReason::kStaleAgentEpoch,
                .agent_id = agent_id,
                .field = field,
                .detail = "evidence session '" + record.identity.agent_session_id +
                          "' != current '" + current_session_id + "'",
            };
        }
    }

    // Mission predicate (M).
    if (contract.require_current_mission) {
        if (record.identity.mission_id != contract.required_mission_id ||
            record.identity.mission_revision != contract.required_mission_revision) {
            return PredicateFailure{
                .reason = RejectionReason::kMissionRevisionMismatch,
                .agent_id = agent_id,
                .field = field,
                .detail = "mission mismatch",
            };
        }
    }

    // Deterministic bound semantics check (§10).
    if (contract.require_deterministic_bounds &&
        record.quality.uncertainty.has_value()) {
        const auto sem = record.quality.uncertainty->descriptor.semantics;
        if (sem != UncertaintySemantics::kDeterministicHardBound &&
            sem != UncertaintySemantics::kEmpiricallyCalibratedBound &&
            sem != UncertaintySemantics::kUnknown) {
            return PredicateFailure{
                .reason = RejectionReason::kUnsupportedUncertaintySemantics,
                .agent_id = agent_id,
                .field = field,
                .detail = "contract requires deterministic bounds but evidence has "
                          "probabilistic uncertainty",
            };
        }
    }

    return std::nullopt;  // All predicates passed.
}

// ---------------------------------------------------------------------------
// Main acceptance decision
// ---------------------------------------------------------------------------

AcceptanceResult StateAcceptanceEngine::RequestSnapshot(
    const StateQualityContract& contract,
    double evaluation_time_ms,
    const EvidenceStore& evidence,
    const std::unordered_map<std::string, ClockQualityState>& clock_states)
    const {

    std::vector<PredicateFailure> failures;

    // Determine the set of agents to evaluate.
    const auto& required = contract.required_agents;
    if (required.empty()) {
        failures.push_back({
            .reason = RejectionReason::kIncompleteAgentSet,
            .detail = "no required agents specified in contract",
        });
        return StructuredRejection{
            .evaluation_time_ms = evaluation_time_ms,
            .contract_id = contract.contract_id,
            .failures = std::move(failures),
        };
    }

    // Compute contract hash for certificate binding.
    const std::string contract_hash = ComputeContractHash(contract);

    AcceptedSnapshot snapshot;
    snapshot.evaluation_time_ms = evaluation_time_ms;
    snapshot.contract_id = contract.contract_id;
    snapshot.contract_version = contract.content_version;
    snapshot.contract_hash = contract_hash;
    snapshot.model_id = contract.propagation_model_id;
    snapshot.model_version = contract.propagation_model_version;

    std::size_t agents_accepted = 0;

    for (const auto& agent_id : required) {
        // Determine current session for epoch check.
        const auto current_session = evidence.CurrentSessionId(agent_id);
        const std::string current_session_id =
            current_session.value_or("");

        bool agent_ok = true;

        for (const auto field : contract.required_fields) {
            // Select causal evidence.
            auto selection = SelectCausalEvidence(
                evidence, agent_id, field, evaluation_time_ms, clock_states);

            if (!selection.has_value()) {
                // Try to distinguish missing evidence from non-causal.
                auto any_evidence = evidence.Recent(agent_id, field, 1);
                if (any_evidence.empty()) {
                    failures.push_back({
                        .reason = RejectionReason::kMissingRequiredEvidence,
                        .agent_id = agent_id,
                        .field = field,
                        .detail = "no evidence available",
                    });
                } else {
                    failures.push_back({
                        .reason = RejectionReason::kCausalSampleUnavailable,
                        .agent_id = agent_id,
                        .field = field,
                        .detail = "no causal evidence with g⁺ ≤ t*",
                    });
                }
                agent_ok = false;
                continue;
            }

            // Check source timestamp availability.
            if (!selection->record.source_time.timestamp_ms.has_value()) {
                failures.push_back({
                    .reason = RejectionReason::kMissingSourceTimestamp,
                    .agent_id = agent_id,
                    .field = field,
                    .detail = "evidence has no source timestamp",
                });
                agent_ok = false;
                continue;
            }

            // Compute propagated uncertainty (§9 Eq.10).
            const double conservative_elapsed =
                selection->generation_interval.ConservativeElapsed(evaluation_time_ms);

            double observation_unc = 0.0;
            if (selection->record.quality.uncertainty.has_value()) {
                observation_unc = static_cast<double>(
                    selection->record.quality.uncertainty->value);
            }

            double max_speed = 0.0;
            if (field == EvidenceFieldId::kPosition) {
                max_speed = static_cast<double>(contract.max_horizontal_speed_mps);
            } else if (field == EvidenceFieldId::kVelocity) {
                // For velocity, propagation uses acceleration bound.
                // For now, use 0 (velocity uncertainty doesn't grow with time
                // in the simple ball model).  A more sophisticated model
                // can replace this.
                max_speed = 0.0;
            }

            const double propagated_unc = ComputePropagatedUncertainty(
                observation_unc, max_speed, conservative_elapsed);

            // Evaluate all predicates.
            auto failure = EvaluateFieldPredicates(
                contract, agent_id, field, *selection,
                evaluation_time_ms, propagated_unc, current_session_id);

            if (failure.has_value()) {
                failures.push_back(std::move(*failure));
                agent_ok = false;
                continue;
            }

            // Record accepted field state.
            snapshot.agent_states[agent_id][static_cast<std::uint8_t>(field)] = {
                .evidence = selection->record,
                .generation_interval = selection->generation_interval,
                .conservative_elapsed_ms = conservative_elapsed,
                .observation_uncertainty = observation_unc,
                .propagated_uncertainty = propagated_unc,
                .clock_uncertainty_ms = selection->clock_uncertainty_ms,
            };
        }

        if (agent_ok) {
            snapshot.accepted_agents.push_back(agent_id);
            ++agents_accepted;
        }
    }

    // Completeness check (S).
    bool completeness_ok = false;
    switch (contract.completeness) {
        case CompletenessRule::kAllRequired:
            completeness_ok = (agents_accepted == required.size());
            break;
        case CompletenessRule::kMinimumCount:
            completeness_ok = (agents_accepted >= contract.min_required_agents);
            break;
    }

    if (!completeness_ok) {
        failures.push_back({
            .reason = RejectionReason::kIncompleteAgentSet,
            .detail = "accepted " + std::to_string(agents_accepted) + "/" +
                      std::to_string(required.size()) + " required agents",
        });
    }

    // Final decision: any failure means REJECTED — no silent downgrade (§11).
    if (!failures.empty()) {
        return StructuredRejection{
            .evaluation_time_ms = evaluation_time_ms,
            .contract_id = contract.contract_id,
            .failures = std::move(failures),
        };
    }

    // Generate snapshot ID.
    if (config_.generate_snapshot_ids) {
        snapshot.snapshot_id = "snap-" + std::to_string(
            const_cast<StateAcceptanceEngine*>(this)->next_snapshot_id_++);
    }

    // Production timestamp.
    snapshot.produced_at_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    return snapshot;
}

}  // namespace swarmkit::core
