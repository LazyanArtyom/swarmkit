// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/core/state_acceptance_engine.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

namespace swarmkit::core {

StateAcceptanceEngine::StateAcceptanceEngine(AcceptanceEngineConfig config)
    : config_(std::move(config)) {}

// ---------------------------------------------------------------------------
// Evidence selection: find the newest causally valid physical evidence (§8)
// ---------------------------------------------------------------------------

std::optional<StateAcceptanceEngine::EvidenceSelection>
StateAcceptanceEngine::SelectCausalEvidence(
    const StateQualityContract& contract,
    const EvidenceStore& evidence,
    const std::string& agent_id,
    EvidenceFieldId field,
    double evaluation_time_ms,
    const std::unordered_map<std::string, ClockQualityState>& clock_states,
    std::vector<PredicateFailure>& failures) const {

    // Retrieve all retained evidence records for this agent/field to ensure
    // we search all candidates beyond arbitrary limits (P0.4).
    auto candidates = evidence.All(agent_id, field);
    if (candidates.empty()) {
        failures.push_back({
            .reason = RejectionReason::kMissingRequiredEvidence,
            .agent_id = agent_id,
            .field = field,
            .detail = "no evidence records stored for agent " + agent_id,
        });
        return std::nullopt;
    }

    std::optional<EvidenceSelection> best_selection;
    double best_source_time = -std::numeric_limits<double>::infinity();
    bool had_clock_quality_failure = false;
    std::string clock_failure_detail;

    for (const auto& record : candidates) {
        if (!record.source_time.timestamp_ms.has_value()) {
            continue;
        }

        const double s = static_cast<double>(*record.source_time.timestamp_ms);
        if (!std::isfinite(s)) continue;

        // Compute generation-time interval [g⁻, g⁺].
        std::optional<GenerationTimeInterval> interval;
        double clock_unc = 0.0;

        auto clock_it = clock_states.find(agent_id);
        if (clock_it != clock_states.end() && clock_it->second.IsValid()) {
            if (contract.require_deterministic_bounds && !clock_it->second.deterministic_bound) {
                had_clock_quality_failure = true;
                clock_failure_detail = "clock state has non-deterministic bound for agent " + agent_id;
                continue;
            }
            interval = ComputeGenerationInterval(record.source_time, clock_it->second);
            clock_unc = clock_it->second.uncertainty_radius_ms;
        } else if (record.source_time.clock_uncertainty_ms.has_value()) {
            const double rho = *record.source_time.clock_uncertainty_ms;
            if (!std::isfinite(rho) || rho < 0.0) {
                had_clock_quality_failure = true;
                clock_failure_detail = "invalid per-sample clock uncertainty bound";
                continue;
            }
            interval = ComputeGenerationIntervalFromSample(record.source_time);
            clock_unc = rho;
        } else {
            had_clock_quality_failure = true;
            clock_failure_detail = "missing clock quality and per-sample clock uncertainty";
            continue;
        }

        if (!interval.has_value()) continue;

        // Causal condition: g⁺ ≤ t* (§8 Eq.7).
        if (!interval->IsCausal(evaluation_time_ms)) {
            continue;
        }

        // Select the candidate with the newest physical source generation time.
        if (!best_selection.has_value() || s > best_source_time) {
            best_source_time = s;
            best_selection = EvidenceSelection{
                .record = record,
                .generation_interval = *interval,
                .clock_uncertainty_ms = clock_unc,
            };
        }
    }

    if (best_selection.has_value()) {
        return best_selection;
    }

    if (had_clock_quality_failure) {
        failures.push_back({
            .reason = RejectionReason::kMissingClockQuality,
            .agent_id = agent_id,
            .field = field,
            .detail = clock_failure_detail,
        });
    } else {
        failures.push_back({
            .reason = RejectionReason::kCausalSampleUnavailable,
            .agent_id = agent_id,
            .field = field,
            .detail = "no causal evidence with g⁺ ≤ t*",
        });
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

    // Deterministic bound semantics check (§10, P0.2).
    if (contract.require_deterministic_bounds) {
        if (!record.quality.uncertainty.has_value()) {
            return PredicateFailure{
                .reason = RejectionReason::kMissingUncertainty,
                .agent_id = agent_id,
                .field = field,
                .detail = "deterministic contract requires uncertainty bound but none was provided",
            };
        }

        const auto& unc = *record.quality.uncertainty;
        if (unc.descriptor.semantics != UncertaintySemantics::kDeterministicHardBound) {
            return PredicateFailure{
                .reason = RejectionReason::kUncertaintySemanticsMismatch,
                .agent_id = agent_id,
                .field = field,
                .detail = "deterministic contract requires DeterministicHardBound semantics",
            };
        }

        if (!std::isfinite(unc.value) || unc.value < 0.0f) {
            return PredicateFailure{
                .reason = RejectionReason::kInvalidUncertaintyBound,
                .agent_id = agent_id,
                .field = field,
                .detail = "invalid uncertainty bound value",
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

    // Frame predicates (P, P0.6).
    if (field == EvidenceFieldId::kPosition) {
        if (contract.required_position_frame != CoordinateFrame::kUnknown &&
            record.identity.coordinate_frame != contract.required_position_frame) {
            return PredicateFailure{
                .reason = RejectionReason::kFrameMismatch,
                .agent_id = agent_id,
                .field = field,
                .detail = "position coordinate frame mismatch",
            };
        }
    }
    if (field == EvidenceFieldId::kVelocity) {
        if (contract.required_velocity_frame != CoordinateFrame::kUnknown &&
            record.identity.coordinate_frame != contract.required_velocity_frame) {
            return PredicateFailure{
                .reason = RejectionReason::kFrameMismatch,
                .agent_id = agent_id,
                .field = field,
                .detail = "velocity coordinate frame mismatch",
            };
        }
    }

    // Agent epoch predicate (P/§6, P0.1): E_msg == E_cur.
    if (contract.require_current_epoch) {
        if (record.identity.agent_session_id != current_session_id) {
            return PredicateFailure{
                .reason = RejectionReason::kStaleAgentEpoch,
                .agent_id = agent_id,
                .field = field,
                .detail = "evidence session '" + record.identity.agent_session_id +
                          "' != current authoritative session '" + current_session_id + "'",
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

    return std::nullopt;  // All predicates passed.
}

// ---------------------------------------------------------------------------
// Main acceptance decision (Mathematically Pure & Stateless, P0.7)
// ---------------------------------------------------------------------------

AcceptanceResult StateAcceptanceEngine::RequestSnapshot(
    const StateQualityContract& contract,
    double evaluation_time_ms,
    const EvidenceStore& evidence,
    const std::unordered_map<std::string, ClockQualityState>& clock_states)
    const {

    // Step 0: Contract validation (P0.5).
    const auto contract_valid = ValidateStateQualityContract(contract);
    if (!contract_valid.IsOk()) {
        std::vector<PredicateFailure> contract_failures;
        contract_failures.push_back({
            .reason = RejectionReason::kInvalidContract,
            .detail = contract_valid.error.user_message.empty() ? "malformed contract"
                                                                : contract_valid.error.user_message,
        });
        return StructuredRejection{
            .evaluation_time_ms = evaluation_time_ms,
            .contract_id = contract.contract_id,
            .failures = std::move(contract_failures),
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
    std::vector<PredicateFailure> all_agent_failures;

    const auto& required = contract.required_agents;

    for (const auto& agent_id : required) {
        // Determine authoritative current session for epoch check (P0.1).
        const auto current_session = evidence.CurrentSessionId(agent_id);
        const std::string current_session_id = current_session.value_or("");

        bool agent_ok = true;
        std::vector<PredicateFailure> agent_failures;

        for (const auto field : contract.required_fields) {
            // Select causal evidence (P0.4).
            auto selection = SelectCausalEvidence(
                contract, evidence, agent_id, field, evaluation_time_ms,
                clock_states, agent_failures);

            if (!selection.has_value()) {
                agent_ok = false;
                continue;
            }

            // Check source timestamp availability.
            if (!selection->record.source_time.timestamp_ms.has_value()) {
                agent_failures.push_back({
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
                max_speed = 0.0;
            }

            const double propagated_unc = ComputePropagatedUncertainty(
                observation_unc, max_speed, conservative_elapsed);

            // Evaluate all predicates.
            auto failure = EvaluateFieldPredicates(
                contract, agent_id, field, *selection,
                evaluation_time_ms, propagated_unc, current_session_id);

            if (failure.has_value()) {
                agent_failures.push_back(std::move(*failure));
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
        } else {
            snapshot.agent_states.erase(agent_id);
            for (auto& f : agent_failures) {
                all_agent_failures.push_back(std::move(f));
            }
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
        all_agent_failures.push_back({
            .reason = RejectionReason::kIncompleteAgentSet,
            .detail = "accepted " + std::to_string(agents_accepted) + "/" +
                      std::to_string(required.size()) + " required agents",
        });
        return StructuredRejection{
            .evaluation_time_ms = evaluation_time_ms,
            .contract_id = contract.contract_id,
            .failures = std::move(all_agent_failures),
        };
    }

    // Deterministic snapshot ID.
    if (config_.generate_snapshot_ids) {
        snapshot.snapshot_id = "snap-" + contract.contract_id + "-" +
                              std::to_string(static_cast<std::int64_t>(evaluation_time_ms));
    }

    // Production timestamp.
    snapshot.produced_at_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    return snapshot;
}

}  // namespace swarmkit::core
