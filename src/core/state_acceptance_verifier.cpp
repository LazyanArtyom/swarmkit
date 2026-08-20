// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/core/state_acceptance_verifier.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

namespace swarmkit::core {
namespace {

std::vector<std::string> Sorted(std::vector<std::string> values) {
    std::sort(values.begin(), values.end());
    return values;
}

bool HasDuplicates(const std::vector<std::string>& values) {
    const auto sorted = Sorted(values);
    return std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end();
}

struct ReconstructedSelection {
    EvidenceRecord record;
    GenerationTimeInterval interval;
    ClockEvaluationEvidence clock;
};

}  // namespace

VerificationResult StateAcceptanceVerifier::Verify(
    const StateAcceptanceCertificate& cert,
    const EvidenceStore& evidence,
    const StateQualityContract& contract,
    const SnapshotRequestContext& request_ctx,
    const std::unordered_map<std::string, ClockQualityState>& clock_states)
    const {

    std::vector<VerificationFailure> failures;
    const auto add_failure = [&](VerificationFailureReason reason, std::string detail) {
        failures.push_back({.reason = reason, .detail = std::move(detail)});
    };
    const auto nearly_equal = [](double lhs, double rhs) {
        return std::isfinite(lhs) && std::isfinite(rhs) &&
               std::abs(lhs - rhs) <= kUncertaintyTolerance;
    };

    // A valid outer hash is only the first gate. Semantic reconstruction below
    // is deliberately independent and still rejects hash-consistent mutations.
    if (!VerifyCertificateIntegrity(cert)) {
        add_failure(VerificationFailureReason::kCertificateHashMismatch,
                    "certificate consistency hash does not match canonical content");
        return VerificationRejection{
            .certificate_id = cert.certificate_id,
            .failures = std::move(failures),
        };
    }

    if (cert.certificate_schema_version != kCertificateSchemaVersion) {
        add_failure(VerificationFailureReason::kCertificateSchemaMismatch,
                    "unsupported certificate schema version '" +
                        cert.certificate_schema_version + "'");
    }
    if (cert.acceptance_semantics_version != kAcceptanceSemanticsVersion) {
        add_failure(VerificationFailureReason::kSemanticVersionMismatch,
                    "unsupported acceptance semantics version '" +
                        cert.acceptance_semantics_version + "'");
    }
    if (cert.certificate_id.empty() || cert.produced_at_ms <= 0) {
        add_failure(VerificationFailureReason::kCertificateSchemaMismatch,
                    "certificate identity or production timestamp is invalid");
    }

    const auto contract_validation = ValidateStateQualityContract(contract);
    if (!contract_validation.IsOk()) {
        add_failure(VerificationFailureReason::kPredicateMismatch,
                    "provided StateQualityContract is invalid");
    }

    const std::string expected_contract_hash = ComputeContractHash(contract);
    if (cert.contract_id != contract.contract_id) {
        add_failure(VerificationFailureReason::kContractVersionMismatch,
                    "certificate contract_id differs from the supplied contract");
    }
    if (cert.contract_hash != expected_contract_hash) {
        add_failure(VerificationFailureReason::kContractHashMismatch,
                    "certificate contract hash differs from the canonical contract hash");
    }
    if (cert.contract_content_version != contract.content_version ||
        cert.contract_schema_version != contract.schema_version) {
        add_failure(VerificationFailureReason::kContractVersionMismatch,
                    "certificate contract schema/content version differs from the supplied contract");
    }

    if (!nearly_equal(cert.evaluation_time_ms, request_ctx.evaluation_time_ms)) {
        add_failure(VerificationFailureReason::kEvaluationTimeMismatch,
                    "certificate t* differs from the persisted request context");
    }
    if (cert.evidence_freeze_ms != request_ctx.evidence_freeze_ms) {
        add_failure(VerificationFailureReason::kEvaluationTimeMismatch,
                    "certificate r* differs from the persisted request context");
    }

    if (HasDuplicates(cert.participants.agent_ids) ||
        Sorted(cert.participants.agent_ids) != Sorted(request_ctx.participants.agent_ids) ||
        cert.participants.membership_revision !=
            request_ctx.participants.membership_revision) {
        add_failure(VerificationFailureReason::kMembershipMismatch,
                    "certificate participant set or membership revision differs from the request");
    }
    for (const auto& required_agent : contract.required_agents) {
        if (std::find(request_ctx.participants.agent_ids.begin(),
                      request_ctx.participants.agent_ids.end(), required_agent) ==
            request_ctx.participants.agent_ids.end()) {
            add_failure(VerificationFailureReason::kMembershipMismatch,
                        "required agent '" + required_agent +
                            "' is absent from the resolved participant set");
        }
    }

    if (cert.propagation_model_id != contract.propagation_model_id ||
        cert.propagation_model_version != contract.propagation_model_version ||
        !nearly_equal(cert.max_horizontal_speed_mps,
                      contract.max_horizontal_speed_mps) ||
        !nearly_equal(cert.max_vertical_speed_mps,
                      contract.max_vertical_speed_mps)) {
        add_failure(VerificationFailureReason::kModelMismatch,
                    "certificate propagation model/version or speed assumptions differ from the contract");
    }

    AcceptedSnapshot reconstructed;
    reconstructed.snapshot_id = "reconstructed-" + cert.certificate_id;
    reconstructed.evaluation_time_ms = request_ctx.evaluation_time_ms;
    reconstructed.evidence_freeze_ms = request_ctx.evidence_freeze_ms;
    reconstructed.contract_id = contract.contract_id;
    reconstructed.contract_version = contract.content_version;
    reconstructed.contract_hash = expected_contract_hash;
    reconstructed.model_id = contract.propagation_model_id;
    reconstructed.model_version = contract.propagation_model_version;
    reconstructed.participants = request_ctx.participants;
    reconstructed.produced_at_ms = cert.produced_at_ms;

    auto required_fields = contract.required_fields;
    if (contract.min_gps_quality != GpsQuality::kUnknown &&
        std::find(required_fields.begin(), required_fields.end(),
                  EvidenceFieldId::kGpsQuality) == required_fields.end()) {
        required_fields.push_back(EvidenceFieldId::kGpsQuality);
    }

    std::size_t agents_accepted = 0;
    std::size_t reconstructed_entry_count = 0;
    double max_effective_rho = 0.0;
    double max_elapsed = 0.0;
    double max_position_uncertainty = 0.0;
    double max_velocity_uncertainty = 0.0;

    for (const auto& agent_id : contract.required_agents) {
        const auto current_session = evidence.CurrentSessionId(agent_id);
        const std::string current_session_id = current_session.value_or("");
        bool agent_ok = true;

        for (const auto field : required_fields) {
            const auto candidates = evidence.All(agent_id, field);
            std::optional<ReconstructedSelection> best;
            double best_g_minus = -std::numeric_limits<double>::infinity();
            std::uint64_t best_sequence = 0;
            std::string best_evidence_id;

            for (const auto& record : candidates) {
                if (record.receive_time_ms > request_ctx.evidence_freeze_ms) continue;
                if (contract.require_current_epoch &&
                    record.identity.agent_session_id != current_session_id) {
                    continue;
                }
                if (field == EvidenceFieldId::kPosition &&
                    contract.required_position_frame != CoordinateFrame::kUnknown &&
                    record.identity.coordinate_frame != contract.required_position_frame) {
                    continue;
                }
                if (field == EvidenceFieldId::kVelocity &&
                    contract.required_velocity_frame != CoordinateFrame::kUnknown &&
                    record.identity.coordinate_frame != contract.required_velocity_frame) {
                    continue;
                }
                if (contract.require_current_mission &&
                    (record.identity.mission_id != contract.required_mission_id ||
                     record.identity.mission_revision !=
                         contract.required_mission_revision)) {
                    continue;
                }
                if (!record.source_time.timestamp_ms.has_value()) continue;

                const double source_time =
                    static_cast<double>(*record.source_time.timestamp_ms);
                if (!std::isfinite(source_time)) continue;

                std::optional<GenerationTimeInterval> interval;
                ClockEvaluationEvidence clock;
                const auto clock_it = clock_states.find(agent_id);
                if (clock_it != clock_states.end() && clock_it->second.IsValid()) {
                    const auto& model = clock_it->second;
                    if (!model.agent_incarnation_id.empty() &&
                        !record.identity.agent_session_id.empty() &&
                        model.agent_incarnation_id !=
                            record.identity.agent_session_id) {
                        continue;
                    }
                    if (contract.require_deterministic_bounds &&
                        (!model.deterministic_bound ||
                         model.agent_incarnation_id.empty())) {
                        continue;
                    }
                    const double g_hat = source_time - model.offset_estimate_ms;
                    const double effective_rho =
                        model.ComputeEffectiveUncertainty(g_hat);
                    interval = GenerationTimeInterval{
                        .lower_ms = g_hat - effective_rho,
                        .upper_ms = g_hat + effective_rho,
                    };
                    clock = {
                        .theta_hat_ms = model.offset_estimate_ms,
                        .base_rho_ms = model.uncertainty_radius_ms,
                        .effective_rho_ms = effective_rho,
                        .max_drift_rate_ppm = model.max_drift_rate_ppm,
                        .model_last_update_reference_ms = model.last_update_ms,
                        .model_version = model.clock_model_version,
                        .agent_incarnation_id = model.agent_incarnation_id,
                        .source_domain = model.source_domain,
                        .synchronization = model.synchronization,
                        .deterministic_bound = model.deterministic_bound,
                    };
                } else if (!contract.require_deterministic_bounds &&
                           record.source_time.clock_uncertainty_ms.has_value()) {
                    const double rho =
                        *record.source_time.clock_uncertainty_ms;
                    if (!std::isfinite(rho) || rho < 0.0) continue;
                    interval = ComputeGenerationIntervalFromSample(record.source_time);
                    clock = {
                        .theta_hat_ms = 0.0,
                        .base_rho_ms = rho,
                        .effective_rho_ms = rho,
                        .max_drift_rate_ppm = 0.0,
                        .model_last_update_reference_ms = 0,
                        .model_version = "per-sample-clock-v1",
                        .agent_incarnation_id =
                            record.identity.agent_session_id,
                        .source_domain = record.source_time.clock_domain,
                        .synchronization = record.source_time.synchronization,
                        .deterministic_bound = false,
                    };
                } else {
                    continue;
                }

                if (!interval.has_value() ||
                    !interval->IsCausal(request_ctx.evaluation_time_ms)) {
                    continue;
                }

                const std::string evidence_id = ComputeCanonicalEvidenceId(record);
                const bool is_better =
                    !best.has_value() || interval->lower_ms > best_g_minus ||
                    (interval->lower_ms == best_g_minus &&
                     (record.identity.sequence > best_sequence ||
                      (record.identity.sequence == best_sequence &&
                       evidence_id > best_evidence_id)));
                if (is_better) {
                    best = ReconstructedSelection{
                        .record = record,
                        .interval = *interval,
                        .clock = clock,
                    };
                    best_g_minus = interval->lower_ms;
                    best_sequence = record.identity.sequence;
                    best_evidence_id = evidence_id;
                }
            }

            if (!best.has_value()) {
                add_failure(VerificationFailureReason::kMissingEvidence,
                            "no eligible evidence for agent '" + agent_id +
                                "' field " +
                                std::to_string(static_cast<int>(field)));
                agent_ok = false;
                continue;
            }

            const auto matches_entry = [&](const CertificateEvidenceEntry& entry) {
                return entry.agent_id == agent_id && entry.field == field;
            };
            const auto first_entry = std::find_if(
                cert.evidence_entries.begin(), cert.evidence_entries.end(),
                matches_entry);
            if (first_entry == cert.evidence_entries.end() ||
                std::count_if(cert.evidence_entries.begin(),
                              cert.evidence_entries.end(), matches_entry) != 1) {
                add_failure(VerificationFailureReason::kMissingEvidence,
                            "certificate must contain exactly one entry for agent '" +
                                agent_id + "' field " +
                                std::to_string(static_cast<int>(field)));
                agent_ok = false;
                continue;
            }

            const auto& entry = *first_entry;
            const auto& record = best->record;
            const auto& clock = best->clock;
            bool field_ok = true;
            const auto mismatch = [&](VerificationFailureReason reason,
                                      std::string detail) {
                add_failure(reason, std::move(detail));
                field_ok = false;
            };

            if (entry.sequence != record.identity.sequence ||
                entry.agent_session_id != record.identity.agent_session_id ||
                entry.source_component != record.identity.source_component ||
                entry.evidence_id != ComputeCanonicalEvidenceId(record) ||
                entry.source_time_ms != record.source_time.timestamp_ms ||
                entry.receive_time_ms != record.receive_time_ms) {
                mismatch(VerificationFailureReason::kDecisionMismatch,
                         "certificate evidence identity/timestamps differ for agent '" +
                             agent_id + "'");
            }
            if (entry.evidence_hash != ComputeEvidenceHash(record)) {
                mismatch(VerificationFailureReason::kEvidenceHashMismatch,
                         "certificate evidence hash differs for agent '" + agent_id + "'");
            }
            if (entry.agent_incarnation_id != clock.agent_incarnation_id) {
                mismatch(VerificationFailureReason::kSessionMismatch,
                         "certificate clock incarnation differs for agent '" + agent_id + "'");
            }
            if (entry.coordinate_frame != record.identity.coordinate_frame) {
                mismatch(VerificationFailureReason::kFrameMismatch,
                         "certificate coordinate frame differs for agent '" + agent_id + "'");
            }
            if (entry.estimator_healthy != record.quality.estimator_healthy ||
                entry.estimator_position_ok !=
                    record.quality.estimator_position_ok ||
                entry.estimator_velocity_ok !=
                    record.quality.estimator_velocity_ok) {
                mismatch(VerificationFailureReason::kPredicateMismatch,
                         "certificate health operands differ for agent '" + agent_id + "'");
            }
            if (entry.mission_id != record.identity.mission_id ||
                entry.mission_revision != record.identity.mission_revision) {
                mismatch(VerificationFailureReason::kProvenanceMismatch,
                         "certificate mission operands differ for agent '" + agent_id + "'");
            }

            const UncertaintySemantics expected_semantics =
                record.quality.uncertainty.has_value()
                    ? record.quality.uncertainty->descriptor.semantics
                    : UncertaintySemantics::kUnknown;
            const double observation_uncertainty =
                record.quality.uncertainty.has_value()
                    ? static_cast<double>(record.quality.uncertainty->value)
                    : 0.0;
            if (entry.uncertainty_semantics != expected_semantics ||
                !nearly_equal(entry.observation_uncertainty,
                              observation_uncertainty)) {
                mismatch(VerificationFailureReason::kPropagationMismatch,
                         "certificate uncertainty operands differ for agent '" + agent_id + "'");
            }

            std::optional<GpsQuality> expected_gps;
            if (field == EvidenceFieldId::kGpsQuality &&
                std::holds_alternative<GpsQuality>(record.value)) {
                expected_gps = std::get<GpsQuality>(record.value);
            }
            if (entry.gps_quality != expected_gps) {
                mismatch(VerificationFailureReason::kPredicateMismatch,
                         "certificate GPS-quality operand differs for agent '" + agent_id + "'");
            }

            if (!nearly_equal(entry.theta_hat_ms, clock.theta_hat_ms) ||
                !nearly_equal(entry.base_rho_ms, clock.base_rho_ms) ||
                !nearly_equal(entry.effective_rho_ms,
                              clock.effective_rho_ms) ||
                !nearly_equal(entry.max_drift_rate_ppm,
                              clock.max_drift_rate_ppm) ||
                entry.clock_model_last_update_reference_ms !=
                    clock.model_last_update_reference_ms ||
                entry.clock_model_version != clock.model_version ||
                entry.clock_domain != clock.source_domain ||
                entry.clock_synchronization != clock.synchronization ||
                entry.clock_deterministic_bound != clock.deterministic_bound) {
                mismatch(VerificationFailureReason::kClockMismatch,
                         "certificate primitive clock-model operands differ for agent '" +
                             agent_id + "'");
            }

            const double conservative_elapsed =
                best->interval.ConservativeElapsed(request_ctx.evaluation_time_ms);
            const double max_speed =
                field == EvidenceFieldId::kPosition
                    ? Compute3DSpeedBound(contract.max_horizontal_speed_mps,
                                          contract.max_vertical_speed_mps)
                    : 0.0;
            const double propagated_uncertainty = ComputePropagatedUncertainty(
                observation_uncertainty, max_speed, conservative_elapsed);

            if (!nearly_equal(entry.generation_interval.lower_ms,
                              best->interval.lower_ms) ||
                !nearly_equal(entry.generation_interval.upper_ms,
                              best->interval.upper_ms) ||
                !nearly_equal(entry.conservative_elapsed_ms,
                              conservative_elapsed)) {
                mismatch(VerificationFailureReason::kClockMismatch,
                         "certificate g-/g+/Delta+ claims differ for agent '" + agent_id + "'");
            }
            if (!nearly_equal(entry.propagated_uncertainty,
                              propagated_uncertainty)) {
                mismatch(VerificationFailureReason::kPropagationMismatch,
                         "certificate propagated uncertainty differs for agent '" + agent_id + "'");
            }

            if (contract.max_evidence_age_ms.has_value() &&
                conservative_elapsed > *contract.max_evidence_age_ms) {
                mismatch(VerificationFailureReason::kPredicateMismatch,
                         "conservative evidence age exceeds the contract for agent '" + agent_id + "'");
            }
            if (contract.max_clock_uncertainty_ms.has_value() &&
                clock.effective_rho_ms > *contract.max_clock_uncertainty_ms) {
                mismatch(VerificationFailureReason::kClockMismatch,
                         "effective clock uncertainty exceeds the contract for agent '" + agent_id + "'");
            }
            if (contract.require_deterministic_bounds &&
                (field == EvidenceFieldId::kPosition ||
                 field == EvidenceFieldId::kVelocity) &&
                (!record.quality.uncertainty.has_value() ||
                 expected_semantics !=
                     UncertaintySemantics::kDeterministicHardBound ||
                 !std::isfinite(observation_uncertainty) ||
                 observation_uncertainty < 0.0 ||
                 !clock.deterministic_bound)) {
                mismatch(VerificationFailureReason::kPredicateMismatch,
                         "deterministic bound semantics are not established for agent '" + agent_id + "'");
            }
            if (field == EvidenceFieldId::kPosition &&
                contract.max_position_uncertainty_m.has_value() &&
                propagated_uncertainty >
                    *contract.max_position_uncertainty_m) {
                mismatch(VerificationFailureReason::kPropagationMismatch,
                         "position uncertainty exceeds the contract for agent '" + agent_id + "'");
            }
            if (field == EvidenceFieldId::kVelocity &&
                contract.max_velocity_uncertainty_mps.has_value() &&
                propagated_uncertainty >
                    *contract.max_velocity_uncertainty_mps) {
                mismatch(VerificationFailureReason::kPropagationMismatch,
                         "velocity uncertainty exceeds the contract for agent '" + agent_id + "'");
            }
            if (contract.require_estimator_healthy &&
                (field == EvidenceFieldId::kPosition ||
                 field == EvidenceFieldId::kVelocity) &&
                !record.quality.estimator_healthy) {
                mismatch(VerificationFailureReason::kPredicateMismatch,
                         "selected evidence is estimator-unhealthy for agent '" + agent_id + "'");
            }
            if (contract.require_estimator_position_ok &&
                field == EvidenceFieldId::kPosition &&
                !record.quality.estimator_position_ok) {
                mismatch(VerificationFailureReason::kPredicateMismatch,
                         "selected position evidence is not estimator-position-ok for agent '" + agent_id + "'");
            }
            if (contract.require_estimator_velocity_ok &&
                field == EvidenceFieldId::kVelocity &&
                !record.quality.estimator_velocity_ok) {
                mismatch(VerificationFailureReason::kPredicateMismatch,
                         "selected velocity evidence is not estimator-velocity-ok for agent '" + agent_id + "'");
            }
            if (contract.min_gps_quality != GpsQuality::kUnknown &&
                field == EvidenceFieldId::kGpsQuality &&
                (!expected_gps.has_value() ||
                 static_cast<std::uint8_t>(*expected_gps) <
                     static_cast<std::uint8_t>(contract.min_gps_quality))) {
                mismatch(VerificationFailureReason::kPredicateMismatch,
                         "GPS-quality predicate is not established for agent '" + agent_id + "'");
            }
            if (contract.require_current_epoch &&
                record.identity.agent_session_id != current_session_id) {
                mismatch(VerificationFailureReason::kSessionMismatch,
                         "selected evidence is not from the authoritative session for agent '" + agent_id + "'");
            }
            if (contract.require_current_mission &&
                (record.identity.mission_id != contract.required_mission_id ||
                 record.identity.mission_revision !=
                     contract.required_mission_revision)) {
                mismatch(VerificationFailureReason::kProvenanceMismatch,
                         "selected evidence mission differs from the contract for agent '" + agent_id + "'");
            }

            if (!field_ok) {
                agent_ok = false;
                continue;
            }

            reconstructed.agent_states[agent_id]
                                      [static_cast<std::uint8_t>(field)] = {
                .evidence = record,
                .generation_interval = best->interval,
                .conservative_elapsed_ms = conservative_elapsed,
                .observation_uncertainty = observation_uncertainty,
                .propagated_uncertainty = propagated_uncertainty,
                .clock_evaluation = clock,
            };
            ++reconstructed_entry_count;
            max_effective_rho =
                std::max(max_effective_rho, clock.effective_rho_ms);
            max_elapsed = std::max(max_elapsed, conservative_elapsed);
            if (field == EvidenceFieldId::kPosition) {
                max_position_uncertainty = std::max(
                    max_position_uncertainty, propagated_uncertainty);
            } else if (field == EvidenceFieldId::kVelocity) {
                max_velocity_uncertainty = std::max(
                    max_velocity_uncertainty, propagated_uncertainty);
            }
        }

        if (agent_ok) {
            reconstructed.accepted_agents.push_back(agent_id);
            ++agents_accepted;
        } else {
            reconstructed_entry_count -= reconstructed.agent_states[agent_id].size();
            reconstructed.agent_states.erase(agent_id);
        }
    }

    bool completeness_ok = false;
    switch (contract.completeness) {
        case CompletenessRule::kAllRequired:
            completeness_ok = agents_accepted == contract.required_agents.size();
            break;
        case CompletenessRule::kMinimumCount:
            completeness_ok = agents_accepted >= contract.min_required_agents;
            break;
    }
    if (!completeness_ok) {
        add_failure(VerificationFailureReason::kCompletenessMismatch,
                    "independently reconstructed decision fails the completeness rule");
    }

    if (HasDuplicates(cert.accepted_agents) ||
        Sorted(cert.accepted_agents) != Sorted(reconstructed.accepted_agents)) {
        add_failure(VerificationFailureReason::kDecisionMismatch,
                    "certificate accepted-agent set differs from the reconstructed decision");
    }
    if (cert.evidence_entries.size() != reconstructed_entry_count) {
        add_failure(VerificationFailureReason::kDecisionMismatch,
                    "certificate contains a missing or extra evidence entry");
    }

    if (!nearly_equal(cert.max_clock_uncertainty_ms, max_effective_rho) ||
        !nearly_equal(cert.max_conservative_elapsed_ms, max_elapsed) ||
        !nearly_equal(cert.max_propagated_position_uncertainty_m,
                      max_position_uncertainty) ||
        !nearly_equal(cert.max_propagated_velocity_uncertainty_mps,
                      max_velocity_uncertainty)) {
        add_failure(VerificationFailureReason::kPropagationMismatch,
                    "certificate timing/uncertainty summary maxima differ from reconstruction");
    }

    if (!failures.empty()) {
        return VerificationRejection{
            .certificate_id = cert.certificate_id,
            .failures = std::move(failures),
        };
    }

    return VerifiedAcceptance{
        .certificate_id = cert.certificate_id,
        .reconstructed_snapshot = std::move(reconstructed),
    };
}

}  // namespace swarmkit::core
