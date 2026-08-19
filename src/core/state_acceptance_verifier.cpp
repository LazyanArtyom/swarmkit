// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/core/state_acceptance_verifier.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>

namespace swarmkit::core {

VerificationResult StateAcceptanceVerifier::Verify(
    const StateAcceptanceCertificate& cert,
    const EvidenceStore& evidence,
    const StateQualityContract& contract,
    const SnapshotRequestContext& request_ctx,
    const std::unordered_map<std::string, ClockQualityState>& clock_states)
    const {

    std::vector<VerificationFailure> failures;

    const double evaluation_time_ms = request_ctx.evaluation_time_ms;
    const std::int64_t evidence_freeze_ms = request_ctx.evidence_freeze_ms;

    // Step 1: Certificate integrity check (h_K binding).
    if (!VerifyCertificateIntegrity(cert)) {
        failures.push_back({
            .reason = VerificationFailureReason::kCertificateHashMismatch,
            .detail = "certificate hash h_K does not match recomputed SHA-256 digest",
        });
        return VerificationRejection{
            .certificate_id = cert.certificate_id,
            .failures = std::move(failures),
        };
    }

    // Step 2: Contract binding check (h_C binding).
    const std::string expected_contract_hash = ComputeContractHash(contract);
    if (cert.contract_hash != expected_contract_hash) {
        failures.push_back({
            .reason = VerificationFailureReason::kContractHashMismatch,
            .detail = "certificate contract hash '" + cert.contract_hash +
                      "' != computed '" + expected_contract_hash + "'",
        });
        return VerificationRejection{
            .certificate_id = cert.certificate_id,
            .failures = std::move(failures),
        };
    }

    if (cert.contract_content_version != contract.content_version ||
        cert.contract_schema_version != contract.schema_version) {
        failures.push_back({
            .reason = VerificationFailureReason::kContractVersionMismatch,
            .detail = "contract version mismatch: cert=(" +
                      std::to_string(cert.contract_schema_version) + "." +
                      std::to_string(cert.contract_content_version) + ") != contract=(" +
                      std::to_string(contract.schema_version) + "." +
                      std::to_string(contract.content_version) + ")",
        });
    }

    if (cert.acceptance_semantics_version != "2.0") {
        failures.push_back({
            .reason = VerificationFailureReason::kSemanticVersionMismatch,
            .detail = "unsupported acceptance semantics version: " +
                      cert.acceptance_semantics_version,
        });
    }

    // Verify r* binding.
    if (cert.evidence_freeze_ms != evidence_freeze_ms) {
        failures.push_back({
            .reason = VerificationFailureReason::kEvaluationTimeMismatch,
            .detail = "evidence_freeze_ms mismatch: cert=" +
                      std::to_string(cert.evidence_freeze_ms) + " ctx=" +
                      std::to_string(evidence_freeze_ms),
        });
    }

    // Verify t* binding.
    if (std::abs(cert.evaluation_time_ms - evaluation_time_ms) > kUncertaintyTolerance) {
        failures.push_back({
            .reason = VerificationFailureReason::kEvaluationTimeMismatch,
            .detail = "evaluation_time_ms mismatch",
        });
    }

    // Step 3: Independent Decision & Evidence Reconstruction
    // The verifier independently reconstructs every predicate
    // without instantiating or invoking StateAcceptanceEngine.

    AcceptedSnapshot reconstructed;
    reconstructed.snapshot_id = "reconstructed-" + cert.certificate_id;
    reconstructed.evaluation_time_ms = cert.evaluation_time_ms;
    reconstructed.evidence_freeze_ms = cert.evidence_freeze_ms;
    reconstructed.contract_id = contract.contract_id;
    reconstructed.contract_version = contract.content_version;
    reconstructed.contract_hash = expected_contract_hash;
    reconstructed.model_id = contract.propagation_model_id;
    reconstructed.model_version = contract.propagation_model_version;
    reconstructed.participants = request_ctx.participants;

    std::size_t agents_accepted = 0;

    for (const auto& agent_id : contract.required_agents) {
        const auto cur_sess = evidence.CurrentSessionId(agent_id);
        const std::string current_session_id = cur_sess.value_or("");

        bool agent_ok = true;

        for (const auto field : contract.required_fields) {
            auto candidates = evidence.All(agent_id, field);
            if (candidates.empty()) {
                failures.push_back({
                    .reason = VerificationFailureReason::kMissingEvidence,
                    .detail = "no evidence stored for agent " + agent_id + " field " +
                              std::to_string(static_cast<int>(field)),
                });
                agent_ok = false;
                break;
            }

            // Independent evidence selection with r* filter,
            // context-invalid pre-filtering, and deterministic tie-breaking (max g⁻).
            std::optional<EvidenceRecord> best_rec;
            std::optional<GenerationTimeInterval> best_interval;
            double best_clock_unc = 0.0;
            double best_g_lower = -std::numeric_limits<double>::infinity();
            std::uint64_t best_sequence = 0;
            std::string best_evidence_id;

            for (const auto& rec : candidates) {
                // P0.1: Evidence-freeze cutoff r*.
                if (rec.receive_time_ms > evidence_freeze_ms) {
                    continue;
                }

                // P0.6: Context-invalid filtering BEFORE ranking.
                if (contract.require_current_epoch) {
                    if (rec.identity.agent_session_id != current_session_id) {
                        continue;
                    }
                }

                // Frame eligibility before ranking.
                if (field == EvidenceFieldId::kPosition &&
                    contract.required_position_frame != CoordinateFrame::kUnknown &&
                    rec.identity.coordinate_frame != contract.required_position_frame) {
                    continue;
                }
                if (field == EvidenceFieldId::kVelocity &&
                    contract.required_velocity_frame != CoordinateFrame::kUnknown &&
                    rec.identity.coordinate_frame != contract.required_velocity_frame) {
                    continue;
                }

                // Mission eligibility before ranking.
                if (contract.require_current_mission) {
                    if (rec.identity.mission_id != contract.required_mission_id ||
                        rec.identity.mission_revision != contract.required_mission_revision) {
                        continue;
                    }
                }

                if (!rec.source_time.timestamp_ms.has_value()) continue;
                const double s = static_cast<double>(*rec.source_time.timestamp_ms);
                if (!std::isfinite(s)) continue;

                std::optional<GenerationTimeInterval> interval;
                double clock_unc = 0.0;

                auto clock_it = clock_states.find(agent_id);
                if (clock_it != clock_states.end() && clock_it->second.IsValid()) {
                    const auto& clk = clock_it->second;

                    // Clock incarnation check.
                    if (!clk.agent_incarnation_id.empty() &&
                        !rec.identity.agent_session_id.empty() &&
                        clk.agent_incarnation_id != rec.identity.agent_session_id) {
                        continue;
                    }

                    if (contract.require_deterministic_bounds && !clk.deterministic_bound) {
                        continue;
                    }

                    // Reference domain estimated generation time g_hat = s - theta_hat
                    const double g_hat = s - clk.offset_estimate_ms;
                    clock_unc = clk.ComputeEffectiveUncertainty(g_hat);
                    interval = GenerationTimeInterval{
                        .lower_ms = g_hat - clock_unc,
                        .upper_ms = g_hat + clock_unc,
                    };
                } else if (!contract.require_deterministic_bounds && rec.source_time.clock_uncertainty_ms.has_value()) {
                    // Non-deterministic legacy mode only
                    const double rho = *rec.source_time.clock_uncertainty_ms;
                    if (!std::isfinite(rho) || rho < 0.0) continue;
                    interval = ComputeGenerationIntervalFromSample(rec.source_time);
                    clock_unc = rho;
                } else {
                    continue;
                }

                if (!interval.has_value()) continue;
                if (!interval->IsCausal(evaluation_time_ms)) continue;

                // Deterministic tie-breaking: max(g⁻), then sequence, then EvidenceId.
                const double g_lower = interval->lower_ms;
                const std::uint64_t seq = rec.identity.sequence;
                const std::string eid =
                    rec.identity.agent_id + ":" +
                    std::to_string(static_cast<int>(rec.identity.field_id)) + ":" +
                    rec.identity.agent_session_id + ":" +
                    std::to_string(rec.identity.sequence);

                bool is_better = false;
                if (!best_rec.has_value()) {
                    is_better = true;
                } else if (g_lower > best_g_lower) {
                    is_better = true;
                } else if (g_lower == best_g_lower) {
                    if (seq > best_sequence) {
                        is_better = true;
                    } else if (seq == best_sequence && eid > best_evidence_id) {
                        is_better = true;
                    }
                }

                if (is_better) {
                    best_g_lower = g_lower;
                    best_sequence = seq;
                    best_evidence_id = eid;
                    best_rec = rec;
                    best_interval = *interval;
                    best_clock_unc = clock_unc;
                }
            }

            if (!best_rec.has_value()) {
                failures.push_back({
                    .reason = VerificationFailureReason::kMissingEvidence,
                    .detail = "could not find causally valid evidence for agent " +
                              agent_id + " at t*=" + std::to_string(evaluation_time_ms),
                });
                agent_ok = false;
                break;
            }

            // Find matching certified evidence entry in the certificate.
            const auto cert_entry_it = std::find_if(
                cert.evidence_entries.begin(), cert.evidence_entries.end(),
                [&](const CertificateEvidenceEntry& e) {
                    return e.agent_id == agent_id && e.field == field;
                });

            if (cert_entry_it == cert.evidence_entries.end()) {
                failures.push_back({
                    .reason = VerificationFailureReason::kMissingEvidence,
                    .detail = "certified entry missing from certificate for agent " +
                              agent_id + " field " + std::to_string(static_cast<int>(field)),
                });
                agent_ok = false;
                break;
            }

            const auto& cert_entry = *cert_entry_it;

            // Verify evidence identity: sequence, session, provenance, content hash (P1.1).
            if (cert_entry.sequence != best_rec->identity.sequence) {
                failures.push_back({
                    .reason = VerificationFailureReason::kDecisionMismatch,
                    .detail = "evidence sequence mismatch for agent " + agent_id +
                              ": cert=" + std::to_string(cert_entry.sequence) +
                              " rec=" + std::to_string(best_rec->identity.sequence),
                });
                agent_ok = false;
                break;
            }

            if (cert_entry.agent_session_id != best_rec->identity.agent_session_id) {
                failures.push_back({
                    .reason = VerificationFailureReason::kSessionMismatch,
                    .detail = "evidence session mismatch for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            if (cert_entry.source_component != best_rec->identity.source_component) {
                failures.push_back({
                    .reason = VerificationFailureReason::kProvenanceMismatch,
                    .detail = "source component mismatch for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            if (cert_entry.source_time_ms != best_rec->source_time.timestamp_ms) {
                failures.push_back({
                    .reason = VerificationFailureReason::kClockMismatch,
                    .detail = "source timestamp mismatch for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            if (cert_entry.coordinate_frame != best_rec->identity.coordinate_frame) {
                failures.push_back({
                    .reason = VerificationFailureReason::kFrameMismatch,
                    .detail = "coordinate frame mismatch for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            const std::string computed_evidence_hash = ComputeEvidenceHash(*best_rec);
            if (cert_entry.evidence_hash != computed_evidence_hash) {
                failures.push_back({
                    .reason = VerificationFailureReason::kEvidenceHashMismatch,
                    .detail = "evidence hash mismatch for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            // Independent predicate evaluation — ALL contract predicates.

            // Session/epoch check (P0.1).
            if (contract.require_current_epoch &&
                best_rec->identity.agent_session_id != current_session_id) {
                failures.push_back({
                    .reason = VerificationFailureReason::kSessionMismatch,
                    .detail = "authoritative session mismatch for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            // Recompute derived values (g⁻, g⁺, Δ⁺, ε) — verifier MUST NOT trust
            // certificate values for these.
            const double conservative_elapsed =
                best_interval->ConservativeElapsed(evaluation_time_ms);

            // Age predicate (A): Δ⁺ ≤ max_evidence_age.
            if (contract.max_evidence_age_ms.has_value() &&
                conservative_elapsed > *contract.max_evidence_age_ms) {
                failures.push_back({
                    .reason = VerificationFailureReason::kPredicateMismatch,
                    .detail = "age exceeded for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            // Clock uncertainty predicate (R): ρ ≤ max_clock_uncertainty.
            if (contract.max_clock_uncertainty_ms.has_value() &&
                best_clock_unc > *contract.max_clock_uncertainty_ms) {
                failures.push_back({
                    .reason = VerificationFailureReason::kClockMismatch,
                    .detail = "clock uncertainty exceeded for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            // Deterministic bounds check (P0.2).
            double obs_unc = 0.0;
            if (best_rec->quality.uncertainty.has_value()) {
                obs_unc = static_cast<double>(best_rec->quality.uncertainty->value);
            }

            if (contract.require_deterministic_bounds) {
                if (!best_rec->quality.uncertainty.has_value() ||
                    best_rec->quality.uncertainty->descriptor.semantics !=
                        UncertaintySemantics::kDeterministicHardBound) {
                    failures.push_back({
                        .reason = VerificationFailureReason::kPredicateMismatch,
                        .detail = "deterministic uncertainty semantics missing for agent " + agent_id,
                    });
                    agent_ok = false;
                    break;
                }

                if (!std::isfinite(best_rec->quality.uncertainty->value) ||
                    best_rec->quality.uncertainty->value < 0.0f) {
                    failures.push_back({
                        .reason = VerificationFailureReason::kPredicateMismatch,
                        .detail = "invalid uncertainty bound for agent " + agent_id,
                    });
                    agent_ok = false;
                    break;
                }
            }

            // Propagated uncertainty: ε(t*) = e_p + V_3D_max · Δ⁺.
            double max_speed = 0.0;
            if (field == EvidenceFieldId::kPosition) {
                max_speed = Compute3DSpeedBound(contract.max_horizontal_speed_mps, contract.max_vertical_speed_mps);
            }

            const double propagated_unc = ComputePropagatedUncertainty(
                obs_unc, max_speed, conservative_elapsed);

            // Position uncertainty predicate (U).
            if (field == EvidenceFieldId::kPosition &&
                contract.max_position_uncertainty_m.has_value() &&
                propagated_unc > *contract.max_position_uncertainty_m) {
                failures.push_back({
                    .reason = VerificationFailureReason::kPropagationMismatch,
                    .detail = "propagated position uncertainty exceeded for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            // Velocity uncertainty predicate (U).
            if (field == EvidenceFieldId::kVelocity &&
                contract.max_velocity_uncertainty_mps.has_value() &&
                propagated_unc > *contract.max_velocity_uncertainty_mps) {
                failures.push_back({
                    .reason = VerificationFailureReason::kPropagationMismatch,
                    .detail = "propagated velocity uncertainty exceeded for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            // Estimator health predicates (H).
            if (contract.require_estimator_healthy &&
                (field == EvidenceFieldId::kPosition || field == EvidenceFieldId::kVelocity)) {
                if (!best_rec->quality.estimator_healthy) {
                    failures.push_back({
                        .reason = VerificationFailureReason::kPredicateMismatch,
                        .detail = "estimator unhealthy for agent " + agent_id,
                    });
                    agent_ok = false;
                    break;
                }
            }

            // Estimator position ok (H).
            if (contract.require_estimator_position_ok && field == EvidenceFieldId::kPosition) {
                if (!best_rec->quality.estimator_position_ok) {
                    failures.push_back({
                        .reason = VerificationFailureReason::kPredicateMismatch,
                        .detail = "estimator position not ok for agent " + agent_id,
                    });
                    agent_ok = false;
                    break;
                }
            }

            // Estimator velocity ok (H).
            if (contract.require_estimator_velocity_ok && field == EvidenceFieldId::kVelocity) {
                if (!best_rec->quality.estimator_velocity_ok) {
                    failures.push_back({
                        .reason = VerificationFailureReason::kPredicateMismatch,
                        .detail = "estimator velocity not ok for agent " + agent_id,
                    });
                    agent_ok = false;
                    break;
                }
            }

            // Frame predicate (P) — position.
            if (field == EvidenceFieldId::kPosition &&
                contract.required_position_frame != CoordinateFrame::kUnknown &&
                best_rec->identity.coordinate_frame != contract.required_position_frame) {
                failures.push_back({
                    .reason = VerificationFailureReason::kFrameMismatch,
                    .detail = "position frame mismatch for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            // Frame predicate (P) — velocity.
            if (field == EvidenceFieldId::kVelocity &&
                contract.required_velocity_frame != CoordinateFrame::kUnknown &&
                best_rec->identity.coordinate_frame != contract.required_velocity_frame) {
                failures.push_back({
                    .reason = VerificationFailureReason::kFrameMismatch,
                    .detail = "velocity frame mismatch for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            // Mission predicate (M).
            if (contract.require_current_mission) {
                if (best_rec->identity.mission_id != contract.required_mission_id ||
                    best_rec->identity.mission_revision != contract.required_mission_revision) {
                    failures.push_back({
                        .reason = VerificationFailureReason::kProvenanceMismatch,
                        .detail = "mission mismatch for agent " + agent_id,
                    });
                    agent_ok = false;
                    break;
                }
            }

            // Cross-check recomputed values against certificate entries (tolerance).
            if (std::abs(cert_entry.clock_uncertainty_ms - best_clock_unc) > kUncertaintyTolerance) {
                failures.push_back({
                    .reason = VerificationFailureReason::kClockMismatch,
                    .detail = "clock uncertainty mismatch for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            if (std::abs(cert_entry.observation_uncertainty - obs_unc) > kUncertaintyTolerance) {
                failures.push_back({
                    .reason = VerificationFailureReason::kPropagationMismatch,
                    .detail = "observation uncertainty mismatch for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            if (std::abs(cert_entry.generation_interval.lower_ms - best_interval->lower_ms) > kUncertaintyTolerance ||
                std::abs(cert_entry.generation_interval.upper_ms - best_interval->upper_ms) > kUncertaintyTolerance) {
                failures.push_back({
                    .reason = VerificationFailureReason::kClockMismatch,
                    .detail = "generation interval mismatch for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            if (std::abs(cert_entry.propagated_uncertainty - propagated_unc) > kUncertaintyTolerance) {
                failures.push_back({
                    .reason = VerificationFailureReason::kPropagationMismatch,
                    .detail = "propagated uncertainty calculation mismatch for agent " + agent_id,
                });
                agent_ok = false;
                break;
            }

            reconstructed.agent_states[agent_id][static_cast<std::uint8_t>(field)] = {
                .evidence = *best_rec,
                .generation_interval = *best_interval,
                .conservative_elapsed_ms = conservative_elapsed,
                .observation_uncertainty = obs_unc,
                .propagated_uncertainty = propagated_unc,
                .clock_uncertainty_ms = best_clock_unc,
            };
        }

        if (agent_ok) {
            reconstructed.accepted_agents.push_back(agent_id);
            ++agents_accepted;
        } else {
            reconstructed.agent_states.erase(agent_id);
        }
    }

    // Step 4: Completeness Rule Verification.
    bool completeness_ok = false;
    switch (contract.completeness) {
        case CompletenessRule::kAllRequired:
            completeness_ok = (agents_accepted == contract.required_agents.size());
            break;
        case CompletenessRule::kMinimumCount:
            completeness_ok = (agents_accepted >= contract.min_required_agents);
            break;
    }

    if (!completeness_ok) {
        failures.push_back({
            .reason = VerificationFailureReason::kCompletenessMismatch,
            .detail = "reconstructed decision failed completeness rule: accepted " +
                      std::to_string(agents_accepted) + "/" +
                      std::to_string(contract.required_agents.size()),
        });
    }

    // Step 5: Verify accepted agent set matches certificate.
    auto cert_agents = cert.accepted_agents;
    auto recon_agents = reconstructed.accepted_agents;
    std::sort(cert_agents.begin(), cert_agents.end());
    std::sort(recon_agents.begin(), recon_agents.end());

    if (cert_agents != recon_agents) {
        failures.push_back({
            .reason = VerificationFailureReason::kDecisionMismatch,
            .detail = "accepted agent set differs from certificate",
        });
    }

    if (!failures.empty()) {
        return VerificationRejection{
            .certificate_id = cert.certificate_id,
            .failures = std::move(failures),
        };
    }

    return VerifiedAcceptance{
        .certificate_id = cert.certificate_id,
        .reconstructed_snapshot = reconstructed,
    };
}

}  // namespace swarmkit::core
