// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/core/state_acceptance_verifier.h"

#include <algorithm>
#include <cmath>

namespace swarmkit::core {

VerificationResult StateAcceptanceVerifier::Verify(
    const StateAcceptanceCertificate& cert,
    const EvidenceStore& evidence,
    const StateQualityContract& contract,
    const std::unordered_map<std::string, ClockQualityState>& clock_states)
    const {

    std::vector<VerificationFailure> failures;

    // Step 1: Certificate integrity check (h_K binding).
    if (!VerifyCertificateIntegrity(cert)) {
        failures.push_back({
            .reason = VerificationFailureReason::kCertificateHashMismatch,
            .detail = "certificate hash h_K does not match recomputed hash",
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

    if (cert.contract_content_version != contract.content_version) {
        failures.push_back({
            .reason = VerificationFailureReason::kContractVersionMismatch,
            .detail = "certificate contract version " +
                      std::to_string(cert.contract_content_version) +
                      " != provided " + std::to_string(contract.content_version),
        });
    }

    // Step 3: Reconstruct the acceptance decision independently.
    // Use a fresh engine instance — verifier must not share state.
    StateAcceptanceEngine reconstructor(AcceptanceEngineConfig{
        .generate_snapshot_ids = false,
    });

    AcceptanceResult result = reconstructor.RequestSnapshot(
        contract, cert.evaluation_time_ms, evidence, clock_states);

    // Step 4: Check that reconstruction succeeded.
    if (std::holds_alternative<StructuredRejection>(result)) {
        const auto& rejection = std::get<StructuredRejection>(result);
        std::string detail = "reconstructed decision: REJECTED (";
        for (const auto& f : rejection.failures) {
            detail += f.detail + "; ";
        }
        detail += ")";
        failures.push_back({
            .reason = VerificationFailureReason::kDecisionReconstructionFailed,
            .detail = std::move(detail),
        });
        return VerificationRejection{
            .certificate_id = cert.certificate_id,
            .failures = std::move(failures),
        };
    }

    const auto& reconstructed = std::get<AcceptedSnapshot>(result);

    // Step 5: Verify evidence entries match.
    for (const auto& cert_entry : cert.evidence_entries) {
        const auto agent_it =
            reconstructed.agent_states.find(cert_entry.agent_id);
        if (agent_it == reconstructed.agent_states.end()) {
            failures.push_back({
                .reason = VerificationFailureReason::kMissingCertifiedEvidence,
                .detail = "agent '" + cert_entry.agent_id +
                          "' not in reconstructed snapshot",
            });
            continue;
        }

        const auto field_key = static_cast<std::uint8_t>(cert_entry.field);
        const auto field_it = agent_it->second.find(field_key);
        if (field_it == agent_it->second.end()) {
            failures.push_back({
                .reason = VerificationFailureReason::kMissingCertifiedEvidence,
                .detail = "field " + std::to_string(field_key) +
                          " for agent '" + cert_entry.agent_id +
                          "' not in reconstructed snapshot",
            });
            continue;
        }

        const auto& recon_state = field_it->second;

        // Sequence must match — same evidence was selected.
        if (cert_entry.sequence != recon_state.evidence.identity.sequence) {
            failures.push_back({
                .reason = VerificationFailureReason::kEvidenceSequenceMismatch,
                .detail = "agent '" + cert_entry.agent_id + "' field " +
                          std::to_string(field_key) + ": cert seq=" +
                          std::to_string(cert_entry.sequence) + " != recon seq=" +
                          std::to_string(recon_state.evidence.identity.sequence),
            });
        }

        // Generation intervals must match within tolerance.
        if (std::abs(cert_entry.generation_interval.lower_ms -
                     recon_state.generation_interval.lower_ms) > kUncertaintyTolerance ||
            std::abs(cert_entry.generation_interval.upper_ms -
                     recon_state.generation_interval.upper_ms) > kUncertaintyTolerance) {
            failures.push_back({
                .reason = VerificationFailureReason::kTimingInconsistency,
                .detail = "generation interval mismatch for agent '" +
                          cert_entry.agent_id + "' field " +
                          std::to_string(field_key),
            });
        }

        // Propagated uncertainty must match within tolerance.
        if (std::abs(cert_entry.propagated_uncertainty -
                     recon_state.propagated_uncertainty) > kUncertaintyTolerance) {
            failures.push_back({
                .reason = VerificationFailureReason::kUncertaintyInconsistency,
                .detail = "propagated uncertainty mismatch for agent '" +
                          cert_entry.agent_id + "' field " +
                          std::to_string(field_key) + ": cert=" +
                          std::to_string(cert_entry.propagated_uncertainty) +
                          " recon=" +
                          std::to_string(recon_state.propagated_uncertainty),
            });
        }
    }

    // Step 6: Verify accepted agent set matches.
    auto cert_agents = cert.accepted_agents;
    auto recon_agents = reconstructed.accepted_agents;
    std::sort(cert_agents.begin(), cert_agents.end());
    std::sort(recon_agents.begin(), recon_agents.end());
    if (cert_agents != recon_agents) {
        failures.push_back({
            .reason = VerificationFailureReason::kDecisionMismatch,
            .detail = "accepted agent sets differ",
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
