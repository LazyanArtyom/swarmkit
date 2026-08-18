// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "swarmkit/core/clock_quality.h"
#include "swarmkit/core/evidence_record.h"
#include "swarmkit/core/state_acceptance_engine.h"

namespace swarmkit::core {

/// Per-field evidence summary in a certificate (§14 item E).
struct CertificateEvidenceEntry {
    std::string agent_id;
    EvidenceFieldId field{};
    std::uint64_t sequence{};
    std::string agent_session_id;
    std::string source_component;

    /// Source timestamp (s) in source domain.
    std::optional<std::int64_t> source_time_ms;

    /// Generation-time interval in reference domain.
    GenerationTimeInterval generation_interval;

    /// Conservative elapsed Δ⁺ at t*.
    double conservative_elapsed_ms{};

    /// Clock uncertainty ρ used.
    double clock_uncertainty_ms{};

    /// Observation-time uncertainty e.
    double observation_uncertainty{};

    /// Propagated uncertainty ε(t*).
    double propagated_uncertainty{};

    bool operator==(const CertificateEvidenceEntry&) const = default;
};

/// State-Acceptance Certificate K = (id, h_C, v_C, t*, E, T, Q, M, V, h_K)
/// from §14 of the dissertation.
///
/// A compact, tamper-evident artifact that records exactly which evidence
/// was used, under which contract, at which time, with which propagation
/// model, to arrive at an acceptance decision.  An independent verifier
/// can reproduce the decision from K plus the original evidence trace.
struct StateAcceptanceCertificate {
    // -- id: Certificate identity --
    std::string certificate_id;

    // -- h_C: Contract binding --
    std::string contract_id;
    std::uint32_t contract_schema_version{};
    std::uint32_t contract_content_version{};
    std::string contract_hash;  ///< SHA-256 of serialized contract.

    // -- t*: Common evaluation time --
    double evaluation_time_ms{};

    // -- E: Evidence entries --
    std::vector<CertificateEvidenceEntry> evidence_entries;

    // -- T: Timing summary --
    double max_clock_uncertainty_ms{};
    double max_conservative_elapsed_ms{};

    // -- Q: Uncertainty summary --
    double max_propagated_position_uncertainty_m{};
    double max_propagated_velocity_uncertainty_mps{};

    // -- M: Model binding --
    std::string propagation_model_id;
    std::string propagation_model_version;
    float max_horizontal_speed_mps{};
    float max_vertical_speed_mps{};

    // -- V: Accepted agent set --
    std::vector<std::string> accepted_agents;

    // -- Acceptance semantics version --
    std::string acceptance_semantics_version{"1.0"};

    // -- Production timestamp (runtime reference clock) --
    std::int64_t produced_at_ms{};

    // -- h_K: Tamper-evident hash binding --
    /// SHA-256 hex digest of the canonical serialization of all preceding fields.
    /// Computed last, after all other fields are populated.
    std::string certificate_hash;

    bool operator==(const StateAcceptanceCertificate&) const = default;
};

/// Build a State-Acceptance Certificate from an AcceptedSnapshot.
///
/// @param snapshot  The accepted state produced by the engine.
/// @param contract  The contract that was satisfied.
/// @return Complete certificate with computed h_K.
[[nodiscard]] StateAcceptanceCertificate BuildCertificate(
    const AcceptedSnapshot& snapshot,
    const StateQualityContract& contract);

/// Compute the canonical hash h_K for a certificate.
/// Used both during building and during verification.
///
/// @param cert  Certificate with all fields except certificate_hash populated.
/// @return SHA-256 hex digest.
[[nodiscard]] std::string ComputeCertificateHash(
    const StateAcceptanceCertificate& cert);

/// Verify the tamper-evident hash binding of a certificate.
///
/// @param cert  Certificate to verify.
/// @return true if h_K matches the recomputed hash.
[[nodiscard]] bool VerifyCertificateIntegrity(
    const StateAcceptanceCertificate& cert);

}  // namespace swarmkit::core
