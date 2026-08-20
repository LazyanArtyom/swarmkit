// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "swarmkit/core/clock_quality.h"
#include "swarmkit/core/evidence_record.h"
#include "swarmkit/core/state_acceptance_engine.h"

namespace swarmkit::core {

inline constexpr char kCertificateSchemaVersion[] = "CERT_V3";
inline constexpr char kAcceptanceSemanticsVersion[] = "2.0";

/// Per-field evidence summary in a certificate (§14 item E).
struct CertificateEvidenceEntry {
    std::string agent_id;
    EvidenceFieldId field{};
    std::uint64_t sequence{};
    std::string agent_session_id;
    std::string source_component;

    /// Canonical evidence identifier for deterministic replay.
    std::string evidence_id;

    /// Agent incarnation ID bound to this evidence.
    std::string agent_incarnation_id;

    /// Source timestamp (s) in source domain.
    std::optional<std::int64_t> source_time_ms;

    /// Evidence receive time r in runtime reference domain (Unix ms).
    std::int64_t receive_time_ms{};

    /// Generation-time interval in reference domain.
    GenerationTimeInterval generation_interval;

    /// Conservative elapsed Δ⁺ at t*.
    double conservative_elapsed_ms{};

    /// Clock offset estimate θ̂ used for this evidence item.
    double theta_hat_ms{};

    /// Effective clock uncertainty ρ_eff used (including drift budget).
    double effective_rho_ms{};

    /// Base synchronization uncertainty ρ_sync.
    double base_rho_ms{};

    /// Maximum drift-rate bound used to expand ρ_sync (ppm).
    double max_drift_rate_ppm{};

    /// Reference-domain time at which the clock model was last updated.
    std::int64_t clock_model_last_update_reference_ms{};

    /// Clock model version identifier.
    std::string clock_model_version;

    /// Whether the selected clock model carries deterministic-bound semantics.
    bool clock_deterministic_bound{false};

    /// Observation-time uncertainty e.
    double observation_uncertainty{};

    /// Propagated uncertainty ε(t*).
    double propagated_uncertainty{};

    /// Coordinate frame of the observation.
    CoordinateFrame coordinate_frame{CoordinateFrame::kUnknown};

    /// Uncertainty semantics of the observation.
    UncertaintySemantics uncertainty_semantics{UncertaintySemantics::kUnknown};

    /// Clock domain.
    ClockDomain clock_domain{ClockDomain::kUnknown};

    /// Clock synchronization state.
    ClockSynchronization clock_synchronization{ClockSynchronization::kUnknown};

    /// GPS quality carried by GPS evidence. Absent for non-GPS fields.
    std::optional<GpsQuality> gps_quality;

    /// Estimator health flags.
    bool estimator_healthy{false};
    bool estimator_position_ok{false};
    bool estimator_velocity_ok{false};

    /// Mission provenance.
    std::string mission_id;
    std::uint64_t mission_revision{0};

    /// SHA-256 digest of the exact EvidenceRecord used (P1.1).
    std::string evidence_hash;

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

    // -- Schema version for deterministic replay --
    std::string certificate_schema_version{kCertificateSchemaVersion};

    // -- h_C: Contract binding --
    std::string contract_id;
    std::uint32_t contract_schema_version{};
    std::uint32_t contract_content_version{};
    std::string contract_hash;  ///< SHA-256 of serialized contract.

    // -- t*: Common evaluation time --
    double evaluation_time_ms{};

    // -- r*: Evidence-freeze cutoff (§8) --
    std::int64_t evidence_freeze_ms{};

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

    // -- I*: Participant snapshot with membership revision --
    ParticipantSnapshot participants;

    // -- Acceptance semantics version --
    std::string acceptance_semantics_version{kAcceptanceSemanticsVersion};

    // -- Production timestamp (runtime reference clock) --
    std::int64_t produced_at_ms{};

    // -- h_K: Integrity hash binding --
    /// SHA-256 hex digest of the canonical serialization of all preceding fields.
    /// Computed last, after all other fields are populated.
    std::string certificate_hash;

    bool operator==(const StateAcceptanceCertificate&) const = default;
};

/// Build a State-Acceptance Certificate from an AcceptedSnapshot.
///
/// @param snapshot    The accepted state produced by the engine.
/// @param contract    The contract that was satisfied.
/// @param request_ctx The request context with t*, r*, participants.
/// @return Complete certificate with computed h_K.
[[nodiscard]] StateAcceptanceCertificate BuildCertificate(
    const AcceptedSnapshot& snapshot,
    const StateQualityContract& contract,
    const SnapshotRequestContext& request_ctx);

/// Compute the canonical SHA-256 hash of an individual EvidenceRecord (P1.1).
[[nodiscard]] std::string ComputeEvidenceHash(
    const EvidenceRecord& record);

/// Canonical total-order identifier used as the final selector tie-break.
/// A content-hash suffix makes reused sequence identities deterministic.
[[nodiscard]] std::string ComputeCanonicalEvidenceId(
    const EvidenceRecord& record);

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

/// Canonical deterministic serialization of a certificate to byte string (P1.2).
[[nodiscard]] std::string SerializeCertificate(
    const StateAcceptanceCertificate& cert);

/// Canonical deserialization of a certificate from byte string.
[[nodiscard]] std::optional<StateAcceptanceCertificate> DeserializeCertificate(
    std::string_view data);

}  // namespace swarmkit::core
