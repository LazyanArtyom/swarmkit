// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <string>
#include <variant>
#include <vector>

#include "swarmkit/core/clock_quality.h"
#include "swarmkit/core/evidence_store.h"
#include "swarmkit/core/state_acceptance_certificate.h"
#include "swarmkit/core/state_acceptance_engine.h"
#include "swarmkit/core/state_quality_contract.h"

namespace swarmkit::core {

/// Reason why the verifier rejected a certificate.
enum class VerificationFailureReason : std::uint8_t {
    /// h_K does not match the recomputed consistency hash.
    kCertificateHashMismatch,
    /// Certificate schema version is unsupported.
    kCertificateSchemaMismatch,
    /// h_C in certificate does not match the provided contract.
    kContractHashMismatch,
    /// Contract version mismatch between certificate and provided contract.
    kContractVersionMismatch,
    /// Acceptance semantics version mismatch.
    kSemanticVersionMismatch,
    /// Missing evidence for a certified entry.
    kMissingEvidence,
    /// Evidence hash does not match computed SHA-256 of stored record.
    kEvidenceHashMismatch,
    /// Authoritative agent session mismatch.
    kSessionMismatch,
    /// Common evaluation time mismatch.
    kEvaluationTimeMismatch,
    /// Resolved participant set or membership revision mismatch.
    kMembershipMismatch,
    /// Propagation model, version, or declared speed assumption mismatch.
    kModelMismatch,
    /// Clock uncertainty / interval reconstruction mismatch.
    kClockMismatch,
    /// Propagated uncertainty mismatch.
    kPropagationMismatch,
    /// Coordinate frame mismatch.
    kFrameMismatch,
    /// Mission / provenance mismatch.
    kProvenanceMismatch,
    /// Predicate evaluation mismatch.
    kPredicateMismatch,
    /// Reconstructed decision disagrees with certificate contents.
    kDecisionMismatch,
    /// Accepted agent set or completeness rule mismatch.
    kCompletenessMismatch,
    /// General decision reconstruction failed.
    kDecisionReconstructionFailed,
};

/// One verification failure with context.
struct VerificationFailure {
    VerificationFailureReason reason{};
    std::string detail;
};

/// Verified acceptance result.
struct VerifiedAcceptance {
    /// The certificate that was verified.
    std::string certificate_id;
    /// Reconstructed snapshot produced independently.
    AcceptedSnapshot reconstructed_snapshot;
};

/// Verification rejection with reasons.
struct VerificationRejection {
    std::string certificate_id;
    std::vector<VerificationFailure> failures;
};

/// Result of verification: either verified acceptance or rejection.
using VerificationResult = std::variant<VerifiedAcceptance, VerificationRejection>;

/// Independent State-Acceptance Verifier (§16-§17).
///
/// Deliberately a SEPARATE code path from StateAcceptanceEngine.
/// The verifier takes a certificate K, the original evidence trace,
/// and the contract C, then reconstructs the acceptance decision
/// independently.  It must NOT share cached results or internal
/// state with the live engine.
///
/// Verification steps:
///   1. Check certificate integrity (h_K binding)
///   2. Check contract binding (h_C binding)
///   3. Reconstruct evidence selection from the trace
///   4. Reconstruct timing intervals and propagated uncertainty
///   5. Evaluate all contract predicates
///   6. Compare reconstructed decision with certificate contents
class StateAcceptanceVerifier {
   public:
    StateAcceptanceVerifier() = default;

    /// Verify a certificate against the original evidence and contract.
    ///
    /// @param cert          Certificate to verify.
    /// @param evidence      Evidence store containing the original trace.
    /// @param contract      The State-Quality Contract.
    /// @param request_ctx   Request context with t*, r*, participants.
    /// @param clock_states  Per-agent clock quality state.
    /// @return VerifiedAcceptance if the decision is reproducible,
    ///         VerificationRejection otherwise.
    [[nodiscard]] VerificationResult Verify(
        const StateAcceptanceCertificate& cert,
        const EvidenceStore& evidence,
        const StateQualityContract& contract,
        const SnapshotRequestContext& request_ctx,
        const std::unordered_map<std::string, ClockQualityState>& clock_states)
        const;

   private:
    /// Tolerance for floating-point comparison of uncertainty values.
    static constexpr double kUncertaintyTolerance = 1e-6;
};

}  // namespace swarmkit::core
