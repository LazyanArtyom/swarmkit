// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "swarmkit/core/evidence_record.h"
#include "swarmkit/core/telemetry.h"

namespace swarmkit::core {

/// Agent-set completeness rule for State-Quality Contract (§11 item S).
enum class CompletenessRule : std::uint8_t {
    /// Every agent in `required_agents` must have accepted evidence.
    kAllRequired,
    /// At least `min_required_agents` must have accepted evidence.
    kMinimumCount,
};

/// State-Quality Contract C = (F, A, R, U, H, P, S, M) from §11.
///
/// An accepted state means every mandatory predicate was established.
/// A rejected state means the runtime refuses to claim they were established.
/// There is no "accepted with warnings" — that would be a silent downgrade.
struct StateQualityContract {
    // -- Contract identity and versioning (§46) --

    std::string contract_id;
    std::uint32_t schema_version{1};
    std::uint32_t content_version{1};

    // -- F: Required fields (§11) --

    std::vector<EvidenceFieldId> required_fields;

    // -- A: Maximum evidence-age predicates (§11) --

    /// Maximum age Δ⁺ in milliseconds for each required field.
    /// When present, applies to all agents.  Per-agent overrides
    /// can be added in a future version.
    std::optional<double> max_evidence_age_ms;

    // -- R: Maximum clock-uncertainty predicates (§11) --

    /// Maximum clock uncertainty radius ρ in milliseconds.
    std::optional<double> max_clock_uncertainty_ms;

    // -- U: Maximum propagated state-uncertainty predicates (§11) --

    /// Maximum propagated position uncertainty ε_p(t*) in metres.
    std::optional<double> max_position_uncertainty_m;

    /// Maximum propagated velocity uncertainty in m/s.
    std::optional<double> max_velocity_uncertainty_mps;

    // -- H: Health/estimator predicates (§11) --

    /// When true, the estimator must report healthy for position.
    bool require_estimator_position_ok{false};

    /// When true, the estimator must report healthy for velocity.
    bool require_estimator_velocity_ok{false};

    /// When true, estimator_state must be kHealthy (not degraded/fault).
    bool require_estimator_healthy{false};

    /// Minimum acceptable GPS quality (kUnknown means no GPS requirement).
    GpsQuality min_gps_quality{GpsQuality::kUnknown};

    // -- P: Provenance, frame, source, identity predicates (§11) --

    /// Required coordinate frame for position evidence.
    /// kUnknown means any frame is accepted.
    CoordinateFrame required_position_frame{CoordinateFrame::kUnknown};

    /// Required coordinate frame for velocity evidence.
    CoordinateFrame required_velocity_frame{CoordinateFrame::kUnknown};

    /// When true, evidence must come from the agent's current session (§6).
    /// E_msg == E_cur.
    bool require_current_epoch{true};

    /// When true, evidence must carry the current mission ID and revision.
    bool require_current_mission{false};

    /// Required mission ID (evaluated only when require_current_mission is true).
    std::string required_mission_id;

    /// Required mission revision (evaluated only when require_current_mission is true).
    std::uint64_t required_mission_revision{};

    // -- S: Required UAV set and completeness rule (§11) --

    /// Set of required agent IDs.
    std::unordered_set<std::string> required_agents;

    /// Completeness rule for the agent set.
    CompletenessRule completeness{CompletenessRule::kAllRequired};

    /// Minimum number of agents when completeness is kMinimumCount.
    std::size_t min_required_agents{};

    // -- M: Mission/session/goal consistency predicates (§11) --
    //    (Covered by require_current_epoch and require_current_mission above.)

    // -- Deterministic vs probabilistic mode (§10) --

    /// When true, the contract requires deterministic hard bounds on
    /// uncertainty.  Evidence with kStandardDeviation or kConfidenceBound
    /// uncertainty semantics will cause rejection.
    bool require_deterministic_bounds{false};

    // -- Propagation model binding (§14 item M) --

    /// Propagation model identifier for certificate versioning.
    std::string propagation_model_id{"ball-enclosure-v1"};

    /// Propagation model version for certificate versioning.
    std::string propagation_model_version{"1.0"};

    /// Maximum horizontal speed V_max used for uncertainty propagation (m/s).
    /// This is a declared motion bound, not measured.
    float max_horizontal_speed_mps{10.0F};

    /// Maximum vertical speed used for uncertainty propagation (m/s).
    float max_vertical_speed_mps{5.0F};
};

/// Compute a SHA-256 content hash of the contract's configuration fields.
/// Used for certificate binding (§14 h_C).
[[nodiscard]] std::string ComputeContractHash(const StateQualityContract& contract);

}  // namespace swarmkit::core
