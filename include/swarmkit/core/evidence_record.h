// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>

#include "swarmkit/core/telemetry.h"

namespace swarmkit::core {

/// Identifies which independently-updated measurement field an evidence record
/// belongs to.  Maps directly to the field set F in a State-Quality Contract
/// (§5 / §11 of the dissertation).
enum class EvidenceFieldId : std::uint8_t {
    kPosition,
    kVelocity,
    kAttitude,
    kBattery,
    kEstimatorHealth,
    kGpsQuality,
    kLinkQuality,
    kArmedState,
    kLandedState,
    kFailsafeState,
};

/// Identity provenance carried by every evidence record (§5).
///
/// These fields collectively identify one exact measurement sample.  The
/// acceptance engine uses them to enforce epoch, frame, source, and mission
/// predicates.
struct EvidenceIdentity {
    std::string agent_id;
    std::string agent_session_id;        ///< Agent process incarnation (epoch, §6).
    EvidenceFieldId field_id{};
    std::uint64_t sequence{};            ///< Per-agent monotonic telemetry sequence.
    std::string source_component;        ///< Backend/estimator/source name.
    CoordinateFrame coordinate_frame{CoordinateFrame::kUnknown};
    std::string estimator_id;
    UncertaintySemantics uncertainty_kind{UncertaintySemantics::kUnknown};
    std::string mission_id;
    std::uint64_t mission_revision{};

    bool operator==(const EvidenceIdentity&) const = default;
};

/// Typed value stored in an evidence record.
///
/// Position values use the 3-element array [north_or_lat, east_or_lon, down_or_alt]
/// whose interpretation depends on the coordinate frame in EvidenceIdentity.
/// Scalar fields (battery, link quality) use the float variant.
/// Boolean fields (armed, landed, failsafe) use the bool variant.
using EvidenceValue = std::variant<
    std::array<double, 3>,   // position (lat,lon,alt or NED)
    std::array<float, 3>,    // velocity (vx, vy, vz) or attitude (roll, pitch, yaw)
    float,                   // scalar (battery %, link quality %, GPS HDOP)
    bool,                    // boolean (armed, landed, failsafe)
    EstimatorState,          // estimator health
    GpsQuality               // GPS quality
>;

/// Uncertainty and health descriptor attached to one evidence record (q in §5).
struct EvidenceQuality {
    std::optional<UncertaintyEstimate> uncertainty;
    bool estimator_healthy{false};
    bool estimator_position_ok{false};
    bool estimator_velocity_ok{false};

    bool operator==(const EvidenceQuality&) const = default;
};

/// One evidence record Z_{i,f,k} as defined in §5 of the dissertation:
///
///   Z = (x̂, s, r, q, γ)
///
/// Where:
///   x̂ = measured/estimated value
///   s = source time in source clock domain
///   r = receive time in runtime/reference domain
///   q = uncertainty and health descriptor
///   γ = identity, frame, source, and mission provenance
struct EvidenceRecord {
    EvidenceValue value;                 ///< x̂_{i,f,k}
    TimestampEvidence source_time;       ///< s_{i,f,k} in source clock domain.
    std::int64_t receive_time_ms{};      ///< r_{i,f,k} in runtime reference domain (Unix ms).
    EvidenceQuality quality;             ///< q_{i,f,k}
    EvidenceIdentity identity;           ///< γ_{i,f,k}

    bool operator==(const EvidenceRecord&) const = default;
};

}  // namespace swarmkit::core
