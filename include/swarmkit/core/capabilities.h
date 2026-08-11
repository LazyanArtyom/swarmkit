// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "swarmkit/core/telemetry.h"

namespace swarmkit::core {

/// Tri-state capability support. Unknown is deliberately distinct from false:
/// it means the producer did not make a support claim.
enum class CapabilitySupport : std::uint8_t {
    kUnknown,
    kUnsupported,
    kSupported,
};

/// Algorithm-neutral evidence features available through normalized telemetry.
/// A supported signal may still be absent from an individual frame; consumers
/// must continue to inspect validity and provenance on every sample.
struct TelemetryEvidenceCapabilities {
    CapabilitySupport source_timestamp{CapabilitySupport::kUnknown};
    std::vector<ClockDomain> source_clock_domains;
    CapabilitySupport position_estimate{CapabilitySupport::kUnknown};
    CapabilitySupport horizontal_position_uncertainty{CapabilitySupport::kUnknown};
    CapabilitySupport vertical_position_uncertainty{CapabilitySupport::kUnknown};
    CapabilitySupport horizontal_velocity{CapabilitySupport::kUnknown};
    CapabilitySupport vertical_velocity{CapabilitySupport::kUnknown};
    CapabilitySupport horizontal_velocity_uncertainty{CapabilitySupport::kUnknown};
    CapabilitySupport vertical_velocity_uncertainty{CapabilitySupport::kUnknown};
    CapabilitySupport speed_uncertainty{CapabilitySupport::kUnknown};
    CapabilitySupport uncertainty_semantics{CapabilitySupport::kUnknown};
    CapabilitySupport estimator_health{CapabilitySupport::kUnknown};
    CapabilitySupport failsafe_state{CapabilitySupport::kUnknown};
    CapabilitySupport active_goal_lineage{CapabilitySupport::kUnknown};
    CapabilitySupport telemetry_sequence{CapabilitySupport::kUnknown};
    CapabilitySupport telemetry_replay{CapabilitySupport::kUnknown};

    bool operator==(const TelemetryEvidenceCapabilities&) const = default;
};

/// Meaning of a reported platform motion limit. None of these values is
/// silently equivalent to a deterministic physical bound.
enum class MotionLimitSemantics : std::uint8_t {
    kUnknown,
    kConfiguredCommandLimit,
    kPlatformCapabilityAssumption,
    kObservedLimit,
    kValidatedBound,
};

struct MotionLimit {
    float value{};
    MotionLimitSemantics semantics{MotionLimitSemantics::kUnknown};
    std::string source;
    std::string profile_id;
    std::string profile_version;

    bool operator==(const MotionLimit&) const = default;
};

/// Backend/platform features independent of the RPC used to retrieve them.
struct BackendCapabilities {
    std::string backend_name{"unknown"};
    std::string protocol{"unknown"};
    std::string vehicle_class{"unknown"};
    bool supports_payload_control{false};
    bool supports_velocity_control{false};
    bool supports_flight_termination{false};
    bool supports_backend_commands{false};
    std::string autopilot_type{"unknown"};
    std::vector<std::string> supported_modes;
    std::vector<std::string> supported_commands;
    std::vector<std::string> supported_payloads;
    std::vector<std::string> backend_command_names;
    TelemetryEvidenceCapabilities evidence;
    std::optional<MotionLimit> max_horizontal_speed;
    std::optional<MotionLimit> max_climb_speed;
    std::optional<MotionLimit> max_descent_speed;
    std::optional<MotionLimit> max_altitude;

    bool operator==(const BackendCapabilities&) const = default;
};

}  // namespace swarmkit::core
