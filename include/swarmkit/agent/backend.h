// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "swarmkit/commands.h"
#include "swarmkit/core/capabilities.h"
#include "swarmkit/core/command_outcome.h"
#include "swarmkit/core/result.h"
#include "swarmkit/core/telemetry.h"

namespace swarmkit::agent {

using swarmkit::commands::Command;
using swarmkit::commands::CommandContext;
using swarmkit::commands::CommandEnvelope;
using swarmkit::commands::CommandPriority;

struct BackendHealth {
    bool ready{false};                              ///< Backend can accept normal commands.
    std::string message{"backend health unknown"};  ///< Human-readable readiness summary.
    std::string backend_name{"unknown"};            ///< Backend implementation name.
    std::string protocol{"unknown"};            ///< Vehicle protocol name, for example "mavlink".
    std::int64_t last_heartbeat_unix_ms{};      ///< Last heartbeat time in Unix milliseconds.
    std::int64_t last_telemetry_unix_ms{};      ///< Last telemetry time in Unix milliseconds.
    std::optional<bool> armed;                  ///< Vehicle armed state, absent until observed.
    std::optional<bool> landed;                 ///< Landed/ground state, absent until observed.
    std::string mode{};                         ///< Current autopilot/backend mode.
    int custom_mode{-1};                        ///< Backend-specific mode id; -1 when unavailable.
    std::optional<bool> failsafe;               ///< Failsafe state, absent until observed.
    std::optional<bool> gps_ok;                 ///< GPS health, absent until observed.
    int gps_fix_type{};                         ///< Backend-specific GPS fix type.
    int satellites_visible{};                   ///< Visible/tracked satellite count.
    float gps_hdop{};                           ///< Horizontal dilution of precision.
    std::optional<bool> ekf_ok;                 ///< Estimator health, absent until observed.
    bool has_relative_altitude{false};          ///< relative_alt_m contains a valid measurement.
    float relative_alt_m{};                     ///< Relative altitude in metres.
    std::optional<float> link_quality_percent;  ///< Optional link quality percentage.
};

/// Algorithm-neutral evidence emitted by a backend or backend decorator.
///
/// This channel is intended for reconstructing execution conditions such as
/// deterministic fault-scheduler decisions.  It is never vehicle telemetry
/// and must not be used as an estimator or ground-truth input.
struct BackendEvidenceEvent {
    std::string source;
    std::string kind;
    std::uint64_t source_sequence{};
    std::optional<std::uint64_t> random_seed;
    std::unordered_map<std::string, std::string> attributes;
};

/// @brief Request passed to BackendRegistry creators.
struct BackendFactoryRequest {
    std::string backend_name;                              ///< Registered backend name.
    std::unordered_map<std::string, std::string> options;  ///< Backend-specific options.
};

/// ---------------------------------------------------------------------------
/// IDroneBackend -- abstract interface for drone vehicle control.
///
/// Implement this interface to connect SwarmKit to a real flight controller
/// (e.g. MAVSDK), a hardware-in-the-loop simulator, or a custom system.
///
/// @par Threading contract
/// Execute() and StartTelemetry()/StopTelemetry() may be called from any
/// thread.  The TelemetryCallback supplied to StartTelemetry() is invoked
/// from the backend's own internal thread.  The callback must be thread-safe,
/// return quickly, and must not call back into the backend. The Agent-supplied
/// callback does not throw.
/// ---------------------------------------------------------------------------
class IDroneBackend {
   public:
    using TelemetryCallback = std::function<void(const swarmkit::core::TelemetryFrame&)>;
    using EvidenceCallback = std::function<void(const BackendEvidenceEvent&)>;

    virtual ~IDroneBackend() = default;

    /// @brief Start backend resources that should be active before the first RPC.
    /// @note Idempotent; repeated calls must not duplicate worker resources.
    [[nodiscard]] virtual swarmkit::core::Result Start() {
        return swarmkit::core::Result::Success();
    }

    /// @brief Execute a flight command described by @p envelope.
    /// @returns Ok on acceptance, Rejected if the command is not valid in the
    ///          current state, or Failed on a hard error.
    [[nodiscard]] virtual core::BackendCommandOutcome Execute(const CommandEnvelope& envelope) = 0;

    /// @brief Begin streaming telemetry for @p drone_id at approximately
    ///        @p rate_hertz frames per second.
    ///
    /// @p callback is invoked from the backend's internal thread for every
    /// frame produced.  Calling StartTelemetry() when a stream is already
    /// running returns Rejected without disturbing the existing stream.
    /// @param rate_hertz Must be greater than zero.
    /// @param callback Must not be empty.
    [[nodiscard]] virtual swarmkit::core::Result StartTelemetry(const std::string& drone_id,
                                                                int rate_hertz,
                                                                TelemetryCallback callback) = 0;

    /// @brief Stop the active telemetry stream for @p drone_id.
    /// @note Safe to call when no stream is running (returns Ok silently).
    /// @post No callback for this stream is running or can begin after this
    ///       function returns. Implementations must wait for in-flight callbacks.
    [[nodiscard]] virtual swarmkit::core::Result StopTelemetry(const std::string& drone_id) = 0;

    /// @brief Report backend-specific readiness/liveness state for health checks.
    [[nodiscard]] virtual BackendHealth GetHealth() const {
        return {};
    }

    /// @brief Report backend/autopilot capabilities exposed to SDK clients.
    [[nodiscard]] virtual core::BackendCapabilities GetCapabilities() const {
        return {};
    }

    /// @brief Install the sink for backend-originated execution evidence.
    ///
    /// Backends that do not produce auxiliary evidence may keep the default
    /// no-op implementation. Decorators must forward underlying events and
    /// serialize callback invocation with their own event production.
    virtual void SetEvidenceCallback(const EvidenceCallback& callback) {
        static_cast<void>(callback);
    }
};

using DroneBackendPtr = std::unique_ptr<IDroneBackend>;

}  // namespace swarmkit::agent
