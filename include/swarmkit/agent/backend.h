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
#include "swarmkit/core/result.h"
#include "swarmkit/core/telemetry.h"

namespace swarmkit::agent {

using swarmkit::commands::Command;
using swarmkit::commands::CommandContext;
using swarmkit::commands::CommandEnvelope;
using swarmkit::commands::CommandPriority;

struct BackendHealth {
    bool ready{true};                           ///< Backend can accept normal commands.
    std::string message{"ready"};               ///< Human-readable readiness summary.
    std::string backend_name{"unknown"};        ///< Backend implementation name.
    std::string protocol{"unknown"};            ///< Vehicle protocol name, for example "mavlink".
    std::int64_t last_heartbeat_unix_ms{};      ///< Last heartbeat time in Unix milliseconds.
    std::int64_t last_telemetry_unix_ms{};      ///< Last telemetry time in Unix milliseconds.
    bool armed{false};                          ///< Vehicle is armed.
    bool landed{false};                         ///< Vehicle reports landed or ground state.
    std::string mode{};                         ///< Current autopilot/backend mode.
    int custom_mode{-1};                        ///< Backend-specific mode id; -1 when unavailable.
    bool failsafe{false};                       ///< Vehicle reports failsafe state.
    bool gps_ok{false};                         ///< GPS quality satisfies backend readiness checks.
    int gps_fix_type{};                         ///< Backend-specific GPS fix type.
    int satellites_visible{};                   ///< Visible/tracked satellite count.
    float gps_hdop{};                           ///< Horizontal dilution of precision.
    bool ekf_ok{true};                          ///< Estimator/EKF state satisfies readiness checks.
    bool has_relative_altitude{false};          ///< relative_alt_m contains a valid measurement.
    float relative_alt_m{};                     ///< Relative altitude in metres.
    std::optional<float> link_quality_percent;  ///< Optional link quality percentage.
};

/// @brief Optional numeric capability limits reported by a backend.
struct BackendNumericLimits {
    std::optional<float> max_horizontal_speed_mps;  ///< Maximum horizontal speed in m/s.
    std::optional<float> max_climb_speed_mps;       ///< Maximum climb speed in m/s.
    std::optional<float> max_descent_speed_mps;     ///< Maximum descent speed in m/s.
    std::optional<float> max_altitude_m;            ///< Maximum supported altitude in metres.
};

/// @brief Backend feature set exposed through the client capabilities RPC.
struct BackendCapabilities {
    std::string backend_name{"unknown"};          ///< Backend implementation name.
    std::string protocol{"unknown"};              ///< Vehicle protocol name.
    std::string vehicle_class{"unknown"};         ///< Vehicle class, for example "multirotor".
    bool supports_payload_control{false};         ///< Payload command support.
    bool supports_velocity_control{false};        ///< Velocity setpoint support.
    bool supports_flight_termination{false};      ///< Emergency termination support.
    bool supports_backend_commands{false};        ///< Backend-specific command support.
    std::string autopilot_type{"unknown"};        ///< Autopilot family/profile if known.
    std::vector<std::string> supported_modes{};     ///< Mode names accepted by CmdSetMode.
    std::vector<std::string> supported_commands{};  ///< Generic command names.
    std::vector<std::string> supported_payloads{};  ///< Payload names/devices.
    std::vector<std::string> supported_telemetry_fields{};  ///< Telemetry fields produced.
    std::vector<std::string> backend_command_names{};       ///< Names accepted by BackendCmd.
    BackendNumericLimits limits{};
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
/// return quickly, and must not call back into the backend.
/// ---------------------------------------------------------------------------
class IDroneBackend {
   public:
    using TelemetryCallback = std::function<void(const swarmkit::core::TelemetryFrame&)>;

    virtual ~IDroneBackend() = default;

    /// @brief Start backend resources that should be active before the first RPC.
    [[nodiscard]] virtual swarmkit::core::Result Start() {
        return swarmkit::core::Result::Success();
    }

    /// @brief Execute a flight command described by @p envelope.
    /// @returns Ok on acceptance, Rejected if the command is not valid in the
    ///          current state, or Failed on a hard error.
    [[nodiscard]] virtual swarmkit::core::Result Execute(const CommandEnvelope& envelope) = 0;

    /// @brief Begin streaming telemetry for @p drone_id at approximately
    ///        @p rate_hertz frames per second.
    ///
    /// @p callback is invoked from the backend's internal thread for every
    /// frame produced.  Calling StartTelemetry() when a stream is already
    /// running returns Rejected without disturbing the existing stream.
    [[nodiscard]] virtual swarmkit::core::Result StartTelemetry(const std::string& drone_id,
                                                                int rate_hertz,
                                                                TelemetryCallback callback) = 0;

    /// @brief Stop the active telemetry stream for @p drone_id.
    /// @note Safe to call when no stream is running (returns Ok silently).
    [[nodiscard]] virtual swarmkit::core::Result StopTelemetry(const std::string& drone_id) = 0;

    /// @brief Report backend-specific readiness/liveness state for health checks.
    [[nodiscard]] virtual BackendHealth GetHealth() const {
        return {};
    }

    /// @brief Report backend/autopilot capabilities exposed to SDK clients.
    [[nodiscard]] virtual BackendCapabilities GetCapabilities() const {
        return {};
    }
};

using DroneBackendPtr = std::unique_ptr<IDroneBackend>;

}  // namespace swarmkit::agent
