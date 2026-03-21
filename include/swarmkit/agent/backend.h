// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <functional>
#include <memory>
#include <string>

#include "swarmkit/commands.h"
#include "swarmkit/core/result.h"
#include "swarmkit/core/telemetry.h"

namespace swarmkit::agent {

using swarmkit::commands::Command;
using swarmkit::commands::CommandContext;
using swarmkit::commands::CommandEnvelope;
using swarmkit::commands::CommandPriority;

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
};

using DroneBackendPtr = std::unique_ptr<IDroneBackend>;

}  // namespace swarmkit::agent
