#pragma once

#include <functional>
#include <memory>
#include <string>

#include "swarmkit/commands.h"
#include "swarmkit/core/result.h"
#include "swarmkit/core/telemetry.h"

namespace swarmkit::agent {

// ---------------------------------------------------------------------------
// IDroneBackend — abstract interface for drone vehicle control.
//
// Implement this interface to connect SwarmKit to a real flight controller
// (e.g. MAVSDK), a hardware-in-the-loop simulator, or a custom system.
//
// Threading contract:
//   Execute() and StartTelemetry()/StopTelemetry() may be called from any
//   thread.  The TelemetryCallback supplied to StartTelemetry() is invoked
//   from the backend's own internal thread.  The callback must be thread-safe,
//   return quickly, and must not call back into the backend.
// ---------------------------------------------------------------------------
class IDroneBackend {
   public:
    using TelemetryCallback = std::function<void(const swarmkit::core::TelemetryFrame&)>;

    virtual ~IDroneBackend() = default;

    // Execute a flight command described by envelope.
    // Returns Ok on acceptance, Rejected if the command is not valid in the
    // current state, or Failed on a hard error.
    [[nodiscard]] virtual swarmkit::core::Result Execute(
        const CommandEnvelope& envelope) = 0;

    // Begin streaming telemetry for drone_id at approximately rate_hz frames
    // per second.  callback is invoked from the backend's internal thread for
    // every frame produced.  Calling StartTelemetry() when a stream is already
    // running returns Rejected without disturbing the existing stream.
    [[nodiscard]] virtual swarmkit::core::Result StartTelemetry(
        const std::string&  drone_id,
        int                 rate_hz,
        TelemetryCallback   callback) = 0;

    // Stop the active telemetry stream for drone_id.
    // Safe to call when no stream is running (returns Ok silently).
    [[nodiscard]] virtual swarmkit::core::Result StopTelemetry(
        const std::string& drone_id) = 0;
};

using DroneBackendPtr = std::unique_ptr<IDroneBackend>;

}  // namespace swarmkit::agent

