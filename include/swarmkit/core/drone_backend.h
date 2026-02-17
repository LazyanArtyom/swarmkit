#ifndef SWARMKIT_CORE_DRONE_BACKEND_H_
#define SWARMKIT_CORE_DRONE_BACKEND_H_

#include <functional>
#include <memory>
#include <string>

#include "swarmkit/core/commands.h"
#include "swarmkit/core/result.h"
#include "swarmkit/core/telemetry.h"

namespace swarmkit::core {

class IDroneBackend {
   public:
    using TelemetryCallback = std::function<void(const TelemetryFrame&)>;

    virtual ~IDroneBackend() = default;

    virtual Result Execute(const CommandContext& context, const Command& command) = 0;

    virtual void SetTelemetryCallback(TelemetryCallback callback) = 0;
    virtual Result StartTelemetry(const std::string& drone_id, int rate_hz) = 0;
    virtual Result StopTelemetry(const std::string& drone_id) = 0;
};

using DroneBackendPtr = std::unique_ptr<IDroneBackend>;

}  // namespace swarmkit::core
#endif  // SWARMKIT_CORE_DRONE_BACKEND_H_