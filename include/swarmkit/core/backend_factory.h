#pragma once

#include "swarmkit/core/drone_backend.h"

namespace swarmkit::core {

// Baseline backend used for desktop development.
DroneBackendPtr MakeMockBackend();

}  // namespace swarmkit::core
