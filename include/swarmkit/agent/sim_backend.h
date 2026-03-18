#pragma once

#include "swarmkit/agent/backend.h"

namespace swarmkit::agent {

// ---------------------------------------------------------------------------
// MakeSimBackend — creates the built-in simulator backend.
//
// The simulator generates synthetic GPS/altitude/battery telemetry on a
// background thread and accepts all commands, logging them without error.
// Use it for local development, demos, and integration testing when no
// physical vehicle or hardware-in-the-loop simulator is available.
// ---------------------------------------------------------------------------
DroneBackendPtr MakeSimBackend();

}  // namespace swarmkit::agent
