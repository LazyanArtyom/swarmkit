#pragma once

#include <string>

#include "swarmkit/agent/backend.h"

namespace swarmkit::agent {

// ---------------------------------------------------------------------------
// AgentConfig — startup parameters for the gRPC agent server.
// ---------------------------------------------------------------------------
struct AgentConfig {
    std::string agent_id{"agent-1"};
    std::string bind_addr{"0.0.0.0:50061"};
    std::string inbox_dir{"/tmp/swarmkit_inbox"};
};

// Starts the agent gRPC server.
// Blocks until the server shuts down.  Returns 0 on clean exit, non-zero on
// error (e.g. port already in use, null backend).
int RunAgentServer(const AgentConfig& config, DroneBackendPtr backend);

}  // namespace swarmkit::agent

