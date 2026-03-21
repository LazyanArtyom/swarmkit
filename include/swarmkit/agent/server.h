#pragma once

#include <string>

#include "swarmkit/agent/backend.h"

namespace swarmkit::agent {

/// ---------------------------------------------------------------------------
/// AgentConfig -- startup parameters for the gRPC agent server.
/// ---------------------------------------------------------------------------
struct AgentConfig {
    std::string agent_id{"agent-1"};    ///< Unique identifier for this agent.
    std::string bind_addr{"0.0.0.0:50061"};  ///< gRPC listen address.
    std::string inbox_dir{"/tmp/swarmkit_inbox"};  ///< Directory for file uploads.
};

/// @brief Start the agent gRPC server.
///
/// Blocks until the server shuts down.  Returns 0 on clean exit, non-zero on
/// error (e.g. port already in use, null backend).
[[nodiscard]] int RunAgentServer(const AgentConfig& config, DroneBackendPtr backend);

}  // namespace swarmkit::agent

