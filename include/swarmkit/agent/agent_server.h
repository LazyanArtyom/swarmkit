#ifndef SWARMKIT_AGENT_AGENT_SERVER_H_
#define SWARMKIT_AGENT_AGENT_SERVER_H_

#include <memory>
#include <string>

namespace swarmkit::core {
class IDroneBackend;
}

namespace swarmkit::agent {

struct AgentConfig {
    std::string agent_id{"agent-1"};
    std::string bind_addr{"0.0.0.0:50061"};
    std::string inbox_dir{"/tmp/swarmkit_inbox"};
};

int RunAgentServer(const AgentConfig& config,
                   std::unique_ptr<swarmkit::core::IDroneBackend> backend);

}  // namespace swarmkit::agent
#endif  // SWARMKIT_AGENT_AGENT_SERVER_H_