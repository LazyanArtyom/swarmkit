#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>

#include "swarmkit/client/client.h"

int main(int argc, char** argv) {
    static constexpr std::string_view kDefaultAddr = "127.0.0.1:50061";

    std::string agent_addr{kDefaultAddr};
    if (argc >= 2) {
        agent_addr = argv[1];
    }

    std::cout << "SwarmKit test client — connecting to " << agent_addr << "\n";

    swarmkit::client::ClientConfig config;
    config.address = agent_addr;
    config.client_agent_id = "test-ping";

    swarmkit::client::Client client(config);
    const auto ping_result = client.Ping();

    if (!ping_result.ok) {
        std::cerr << "Ping FAILED: " << ping_result.error_message << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Ping OK\n"
              << "  agent_id : " << ping_result.agent_id << "\n"
              << "  version  : " << ping_result.version << "\n"
              << "  time_ms  : " << ping_result.unix_time_ms << "\n";

    return EXIT_SUCCESS;
}
