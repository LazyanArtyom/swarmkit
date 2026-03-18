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
    config.client_id = "test-ping";

    swarmkit::client::Client client(config);
    const auto kPingResult = client.Ping();

    if (!kPingResult.ok) {
        std::cerr << "Ping FAILED: " << kPingResult.error_message << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Ping OK\n"
              << "  agent_id : " << kPingResult.agent_id << "\n"
              << "  version  : " << kPingResult.version << "\n"
              << "  time_ms  : " << kPingResult.unix_time_ms << "\n";

    return EXIT_SUCCESS;
}
