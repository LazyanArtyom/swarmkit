#include <cstdlib>
#include <iostream>
#include <string>

#include "swarmkit/client/client.h"
#include "swarmkit/core/logger.h"

int main(int argc, char** argv) {
    swarmkit::core::LoggerConfig cfg;
    cfg.sink_type = swarmkit::core::LogSinkType::kStdout;
    cfg.level = swarmkit::core::LogLevel::kInfo;
    swarmkit::core::Logger::Init(cfg);

    std::string addr = "127.0.0.1:50061";
    if (argc >= 2) addr = argv[1];

    swarmkit::client::ClientConfig cc;
    cc.address = addr;
    cc.client_agent_id = "cli";

    swarmkit::client::Client client(cc);
    const auto res = client.Ping();

    if (!res.ok) {
        swarmkit::core::Logger::ErrorFmt("Ping failed: {}", res.error_message);
        return EXIT_FAILURE;
    }

    swarmkit::core::Logger::InfoFmt("Ping OK: agent_id={} version={} time_ms={}", res.agent_id,
                                    res.version, res.unix_time_ms);
    return EXIT_SUCCESS;
}