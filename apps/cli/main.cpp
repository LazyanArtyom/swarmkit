#include <cstdlib>
#include <string>

#include "swarmkit/client/client.h"
#include "swarmkit/core/logger.h"

int main(int argc, char** argv) {
    swarmkit::core::LoggerConfig cfg;
    cfg.sink_type = swarmkit::core::LogSinkType::kStdout;
    cfg.level = swarmkit::core::LogLevel::kInfo;
    swarmkit::core::Logger::Init(cfg);

    std::string addr = "127.0.0.1:50061";
    if (argc >= 2) {
        addr = argv[1];
    }

    swarmkit::client::ClientConfig config;
    config.address = addr;
    config.client_agent_id = "cli";

    swarmkit::client::Client client(config);
    const auto kRes = client.Ping();

    if (!kRes.ok) {
        swarmkit::core::Logger::ErrorFmt("Ping failed: {}", kRes.error_message);
        return EXIT_FAILURE;
    }

    swarmkit::core::Logger::InfoFmt("Ping OK: agent_id={} version={} time_ms={}", kRes.agent_id,
                                    kRes.version, kRes.unix_time_ms);
    return EXIT_SUCCESS;
}