#include <cstdlib>
#include <string>

#include "swarmkit/agent/server.h"
#include "swarmkit/agent/sim_backend.h"
#include "swarmkit/core/logger.h"

namespace {

constexpr const char* kDefaultBindAddr = "0.0.0.0:50061";
constexpr const char* kDefaultInboxDir = "/tmp/swarmkit_inbox";
constexpr const char* kDefaultAgentId = "agent-1";

std::string GetArg(int argc, char** argv, const std::string& key, const std::string& def) {
    for (int idx = 1; idx + 1 < argc; ++idx) {
        if (std::string(argv[idx]) == key) {
            return {argv[idx + 1]};
        }
    }
    return def;
}

bool HasFlag(int argc, char** argv, const std::string& flag) {
    for (int idx = 1; idx < argc; ++idx) {
        if (std::string(argv[idx]) == flag) {
            return true;
        }
    }
    return false;
}

swarmkit::core::LogLevel ParseLogLevel(const std::string& value) {
    if (value == "trace") {
        return swarmkit::core::LogLevel::kTrace;
    }
    if (value == "debug") {
        return swarmkit::core::LogLevel::kDebug;
    }
    if (value == "warn") {
        return swarmkit::core::LogLevel::kWarn;
    }
    if (value == "error") {
        return swarmkit::core::LogLevel::kError;
    }
    if (value == "critical") {
        return swarmkit::core::LogLevel::kCritical;
    }
    if (value == "off") {
        return swarmkit::core::LogLevel::kOff;
    }
    return swarmkit::core::LogLevel::kInfo;
}

void PrintUsage() {
    swarmkit::core::Logger::Info(
        "Usage: swarmkit-agent [OPTIONS]\n"
        "\n"
        "Options:\n"
        "  --id      AGENT_ID    Agent identifier (default: agent-1)\n"
        "  --bind    HOST:PORT   Bind address    (default: 0.0.0.0:50061)\n"
        "  --inbox   DIR         Inbox directory (default: /tmp/swarmkit_inbox)\n"
        "  --log-file  PATH      Log to rotating file instead of stdout\n"
        "  --log-level LEVEL     trace|debug|info|warn|error|critical|off\n"
        "  --help                Print this message\n");
}

}  // namespace

int main(int argc, char** argv) {
    swarmkit::core::LoggerConfig logger_cfg;
    logger_cfg.sink_type = swarmkit::core::LogSinkType::kStdout;
    logger_cfg.level = swarmkit::core::LogLevel::kInfo;

    const std::string kLogFile = GetArg(argc, argv, "--log-file", "");
    if (!kLogFile.empty()) {
        logger_cfg.sink_type = swarmkit::core::LogSinkType::kRotatingFile;
        logger_cfg.log_file_path = kLogFile;
    }

    logger_cfg.level = ParseLogLevel(GetArg(argc, argv, "--log-level", "info"));
    swarmkit::core::Logger::Init(logger_cfg);

    if (HasFlag(argc, argv, "--help")) {
        PrintUsage();
        return EXIT_SUCCESS;
    }

    swarmkit::agent::AgentConfig agent_cfg;
    agent_cfg.agent_id = GetArg(argc, argv, "--id", kDefaultAgentId);
    agent_cfg.bind_addr = GetArg(argc, argv, "--bind", kDefaultBindAddr);
    agent_cfg.inbox_dir = GetArg(argc, argv, "--inbox", kDefaultInboxDir);

    return swarmkit::agent::RunAgentServer(agent_cfg, swarmkit::agent::MakeSimBackend());
}
