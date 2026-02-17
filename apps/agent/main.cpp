#include <cstdlib>
#include <string>

#include "swarmkit/agent/agent_server.h"
#include "swarmkit/core/backend_factory.h"
#include "swarmkit/core/logger.h"

namespace {
constexpr const char* kDefaultBindAddr = "0.0.0.0:50061";
constexpr const char* kDefaultInboxDir = "/tmp/swarmkit_inbox";
constexpr const char* kDefaultAgentId = "agent-1";

std::string GetArgValue(int argc, char** argv, const std::string& key, const std::string& def) {
    for (int arg_index = 1; arg_index + 1 < argc; ++arg_index) {
        if (std::string(argv[arg_index]) == key) {
            return std::string(argv[arg_index + 1]);
        }
    }
    return def;
}

bool HasFlag(int argc, char** argv, const std::string& flag) {
    for (int arg_index = 1; arg_index < argc; ++arg_index) {
        if (std::string(argv[arg_index]) == flag) {
            return true;
        }
    }
    return false;
}

void PrintUsage() {
    swarmkit::core::Logger::Info(
        "swarmkit_agent [--id AGENT_ID] [--bind HOST:PORT] [--inbox DIR] "
        "[--log-file PATH] [--log-level info|debug|warn|error]\n");
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

}  // namespace

int main(int argc, char** argv) {
    swarmkit::core::LoggerConfig logger_config;
    logger_config.sink_type = swarmkit::core::LogSinkType::kStdout;
    logger_config.level = swarmkit::core::LogLevel::kInfo;

    const std::string log_file = GetArgValue(argc, argv, "--log-file", "");
    if (!log_file.empty()) {
        logger_config.sink_type = swarmkit::core::LogSinkType::kRotatingFile;
        logger_config.log_file_path = log_file;
    }

    const std::string log_level_value = GetArgValue(argc, argv, "--log-level", "info");
    logger_config.level = ParseLogLevel(log_level_value);

    swarmkit::core::Logger::Init(logger_config);

    if (HasFlag(argc, argv, "--help")) {
        PrintUsage();
        return 0;
    }

    swarmkit::agent::AgentConfig agent_config;
    agent_config.agent_id = GetArgValue(argc, argv, "--id", kDefaultAgentId);
    agent_config.bind_addr = GetArgValue(argc, argv, "--bind", kDefaultBindAddr);
    agent_config.inbox_dir = GetArgValue(argc, argv, "--inbox", kDefaultInboxDir);

    auto backend = swarmkit::core::MakeMockBackend();
    return swarmkit::agent::RunAgentServer(agent_config, std::move(backend));
}