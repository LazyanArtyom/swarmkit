#include "swarmkit/core/logger.h"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <mutex>
#include <string>
#include <utility>

namespace swarmkit::core {
namespace {

constexpr const char* kDefaultLoggerName = "swarmkit";
constexpr const char* kDefaultPattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v";

struct LoggerState {
    std::mutex mutex;
    bool initialized{false};
    LoggerConfig config;
    std::shared_ptr<spdlog::logger> logger;
};

LoggerState& GetState() {
    static LoggerState state;
    return state;
}

spdlog::level::level_enum ToSpdLevel(LogLevel level) {
    switch (level) {
        case LogLevel::kTrace:
            return spdlog::level::trace;
        case LogLevel::kDebug:
            return spdlog::level::debug;
        case LogLevel::kInfo:
            return spdlog::level::info;
        case LogLevel::kWarn:
            return spdlog::level::warn;
        case LogLevel::kError:
            return spdlog::level::err;
        case LogLevel::kCritical:
            return spdlog::level::critical;
        case LogLevel::kOff:
            return spdlog::level::off;
    }
    return spdlog::level::info;
}

std::shared_ptr<spdlog::logger> CreateLoggerUnlocked(const LoggerConfig& config) {
    std::shared_ptr<spdlog::logger> created_logger;

    if (config.sink_type == LogSinkType::kRotatingFile) {
        created_logger =
            spdlog::rotating_logger_mt(kDefaultLoggerName, config.log_file_path,
                                       static_cast<std::size_t>(config.max_file_size_bytes),
                                       static_cast<std::size_t>(config.max_files));
    } else {
        created_logger = spdlog::stdout_color_mt(kDefaultLoggerName);
    }

    const std::string pattern = config.pattern.empty() ? kDefaultPattern : config.pattern;
    created_logger->set_pattern(pattern);
    created_logger->set_level(ToSpdLevel(config.level));
    created_logger->flush_on(spdlog::level::info);

    spdlog::set_default_logger(created_logger);
    return created_logger;
}

}  // namespace

void Logger::Init(const LoggerConfig& config) {
    LoggerState& state = GetState();
    std::lock_guard<std::mutex> lock(state.mutex);

    state.config = config;

    if (state.initialized) {
        // Reconfigure existing logger.
        state.logger->set_level(ToSpdLevel(state.config.level));
        const std::string pattern =
            state.config.pattern.empty() ? kDefaultPattern : state.config.pattern;
        state.logger->set_pattern(pattern);
        return;
    }

    state.logger = CreateLoggerUnlocked(state.config);
    state.initialized = true;
}

void Logger::Shutdown() {
    LoggerState& state = GetState();
    std::lock_guard<std::mutex> lock(state.mutex);

    if (!state.initialized) {
        return;
    }

    state.logger.reset();
    state.initialized = false;

    spdlog::shutdown();
}

void Logger::EnsureInitialized() {
    LoggerState& state = GetState();
    std::lock_guard<std::mutex> lock(state.mutex);

    if (state.initialized) {
        return;
    }

    LoggerConfig default_config;
    state.config = default_config;
    state.logger = CreateLoggerUnlocked(state.config);
    state.initialized = true;
}

void Logger::Log(LogLevel level, fmt::string_view format, fmt::format_args arguments) {
    EnsureInitialized();

    LoggerState& state = GetState();
    std::shared_ptr<spdlog::logger> logger_copy;

    {
        std::lock_guard<std::mutex> lock(state.mutex);
        logger_copy = state.logger;
    }

    if (!logger_copy) {
        return;
    }

    const std::string message = fmt::vformat(format, arguments);

    switch (level) {
        case LogLevel::kTrace:
            logger_copy->trace(message);
            break;
        case LogLevel::kDebug:
            logger_copy->debug(message);
            break;
        case LogLevel::kInfo:
            logger_copy->info(message);
            break;
        case LogLevel::kWarn:
            logger_copy->warn(message);
            break;
        case LogLevel::kError:
            logger_copy->error(message);
            break;
        case LogLevel::kCritical:
            logger_copy->critical(message);
            break;
        case LogLevel::kOff:
            break;
    }
}

void Logger::Trace(const std::string& message) {
    Log(LogLevel::kTrace, "{}", fmt::make_format_args(message));
}
void Logger::Debug(const std::string& message) {
    Log(LogLevel::kDebug, "{}", fmt::make_format_args(message));
}
void Logger::Info(const std::string& message) {
    Log(LogLevel::kInfo, "{}", fmt::make_format_args(message));
}
void Logger::Warn(const std::string& message) {
    Log(LogLevel::kWarn, "{}", fmt::make_format_args(message));
}
void Logger::Error(const std::string& message) {
    Log(LogLevel::kError, "{}", fmt::make_format_args(message));
}
void Logger::Critical(const std::string& message) {
    Log(LogLevel::kCritical, "{}", fmt::make_format_args(message));
}

}  // namespace swarmkit::core