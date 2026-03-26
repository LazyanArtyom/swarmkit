// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/core/logger.h"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <cstdio>
#include <filesystem>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace swarmkit::core {
namespace {

constexpr std::string_view kDefaultLoggerName = "swarmkit";
constexpr std::string_view kDefaultPattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v";

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

void PrintLoggerInitError(std::string_view message) {
    std::fprintf(stderr, "SwarmKit logger: %.*s\n", static_cast<int>(message.size()),
                 message.data());
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

LogLevel SanitizeFlushLevel(LogLevel level) {
    return level == LogLevel::kOff ? LogLevel::kCritical : level;
}

std::shared_ptr<spdlog::logger> CreateLoggerUnlocked(const LoggerConfig& config) {
    std::vector<spdlog::sink_ptr> sinks;
    sinks.reserve(2);

    if (config.sink_type == LogSinkType::kStdout ||
        config.sink_type == LogSinkType::kStdoutAndRotatingFile) {
        sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    }

    if (config.sink_type == LogSinkType::kRotatingFile ||
        config.sink_type == LogSinkType::kStdoutAndRotatingFile) {
        const std::filesystem::path kLogPath = config.log_file_path;
        if (kLogPath.has_parent_path()) {
            std::error_code error_code;
            std::filesystem::create_directories(kLogPath.parent_path(), error_code);
            if (error_code) {
                throw spdlog::spdlog_ex("failed to create log directory '" +
                                        kLogPath.parent_path().string() +
                                        "': " + error_code.message());
            }
        }

        sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            config.log_file_path, static_cast<std::size_t>(config.max_file_size_bytes),
            static_cast<std::size_t>(config.max_files)));
    }

    auto created_logger = std::make_shared<spdlog::logger>(std::string{kDefaultLoggerName},
                                                           sinks.begin(), sinks.end());
    const std::string kPattern =
        config.pattern.empty() ? std::string{kDefaultPattern} : config.pattern;
    created_logger->set_pattern(kPattern);
    created_logger->set_level(ToSpdLevel(config.level));
    created_logger->flush_on(ToSpdLevel(SanitizeFlushLevel(config.flush_level)));
    created_logger->set_error_handler([](const std::string& message) {
        PrintLoggerInitError(std::string("spdlog internal error: ") + message);
    });

    spdlog::drop(std::string{kDefaultLoggerName});
    spdlog::set_default_logger(created_logger);
    return created_logger;
}

}  // namespace

std::expected<LogLevel, core::Result> ParseLogLevel(std::string_view value) {
    if (value == "trace") {
        return LogLevel::kTrace;
    }
    if (value == "debug") {
        return LogLevel::kDebug;
    }
    if (value == "info") {
        return LogLevel::kInfo;
    }
    if (value == "warn") {
        return LogLevel::kWarn;
    }
    if (value == "error") {
        return LogLevel::kError;
    }
    if (value == "critical") {
        return LogLevel::kCritical;
    }
    if (value == "off") {
        return LogLevel::kOff;
    }

    return std::unexpected(
        core::Result::Rejected("unsupported log level '" + std::string(value) + "'"));
}

std::expected<LogSinkType, core::Result> ParseLogSinkType(std::string_view value) {
    if (value == "stdout" || value == "console") {
        return LogSinkType::kStdout;
    }
    if (value == "file") {
        return LogSinkType::kRotatingFile;
    }
    if (value == "both") {
        return LogSinkType::kStdoutAndRotatingFile;
    }

    return std::unexpected(
        core::Result::Rejected("unsupported log sink '" + std::string(value) + "'"));
}

std::string_view ToString(LogLevel level) {
    switch (level) {
        case LogLevel::kTrace:
            return "trace";
        case LogLevel::kDebug:
            return "debug";
        case LogLevel::kInfo:
            return "info";
        case LogLevel::kWarn:
            return "warn";
        case LogLevel::kError:
            return "error";
        case LogLevel::kCritical:
            return "critical";
        case LogLevel::kOff:
            return "off";
    }
    return "info";
}

std::string_view ToString(LogSinkType sink_type) {
    switch (sink_type) {
        case LogSinkType::kStdout:
            return "stdout";
        case LogSinkType::kRotatingFile:
            return "file";
        case LogSinkType::kStdoutAndRotatingFile:
            return "both";
    }
    return "stdout";
}

core::Result ValidateLoggerConfig(const LoggerConfig& config) {
    if (config.max_file_size_bytes == 0) {
        return core::Result::Rejected("max_file_size_bytes must be > 0");
    }
    if (config.max_files == 0) {
        return core::Result::Rejected("max_files must be > 0");
    }
    if ((config.sink_type == LogSinkType::kRotatingFile ||
         config.sink_type == LogSinkType::kStdoutAndRotatingFile) &&
        config.log_file_path.empty()) {
        return core::Result::Rejected("log_file_path must not be empty when file logging is used");
    }
    return core::Result::Success();
}

void Logger::Init(const LoggerConfig& config) {
    LoggerState& state = GetState();
    std::lock_guard<std::mutex> lock(state.mutex);

    LoggerConfig active_config = config;
    if (const core::Result kValidation = ValidateLoggerConfig(config); !kValidation.IsOk()) {
        PrintLoggerInitError(std::string("invalid logger config: ") + kValidation.message +
                             " - falling back to stdout");
        active_config = LoggerConfig{};
    }

    try {
        state.config = active_config;
        state.logger = CreateLoggerUnlocked(state.config);
    } catch (const spdlog::spdlog_ex& error) {
        PrintLoggerInitError(std::string("failed to initialize configured logger: ") +
                             error.what() + " - falling back to stdout");
        state.config = LoggerConfig{};
        state.logger = CreateLoggerUnlocked(state.config);
    }
    state.initialized = true;
}

void Logger::Shutdown() {
    LoggerState& state = GetState();
    std::lock_guard<std::mutex> lock(state.mutex);

    if (!state.initialized) {
        return;
    }

    if (state.logger) {
        state.logger->flush();
    }

    spdlog::set_default_logger(nullptr);
    spdlog::drop(std::string{kDefaultLoggerName});
    state.logger.reset();
    state.initialized = false;

    spdlog::shutdown();
}

void Logger::Flush() {
    EnsureInitialized();

    LoggerState& state = GetState();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.logger) {
        state.logger->flush();
    }
}

bool Logger::IsEnabled(LogLevel level) {
    EnsureInitialized();

    LoggerState& state = GetState();
    std::lock_guard<std::mutex> lock(state.mutex);

    if (!state.logger) {
        return false;
    }

    return state.logger->should_log(ToSpdLevel(level));
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

    const std::string kMessage = fmt::vformat(format, arguments);

    switch (level) {
        case LogLevel::kTrace:
            logger_copy->trace(kMessage);
            break;
        case LogLevel::kDebug:
            logger_copy->debug(kMessage);
            break;
        case LogLevel::kInfo:
            logger_copy->info(kMessage);
            break;
        case LogLevel::kWarn:
            logger_copy->warn(kMessage);
            break;
        case LogLevel::kError:
            logger_copy->error(kMessage);
            break;
        case LogLevel::kCritical:
            logger_copy->critical(kMessage);
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
