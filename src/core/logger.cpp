// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/core/logger.h"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <mutex>
#include <string>
#include <string_view>

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
            spdlog::rotating_logger_mt(std::string{kDefaultLoggerName}, config.log_file_path,
                                       static_cast<std::size_t>(config.max_file_size_bytes),
                                       static_cast<std::size_t>(config.max_files));
    } else {
        created_logger = spdlog::stdout_color_mt(std::string{kDefaultLoggerName});
    }

    const std::string kPattern =
        config.pattern.empty() ? std::string{kDefaultPattern} : config.pattern;
    created_logger->set_pattern(kPattern);
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
        state.logger->set_level(ToSpdLevel(state.config.level));
        const std::string kActivePattern =
            state.config.pattern.empty() ? std::string{kDefaultPattern} : state.config.pattern;
        state.logger->set_pattern(kActivePattern);
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
