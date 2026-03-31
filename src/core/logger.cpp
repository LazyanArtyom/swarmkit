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
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace swarmkit::core {
namespace {

constexpr std::string_view kDefaultLoggerName = "swarmkit";
constexpr std::string_view kDefaultPattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v";

void PrintLoggerInitError(std::string_view message);
spdlog::level::level_enum ToSpdLevel(LogLevel level);
LogLevel SanitizeFlushLevel(LogLevel level);

class SpdlogBackend final : public ILogBackend {
   public:
    explicit SpdlogBackend(const LoggerConfig& config)
        : logger_(CreateLogger(config)) {}

    [[nodiscard]] bool IsEnabled(LogLevel level) const override {
        return logger_ && logger_->should_log(ToSpdLevel(level));
    }

    void Log(LogLevel level, std::string_view message) override {
        if (!logger_) {
            return;
        }

        switch (level) {
            case LogLevel::kTrace:
                logger_->trace(message);
                break;
            case LogLevel::kDebug:
                logger_->debug(message);
                break;
            case LogLevel::kInfo:
                logger_->info(message);
                break;
            case LogLevel::kWarn:
                logger_->warn(message);
                break;
            case LogLevel::kError:
                logger_->error(message);
                break;
            case LogLevel::kCritical:
                logger_->critical(message);
                break;
            case LogLevel::kOff:
                break;
        }
    }

    void Flush() override {
        if (logger_) {
            logger_->flush();
        }
    }

   private:
    static std::shared_ptr<spdlog::logger> CreateLogger(const LoggerConfig& config) {
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

        auto logger = std::make_shared<spdlog::logger>(std::string{kDefaultLoggerName},
                                                       sinks.begin(), sinks.end());
        const std::string kPattern =
            config.pattern.empty() ? std::string{kDefaultPattern} : config.pattern;
        logger->set_pattern(kPattern);
        logger->set_level(ToSpdLevel(config.level));
        logger->flush_on(ToSpdLevel(SanitizeFlushLevel(config.flush_level)));
        logger->set_error_handler([](const std::string& message) {
            PrintLoggerInitError(std::string("spdlog internal error: ") + message);
        });
        return logger;
    }

    std::shared_ptr<spdlog::logger> logger_;
};

struct LoggerState {
    std::mutex mutex;
    std::shared_ptr<ILogBackend> backend;
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
    SetBackend(CreateDefaultBackend(config));
}

void Logger::SetBackend(std::shared_ptr<ILogBackend> backend) {
    if (!backend) {
        backend = CreateDefaultBackend(LoggerConfig{});
    }

    LoggerState& state = GetState();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.backend = std::move(backend);
}

std::shared_ptr<ILogBackend> Logger::CreateDefaultBackend(const LoggerConfig& config) {
    LoggerConfig active_config = config;
    if (const core::Result kValidation = ValidateLoggerConfig(config); !kValidation.IsOk()) {
        PrintLoggerInitError(std::string("invalid logger config: ") + kValidation.message +
                             " - falling back to stdout");
        active_config = LoggerConfig{};
    }

    try {
        return std::make_shared<SpdlogBackend>(active_config);
    } catch (const spdlog::spdlog_ex& error) {
        PrintLoggerInitError(std::string("failed to initialize configured logger: ") +
                             error.what() + " - falling back to stdout");
        return std::make_shared<SpdlogBackend>(LoggerConfig{});
    }
}

void Logger::Shutdown() {
    LoggerState& state = GetState();
    std::shared_ptr<ILogBackend> backend;

    {
        std::lock_guard<std::mutex> lock(state.mutex);
        backend = std::move(state.backend);
    }

    if (backend) {
        backend->Flush();
    }
}

void Logger::Flush() {
    EnsureInitialized();

    LoggerState& state = GetState();
    std::shared_ptr<ILogBackend> backend;
    {
        std::lock_guard<std::mutex> lock(state.mutex);
        backend = state.backend;
    }
    if (backend) {
        backend->Flush();
    }
}

bool Logger::IsEnabled(LogLevel level) {
    EnsureInitialized();

    LoggerState& state = GetState();
    std::shared_ptr<ILogBackend> backend;
    {
        std::lock_guard<std::mutex> lock(state.mutex);
        backend = state.backend;
    }
    return backend && backend->IsEnabled(level);
}

void Logger::EnsureInitialized() {
    LoggerState& state = GetState();
    std::lock_guard<std::mutex> lock(state.mutex);

    if (state.backend) {
        return;
    }

    state.backend = CreateDefaultBackend(LoggerConfig{});
}

void Logger::Log(LogLevel level, fmt::string_view format, fmt::format_args arguments) {
    if (level == LogLevel::kOff) {
        return;
    }

    EnsureInitialized();

    LoggerState& state = GetState();
    std::shared_ptr<ILogBackend> backend;

    {
        std::lock_guard<std::mutex> lock(state.mutex);
        backend = state.backend;
    }

    if (!backend) {
        return;
    }

    const std::string kMessage = fmt::vformat(format, arguments);
    backend->Log(level, kMessage);
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
