// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <fmt/format.h>

#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include "swarmkit/core/result.h"

namespace swarmkit::core {

/// Bytes in one mebibyte (1024 * 1024).
constexpr std::uint64_t kBytesPerMebibyte = 1024ULL * 1024U;
/// Default maximum file size for rotating logs (10 MiB).
constexpr std::uint64_t kDefaultMaxFileSizeBytes = 10U * kBytesPerMebibyte;

/// Destination for log output.
enum class LogSinkType : std::uint8_t {
    kStdout,
    kRotatingFile,
    kStdoutAndRotatingFile,
};

/// Severity levels ordered from most to least verbose.
enum class LogLevel : std::uint8_t {
    kTrace,
    kDebug,
    kInfo,
    kWarn,
    kError,
    kCritical,
    kOff,
};

/// Configuration for Logger initialisation.
struct LoggerConfig {
    LogSinkType sink_type{LogSinkType::kStdout};
    LogLevel level{LogLevel::kInfo};
    LogLevel flush_level{LogLevel::kInfo};

    /// File path used when sink_type == kRotatingFile.
    std::string log_file_path{"swarmkit.log"};
    /// Maximum size of a single log file before rotation.
    std::uint64_t max_file_size_bytes{kDefaultMaxFileSizeBytes};
    /// Number of rotated files to keep.
    std::uint32_t max_files{3U};

    /// spdlog pattern string; empty uses the built-in default.
    std::string pattern;
};

[[nodiscard]] std::expected<LogLevel, core::Result> ParseLogLevel(std::string_view value);
[[nodiscard]] std::expected<LogSinkType, core::Result> ParseLogSinkType(std::string_view value);
[[nodiscard]] std::string_view ToString(LogLevel level);
[[nodiscard]] std::string_view ToString(LogSinkType sink_type);
[[nodiscard]] core::Result ValidateLoggerConfig(const LoggerConfig& config);

/// Pluggable logger backend used by the global Logger facade.
class ILogBackend {
   public:
    virtual ~ILogBackend() = default;

    [[nodiscard]] virtual bool IsEnabled(LogLevel level) const = 0;
    virtual void Log(LogLevel level, std::string_view message) = 0;
    virtual void Flush() = 0;
};

/// Global, thread-safe logger facade backed by a configurable backend.
class Logger final {
   public:
    Logger() = delete;
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    /// Initialise (or reconfigure) the global logger using the built-in backend.
    static void Init(const LoggerConfig& config);

    /// Replace the active backend. Passing nullptr restores the built-in default backend.
    static void SetBackend(std::shared_ptr<ILogBackend> backend);

    /// Create the built-in backend using the given configuration.
    [[nodiscard]] static std::shared_ptr<ILogBackend> CreateDefaultBackend(
        const LoggerConfig& config);

    /// Flush and tear down the global logger.
    static void Shutdown();

    /// Flush the active logger sinks.
    static void Flush();

    /// Returns true if a message at the given level would be emitted.
    [[nodiscard]] static bool IsEnabled(LogLevel level);

    // -----------------------------------------------------------------------
    // Plain-string logging
    // -----------------------------------------------------------------------

    static void Trace(std::string_view message);
    static void Debug(std::string_view message);
    static void Info(std::string_view message);
    static void Warn(std::string_view message);
    static void Error(std::string_view message);
    static void Critical(std::string_view message);

    // -----------------------------------------------------------------------
    // fmt-style logging
    // -----------------------------------------------------------------------

    template <typename... Args>
    static void TraceFmt(fmt::format_string<Args...> format, Args&&... args) {
        LogFmt(LogLevel::kTrace, format, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void DebugFmt(fmt::format_string<Args...> format, Args&&... args) {
        LogFmt(LogLevel::kDebug, format, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void InfoFmt(fmt::format_string<Args...> format, Args&&... args) {
        LogFmt(LogLevel::kInfo, format, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void WarnFmt(fmt::format_string<Args...> format, Args&&... args) {
        LogFmt(LogLevel::kWarn, format, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void ErrorFmt(fmt::format_string<Args...> format, Args&&... args) {
        LogFmt(LogLevel::kError, format, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void CriticalFmt(fmt::format_string<Args...> format, Args&&... args) {
        LogFmt(LogLevel::kCritical, format, std::forward<Args>(args)...);
    }

   private:
    static void EnsureInitialized();
    static void Log(LogLevel level, fmt::string_view format, fmt::format_args arguments);

    template <typename... Args>
    static void LogFmt(LogLevel level, fmt::format_string<Args...> format, Args&&... args) {
        Log(level, fmt::string_view(format), fmt::make_format_args(args...));
    }
};

}  // namespace swarmkit::core
