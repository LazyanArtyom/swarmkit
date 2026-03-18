#pragma once

#include <fmt/format.h>

#include <cstdint>
#include <string>
#include <utility>

namespace swarmkit::core {

enum class LogSinkType {
    kStdout,
    kRotatingFile,
};

enum class LogLevel {
    kTrace,
    kDebug,
    kInfo,
    kWarn,
    kError,
    kCritical,
    kOff,
};

struct LoggerConfig {
    LogSinkType sink_type{LogSinkType::kStdout};
    LogLevel level{LogLevel::kInfo};

    // Used when sink_type == kRotatingFile
    std::string log_file_path{"swarmkit.log"};
    std::uint64_t max_file_size_bytes{10U * 1024U * 1024U};
    std::uint32_t max_files{3U};

    // Example: "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v"
    std::string pattern{};
};

class Logger final {
   public:
    Logger() = delete;
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    static void Init(const LoggerConfig& config);
    static void Shutdown();

    static void Trace(const std::string& message);
    static void Debug(const std::string& message);
    static void Info(const std::string& message);
    static void Warn(const std::string& message);
    static void Error(const std::string& message);
    static void Critical(const std::string& message);

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

