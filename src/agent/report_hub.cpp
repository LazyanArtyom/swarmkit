// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "report_hub.h"

#include <google/protobuf/util/json_util.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#endif

#include "swarmkit/core/logger.h"

namespace swarmkit::agent::internal {
namespace {

[[nodiscard]] std::int64_t NowUnixMs() {
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

[[nodiscard]] std::string JsonEscape(std::string_view value) {
    std::string out;
    out.reserve(value.size());
    for (const char character : value) {
        switch (character) {
            case '"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out += character;
                break;
        }
    }
    return out;
}

[[nodiscard]] std::string ReportToJson(const swarmkit::v1::AgentReport& report) {
    google::protobuf::util::JsonPrintOptions options;
    options.add_whitespace = false;
    options.preserve_proto_field_names = true;
    std::string json;
    const auto status = google::protobuf::util::MessageToJsonString(report, &json, options);
    if (status.ok()) {
        return json;
    }
    return R"({"sequence":)" + std::to_string(report.sequence()) + R"(,"unix_time_ms":)" +
           std::to_string(report.unix_time_ms()) + R"(,"drone_id":")" +
           JsonEscape(report.drone_id()) + R"(","message":")" + JsonEscape(report.message()) +
           R"("})";
}

[[nodiscard]] bool ParseReportJson(std::string_view line, swarmkit::v1::AgentReport* report) {
    if (report == nullptr || line.empty()) {
        return false;
    }
    google::protobuf::util::JsonParseOptions options;
    options.ignore_unknown_fields = true;
    return google::protobuf::util::JsonStringToMessage(std::string(line), report, options).ok();
}

[[nodiscard]] std::string DefaultSequenceStateFile(const std::string& report_log_file) {
    if (report_log_file.empty()) {
        return {};
    }
    return report_log_file + ".seq";
}

[[nodiscard]] std::string RotatedLogPath(const std::string& report_log_file, int index) {
    return report_log_file + "." + std::to_string(index);
}

[[nodiscard]] std::uint64_t ParseSequenceStateValue(const std::string& value) {
    try {
        return static_cast<std::uint64_t>(std::stoull(value));
    } catch (const std::exception&) {
        return 0;
    }
}

[[nodiscard]] bool SyncFileToDisk(const std::string& path) {
    if (path.empty()) {
        return true;
    }
#ifdef _WIN32
    static_cast<void>(path);
    return true;
#else
    const int file_descriptor = ::open(path.c_str(), O_RDONLY);
    if (file_descriptor < 0) {
        return false;
    }
    const bool synced = ::fsync(file_descriptor) == 0;
    static_cast<void>(::close(file_descriptor));
    return synced;
#endif
}

void RenameIfExists(const std::string& from, const std::string& target) {
    std::error_code error;
    if (!std::filesystem::exists(from, error)) {
        return;
    }
    std::filesystem::remove(target, error);
    error.clear();
    std::filesystem::rename(from, target, error);
    if (error) {
        core::Logger::WarnFmt("ReportHub: failed to rotate report log '{}' -> '{}': {}", from,
                              target, error.message());
    }
}

}  // namespace

void ReportQueue::Push(swarmkit::v1::AgentReport report) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (shutdown_) {
            return;
        }
        queue_.push(std::move(report));
    }
    cv_.notify_one();
}

bool ReportQueue::Pop(swarmkit::v1::AgentReport* out, std::chrono::milliseconds timeout) {
    if (out == nullptr) {
        return false;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    const bool ready = cv_.wait_for(lock, timeout, [this] { return !queue_.empty() || shutdown_; });
    if (!ready || queue_.empty()) {
        return false;
    }
    *out = std::move(queue_.front());
    queue_.pop();
    return true;
}

void ReportQueue::Shutdown() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
    }
    cv_.notify_all();
}

ReportHub::ReportHub() : ReportHub(ReportHubOptions{}) {}

ReportHub::ReportHub(std::string report_log_file)
    : ReportHub(ReportHubOptions{.report_log_file = std::move(report_log_file)}) {}

ReportHub::ReportHub(ReportHubOptions options) : options_(std::move(options)) {
    if (options_.sequence_state_file.empty()) {
        options_.sequence_state_file = DefaultSequenceStateFile(options_.report_log_file);
    }
    InitializePersistence();
}

ReportWatchToken ReportHub::Watch(std::string drone_id, std::uint64_t after_sequence,
                                  const std::shared_ptr<ReportQueue>& queue) {
    std::vector<swarmkit::v1::AgentReport> replay;
    ReportWatchToken token;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        token.watch_id = ++next_watch_id_;
        watchers_.emplace(token.watch_id, Watcher{
                                              .drone_id = std::move(drone_id),
                                              .queue = queue,
                                          });
        std::uint64_t max_replayed_sequence = after_sequence;
        if (after_sequence > 0 || options_.replay_from_log) {
            replay = LoadReplay(watchers_.at(token.watch_id), after_sequence);
            for (const auto& report : replay) {
                max_replayed_sequence = std::max(max_replayed_sequence, report.sequence());
            }
        }
        for (const auto& report : backlog_) {
            if (report.sequence() > max_replayed_sequence &&
                Matches(watchers_.at(token.watch_id), report)) {
                replay.push_back(report);
            }
        }
    }
    for (auto& report : replay) {
        if (queue) {
            queue->Push(std::move(report));
        }
    }
    return token;
}

void ReportHub::Unwatch(ReportWatchToken token) {
    std::lock_guard<std::mutex> lock(mutex_);
    watchers_.erase(token.watch_id);
}

void ReportHub::Publish(swarmkit::v1::AgentReport report) {
    std::vector<std::shared_ptr<ReportQueue>> queues;
    swarmkit::v1::AgentReport finalized_report;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        report.set_sequence(++next_sequence_);
        report.set_unix_time_ms(NowUnixMs());
        finalized_report = report;
        backlog_.push_back(report);
        while (backlog_.size() > options_.max_in_memory_backlog) {
            backlog_.pop_front();
        }
        for (auto iter = watchers_.begin(); iter != watchers_.end();) {
            if (auto queue = iter->second.queue.lock()) {
                if (Matches(iter->second, report)) {
                    queues.push_back(std::move(queue));
                }
                ++iter;
            } else {
                iter = watchers_.erase(iter);
            }
        }
        WriteReportLogLine(finalized_report);
        PersistSequenceState(finalized_report.sequence());
    }
    for (const auto& queue : queues) {
        queue->Push(finalized_report);
    }
}

std::vector<swarmkit::v1::AgentReport> ReportHub::LoadReplay(const Watcher& watcher,
                                                             std::uint64_t after_sequence) const {
    std::vector<swarmkit::v1::AgentReport> reports;
    if (!options_.replay_from_log || options_.report_log_file.empty()) {
        return reports;
    }

    for (const std::string& path : ReportLogReadPaths()) {
        std::ifstream input(path);
        if (!input.is_open()) {
            continue;
        }
        std::string line;
        while (std::getline(input, line)) {
            swarmkit::v1::AgentReport report;
            if (!ParseReportJson(line, &report)) {
                core::Logger::WarnFmt("ReportHub: skipped malformed report log line in '{}'", path);
                continue;
            }
            if (report.sequence() > after_sequence && Matches(watcher, report)) {
                reports.push_back(std::move(report));
            }
        }
    }
    return reports;
}

void ReportHub::InitializePersistence() {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint64_t sequence_state = LoadSequenceState();
    const std::uint64_t log_sequence = LoadMaxSequenceFromLogs();
    next_sequence_ = std::max(sequence_state, log_sequence);
    OpenReportLog(true);
    if (next_sequence_ > 0) {
        PersistSequenceState(next_sequence_);
    }
}

void ReportHub::OpenReportLog(bool append) {
    if (options_.report_log_file.empty()) {
        return;
    }
    report_log_.open(options_.report_log_file, append ? std::ios::app : std::ios::trunc);
    if (!report_log_.is_open()) {
        core::Logger::WarnFmt("ReportHub: failed to open report_log_file '{}'",
                              options_.report_log_file);
        report_log_bytes_ = 0;
        return;
    }
    std::error_code error;
    report_log_bytes_ = std::filesystem::exists(options_.report_log_file, error)
                            ? std::filesystem::file_size(options_.report_log_file, error)
                            : 0;
    if (error) {
        report_log_bytes_ = 0;
    }
    core::Logger::InfoFmt("ReportHub: JSONL report logging enabled at '{}'",
                          options_.report_log_file);
}

void ReportHub::RotateReportLogIfNeeded(std::size_t pending_bytes) {
    if (!report_log_.is_open() || options_.max_log_file_size_bytes == 0 || report_log_bytes_ == 0 ||
        report_log_bytes_ + pending_bytes <= options_.max_log_file_size_bytes) {
        return;
    }

    report_log_.flush();
    report_log_.close();
    if (options_.max_log_files <= 0) {
        std::error_code error;
        std::filesystem::remove(options_.report_log_file, error);
        OpenReportLog(false);
        return;
    }

    const std::string oldest_path =
        RotatedLogPath(options_.report_log_file, options_.max_log_files);
    std::error_code error;
    std::filesystem::remove(oldest_path, error);
    for (int index = options_.max_log_files - 1; index >= 1; --index) {
        RenameIfExists(RotatedLogPath(options_.report_log_file, index),
                       RotatedLogPath(options_.report_log_file, index + 1));
    }
    RenameIfExists(options_.report_log_file, RotatedLogPath(options_.report_log_file, 1));
    OpenReportLog(false);
}

void ReportHub::WriteReportLogLine(const swarmkit::v1::AgentReport& report) {
    if (!report_log_.is_open()) {
        return;
    }
    const std::string line = ReportToJson(report);
    const std::size_t bytes_to_write = line.size() + 1U;
    RotateReportLogIfNeeded(bytes_to_write);
    if (!report_log_.is_open()) {
        return;
    }
    report_log_ << line << '\n';
    report_log_bytes_ += bytes_to_write;
    if (options_.flush_each_write || options_.fsync_each_write) {
        report_log_.flush();
    }
    if (options_.fsync_each_write && !SyncFileToDisk(options_.report_log_file)) {
        core::Logger::WarnFmt("ReportHub: fsync failed for report log '{}'",
                              options_.report_log_file);
    }
}

void ReportHub::PersistSequenceState(std::uint64_t sequence) {
    if (options_.sequence_state_file.empty()) {
        return;
    }
    const std::string temp_path = options_.sequence_state_file + ".tmp";
    {
        std::ofstream output(temp_path, std::ios::trunc);
        if (!output.is_open()) {
            core::Logger::WarnFmt("ReportHub: failed to open sequence state file '{}'", temp_path);
            return;
        }
        output << sequence << '\n';
        output.flush();
    }
    if (options_.fsync_each_write && !SyncFileToDisk(temp_path)) {
        core::Logger::WarnFmt("ReportHub: fsync failed for sequence state '{}'", temp_path);
    }

    std::error_code error;
    std::filesystem::rename(temp_path, options_.sequence_state_file, error);
    if (error) {
        std::filesystem::remove(options_.sequence_state_file, error);
        error.clear();
        std::filesystem::rename(temp_path, options_.sequence_state_file, error);
    }
    if (error) {
        core::Logger::WarnFmt("ReportHub: failed to persist sequence state '{}': {}",
                              options_.sequence_state_file, error.message());
    }
}

std::uint64_t ReportHub::LoadSequenceState() const {
    if (options_.sequence_state_file.empty()) {
        return 0;
    }
    std::ifstream input(options_.sequence_state_file);
    if (!input.is_open()) {
        return 0;
    }
    std::string value;
    std::getline(input, value);
    return ParseSequenceStateValue(value);
}

std::uint64_t ReportHub::LoadMaxSequenceFromLogs() const {
    std::uint64_t max_sequence = 0;
    if (!options_.replay_from_log || options_.report_log_file.empty()) {
        return max_sequence;
    }

    for (const std::string& path : ReportLogReadPaths()) {
        std::ifstream input(path);
        if (!input.is_open()) {
            continue;
        }
        std::string line;
        while (std::getline(input, line)) {
            swarmkit::v1::AgentReport report;
            if (ParseReportJson(line, &report)) {
                max_sequence = std::max(max_sequence, report.sequence());
            }
        }
    }
    return max_sequence;
}

std::vector<std::string> ReportHub::ReportLogReadPaths() const {
    std::vector<std::string> paths;
    if (options_.report_log_file.empty()) {
        return paths;
    }
    std::error_code error;
    for (int index = options_.max_log_files; index >= 1; --index) {
        const std::string rotated_path = RotatedLogPath(options_.report_log_file, index);
        if (std::filesystem::exists(rotated_path, error)) {
            paths.push_back(rotated_path);
        }
        error.clear();
    }
    if (std::filesystem::exists(options_.report_log_file, error)) {
        paths.push_back(options_.report_log_file);
    }
    return paths;
}

bool ReportHub::Matches(const Watcher& watcher, const swarmkit::v1::AgentReport& report) {
    return watcher.drone_id.empty() || watcher.drone_id == "all" ||
           watcher.drone_id == report.drone_id();
}

}  // namespace swarmkit::agent::internal
