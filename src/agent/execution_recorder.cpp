// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "execution_recorder.h"

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>

#include <array>
#include <filesystem>
#include <system_error>
#include <utility>

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#endif

#include "sha256.h"
#include "swarmkit/core/logger.h"

namespace swarmkit::agent::internal {
namespace {

constexpr std::array<char, 8> kFileMagic{'S', 'W', 'K', 'E', 'V', '2', '\r', '\n'};
constexpr std::uint32_t kSchemaVersion = 1;
constexpr std::size_t kRecordHeaderBytes = sizeof(std::uint64_t) + 32;

[[nodiscard]] std::array<char, sizeof(std::uint64_t)> EncodeLittleEndian(std::uint64_t value) {
    std::array<char, sizeof(std::uint64_t)> bytes{};
    for (std::size_t index = 0; index < bytes.size(); ++index) {
        bytes[index] = static_cast<char>((value >> (index * 8U)) & 0xFFU);
    }
    return bytes;
}

[[nodiscard]] std::string SegmentPath(const std::string& base, int index) {
    return index == 0 ? base : base + "." + std::to_string(index);
}

}  // namespace

ExecutionRecorder::ExecutionRecorder(ExecutionRecorderOptions options)
    : options_(std::move(options)), enabled_(!options_.config.file_path.empty()) {
    if (!enabled_) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!OpenSegmentLocked(true)) {
        return;
    }
    swarmkit::v1::ExecutionEventEnvelope envelope;
    auto* session = envelope.mutable_session();
    session->set_state(swarmkit::v1::EXECUTION_SESSION_STARTED);
    *session->mutable_metadata() = options_.metadata;
    session->set_detail("Agent execution evidence session started");
    static_cast<void>(WriteEnvelopeLocked(&envelope));
}

ExecutionRecorder::~ExecutionRecorder() {
    Close();
}

bool ExecutionRecorder::Enabled() const noexcept {
    return enabled_;
}

bool ExecutionRecorder::Valid() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return valid_;
}

std::string ExecutionRecorder::FailureMessage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return failure_message_;
}

void ExecutionRecorder::RecordEnvelope(swarmkit::v1::ExecutionEventEnvelope envelope) {
    if (!enabled_) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    static_cast<void>(WriteEnvelopeLocked(&envelope));
}

void ExecutionRecorder::RecordTelemetry(const swarmkit::v1::TelemetryFrame& telemetry) {
    if (!enabled_) {
        return;
    }
    swarmkit::v1::ExecutionEventEnvelope envelope;
    *envelope.mutable_telemetry() = telemetry;
    RecordEnvelope(std::move(envelope));

    swarmkit::v1::VehicleHealthEvent health;
    health.set_drone_id(telemetry.drone_id());
    *health.mutable_validity() = telemetry.validity();
    health.set_estimator_state(telemetry.estimator_state());
    health.set_estimator_position_ok(telemetry.estimator_position_ok());
    health.set_estimator_velocity_ok(telemetry.estimator_velocity_ok());
    health.set_failsafe(telemetry.failsafe());
    health.set_armed(telemetry.armed());
    health.set_landed(telemetry.landed());

    bool changed = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto previous = last_health_by_drone_.find(telemetry.drone_id());
        changed = previous == last_health_by_drone_.end() ||
                  previous->second.SerializeAsString() != health.SerializeAsString();
        if (changed) {
            last_health_by_drone_[telemetry.drone_id()] = health;
        }
    }
    if (changed) {
        swarmkit::v1::ExecutionEventEnvelope health_envelope;
        *health_envelope.mutable_health() = std::move(health);
        RecordEnvelope(std::move(health_envelope));
    }
}

void ExecutionRecorder::RecordCommandRequested(
    const swarmkit::v1::CommandRequest& request,
    const swarmkit::v1::ExecutionHandle* execution_handle) {
    swarmkit::v1::ExecutionEventEnvelope envelope;
    auto* command = envelope.mutable_command();
    command->set_stage(swarmkit::v1::COMMAND_EXECUTION_REQUESTED);
    *command->mutable_request() = request;
    if (execution_handle != nullptr) {
        *command->mutable_execution_handle() = *execution_handle;
    }
    RecordEnvelope(std::move(envelope));
}

void ExecutionRecorder::RecordCommandCompleted(
    const swarmkit::v1::CommandRequest& request, const swarmkit::v1::BackendCommandOutcome& outcome,
    const swarmkit::v1::ExecutionHandle* execution_handle) {
    swarmkit::v1::ExecutionEventEnvelope envelope;
    auto* command = envelope.mutable_command();
    command->set_stage(swarmkit::v1::COMMAND_EXECUTION_COMPLETED);
    *command->mutable_request() = request;
    *command->mutable_outcome() = outcome;
    if (execution_handle != nullptr) {
        *command->mutable_execution_handle() = *execution_handle;
    }
    RecordEnvelope(std::move(envelope));
}

void ExecutionRecorder::RecordGoal(const swarmkit::v1::ActiveGoal& goal,
                                   const swarmkit::v1::ExecutionHandle& execution_handle,
                                   swarmkit::v1::GoalStatus status, std::string detail) {
    swarmkit::v1::ExecutionEventEnvelope envelope;
    auto* event = envelope.mutable_goal();
    event->set_status(status);
    *event->mutable_goal() = goal;
    *event->mutable_execution_handle() = execution_handle;
    event->set_detail(std::move(detail));
    RecordEnvelope(std::move(envelope));
}

void ExecutionRecorder::RecordReport(const swarmkit::v1::AgentReport& report) {
    swarmkit::v1::ExecutionEventEnvelope envelope;
    *envelope.mutable_report() = report;
    RecordEnvelope(std::move(envelope));
}

void ExecutionRecorder::RecordAuthority(const swarmkit::v1::AuthorityEvent& authority) {
    swarmkit::v1::ExecutionEventEnvelope envelope;
    *envelope.mutable_authority() = authority;
    RecordEnvelope(std::move(envelope));
}

void ExecutionRecorder::Close() {
    if (!enabled_) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
        return;
    }
    if (valid_ && output_.is_open()) {
        swarmkit::v1::ExecutionEventEnvelope envelope;
        auto* session = envelope.mutable_session();
        session->set_state(swarmkit::v1::EXECUTION_SESSION_COMPLETED);
        *session->mutable_metadata() = options_.metadata;
        session->set_detail("Agent execution evidence session closed cleanly");
        static_cast<void>(WriteEnvelopeLocked(&envelope));
    }
    if (output_.is_open()) {
        output_.flush();
        SyncLocked();
        output_.close();
    }
    closed_ = true;
}

bool ExecutionRecorder::WriteEnvelopeLocked(swarmkit::v1::ExecutionEventEnvelope* envelope) {
    if (!valid_ || closed_ || envelope == nullptr || !output_.is_open()) {
        return false;
    }
    envelope->set_schema_version(kSchemaVersion);
    envelope->set_event_sequence(++next_event_sequence_);
    envelope->set_agent_session_id(options_.agent_session_id);
    envelope->set_agent_unix_time_ms(options_.providers.WallTimeMs());
    envelope->set_agent_monotonic_time_ns(options_.providers.MonotonicTimeNs());
    *envelope->mutable_run_metadata() = options_.metadata;

    std::string payload;
    if (!SerializeDeterministically(*envelope, &payload)) {
        InvalidateLocked("deterministic protobuf serialization failed");
        return false;
    }
    const std::size_t record_bytes = kRecordHeaderBytes + payload.size();
    if (!EnsureCapacityLocked(record_bytes)) {
        return false;
    }

    const auto length = EncodeLittleEndian(payload.size());
    core::internal::Sha256 hasher;
    hasher.Update(payload.data(), payload.size());
    const auto checksum = hasher.Final();
    output_.write(length.data(), static_cast<std::streamsize>(length.size()));
    output_.write(reinterpret_cast<const char*>(checksum.data()),
                  static_cast<std::streamsize>(checksum.size()));
    output_.write(payload.data(), static_cast<std::streamsize>(payload.size()));
    if (!output_.good()) {
        InvalidateLocked("execution evidence write failed");
        return false;
    }
    segment_bytes_ += record_bytes;
    if (options_.config.flush_each_record || options_.config.fsync_each_record) {
        output_.flush();
        if (!output_.good()) {
            InvalidateLocked("execution evidence flush failed");
            return false;
        }
    }
    if (options_.config.fsync_each_record) {
        SyncLocked();
        if (!valid_) {
            return false;
        }
    }
    return true;
}

bool ExecutionRecorder::SerializeDeterministically(
    const swarmkit::v1::ExecutionEventEnvelope& envelope, std::string* output) const {
    if (output == nullptr) {
        return false;
    }
    output->clear();
    google::protobuf::io::StringOutputStream raw(output);
    google::protobuf::io::CodedOutputStream coded(&raw);
    coded.SetSerializationDeterministic(true);
    const bool serialized = envelope.SerializeToCodedStream(&coded);
    coded.Trim();
    return serialized && !coded.HadError();
}

bool ExecutionRecorder::EnsureCapacityLocked(std::size_t record_bytes) {
    const auto limit = static_cast<std::uint64_t>(options_.config.max_segment_bytes);
    if (kFileMagic.size() + record_bytes > limit) {
        InvalidateLocked("execution evidence record exceeds configured segment capacity");
        return false;
    }
    if (segment_bytes_ + record_bytes <= limit) {
        return true;
    }
    if (options_.config.loss_policy == EvidenceLossPolicy::kInvalidateRun) {
        InvalidateLocked("execution evidence retention exhausted; scientific run invalidated");
        return false;
    }
    return RotateLocked();
}

bool ExecutionRecorder::OpenSegmentLocked(bool initial) {
    const std::filesystem::path path(options_.config.file_path);
    std::error_code error;
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path(), error);
        if (error) {
            InvalidateLocked("cannot create evidence directory: " + error.message());
            return false;
        }
    }
    if (initial) {
        for (int index = 0; index < options_.config.max_segments; ++index) {
            const std::filesystem::path segment = SegmentPath(path.string(), index);
            error.clear();
            if (!std::filesystem::exists(segment, error)) {
                if (error) {
                    InvalidateLocked("cannot inspect evidence segment: " + error.message());
                    return false;
                }
                continue;
            }
            if (!options_.config.overwrite_existing) {
                InvalidateLocked("execution evidence segment already exists: " + segment.string());
                return false;
            }
            std::filesystem::remove(segment, error);
            if (error) {
                InvalidateLocked("cannot replace evidence segment: " + error.message());
                return false;
            }
        }
    }
    output_.open(path, std::ios::binary | std::ios::trunc);
    if (!output_.is_open()) {
        InvalidateLocked("cannot open execution evidence file: " + path.string());
        return false;
    }
    output_.write(kFileMagic.data(), static_cast<std::streamsize>(kFileMagic.size()));
    segment_bytes_ = kFileMagic.size();
    if (!output_.good()) {
        InvalidateLocked("cannot write execution evidence file header");
        return false;
    }
    return true;
}

bool ExecutionRecorder::RotateLocked() {
    output_.flush();
    SyncLocked();
    output_.close();
    if (!valid_) {
        return false;
    }
    const int max_segments = options_.config.max_segments;
    std::error_code error;
    std::filesystem::remove(SegmentPath(options_.config.file_path, max_segments - 1), error);
    for (int index = max_segments - 2; index >= 0; --index) {
        const std::string from = SegmentPath(options_.config.file_path, index);
        const std::string to = SegmentPath(options_.config.file_path, index + 1);
        error.clear();
        if (std::filesystem::exists(from, error)) {
            std::filesystem::rename(from, to, error);
            if (error) {
                InvalidateLocked("execution evidence rotation failed: " + error.message());
                return false;
            }
        }
    }
    return OpenSegmentLocked(false);
}

void ExecutionRecorder::InvalidateLocked(std::string message) {
    valid_ = false;
    failure_message_ = std::move(message);
    core::Logger::ErrorFmt("ExecutionRecorder: {}", failure_message_);
}

void ExecutionRecorder::SyncLocked() {
#ifndef _WIN32
    const int descriptor = ::open(options_.config.file_path.c_str(), O_RDONLY);
    if (descriptor >= 0) {
        if (::fsync(descriptor) != 0) {
            InvalidateLocked("fsync failed for execution evidence file");
        }
        static_cast<void>(::close(descriptor));
    } else {
        InvalidateLocked("cannot open execution evidence file for fsync");
    }
#endif
}

}  // namespace swarmkit::agent::internal
