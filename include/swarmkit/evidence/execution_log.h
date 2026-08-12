// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <functional>
#include <string>
#include <vector>

#include "swarmkit/core/result.h"
#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::evidence {

inline constexpr std::uint32_t kExecutionEvidenceSchemaVersion = 1;
inline constexpr std::size_t kDefaultMaximumRecordBytes =
    std::size_t{16} * std::size_t{1024} * std::size_t{1024};
inline constexpr int kDefaultMaximumSegments = 64;

enum class ExecutionLogErrorCode : std::uint8_t {
    kNotFound,
    kIo,
    kInvalidMagic,
    kTruncatedRecord,
    kRecordTooLarge,
    kChecksumMismatch,
    kInvalidProtobuf,
    kUnsupportedSchema,
    kSequenceDiscontinuity,
    kSessionMismatch,
    kMetadataMismatch,
    kIncompleteRun,
    kInvalidStructure,
};

struct ExecutionLogError {
    ExecutionLogErrorCode code{ExecutionLogErrorCode::kIo};
    std::string path;
    std::uint64_t record_index{};
    std::string message;
};

struct ExecutionLogReadOptions {
    std::size_t maximum_record_bytes{kDefaultMaximumRecordBytes};
    int maximum_segments{kDefaultMaximumSegments};
    bool require_contiguous_event_sequences{true};
    bool require_complete_run{true};
};

struct ExecutionLog {
    std::vector<swarmkit::v1::ExecutionEventEnvelope> events;
    swarmkit::v1::ExecutionRunMetadata metadata;
    std::string agent_session_id;
    bool has_session_start{false};
    bool has_clean_completion{false};
};

/// Read and validate a base evidence file plus its numbered rotation segments.
/// Numbered segments are consumed oldest-first and the base file newest-last.
[[nodiscard]] std::expected<ExecutionLog, ExecutionLogError> ReadExecutionLog(
    const std::string& base_path, const ExecutionLogReadOptions& options = {});

using ExecutionEventHandler =
    std::function<core::Result(const swarmkit::v1::ExecutionEventEnvelope&)>;

/// Replay validated envelopes without changing sequence, session, or timestamps.
[[nodiscard]] core::Result ReplayExecutionLog(const ExecutionLog& log,
                                              const ExecutionEventHandler& handler);

}  // namespace swarmkit::evidence
