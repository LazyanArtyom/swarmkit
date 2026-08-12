// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/evidence/execution_log.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string_view>

#include "sha256.h"

namespace swarmkit::evidence {
namespace {

constexpr std::array<char, 8> kFileMagic{'S', 'W', 'K', 'E', 'V', '2', '\r', '\n'};
constexpr std::size_t kChecksumBytes = 32;

[[nodiscard]] std::uint64_t DecodeLittleEndian(
    const std::array<char, sizeof(std::uint64_t)>& bytes) {
    std::uint64_t value = 0;
    for (std::size_t index = 0; index < bytes.size(); ++index) {
        value |= static_cast<std::uint64_t>(static_cast<unsigned char>(bytes[index]))
                 << (index * 8U);
    }
    return value;
}

[[nodiscard]] ExecutionLogError Error(ExecutionLogErrorCode code, const std::string& path,
                                      std::uint64_t record_index, std::string message) {
    return {
        .code = code, .path = path, .record_index = record_index, .message = std::move(message)};
}

[[nodiscard]] bool SameMetadata(const swarmkit::v1::ExecutionRunMetadata& left,
                                const swarmkit::v1::ExecutionRunMetadata& right) {
    if (left.run_id() != right.run_id() || left.scenario_id() != right.scenario_id() ||
        left.has_random_seed() != right.has_random_seed() ||
        (left.has_random_seed() && left.random_seed() != right.random_seed()) ||
        left.configuration_sha256() != right.configuration_sha256() ||
        left.software_version() != right.software_version() ||
        left.backend_name() != right.backend_name() ||
        left.backend_protocol() != right.backend_protocol() ||
        left.calibration_profile_id() != right.calibration_profile_id() ||
        left.calibration_version() != right.calibration_version() ||
        left.labels_size() != right.labels_size()) {
        return false;
    }
    return std::ranges::all_of(left.labels(), [&right](const auto& entry) {
        const auto iter = right.labels().find(entry.first);
        return iter != right.labels().end() && iter->second == entry.second;
    });
}

[[nodiscard]] std::expected<std::vector<std::string>, ExecutionLogError> SegmentPaths(
    const std::string& base_path, int maximum_segments) {
    namespace fs = std::filesystem;
    std::error_code error;
    if (!fs::exists(base_path, error)) {
        return std::unexpected(Error(ExecutionLogErrorCode::kNotFound, base_path, 0,
                                     error ? "cannot inspect evidence log: " + error.message()
                                           : "execution evidence log does not exist"));
    }
    if (maximum_segments < 1) {
        return std::unexpected(Error(ExecutionLogErrorCode::kInvalidStructure, base_path, 0,
                                     "maximum_segments must be positive"));
    }

    std::vector<std::string> paths;
    bool saw_numbered_segment = false;
    bool saw_gap = false;
    for (int index = maximum_segments - 1; index >= 1; --index) {
        const std::string candidate = base_path + "." + std::to_string(index);
        error.clear();
        const bool exists = fs::exists(candidate, error);
        if (error) {
            return std::unexpected(Error(ExecutionLogErrorCode::kIo, candidate, 0,
                                         "cannot inspect evidence segment: " + error.message()));
        }
        if (exists) {
            if (saw_gap) {
                return std::unexpected(
                    Error(ExecutionLogErrorCode::kInvalidStructure, candidate, 0,
                          "numbered evidence segments are not a contiguous rotation set"));
            }
            saw_numbered_segment = true;
            paths.push_back(candidate);
        } else if (saw_numbered_segment) {
            saw_gap = true;
        }
    }
    paths.push_back(base_path);
    return paths;
}

[[nodiscard]] std::expected<bool, ExecutionLogError> ReadSegment(
    const std::string& path, const ExecutionLogReadOptions& options, ExecutionLog* log,
    std::uint64_t* record_index, std::uint64_t* previous_event_sequence) {
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) {
        return std::unexpected(Error(ExecutionLogErrorCode::kIo, path, *record_index,
                                     "cannot open execution evidence segment"));
    }

    std::array<char, kFileMagic.size()> magic{};
    input.read(magic.data(), static_cast<std::streamsize>(magic.size()));
    if (input.gcount() != static_cast<std::streamsize>(magic.size()) || magic != kFileMagic) {
        return std::unexpected(Error(ExecutionLogErrorCode::kInvalidMagic, path, *record_index,
                                     "invalid or truncated execution evidence header"));
    }

    while (true) {
        std::array<char, sizeof(std::uint64_t)> encoded_length{};
        input.read(encoded_length.data(), static_cast<std::streamsize>(encoded_length.size()));
        const std::streamsize length_bytes = input.gcount();
        if (length_bytes == 0 && input.eof()) {
            break;
        }
        if (length_bytes != static_cast<std::streamsize>(encoded_length.size())) {
            return std::unexpected(Error(ExecutionLogErrorCode::kTruncatedRecord, path,
                                         *record_index + 1,
                                         "truncated execution evidence record length"));
        }
        const std::uint64_t payload_size = DecodeLittleEndian(encoded_length);
        if (payload_size == 0 || payload_size > options.maximum_record_bytes ||
            payload_size >
                static_cast<std::uint64_t>(std::numeric_limits<std::streamsize>::max())) {
            return std::unexpected(Error(ExecutionLogErrorCode::kRecordTooLarge, path,
                                         *record_index + 1,
                                         "execution evidence record length is invalid"));
        }

        std::array<std::uint8_t, kChecksumBytes> expected_checksum{};
        input.read(reinterpret_cast<char*>(expected_checksum.data()),
                   static_cast<std::streamsize>(expected_checksum.size()));
        if (input.gcount() != static_cast<std::streamsize>(expected_checksum.size())) {
            return std::unexpected(Error(ExecutionLogErrorCode::kTruncatedRecord, path,
                                         *record_index + 1,
                                         "truncated execution evidence checksum"));
        }

        std::string payload(static_cast<std::size_t>(payload_size), '\0');
        input.read(payload.data(), static_cast<std::streamsize>(payload.size()));
        if (input.gcount() != static_cast<std::streamsize>(payload.size())) {
            return std::unexpected(Error(ExecutionLogErrorCode::kTruncatedRecord, path,
                                         *record_index + 1,
                                         "truncated execution evidence payload"));
        }

        core::internal::Sha256 hasher;
        hasher.Update(payload.data(), payload.size());
        if (hasher.Final() != expected_checksum) {
            return std::unexpected(Error(ExecutionLogErrorCode::kChecksumMismatch, path,
                                         *record_index + 1,
                                         "execution evidence checksum mismatch"));
        }

        swarmkit::v1::ExecutionEventEnvelope envelope;
        if (!envelope.ParseFromString(payload)) {
            return std::unexpected(Error(ExecutionLogErrorCode::kInvalidProtobuf, path,
                                         *record_index + 1,
                                         "execution evidence payload is not a valid envelope"));
        }
        ++(*record_index);
        if (envelope.schema_version() != kExecutionEvidenceSchemaVersion) {
            return std::unexpected(Error(ExecutionLogErrorCode::kUnsupportedSchema, path,
                                         *record_index,
                                         "unsupported execution evidence schema version"));
        }
        if (options.require_contiguous_event_sequences && *previous_event_sequence != 0 &&
            envelope.event_sequence() != *previous_event_sequence + 1) {
            return std::unexpected(Error(ExecutionLogErrorCode::kSequenceDiscontinuity, path,
                                         *record_index,
                                         "execution event sequence is not contiguous"));
        }
        if (envelope.event_sequence() == 0) {
            return std::unexpected(Error(ExecutionLogErrorCode::kSequenceDiscontinuity, path,
                                         *record_index,
                                         "execution event sequence must be non-zero"));
        }
        *previous_event_sequence = envelope.event_sequence();

        if (log->agent_session_id.empty()) {
            log->agent_session_id = envelope.agent_session_id();
            log->metadata = envelope.run_metadata();
        } else if (envelope.agent_session_id() != log->agent_session_id) {
            return std::unexpected(Error(ExecutionLogErrorCode::kSessionMismatch, path,
                                         *record_index,
                                         "one evidence log contains multiple Agent sessions"));
        } else if (!SameMetadata(envelope.run_metadata(), log->metadata)) {
            return std::unexpected(Error(ExecutionLogErrorCode::kMetadataMismatch, path,
                                         *record_index,
                                         "execution run metadata changed within the log"));
        }

        if (envelope.has_session()) {
            if (envelope.session().state() == swarmkit::v1::EXECUTION_SESSION_STARTED) {
                if (!log->events.empty() || log->has_session_start) {
                    return std::unexpected(Error(ExecutionLogErrorCode::kInvalidStructure, path,
                                                 *record_index,
                                                 "session start is not the first event"));
                }
                log->has_session_start = true;
            } else if (envelope.session().state() == swarmkit::v1::EXECUTION_SESSION_COMPLETED) {
                if (log->has_clean_completion) {
                    return std::unexpected(Error(ExecutionLogErrorCode::kInvalidStructure, path,
                                                 *record_index,
                                                 "execution log has multiple completion events"));
                }
                log->has_clean_completion = true;
            }
        }
        if (log->has_clean_completion && !envelope.has_session()) {
            return std::unexpected(Error(ExecutionLogErrorCode::kInvalidStructure, path,
                                         *record_index, "event appears after session completion"));
        }
        log->events.push_back(std::move(envelope));
    }
    return true;
}

}  // namespace

std::expected<ExecutionLog, ExecutionLogError> ReadExecutionLog(
    const std::string& base_path, const ExecutionLogReadOptions& options) {
    if (options.maximum_record_bytes == 0) {
        return std::unexpected(Error(ExecutionLogErrorCode::kInvalidStructure, base_path, 0,
                                     "maximum_record_bytes must be positive"));
    }
    auto paths = SegmentPaths(base_path, options.maximum_segments);
    if (!paths.has_value()) {
        return std::unexpected(paths.error());
    }

    ExecutionLog log;
    std::uint64_t record_index = 0;
    std::uint64_t previous_event_sequence = 0;
    for (const auto& path : *paths) {
        auto read = ReadSegment(path, options, &log, &record_index, &previous_event_sequence);
        if (!read.has_value()) {
            return std::unexpected(read.error());
        }
    }
    if (log.events.empty()) {
        return std::unexpected(Error(ExecutionLogErrorCode::kIncompleteRun, base_path, 0,
                                     "execution evidence log contains no events"));
    }
    if (options.require_complete_run &&
        (!log.has_session_start || !log.has_clean_completion || !log.events.back().has_session() ||
         log.events.back().session().state() != swarmkit::v1::EXECUTION_SESSION_COMPLETED)) {
        return std::unexpected(Error(ExecutionLogErrorCode::kIncompleteRun, base_path, record_index,
                                     "execution evidence log is not a clean complete run"));
    }
    return log;
}

core::Result ReplayExecutionLog(const ExecutionLog& log, const ExecutionEventHandler& handler) {
    if (!handler) {
        return core::Result::Rejected("execution replay handler must not be empty");
    }
    for (const auto& event : log.events) {
        const core::Result result = handler(event);
        if (!result.IsOk()) {
            return result;
        }
    }
    return core::Result::Success();
}

}  // namespace swarmkit::evidence
