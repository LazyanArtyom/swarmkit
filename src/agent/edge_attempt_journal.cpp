// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include "edge_attempt_journal.h"

#include <cerrno>
#include <cstdint>
#include <filesystem>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace swarmkit::agent::internal {
namespace {

constexpr char kMagic[] = "VECRJ01\n";
constexpr std::size_t kMagicSize = sizeof(kMagic) - 1;
constexpr std::size_t kMaximumJournalBytes = 64U * 1024U * 1024U;
constexpr std::size_t kMaximumRecordBytes = 4U * 1024U * 1024U;

void Put32(std::string* out, std::uint32_t value) {
    for (int i = 0; i < 4; ++i) out->push_back(static_cast<char>((value >> (8 * i)) & 0xffU));
}

[[nodiscard]] bool Take32(const std::string& bytes, std::size_t* offset,
                          std::uint32_t* value) {
    if (bytes.size() - *offset < 4) return false;
    *value = 0;
    for (int i = 0; i < 4; ++i)
        *value |= static_cast<std::uint32_t>(
                      static_cast<unsigned char>(bytes[*offset + i])) << (8 * i);
    *offset += 4;
    return true;
}

[[nodiscard]] std::uint32_t Checksum(const std::string& bytes) {
    std::uint32_t hash = 2166136261U;
    for (const unsigned char value : bytes) {
        hash ^= value;
        hash *= 16777619U;
    }
    return hash;
}

[[nodiscard]] bool WriteAll(int fd, const char* data, std::size_t size) {
    while (size > 0) {
        const auto written = ::write(fd, data, size);
        if (written < 0 && errno == EINTR) continue;
        if (written <= 0) return false;
        data += written;
        size -= static_cast<std::size_t>(written);
    }
    return true;
}

[[nodiscard]] bool ReadAll(int fd, char* data, std::size_t size) {
    std::size_t offset = 0;
    while (offset < size) {
        const auto read = ::pread(fd, data + offset, size - offset,
                                  static_cast<off_t>(offset));
        if (read < 0 && errno == EINTR) continue;
        if (read <= 0) return false;
        offset += static_cast<std::size_t>(read);
    }
    return true;
}

[[nodiscard]] bool TakeString(const std::string& bytes, std::size_t* offset,
                              std::string* value) {
    std::uint32_t length = 0;
    if (!Take32(bytes, offset, &length) || length > bytes.size() - *offset) return false;
    value->assign(bytes, *offset, length);
    *offset += length;
    return true;
}

[[nodiscard]] std::string Encode(const EdgeAttemptJournal::Entry& entry) {
    std::string bytes;
    const auto operation = entry.operation.SerializeAsString();
    const auto report = entry.report.SerializeAsString();
    bytes.push_back(entry.dispatch_accepted ? '\1' : '\0');
    Put32(&bytes, static_cast<std::uint32_t>(operation.size()));
    bytes += operation;
    Put32(&bytes, static_cast<std::uint32_t>(entry.client_id.size()));
    bytes += entry.client_id;
    Put32(&bytes, static_cast<std::uint32_t>(report.size()));
    bytes += report;
    return bytes;
}

[[nodiscard]] std::optional<EdgeAttemptJournal::Entry> Decode(const std::string& bytes) {
    if (bytes.empty() || (bytes[0] != '\0' && bytes[0] != '\1')) return std::nullopt;
    EdgeAttemptJournal::Entry entry;
    entry.dispatch_accepted = bytes[0] == '\1';
    std::size_t offset = 1;
    std::string operation;
    std::string report;
    if (!TakeString(bytes, &offset, &operation) ||
        !TakeString(bytes, &offset, &entry.client_id) ||
        !TakeString(bytes, &offset, &report) || offset != bytes.size() ||
        !entry.operation.ParseFromString(operation) ||
        !entry.report.ParseFromString(report) ||
        entry.operation.drone_id().empty() || entry.operation.attempt_id().empty() ||
        entry.client_id.empty() ||
        entry.operation.drone_id() != entry.report.drone_id() ||
        entry.operation.attempt_id() != entry.report.attempt_id()) return std::nullopt;
    return entry;
}

}  // namespace

EdgeAttemptJournal::EdgeAttemptJournal(std::string path) {
    state_ = core::Result::Rejected("edge attempt journal unavailable");
    if (path.empty()) return;
    std::error_code error;
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent, error);
    if (error) {
        state_ = core::Result::Rejected("cannot create edge attempt journal directory");
        return;
    }
    fd_ = ::open(path.c_str(), O_RDWR | O_CREAT | O_APPEND, 0600);
    if (fd_ < 0 || ::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
        state_ = core::Result::Rejected("cannot exclusively open edge attempt journal");
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
        return;
    }
    const auto file_size = ::lseek(fd_, 0, SEEK_END);
    if (file_size < 0 || static_cast<std::size_t>(file_size) > kMaximumJournalBytes) {
        state_ = core::Result::Rejected("edge attempt journal size invalid");
        return;
    }
    if (file_size == 0) {
        if (!WriteAll(fd_, kMagic, kMagicSize) || ::fsync(fd_) != 0) {
            state_ = core::Result::Rejected("cannot initialize edge attempt journal");
            return;
        }
        state_ = core::Result::Success();
        return;
    }
    std::string file(static_cast<std::size_t>(file_size), '\0');
    if (!ReadAll(fd_, file.data(), file.size()) ||
        file.substr(0, kMagicSize) != kMagic) {
        state_ = core::Result::Rejected("edge attempt journal header invalid");
        return;
    }
    std::size_t offset = kMagicSize;
    while (offset < file.size()) {
        std::uint32_t length = 0;
        std::uint32_t checksum = 0;
        if (!Take32(file, &offset, &length) || !Take32(file, &offset, &checksum) ||
            length == 0 || length > kMaximumRecordBytes || length > file.size() - offset) {
            state_ = core::Result::Rejected("edge attempt journal record truncated");
            return;
        }
        const auto payload = file.substr(offset, length);
        offset += length;
        const auto entry = Decode(payload);
        if (Checksum(payload) != checksum || !entry) {
            state_ = core::Result::Rejected("edge attempt journal record corrupt");
            return;
        }
        const auto key = std::make_pair(entry->operation.drone_id(),
                                        entry->operation.attempt_id());
        if (const auto it = entries_.find(key); it != entries_.end() &&
            (it->second.operation.SerializeAsString() !=
                 entry->operation.SerializeAsString() ||
             it->second.client_id != entry->client_id)) {
            state_ = core::Result::Rejected("edge attempt journal identity conflict");
            return;
        }
        entries_[key] = *entry;
    }
    for (auto& [_, entry] : entries_) {
        if (entry.report.phase() != swarmkit::v1::EDGE_FULFILLED &&
            entry.report.phase() != swarmkit::v1::EDGE_INTERRUPTED) {
            entry.report.set_phase(swarmkit::v1::EDGE_UNKNOWN);
            entry.report.set_at_b_verified(false);
            entry.report.set_reason("agent restarted before terminal edge outcome; physical state unknown");
            entry.dispatch_accepted = false;
        }
    }
    state_ = core::Result::Success();
}

EdgeAttemptJournal::~EdgeAttemptJournal() {
    if (fd_ >= 0) {
        ::flock(fd_, LOCK_UN);
        ::close(fd_);
    }
}

core::Result EdgeAttemptJournal::Ready() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

core::Result EdgeAttemptJournal::Append(const Entry& entry) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.IsOk()) return state_;
    const auto key = std::make_pair(entry.operation.drone_id(), entry.operation.attempt_id());
    if (const auto it = entries_.find(key); it != entries_.end()) {
        if (it->second.operation.SerializeAsString() != entry.operation.SerializeAsString() ||
            it->second.client_id != entry.client_id)
            return core::Result::Rejected("edge attempt journal identity conflict");
        const auto prior_phase = it->second.report.phase();
        if (prior_phase == swarmkit::v1::EDGE_FULFILLED ||
            prior_phase == swarmkit::v1::EDGE_INTERRUPTED)
            return prior_phase == entry.report.phase()
                       ? core::Result::Success("terminal edge record retained")
                       : core::Result::Rejected("terminal edge record cannot be replaced");
    }
    const auto payload = Encode(entry);
    const auto file_size = ::lseek(fd_, 0, SEEK_END);
    if (payload.size() > kMaximumRecordBytes || file_size < 0 ||
        file_size > static_cast<off_t>(kMaximumJournalBytes - payload.size() - 8)) {
        state_ = core::Result::Rejected("edge attempt journal capacity reached");
        return state_;
    }
    std::string frame;
    Put32(&frame, static_cast<std::uint32_t>(payload.size()));
    Put32(&frame, Checksum(payload));
    frame += payload;
    if (!WriteAll(fd_, frame.data(), frame.size()) || ::fsync(fd_) != 0) {
        state_ = core::Result::Rejected("edge attempt journal write failed");
        return state_;
    }
    entries_[key] = entry;
    return core::Result::Success();
}

std::optional<EdgeAttemptJournal::Entry> EdgeAttemptJournal::Get(
    const std::string& drone_id, const std::string& attempt_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.IsOk()) return std::nullopt;
    const auto it = entries_.find({drone_id, attempt_id});
    if (it == entries_.end()) return std::nullopt;
    return it->second;
}

}  // namespace swarmkit::agent::internal
