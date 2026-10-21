// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#pragma once

#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "swarmkit/core/result.h"
#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::agent::internal {

// Append-only, fsynced reservation/status journal. A reservation is durable
// before a backend command is issued, so restart ambiguity fails closed.
class EdgeAttemptJournal {
   public:
    enum class Fault {
        kNone,
        kNoSpace,
        kQuota,
        kReadOnly,
        kPartialWrite,
        kFsync,
    };
    struct Entry {
        swarmkit::v1::EdgeOperation operation;
        std::string client_id;
        swarmkit::v1::EdgeReport report;
        bool dispatch_accepted{};
        bool physical_navigation_unresolved{};
    };

    using FaultInjector = std::function<Fault(const Entry&)>;
    explicit EdgeAttemptJournal(std::string path, FaultInjector fault_injector = {});
    ~EdgeAttemptJournal();
    EdgeAttemptJournal(const EdgeAttemptJournal&) = delete;
    EdgeAttemptJournal& operator=(const EdgeAttemptJournal&) = delete;

    [[nodiscard]] core::Result Ready() const;
    [[nodiscard]] core::Result Append(const Entry& entry);
    [[nodiscard]] std::optional<Entry> Get(const std::string& drone_id,
                                           const std::string& attempt_id) const;
    [[nodiscard]] std::optional<Entry> Unresolved(const std::string& drone_id) const;

   private:
    mutable std::mutex mutex_;
    int fd_{-1};
    core::Result state_;
    FaultInjector fault_injector_;
    std::map<std::pair<std::string, std::string>, Entry> entries_;
};

}  // namespace swarmkit::agent::internal
