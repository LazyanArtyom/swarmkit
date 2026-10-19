// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#pragma once

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
    struct Entry {
        swarmkit::v1::EdgeOperation operation;
        std::string client_id;
        swarmkit::v1::EdgeReport report;
        bool dispatch_accepted{};
    };

    explicit EdgeAttemptJournal(std::string path);
    ~EdgeAttemptJournal();
    EdgeAttemptJournal(const EdgeAttemptJournal&) = delete;
    EdgeAttemptJournal& operator=(const EdgeAttemptJournal&) = delete;

    [[nodiscard]] core::Result Ready() const;
    [[nodiscard]] core::Result Append(const Entry& entry);
    [[nodiscard]] std::optional<Entry> Get(const std::string& drone_id,
                                           const std::string& attempt_id) const;

   private:
    mutable std::mutex mutex_;
    int fd_{-1};
    core::Result state_;
    std::map<std::pair<std::string, std::string>, Entry> entries_;
};

}  // namespace swarmkit::agent::internal
