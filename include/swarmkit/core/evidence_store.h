// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "swarmkit/core/evidence_record.h"
#include "swarmkit/core/telemetry.h"

namespace swarmkit::core {

/// Configuration for an evidence store instance.
struct EvidenceStoreConfig {
    /// Maximum evidence records retained per (agent, field) pair.
    std::size_t max_records_per_field{256};
};

/// Per-field ring buffer of evidence records for one agent.
///
/// Provides the "latest causal evidence" query needed by the acceptance engine:
/// find the most recent record for a given field whose upper generation-time
/// bound g⁺ does not exceed the evaluation time t*.
class AgentEvidenceBuffer {
   public:
    explicit AgentEvidenceBuffer(std::size_t max_records_per_field);

    /// Insert one evidence record.  Drops the oldest record if the ring is full.
    void Insert(EvidenceRecord record);

    /// Query the most recent record for @p field whose source time can be
    /// considered causal relative to evaluation time.  The caller (acceptance
    /// engine) performs the actual g⁺ ≤ t* check after resolving the clock
    /// interval; this method returns records in reverse-chronological order
    /// for that purpose.
    ///
    /// @param field  Target evidence field.
    /// @param max_count  Maximum number of recent records to return.
    /// @return Records ordered newest-first.
    [[nodiscard]] std::vector<EvidenceRecord> Recent(
        EvidenceFieldId field, std::size_t max_count = 1) const;

    /// Return all stored records for @p field in insertion order.
    [[nodiscard]] std::vector<EvidenceRecord> All(EvidenceFieldId field) const;

    /// Total records stored across all fields.
    [[nodiscard]] std::size_t TotalRecords() const;

    /// Current agent session ID from the most recently inserted record (if any).
    [[nodiscard]] std::optional<std::string> CurrentSessionId() const;

   private:
    struct FieldRing {
        std::vector<EvidenceRecord> records;
        std::size_t write_pos{};
        std::size_t count{};
        std::size_t capacity{};

        explicit FieldRing(std::size_t cap) : records(cap), capacity(cap) {}

        void Push(EvidenceRecord rec);
        [[nodiscard]] std::vector<EvidenceRecord> RecentN(std::size_t n) const;
        [[nodiscard]] std::vector<EvidenceRecord> AllOrdered() const;
    };

    std::size_t max_per_field_;
    std::unordered_map<std::uint8_t, FieldRing> rings_;
    std::optional<std::string> current_session_id_;

    [[nodiscard]] FieldRing& GetOrCreate(EvidenceFieldId field);
    [[nodiscard]] const FieldRing* Find(EvidenceFieldId field) const;
};

/// Multi-agent evidence store.
///
/// Thread-safe.  The acceptance engine queries this to obtain evidence
/// for all required agents when evaluating a State-Quality Contract.
class EvidenceStore {
   public:
    explicit EvidenceStore(EvidenceStoreConfig config = {});

    /// Insert an evidence record for the identified agent.
    void Insert(const std::string& agent_id, EvidenceRecord record);

    /// Insert all evidence records decomposed from a TelemetryFrame.
    void InsertFrame(const TelemetryFrame& frame);

    /// Query recent evidence for a specific agent and field.
    [[nodiscard]] std::vector<EvidenceRecord> Recent(
        const std::string& agent_id, EvidenceFieldId field,
        std::size_t max_count = 1) const;

    /// Return all stored evidence for a specific agent and field.
    [[nodiscard]] std::vector<EvidenceRecord> All(
        const std::string& agent_id, EvidenceFieldId field) const;

    /// Return all known agent IDs.
    [[nodiscard]] std::vector<std::string> AgentIds() const;

    /// Current session ID for an agent (if evidence has been received).
    [[nodiscard]] std::optional<std::string> CurrentSessionId(
        const std::string& agent_id) const;

    /// Total evidence records across all agents.
    [[nodiscard]] std::size_t TotalRecords() const;

   private:
    EvidenceStoreConfig config_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, AgentEvidenceBuffer> agents_;
};

/// Decompose a TelemetryFrame into individual evidence records.
///
/// Each independently-updated measurement group in the frame becomes a
/// separate EvidenceRecord.  Only fields with validity=true in the frame
/// are emitted.  This bridges the existing flat TelemetryFrame representation
/// into the per-field evidence model required by the acceptance engine.
[[nodiscard]] std::vector<EvidenceRecord> DecomposeToEvidence(
    const TelemetryFrame& frame);

}  // namespace swarmkit::core
