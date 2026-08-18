// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/core/evidence_store.h"

#include <algorithm>

namespace swarmkit::core {

// ---------------------------------------------------------------------------
// AgentEvidenceBuffer::FieldRing
// ---------------------------------------------------------------------------

void AgentEvidenceBuffer::FieldRing::Push(EvidenceRecord rec) {
    if (capacity == 0) return;
    records[write_pos] = std::move(rec);
    write_pos = (write_pos + 1) % capacity;
    if (count < capacity) ++count;
}

std::vector<EvidenceRecord> AgentEvidenceBuffer::FieldRing::RecentN(
    std::size_t n) const {
    const std::size_t to_return = std::min(n, count);
    std::vector<EvidenceRecord> result;
    result.reserve(to_return);
    for (std::size_t i = 0; i < to_return; ++i) {
        // Walk backwards from the most recently written position.
        const std::size_t idx =
            (write_pos + capacity - 1 - i) % capacity;
        result.push_back(records[idx]);
    }
    return result;
}

std::vector<EvidenceRecord> AgentEvidenceBuffer::FieldRing::AllOrdered() const {
    std::vector<EvidenceRecord> result;
    result.reserve(count);
    // Oldest first.
    const std::size_t start =
        (count < capacity) ? 0 : write_pos;
    for (std::size_t i = 0; i < count; ++i) {
        result.push_back(records[(start + i) % capacity]);
    }
    return result;
}

// ---------------------------------------------------------------------------
// AgentEvidenceBuffer
// ---------------------------------------------------------------------------

AgentEvidenceBuffer::AgentEvidenceBuffer(std::size_t max_records_per_field)
    : max_per_field_(max_records_per_field) {}

void AgentEvidenceBuffer::Insert(EvidenceRecord record) {
    current_session_id_ = record.identity.agent_session_id;
    GetOrCreate(record.identity.field_id).Push(std::move(record));
}

std::vector<EvidenceRecord> AgentEvidenceBuffer::Recent(
    EvidenceFieldId field, std::size_t max_count) const {
    const auto* ring = Find(field);
    if (!ring) return {};
    return ring->RecentN(max_count);
}

std::vector<EvidenceRecord> AgentEvidenceBuffer::All(
    EvidenceFieldId field) const {
    const auto* ring = Find(field);
    if (!ring) return {};
    return ring->AllOrdered();
}

std::size_t AgentEvidenceBuffer::TotalRecords() const {
    std::size_t total = 0;
    for (const auto& [_, ring] : rings_) {
        total += ring.count;
    }
    return total;
}

std::optional<std::string> AgentEvidenceBuffer::CurrentSessionId() const {
    return current_session_id_;
}

AgentEvidenceBuffer::FieldRing& AgentEvidenceBuffer::GetOrCreate(
    EvidenceFieldId field) {
    const auto key = static_cast<std::uint8_t>(field);
    auto it = rings_.find(key);
    if (it == rings_.end()) {
        it = rings_.emplace(key, FieldRing(max_per_field_)).first;
    }
    return it->second;
}

const AgentEvidenceBuffer::FieldRing* AgentEvidenceBuffer::Find(
    EvidenceFieldId field) const {
    const auto it = rings_.find(static_cast<std::uint8_t>(field));
    if (it == rings_.end()) return nullptr;
    return &it->second;
}

// ---------------------------------------------------------------------------
// EvidenceStore
// ---------------------------------------------------------------------------

EvidenceStore::EvidenceStore(EvidenceStoreConfig config)
    : config_(std::move(config)) {}

void EvidenceStore::Insert(const std::string& agent_id,
                           EvidenceRecord record) {
    std::lock_guard lock(mutex_);
    auto it = agents_.find(agent_id);
    if (it == agents_.end()) {
        it = agents_
                 .emplace(agent_id,
                          AgentEvidenceBuffer(config_.max_records_per_field))
                 .first;
    }
    it->second.Insert(std::move(record));
}

void EvidenceStore::InsertFrame(const TelemetryFrame& frame) {
    auto records = DecomposeToEvidence(frame);
    std::lock_guard lock(mutex_);
    auto it = agents_.find(frame.drone_id);
    if (it == agents_.end()) {
        it = agents_
                 .emplace(frame.drone_id,
                          AgentEvidenceBuffer(config_.max_records_per_field))
                 .first;
    }
    for (auto& rec : records) {
        it->second.Insert(std::move(rec));
    }
}

std::vector<EvidenceRecord> EvidenceStore::Recent(
    const std::string& agent_id, EvidenceFieldId field,
    std::size_t max_count) const {
    std::lock_guard lock(mutex_);
    auto it = agents_.find(agent_id);
    if (it == agents_.end()) return {};
    return it->second.Recent(field, max_count);
}

std::vector<EvidenceRecord> EvidenceStore::All(
    const std::string& agent_id, EvidenceFieldId field) const {
    std::lock_guard lock(mutex_);
    auto it = agents_.find(agent_id);
    if (it == agents_.end()) return {};
    return it->second.All(field);
}

std::vector<std::string> EvidenceStore::AgentIds() const {
    std::lock_guard lock(mutex_);
    std::vector<std::string> ids;
    ids.reserve(agents_.size());
    for (const auto& [id, _] : agents_) {
        ids.push_back(id);
    }
    return ids;
}

std::optional<std::string> EvidenceStore::CurrentSessionId(
    const std::string& agent_id) const {
    std::lock_guard lock(mutex_);
    auto it = agents_.find(agent_id);
    if (it == agents_.end()) return std::nullopt;
    return it->second.CurrentSessionId();
}

std::size_t EvidenceStore::TotalRecords() const {
    std::lock_guard lock(mutex_);
    std::size_t total = 0;
    for (const auto& [_, buf] : agents_) {
        total += buf.TotalRecords();
    }
    return total;
}

// ---------------------------------------------------------------------------
// DecomposeToEvidence
// ---------------------------------------------------------------------------

namespace {

EvidenceIdentity MakeIdentity(const TelemetryFrame& frame,
                              EvidenceFieldId field,
                              CoordinateFrame coord_frame,
                              const std::string& source) {
    return {
        .agent_id = frame.drone_id,
        .agent_session_id = frame.agent_session_id,
        .field_id = field,
        .sequence = frame.telemetry_sequence,
        .source_component = source,
        .coordinate_frame = coord_frame,
        .estimator_id = {},
        .uncertainty_kind = UncertaintySemantics::kUnknown,
        .mission_id = frame.execution_handle
                          ? frame.execution_handle->context
                                ? frame.execution_handle->context->mission_id
                                : std::string{}
                          : std::string{},
        .mission_revision = frame.execution_handle
                                ? frame.execution_handle->context
                                      ? frame.execution_handle->context
                                            ->mission_revision
                                      : std::uint64_t{0}
                                : std::uint64_t{0},
    };
}

EvidenceQuality MakeQuality(const TelemetryFrame& frame,
                            std::optional<UncertaintyEstimate> uncertainty) {
    return {
        .uncertainty = std::move(uncertainty),
        .estimator_healthy =
            frame.estimator_state == EstimatorState::kHealthy,
        .estimator_position_ok = frame.estimator_position_ok,
        .estimator_velocity_ok = frame.estimator_velocity_ok,
    };
}

}  // namespace

std::vector<EvidenceRecord> DecomposeToEvidence(const TelemetryFrame& frame) {
    std::vector<EvidenceRecord> records;
    records.reserve(6);  // Typical: position, velocity, attitude, battery, estimator, GPS.

    const std::int64_t receive_ms = frame.agent_receive_unix_time_ms;

    // Position evidence.
    if (frame.validity.position) {
        records.push_back({
            .value = std::array<double, 3>{frame.lat_deg, frame.lon_deg,
                                           static_cast<double>(frame.rel_alt_m)},
            .source_time = frame.provenance.position.source_time,
            .receive_time_ms = receive_ms,
            .quality = MakeQuality(frame, frame.accuracy.horizontal_position),
            .identity = MakeIdentity(frame, EvidenceFieldId::kPosition,
                                     frame.position_frame,
                                     frame.provenance.position.source),
        });
    }

    // Velocity evidence.
    if (frame.validity.velocity) {
        records.push_back({
            .value = std::array<float, 3>{frame.vx_mps, frame.vy_mps,
                                          frame.vz_mps},
            .source_time = frame.provenance.velocity.source_time,
            .receive_time_ms = receive_ms,
            .quality = MakeQuality(frame, frame.accuracy.horizontal_velocity),
            .identity = MakeIdentity(frame, EvidenceFieldId::kVelocity,
                                     frame.velocity_frame,
                                     frame.provenance.velocity.source),
        });
    }

    // Attitude evidence.
    if (frame.validity.attitude) {
        records.push_back({
            .value = std::array<float, 3>{frame.roll_deg, frame.pitch_deg,
                                          frame.yaw_deg},
            .source_time = frame.provenance.position.source_time,
            .receive_time_ms = receive_ms,
            .quality = MakeQuality(frame, std::nullopt),
            .identity = MakeIdentity(frame, EvidenceFieldId::kAttitude,
                                     CoordinateFrame::kUnknown,
                                     frame.provenance.position.source),
        });
    }

    // Battery evidence.
    if (frame.validity.battery) {
        records.push_back({
            .value = frame.battery_percent,
            .source_time = frame.provenance.vehicle_state.source_time,
            .receive_time_ms = receive_ms,
            .quality = MakeQuality(frame, std::nullopt),
            .identity = MakeIdentity(frame, EvidenceFieldId::kBattery,
                                     CoordinateFrame::kUnknown,
                                     frame.provenance.vehicle_state.source),
        });
    }

    // Estimator health evidence.
    if (frame.validity.estimator) {
        records.push_back({
            .value = frame.estimator_state,
            .source_time = frame.provenance.estimator.source_time,
            .receive_time_ms = receive_ms,
            .quality = MakeQuality(frame, std::nullopt),
            .identity = MakeIdentity(frame, EvidenceFieldId::kEstimatorHealth,
                                     CoordinateFrame::kUnknown,
                                     frame.provenance.estimator.source),
        });
    }

    // GPS quality evidence.
    if (frame.validity.gps) {
        records.push_back({
            .value = frame.gps_quality,
            .source_time = frame.provenance.vehicle_state.source_time,
            .receive_time_ms = receive_ms,
            .quality = MakeQuality(frame, std::nullopt),
            .identity = MakeIdentity(frame, EvidenceFieldId::kGpsQuality,
                                     CoordinateFrame::kUnknown,
                                     frame.provenance.vehicle_state.source),
        });
    }

    // Armed/Landed/Failsafe boolean evidence.
    if (frame.validity.armed) {
        records.push_back({
            .value = frame.armed,
            .source_time = frame.provenance.vehicle_state.source_time,
            .receive_time_ms = receive_ms,
            .quality = MakeQuality(frame, std::nullopt),
            .identity = MakeIdentity(frame, EvidenceFieldId::kArmedState,
                                     CoordinateFrame::kUnknown,
                                     frame.provenance.vehicle_state.source),
        });
    }

    if (frame.validity.landed) {
        records.push_back({
            .value = frame.landed,
            .source_time = frame.provenance.vehicle_state.source_time,
            .receive_time_ms = receive_ms,
            .quality = MakeQuality(frame, std::nullopt),
            .identity = MakeIdentity(frame, EvidenceFieldId::kLandedState,
                                     CoordinateFrame::kUnknown,
                                     frame.provenance.vehicle_state.source),
        });
    }

    if (frame.validity.failsafe) {
        records.push_back({
            .value = frame.failsafe,
            .source_time = frame.provenance.vehicle_state.source_time,
            .receive_time_ms = receive_ms,
            .quality = MakeQuality(frame, std::nullopt),
            .identity = MakeIdentity(frame, EvidenceFieldId::kFailsafeState,
                                     CoordinateFrame::kUnknown,
                                     frame.provenance.vehicle_state.source),
        });
    }

    return records;
}

}  // namespace swarmkit::core
