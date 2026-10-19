// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include "edge_execution_runtime.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <utility>
#include <vector>

#include "command_preconditions.h"
#include "proto_execution.h"
#include "swarmkit/core/logger.h"

namespace swarmkit::agent::internal {
namespace {

[[nodiscard]] swarmkit::v1::EdgePhase ToPhase(edge::Phase phase) {
    switch (phase) {
        case edge::Phase::kMoving: return swarmkit::v1::EDGE_MOVING;
        case edge::Phase::kRecovering: return swarmkit::v1::EDGE_RECOVERING;
        case edge::Phase::kFulfilled: return swarmkit::v1::EDGE_FULFILLED;
        case edge::Phase::kInterrupted: return swarmkit::v1::EDGE_INTERRUPTED;
        case edge::Phase::kUnknown: return swarmkit::v1::EDGE_UNKNOWN;
    }
    return swarmkit::v1::EDGE_PHASE_UNSPECIFIED;
}

[[nodiscard]] bool Terminal(edge::Phase phase) {
    return phase == edge::Phase::kFulfilled || phase == edge::Phase::kInterrupted;
}

[[nodiscard]] bool SafetyOverrideMode(const std::string& mode) {
    return mode == "SIM_RTL" || mode == "SIM_LAND" || mode == "SIM_TERMINATED" ||
           mode == "RTL" || mode == "LAND" || mode == "QRTL" || mode == "QLAND" ||
           mode == "FAILSAFE";
}

[[nodiscard]] bool ValidGeo(const swarmkit::v1::GeoPoint& point) {
    return std::isfinite(point.lat_deg()) && std::isfinite(point.lon_deg()) &&
           std::isfinite(point.alt_m()) && point.lat_deg() >= -85.0 &&
           point.lat_deg() <= 85.0 && point.lon_deg() >= -180.0 && point.lon_deg() <= 180.0;
}

[[nodiscard]] std::string JournalPath(const AgentConfig& config) {
    if (!config.vecr_attempt_journal_path.empty())
        return config.vecr_attempt_journal_path;
    constexpr char kHex[] = "0123456789abcdef";
    std::string safe_id;
    safe_id.reserve(config.agent_id.size() * 2);
    for (const unsigned char character : config.agent_id) {
        safe_id.push_back(kHex[character >> 4]);
        safe_id.push_back(kHex[character & 0x0fU]);
    }
    return (std::filesystem::path(config.data.artifact_dir) /
            (safe_id + "-edge-attempts.journal")).string();
}

[[nodiscard]] swarmkit::v1::EdgeReport ReservedReport(
    const swarmkit::v1::EdgeOperation& operation, const core::ExecutionHandle& handle) {
    swarmkit::v1::EdgeReport report;
    report.set_mission_id(operation.mission_id());
    report.set_mission_revision(operation.mission_revision());
    report.set_edge_id(operation.edge_id());
    report.set_attempt_id(operation.attempt_id());
    report.set_drone_id(operation.drone_id());
    report.set_phase(swarmkit::v1::EDGE_UNKNOWN);
    report.set_reason("edge command reserved; physical dispatch outcome unknown");
    core::internal::PopulateExecutionHandle(handle, report.mutable_execution_handle());
    return report;
}

}  // namespace

EdgeExecutionRuntime::Attempt::Attempt(edge::EdgeRequest request,
                                       const swarmkit::v1::EdgeOperation& source,
                                       core::ExecutionHandle execution,
                                       commands::CommandContext command, TelemetryLease lease_value)
    : supervisor(std::move(request)), operation(source), handle(std::move(execution)),
      context(std::move(command)), lease(std::move(lease_value)),
      last_sequence(lease.initial_sequence) {}

EdgeExecutionRuntime::EdgeExecutionRuntime(TelemetryManager* telemetry, ReportHub* reports,
                                           IDroneBackend* backend, CommandArbiter* arbiter,
                                           const AgentConfig* config, RuntimeProviders providers)
    : telemetry_(telemetry), reports_(reports), backend_(backend), arbiter_(arbiter),
      config_(config), providers_(std::move(providers)), journal_(JournalPath(*config)) {}

EdgeExecutionRuntime::~EdgeExecutionRuntime() { Shutdown(); }

std::optional<edge::EdgeRequest> EdgeExecutionRuntime::Convert(
    const swarmkit::v1::EdgeOperation& operation) {
    if (operation.route_size() != 2 ||
        operation.obligation() == swarmkit::v1::EDGE_OBLIGATION_UNSPECIFIED ||
        operation.obligation() > swarmkit::v1::EDGE_TRAVERSAL_AND_SURVEY ||
        !ValidGeo(operation.route(0)) || operation.commanded_speed_mps() <= 0.0F ||
        !std::isfinite(operation.commanded_speed_mps())) return std::nullopt;
    edge::EdgeRequest request;
    request.mission_id = operation.mission_id();
    request.mission_revision = operation.mission_revision();
    request.edge_id = operation.edge_id();
    request.attempt_id = operation.attempt_id();
    request.drone_id = operation.drone_id();
    request.obligation = operation.obligation() == swarmkit::v1::EDGE_ARRIVAL
                             ? edge::Obligation::kArrival
                             : operation.obligation() == swarmkit::v1::EDGE_TRAVERSAL
                                   ? edge::Obligation::kTraversal
                                   : operation.obligation() == swarmkit::v1::EDGE_SURVEY
                                         ? edge::Obligation::kSurvey
                                         : edge::Obligation::kTraversalAndSurvey;
    request.corridor_half_width_m = operation.corridor_half_width_m();
    request.arrival_radius_m = operation.arrival_radius_m();
    request.arrival_max_speed_mps = operation.arrival_max_speed_mps();
    request.target_altitude_m = operation.route(operation.route_size() - 1).alt_m();
    request.maximum_missing_m = operation.maximum_missing_m();
    request.maximum_speed_mps = operation.maximum_speed_mps();
    request.recovery_overlap_m = operation.recovery_overlap_m();
    request.max_recovery_attempts = operation.max_recovery_attempts();
    request.sustained_deviation_samples = operation.sustained_deviation_samples();
    request.recovery_cooldown_ms = operation.recovery_cooldown_ms();
    request.timeout_ms = operation.timeout_ms();
    const auto& origin = operation.route(0);
    for (const auto& point : operation.route()) {
        if (!ValidGeo(point) || std::abs(point.alt_m() - origin.alt_m()) > 0.01 ||
            std::abs(point.lon_deg() - origin.lon_deg()) > 0.01 ||
            std::abs(point.lat_deg() - origin.lat_deg()) > 0.01) return std::nullopt;
        request.route.points.push_back(edge::Wgs84ToLocal(
            origin.lat_deg(), origin.lon_deg(), point.lat_deg(), point.lon_deg()));
    }
    return edge::EdgeExecutionSupervisor(request).Valid() ? std::optional{std::move(request)}
                                                           : std::nullopt;
}

swarmkit::v1::EdgeReport EdgeExecutionRuntime::ToReport(const Attempt& attempt,
                                                        const edge::Status& status) {
    swarmkit::v1::EdgeReport report;
    const auto& req = attempt.supervisor.Request();
    report.set_mission_id(req.mission_id);
    report.set_mission_revision(req.mission_revision);
    report.set_edge_id(req.edge_id);
    report.set_attempt_id(req.attempt_id);
    report.set_drone_id(req.drone_id);
    report.set_phase(ToPhase(status.phase));
    report.set_at_b_verified(status.at_b_verified);
    report.set_traversal_verified(status.traversal_verified);
    report.set_survey_verified(status.survey_verified);
    report.set_traversal_quality(
        status.traversal_quality == edge::EvidenceQuality::kBounded
            ? swarmkit::v1::EDGE_EVIDENCE_BOUNDED
            : status.traversal_quality == edge::EvidenceQuality::kEstimated
                  ? swarmkit::v1::EDGE_EVIDENCE_ESTIMATED
                  : swarmkit::v1::EDGE_EVIDENCE_UNKNOWN);
    report.set_survey_covered_m(status.survey_covered_m);
    report.set_survey_prefix_m(status.survey_prefix_m);
    for (const auto& missing : status.survey_missing) {
        auto* interval = report.add_survey_missing();
        interval->set_begin_m(missing.begin_m);
        interval->set_end_m(missing.end_m);
    }
    for (const auto& missing : status.traversal_missing) {
        auto* interval = report.add_traversal_missing();
        interval->set_begin_m(missing.begin_m);
        interval->set_end_m(missing.end_m);
    }
    report.set_recovery_attempts(status.recovery_attempts);
    report.set_reason(status.reason);
    if (status.pending_reentry) {
        const auto& origin = attempt.operation.route(0);
        const edge::GeoPoint geo = edge::LocalToWgs84(origin.lat_deg(), origin.lon_deg(),
                                                      *status.pending_reentry);
        report.mutable_pending_reentry()->set_lat_deg(geo.lat_deg);
        report.mutable_pending_reentry()->set_lon_deg(geo.lon_deg);
        report.mutable_pending_reentry()->set_alt_m(origin.alt_m());
    }
    core::internal::PopulateExecutionHandle(attempt.handle, report.mutable_execution_handle());
    return report;
}

void EdgeExecutionRuntime::Publish(const Attempt& attempt, const edge::Status& status) {
    const auto edge_report = ToReport(attempt, status);
    if (const auto saved = journal_.Append({
            .operation = attempt.operation, .client_id = attempt.handle.client_id,
            .report = edge_report, .dispatch_accepted = true});
        !saved.IsOk())
        core::Logger::ErrorFmt("VECR attempt journal append failed: {}", saved.message);
    swarmkit::v1::AgentReport report;
    report.set_drone_id(attempt.handle.drone_id);
    report.set_correlation_id(attempt.handle.correlation_id);
    report.set_type(swarmkit::v1::EDGE_REPORT);
    const bool safety_critical = status.reason.find("failsafe") != std::string::npos ||
                                 status.reason.find("battery") != std::string::npos ||
                                 status.reason.find("safety mode") != std::string::npos;
    report.set_severity(status.phase == edge::Phase::kInterrupted
                            ? (safety_critical ? swarmkit::v1::REPORT_CRITICAL
                                               : swarmkit::v1::REPORT_WARNING)
                            : swarmkit::v1::REPORT_INFO);
    report.set_message(status.reason);
    core::internal::PopulateExecutionHandle(attempt.handle, report.mutable_execution_handle());
    *report.mutable_edge() = edge_report;
    reports_->Publish(std::move(report));
}

core::Result EdgeExecutionRuntime::Start(const swarmkit::v1::EdgeOperation& operation,
                                         const core::ExecutionHandle& handle,
                                         const commands::CommandContext& context,
                                         swarmkit::v1::EdgeReport* status,
                                         core::BackendCommandOutcome* dispatch) {
    std::lock_guard<std::mutex> start_lock(start_mutex_);
    auto request = Convert(operation);
    if (!request) return core::Result::Rejected("invalid edge operation or unsupported route frame");
    request->minimum_recovery_battery_percent =
        config_->vehicle_profile.min_battery_percent;
    if (operation.drone_id() != context.drone_id || handle.drone_id != operation.drone_id() ||
        handle.physical_attempt_id != operation.attempt_id())
        return core::Result::Rejected("edge/context/physical attempt identity mismatch");
    if (const auto ready = journal_.Ready(); !ready.IsOk()) return ready;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto key = std::make_pair(operation.drone_id(), operation.attempt_id());
        if (const auto it = history_.find(key); it != history_.end()) {
            const auto& prior = it->second;
            if (prior->operation.SerializeAsString() != operation.SerializeAsString() ||
                prior->handle.client_id != handle.client_id)
                return core::Result::Rejected("edge attempt ID reused with different request");
            std::lock_guard<std::mutex> attempt_lock(prior->mutex);
            *status = ToReport(*prior, prior->supervisor.Snapshot());
            *dispatch = prior->initial_dispatch;
            return core::Result::Success("existing edge attempt returned without redispatch");
        }
    }
    if (const auto prior = journal_.Get(operation.drone_id(), operation.attempt_id())) {
        if (prior->operation.SerializeAsString() != operation.SerializeAsString() ||
            prior->client_id != handle.client_id)
            return core::Result::Rejected("edge attempt ID reused with different request");
        *status = prior->report;
        if (prior->dispatch_accepted)
            return core::Result::Success("durable edge attempt returned without redispatch");
        if (prior->report.phase() == swarmkit::v1::EDGE_INTERRUPTED)
            return core::Result::Rejected("initial edge command was rejected; no redispatch");
        return core::Result::Rejected(
            "edge attempt previously reserved; physical dispatch outcome requires reconciliation");
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (history_.size() >= 4096)
            return core::Result::Rejected("edge attempt history capacity reached");
    }
    const auto capabilities = backend_->GetCapabilities();
    if (std::find(capabilities.supported_commands.begin(), capabilities.supported_commands.end(),
                  "goto") == capabilities.supported_commands.end())
        return core::Result::Rejected("backend does not declare goto capability");
    const auto health = backend_->GetHealth();
    const core::Result ready = ValidateAutonomousReadiness(
        health, "execute edge", config_->safety.allow_unsafe_bench_commands);
    if (!ready.IsOk()) return ready;
    std::shared_ptr<Attempt> old;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (const auto it = attempts_.find(operation.drone_id()); it != attempts_.end()) {
            old = it->second;
            std::lock_guard<std::mutex> attempt_lock(old->mutex);
            if (!Terminal(old->supervisor.Snapshot().phase))
                return core::Result::Rejected("another edge attempt is active for this drone");
            attempts_.erase(it);
        }
    }
    if (old && old->worker.joinable()) old->worker.join();
    const core::Result authority = arbiter_->CheckAndGrant(
        context, std::chrono::milliseconds{config_->default_authority_ttl_ms});
    if (!authority.IsOk()) return authority;
    TelemetryLease lease;
    const core::Result lease_result = telemetry_->AcquireLease(
        operation.drone_id(), config_->default_telemetry_rate_hz, {}, &lease);
    if (!lease_result.IsOk()) return lease_result;
    commands::CommandEnvelope envelope;
    envelope.context = context;
    const auto& target = operation.route(operation.route_size() - 1);
    envelope.command = commands::NavCmd{commands::CmdGoto{
        .lat_deg = target.lat_deg(), .lon_deg = target.lon_deg(), .alt_m = target.alt_m(),
        .speed_mps = operation.commanded_speed_mps()}};
    const auto reserved = ReservedReport(operation, handle);
    if (const auto saved = journal_.Append({
            .operation = operation, .client_id = handle.client_id,
            .report = reserved, .dispatch_accepted = false});
        !saved.IsOk()) {
        telemetry_->ReleaseLease(lease);
        return saved;
    }
    *dispatch = backend_->Execute(envelope);
    if (!dispatch->result.IsOk()) {
        auto rejected = reserved;
        if (dispatch->dispatch_state == core::BackendDispatchState::kRejected) {
            rejected.set_phase(swarmkit::v1::EDGE_INTERRUPTED);
            rejected.set_reason("initial edge command rejected: " + dispatch->result.message);
        } else {
            rejected.set_reason("initial edge dispatch outcome uncertain: " +
                                dispatch->result.message);
        }
        if (const auto saved = journal_.Append({
                .operation = operation, .client_id = handle.client_id,
                .report = rejected, .dispatch_accepted = false});
            !saved.IsOk())
            core::Logger::ErrorFmt("VECR attempt journal append failed: {}", saved.message);
        *status = std::move(rejected);
        telemetry_->ReleaseLease(lease);
        return dispatch->result;
    }
    auto attempt = std::make_shared<Attempt>(std::move(*request), operation, handle, context,
                                              std::move(lease));
    attempt->initial_dispatch = *dispatch;
    attempt->started_agent_ms = providers_.MonotonicTimeNs() / 1'000'000;
    attempt->last_frame_agent_ms = attempt->started_agent_ms;
    *status = ToReport(*attempt, attempt->supervisor.Snapshot());
    {
        std::lock_guard<std::mutex> lock(mutex_);
        attempts_[operation.drone_id()] = attempt;
        history_.emplace(std::make_pair(operation.drone_id(), operation.attempt_id()), attempt);
    }
    Publish(*attempt, attempt->supervisor.Snapshot());
    attempt->worker = std::thread([this, attempt] { Work(attempt); });
    return core::Result::Success("edge command accepted; physical result pending");
}

std::optional<swarmkit::v1::EdgeReport> EdgeExecutionRuntime::Get(
    const std::string& drone_id, const std::string& attempt_id) const {
    std::shared_ptr<Attempt> attempt;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = history_.find(std::make_pair(drone_id, attempt_id));
        if (it != history_.end()) attempt = it->second;
    }
    if (!attempt) {
        const auto entry = journal_.Get(drone_id, attempt_id);
        return entry ? std::optional{entry->report} : std::nullopt;
    }
    std::lock_guard<std::mutex> lock(attempt->mutex);
    return ToReport(*attempt, attempt->supervisor.Snapshot());
}

bool EdgeExecutionRuntime::Active(const std::string& drone_id) const {
    std::shared_ptr<Attempt> attempt;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = attempts_.find(drone_id);
        if (it == attempts_.end()) return false;
        attempt = it->second;
    }
    std::lock_guard<std::mutex> lock(attempt->mutex);
    return !Terminal(attempt->supervisor.Snapshot().phase);
}

core::Result EdgeExecutionRuntime::SubmitCapture(
    const swarmkit::v1::SubmitEdgeCaptureRequest& capture, swarmkit::v1::EdgeReport* status) {
    if (capture.agent_monotonic_ms() > providers_.MonotonicTimeNs() / 1'000'000 + 100)
        return core::Result::Rejected("capture timestamp is in the future of agent clock");
    std::shared_ptr<Attempt> attempt;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = attempts_.find(capture.drone_id());
        if (it == attempts_.end()) return core::Result::Rejected("edge attempt not found");
        attempt = it->second;
    }
    edge::Status updated;
    bool should_publish = false;
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        const auto& requested = attempt->supervisor.Request();
        if (capture.mission_id() != requested.mission_id ||
            capture.mission_revision() != requested.mission_revision ||
            capture.edge_id() != requested.edge_id ||
            capture.attempt_id() != requested.attempt_id ||
            capture.calibration_id() != config_->vecr_sensor_calibration_id)
            return core::Result::Rejected("capture binding or calibration invalid");
        const auto before = attempt->supervisor.Snapshot();
        const edge::NadirCaptureInput input{
            .mission_id = capture.mission_id(),
            .mission_revision = capture.mission_revision(),
            .edge_id = capture.edge_id(),
            .attempt_id = capture.attempt_id(), .capture_id = capture.capture_id(),
            .calibration_id = capture.calibration_id(),
            .monotonic_ms = capture.agent_monotonic_ms(),
            .camera_center = {capture.camera_center_east_m(),
                              capture.camera_center_north_m()},
            .altitude_above_ground_m = capture.altitude_above_ground_m(),
            .altitude_error_hard_bound_m = capture.altitude_error_hard_bound_m(),
            .horizontal_error_hard_bound_m = capture.horizontal_error_hard_bound_m(),
            .along_fov_deg = capture.along_fov_deg(),
            .cross_fov_deg = capture.cross_fov_deg(),
            .capture_succeeded = capture.capture_succeeded(),
            .image_quality_established = capture.image_quality_established(),
            .nadir_route_alignment_established = capture.nadir_route_alignment_established(),
            .flat_ground_established = capture.flat_ground_established()};
        if (!attempt->last_bounded_position ||
            !attempt->last_position_hard_bound_m ||
            input.monotonic_ms < attempt->last_position_ms - 500 ||
            input.monotonic_ms > attempt->last_position_ms + 500)
            return core::Result::Rejected("capture has no nearby bounded vehicle pose");
        const double elapsed_s = std::abs(input.monotonic_ms - attempt->last_position_ms) / 1000.0;
        const double possible_displacement_m =
            *attempt->last_position_hard_bound_m + input.horizontal_error_hard_bound_m +
            requested.maximum_speed_mps * elapsed_s;
        if (!std::isfinite(possible_displacement_m) ||
            std::hypot(input.camera_center.east_m - attempt->last_bounded_position->east_m,
                       input.camera_center.north_m - attempt->last_bounded_position->north_m) >
                possible_displacement_m + 1e-6)
            return core::Result::Rejected(
                "capture pose inconsistent with vehicle telemetry: capture_east=" +
                std::to_string(input.camera_center.east_m) + " vehicle_east=" +
                std::to_string(attempt->last_bounded_position->east_m) + " dt_ms=" +
                std::to_string(input.monotonic_ms - attempt->last_position_ms));
        const auto evidence = edge::BuildNadirStripEvidence(
            requested.route, requested.corridor_half_width_m, input);
        if (!evidence) return core::Result::Rejected("camera footprint cannot prove full strip");
        updated = attempt->supervisor.ObserveCapture(*evidence);
        *status = ToReport(*attempt, updated);
        if (updated.survey_covered_m <= before.survey_covered_m) {
            if (attempt->supervisor.WasCaptureAccepted(*evidence))
                return core::Result::Success("capture already accepted");
            return core::Result::Rejected(
                "capture not accepted: invalid, stale, uncertain or terminal attempt");
        }
        should_publish = updated.phase != before.phase ||
                         updated.survey_covered_m - attempt->last_reported_covered >= 5.0;
        if (should_publish) attempt->last_reported_covered = updated.survey_covered_m;
    }
    if (should_publish) Publish(*attempt, updated);
    return core::Result::Success("capture evidence accepted");
}

core::Result EdgeExecutionRuntime::Cancel(const core::ExecutionHandle& expected,
                                          swarmkit::v1::EdgeReport* status) {
    std::shared_ptr<Attempt> attempt;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = attempts_.find(expected.drone_id);
        if (it == attempts_.end()) return core::Result::Rejected("edge attempt not found");
        attempt = it->second;
    }
    edge::Status cancelled;
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        if (attempt->handle != expected)
            return core::Result::Rejected("edge execution handle mismatch");
        cancelled = attempt->supervisor.Interrupt("edge cancelled by authorized client");
        *status = ToReport(*attempt, cancelled);
        attempt->stop.store(true, std::memory_order_relaxed);
    }
    Publish(*attempt, cancelled);
    return core::Result::Success("edge cancelled");
}

void EdgeExecutionRuntime::Preempt(const std::string& drone_id,
                                    const std::string& affected_client_id) {
    std::shared_ptr<Attempt> attempt;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = attempts_.find(drone_id);
        if (it == attempts_.end()) return;
        attempt = it->second;
    }
    edge::Status status;
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        if (attempt->handle.client_id != affected_client_id ||
            Terminal(attempt->supervisor.Snapshot().phase)) return;
        status = attempt->supervisor.Interrupt("edge authority preempted by higher priority");
        attempt->stop.store(true, std::memory_order_relaxed);
    }
    Publish(*attempt, status);
}

void EdgeExecutionRuntime::DispatchRecovery(const std::shared_ptr<Attempt>& attempt,
                                             edge::Point target, std::int64_t now_ms) {
    const auto capabilities = backend_->GetCapabilities();
    const auto health = backend_->GetHealth();
    bool allowed = config_->vecr_sim_recovery_enabled && capabilities.backend_name == "sim" &&
                   health.ready && health.failsafe == false && health.gps_ok == true &&
                   health.ekf_ok == true && health.armed == true &&
                   (health.mode == "SIM_GOTO" || health.mode == "SIM_HOLD");
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        allowed = allowed && !attempt->stop.load(std::memory_order_relaxed) &&
                  attempt->supervisor.Snapshot().phase == edge::Phase::kRecovering &&
                  attempt->last_battery_percent &&
                  *attempt->last_battery_percent >=
                      attempt->supervisor.Request().minimum_recovery_battery_percent;
    }
    if (allowed) {
        allowed = arbiter_->CheckAndGrant(
            attempt->context, std::chrono::milliseconds{config_->default_authority_ttl_ms}).IsOk();
    }
    edge::Status updated;
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        if (attempt->stop.load(std::memory_order_relaxed) ||
            Terminal(attempt->supervisor.Snapshot().phase)) return;
        allowed = allowed && attempt->supervisor.Snapshot().pending_reentry.has_value() &&
                  attempt->last_battery_percent &&
                  *attempt->last_battery_percent >=
                      attempt->supervisor.Request().minimum_recovery_battery_percent;
        if (allowed) {
            const auto& origin = attempt->operation.route(0);
            const edge::GeoPoint geo = edge::LocalToWgs84(origin.lat_deg(), origin.lon_deg(), target);
            commands::CommandEnvelope command;
            command.context = attempt->context;
            command.command = commands::NavCmd{commands::CmdGoto{
                .lat_deg = geo.lat_deg, .lon_deg = geo.lon_deg, .alt_m = origin.alt_m(),
                .speed_mps = attempt->operation.commanded_speed_mps()}};
            allowed = backend_->Execute(command).result.IsOk();
        }
        updated = attempt->supervisor.RecoveryDispatched(allowed, now_ms);
        if (allowed) {
            attempt->reentry_target = target;
            attempt->reentry_candidate_ms = -1;
        }
    }
    Publish(*attempt, updated);
}

void EdgeExecutionRuntime::DispatchReturnToB(const std::shared_ptr<Attempt>& attempt) {
    const auto health = backend_->GetHealth();
    bool allowed = health.ready && health.failsafe == false && health.gps_ok == true &&
                   health.ekf_ok == true && health.armed == true &&
                   (health.mode == "SIM_GOTO" || health.mode == "SIM_HOLD");
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        allowed = allowed && !attempt->stop.load(std::memory_order_relaxed) &&
                  !Terminal(attempt->supervisor.Snapshot().phase) &&
                  attempt->last_battery_percent &&
                  *attempt->last_battery_percent >=
                      attempt->supervisor.Request().minimum_recovery_battery_percent;
    }
    if (allowed)
        allowed = arbiter_->CheckAndGrant(
            attempt->context,
            std::chrono::milliseconds{config_->default_authority_ttl_ms}).IsOk();
    std::optional<edge::Status> interrupted;
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        if (attempt->stop.load(std::memory_order_relaxed) ||
            Terminal(attempt->supervisor.Snapshot().phase)) return;
        allowed = allowed && attempt->last_battery_percent &&
                  *attempt->last_battery_percent >=
                      attempt->supervisor.Request().minimum_recovery_battery_percent;
        if (allowed) {
            const auto& target = attempt->operation.route(attempt->operation.route_size() - 1);
            commands::CommandEnvelope command;
            command.context = attempt->context;
            command.command = commands::NavCmd{commands::CmdGoto{
                .lat_deg = target.lat_deg(), .lon_deg = target.lon_deg(), .alt_m = target.alt_m(),
                .speed_mps = attempt->operation.commanded_speed_mps()}};
            allowed = backend_->Execute(command).result.IsOk();
        }
        if (!allowed)
            interrupted = attempt->supervisor.Interrupt(
                "cannot safely continue from recovery re-entry to B");
    }
    if (interrupted) Publish(*attempt, *interrupted);
}

void EdgeExecutionRuntime::Work(const std::shared_ptr<Attempt>& attempt) {
    while (!attempt->stop.load(std::memory_order_relaxed)) {
        const auto read = TelemetryManager::WaitForFrame(
            attempt->lease, attempt->last_sequence, std::chrono::milliseconds{200});
        if (read.status == TelemetryReadStatus::kShutdown) break;
        if (read.status != TelemetryReadStatus::kFrame) {
            const std::int64_t now_ms = providers_.MonotonicTimeNs() / 1'000'000;
            edge::Status timeout_status;
            bool publish_timeout = false;
            {
                std::lock_guard<std::mutex> lock(attempt->mutex);
                if (now_ms - attempt->started_agent_ms > attempt->supervisor.Request().timeout_ms) {
                    timeout_status = attempt->supervisor.Interrupt("edge telemetry/attempt timeout");
                    publish_timeout = true;
                } else if (now_ms - attempt->last_frame_agent_ms > 1000 &&
                           attempt->supervisor.Snapshot().phase != edge::Phase::kUnknown) {
                    timeout_status = attempt->supervisor.ObservePosition({
                        .monotonic_ms = now_ms, .valid = false});
                    publish_timeout = true;
                }
            }
            if (publish_timeout) Publish(*attempt, timeout_status);
            if (Terminal(timeout_status.phase)) break;
            continue;
        }
        attempt->last_sequence = read.frame.telemetry_sequence;
        attempt->last_frame_agent_ms = providers_.MonotonicTimeNs() / 1'000'000;
        const auto& frame = read.frame;
        const auto& origin = attempt->operation.route(0);
        edge::PositionObservation observation;
        observation.monotonic_ms = frame.agent_receive_monotonic_time_ns / 1'000'000;
        observation.valid = frame.HasPosition() && frame.position_frame == core::CoordinateFrame::kWgs84 &&
                            frame.validity.estimator &&
                            frame.estimator_state == core::EstimatorState::kHealthy;
        if (observation.valid) {
            observation.position = edge::Wgs84ToLocal(origin.lat_deg(), origin.lon_deg(),
                                                       frame.lat_deg, frame.lon_deg);
        }
        observation.failsafe = frame.validity.failsafe && frame.failsafe;
        observation.safe_to_navigate = observation.valid && !observation.failsafe &&
                                       frame.validity.failsafe &&
                                       frame.validity.armed && frame.armed;
        if (frame.HasBattery()) observation.battery_percent = frame.battery_percent;
        if (frame.HasVelocity() && frame.velocity_frame == core::CoordinateFrame::kLocalNed)
            observation.speed_mps = std::hypot(frame.vx_mps, frame.vy_mps);
        if (frame.HasRelativeAltitude()) observation.altitude_m = frame.rel_alt_m;
        if (frame.accuracy.horizontal_position &&
            frame.accuracy.horizontal_position->descriptor.semantics ==
                core::UncertaintySemantics::kDeterministicHardBound)
            observation.horizontal_error_hard_bound_m =
                frame.accuracy.horizontal_position->value;
        if (frame.accuracy.horizontal_velocity &&
            frame.accuracy.horizontal_velocity->descriptor.semantics ==
                core::UncertaintySemantics::kDeterministicHardBound)
            observation.speed_error_hard_bound_mps =
                frame.accuracy.horizontal_velocity->value;
        if (frame.accuracy.vertical_position &&
            frame.accuracy.vertical_position->descriptor.semantics ==
                core::UncertaintySemantics::kDeterministicHardBound)
            observation.vertical_error_hard_bound_m =
                frame.accuracy.vertical_position->value;
        edge::Status updated;
        bool publish = false;
        std::optional<edge::Point> reentry;
        bool return_to_b = false;
        {
            std::lock_guard<std::mutex> lock(attempt->mutex);
            updated = frame.validity.mode && SafetyOverrideMode(frame.mode)
                          ? attempt->supervisor.Interrupt("autopilot safety mode active")
                          : attempt->supervisor.ObservePosition(observation);
            attempt->last_battery_percent = observation.battery_percent;
            if (observation.valid && observation.horizontal_error_hard_bound_m &&
                std::isfinite(*observation.horizontal_error_hard_bound_m) &&
                *observation.horizontal_error_hard_bound_m >= 0) {
                attempt->last_bounded_position = observation.position;
                attempt->last_position_hard_bound_m =
                    observation.horizontal_error_hard_bound_m;
                attempt->last_position_ms = observation.monotonic_ms;
            } else {
                attempt->last_bounded_position.reset();
                attempt->last_position_hard_bound_m.reset();
            }
            publish = updated.phase != attempt->last_reported_phase ||
                      updated.at_b_verified != attempt->last_reported_arrival;
            if (publish) {
                attempt->last_reported_phase = updated.phase;
                attempt->last_reported_arrival = updated.at_b_verified;
            }
            reentry = updated.pending_reentry;
            if (attempt->reentry_target) {
                const auto& request = attempt->supervisor.Request();
                const bool at_reentry = observation.valid &&
                    observation.horizontal_error_hard_bound_m &&
                    observation.speed_mps && std::isfinite(*observation.speed_mps) &&
                    *observation.speed_mps >= 0 && observation.speed_error_hard_bound_mps &&
                    observation.altitude_m && observation.vertical_error_hard_bound_m &&
                    std::hypot(std::hypot(
                        observation.position.east_m - attempt->reentry_target->east_m,
                        observation.position.north_m - attempt->reentry_target->north_m) +
                        *observation.horizontal_error_hard_bound_m,
                        std::abs(*observation.altitude_m - request.target_altitude_m) +
                        *observation.vertical_error_hard_bound_m) <= request.arrival_radius_m &&
                    *observation.speed_mps + *observation.speed_error_hard_bound_mps <=
                        request.arrival_max_speed_mps;
                if (at_reentry) {
                    if (attempt->reentry_candidate_ms < 0)
                        attempt->reentry_candidate_ms = observation.monotonic_ms;
                    if (observation.monotonic_ms - attempt->reentry_candidate_ms >= 200) {
                        attempt->reentry_target.reset();
                        attempt->reentry_candidate_ms = -1;
                        return_to_b = true;
                    }
                } else {
                    attempt->reentry_candidate_ms = -1;
                }
            }
        }
        if (publish) Publish(*attempt, updated);
        if (reentry) DispatchRecovery(attempt, *reentry, observation.monotonic_ms);
        if (return_to_b) DispatchReturnToB(attempt);
        if (Terminal(updated.phase)) break;
    }
    telemetry_->ReleaseLease(attempt->lease);
}

void EdgeExecutionRuntime::Shutdown() {
    std::vector<std::shared_ptr<Attempt>> attempts;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& [_, attempt] : attempts_) attempts.push_back(attempt);
        attempts_.clear();
        history_.clear();
    }
    for (auto& attempt : attempts) attempt->stop.store(true, std::memory_order_relaxed);
    for (auto& attempt : attempts) if (attempt->worker.joinable()) attempt->worker.join();
}

}  // namespace swarmkit::agent::internal
