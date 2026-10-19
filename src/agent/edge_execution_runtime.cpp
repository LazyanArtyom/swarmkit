// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include "edge_execution_runtime.h"

#include <algorithm>
#include <chrono>
#include <cmath>
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

[[nodiscard]] bool ValidGeo(const swarmkit::v1::GeoPoint& point) {
    return std::isfinite(point.lat_deg()) && std::isfinite(point.lon_deg()) &&
           std::isfinite(point.alt_m()) && point.lat_deg() >= -85.0 &&
           point.lat_deg() <= 85.0 && point.lon_deg() >= -180.0 && point.lon_deg() <= 180.0;
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
      config_(config), providers_(std::move(providers)) {}

EdgeExecutionRuntime::~EdgeExecutionRuntime() { Shutdown(); }

std::optional<edge::EdgeRequest> EdgeExecutionRuntime::Convert(
    const swarmkit::v1::EdgeOperation& operation) {
    if (operation.route_size() < 2 || operation.route_size() > 128 ||
        operation.obligation() == swarmkit::v1::EDGE_OBLIGATION_UNSPECIFIED ||
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
                                   ? edge::Obligation::kTraversal : edge::Obligation::kSurvey;
    request.corridor_half_width_m = operation.corridor_half_width_m();
    request.arrival_radius_m = operation.arrival_radius_m();
    request.arrival_max_speed_mps = operation.arrival_max_speed_mps();
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
    swarmkit::v1::AgentReport report;
    report.set_drone_id(attempt.handle.drone_id);
    report.set_correlation_id(attempt.handle.correlation_id);
    report.set_type(swarmkit::v1::EDGE_REPORT);
    report.set_severity(status.phase == edge::Phase::kInterrupted ? swarmkit::v1::REPORT_WARNING
                                                                : swarmkit::v1::REPORT_INFO);
    report.set_message(status.reason);
    core::internal::PopulateExecutionHandle(attempt.handle, report.mutable_execution_handle());
    *report.mutable_edge() = ToReport(attempt, status);
    reports_->Publish(std::move(report));
}

core::Result EdgeExecutionRuntime::Start(const swarmkit::v1::EdgeOperation& operation,
                                         const core::ExecutionHandle& handle,
                                         const commands::CommandContext& context,
                                         swarmkit::v1::EdgeReport* status,
                                         core::BackendCommandOutcome* dispatch) {
    auto request = Convert(operation);
    if (!request) return core::Result::Rejected("invalid edge operation or unsupported route frame");
    if (operation.drone_id() != context.drone_id || handle.drone_id != operation.drone_id() ||
        handle.physical_attempt_id != operation.attempt_id())
        return core::Result::Rejected("edge/context/physical attempt identity mismatch");
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
    *dispatch = backend_->Execute(envelope);
    if (!dispatch->result.IsOk()) {
        telemetry_->ReleaseLease(lease);
        return dispatch->result;
    }
    auto attempt = std::make_shared<Attempt>(std::move(*request), operation, handle, context,
                                              std::move(lease));
    attempt->started_agent_ms = providers_.MonotonicTimeNs() / 1'000'000;
    attempt->last_frame_agent_ms = attempt->started_agent_ms;
    *status = ToReport(*attempt, attempt->supervisor.Snapshot());
    {
        std::lock_guard<std::mutex> lock(mutex_);
        attempts_[operation.drone_id()] = attempt;
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
        const auto it = attempts_.find(drone_id);
        if (it == attempts_.end()) return std::nullopt;
        attempt = it->second;
    }
    std::lock_guard<std::mutex> lock(attempt->mutex);
    if (attempt->supervisor.Request().attempt_id != attempt_id) return std::nullopt;
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
        if (capture.attempt_id() != attempt->supervisor.Request().attempt_id ||
            !capture.has_guaranteed_full_strip_interval())
            return core::Result::Rejected("capture attempt identity or interval missing");
        const auto before = attempt->supervisor.Snapshot();
        const edge::CaptureEvidence evidence{
            .attempt_id = capture.attempt_id(), .capture_id = capture.capture_id(),
            .calibration_id = capture.calibration_id(),
            .monotonic_ms = capture.agent_monotonic_ms(),
            .guaranteed_full_strip_interval =
                {capture.guaranteed_full_strip_interval().begin_m(),
                 capture.guaranteed_full_strip_interval().end_m()},
            .capture_succeeded = capture.capture_succeeded(),
            .full_strip_quality_established = capture.full_strip_quality_established()};
        updated = attempt->supervisor.ObserveCapture(evidence);
        *status = ToReport(*attempt, updated);
        if (updated.survey_covered_m <= before.survey_covered_m) {
            if (attempt->supervisor.WasCaptureAccepted(evidence))
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
                   health.ekf_ok == true && health.armed == true;
    if (allowed) {
        allowed = arbiter_->CheckAndGrant(
            attempt->context, std::chrono::milliseconds{config_->default_authority_ttl_ms}).IsOk();
    }
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
    edge::Status updated;
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        updated = attempt->supervisor.RecoveryDispatched(allowed, now_ms);
        if (allowed) attempt->reentry_target = target;
    }
    Publish(*attempt, updated);
}

void EdgeExecutionRuntime::DispatchReturnToB(const std::shared_ptr<Attempt>& attempt) {
    const auto health = backend_->GetHealth();
    bool allowed = health.ready && health.failsafe == false && health.gps_ok == true &&
                   health.ekf_ok == true && health.armed == true &&
                   arbiter_->CheckAndGrant(
                       attempt->context,
                       std::chrono::milliseconds{config_->default_authority_ttl_ms}).IsOk();
    if (allowed) {
        const auto& target = attempt->operation.route(attempt->operation.route_size() - 1);
        commands::CommandEnvelope command;
        command.context = attempt->context;
        command.command = commands::NavCmd{commands::CmdGoto{
            .lat_deg = target.lat_deg(), .lon_deg = target.lon_deg(), .alt_m = target.alt_m(),
            .speed_mps = attempt->operation.commanded_speed_mps()}};
        allowed = backend_->Execute(command).result.IsOk();
    }
    if (!allowed) {
        edge::Status interrupted;
        {
            std::lock_guard<std::mutex> lock(attempt->mutex);
            interrupted = attempt->supervisor.Interrupt(
                "cannot safely continue from recovery re-entry to B");
        }
        Publish(*attempt, interrupted);
    }
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
                                       frame.validity.armed && frame.armed;
        if (frame.HasVelocity()) observation.speed_mps = std::hypot(frame.vx_mps, frame.vy_mps);
        if (frame.accuracy.horizontal_position &&
            frame.accuracy.horizontal_position->descriptor.semantics ==
                core::UncertaintySemantics::kDeterministicHardBound)
            observation.horizontal_error_hard_bound_m =
                frame.accuracy.horizontal_position->value;
        edge::Status updated;
        bool publish = false;
        std::optional<edge::Point> reentry;
        bool return_to_b = false;
        {
            std::lock_guard<std::mutex> lock(attempt->mutex);
            updated = attempt->supervisor.ObservePosition(observation);
            publish = updated.phase != attempt->last_reported_phase ||
                      updated.at_b_verified != attempt->last_reported_arrival;
            if (publish) {
                attempt->last_reported_phase = updated.phase;
                attempt->last_reported_arrival = updated.at_b_verified;
            }
            reentry = updated.pending_reentry;
            if (attempt->reentry_target && observation.valid &&
                std::hypot(observation.position.east_m - attempt->reentry_target->east_m,
                           observation.position.north_m - attempt->reentry_target->north_m) <=
                    attempt->supervisor.Request().arrival_radius_m &&
                observation.speed_mps && *observation.speed_mps <=
                    attempt->supervisor.Request().arrival_max_speed_mps) {
                attempt->reentry_target.reset();
                return_to_b = true;
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
    }
    for (auto& attempt : attempts) attempt->stop.store(true, std::memory_order_relaxed);
    for (auto& attempt : attempts) if (attempt->worker.joinable()) attempt->worker.join();
}

}  // namespace swarmkit::agent::internal
