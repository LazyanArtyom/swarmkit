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
        case edge::Phase::kMoving:
            return swarmkit::v1::EDGE_MOVING;
        case edge::Phase::kDeviating:
            return swarmkit::v1::EDGE_DEVIATING;
        case edge::Phase::kRecovering:
            return swarmkit::v1::EDGE_RECOVERING;
        case edge::Phase::kArrived:
            return swarmkit::v1::EDGE_ARRIVED;
        case edge::Phase::kFailed:
            return swarmkit::v1::EDGE_FAILED;
        case edge::Phase::kCancelled:
            return swarmkit::v1::EDGE_CANCELLED;
        case edge::Phase::kUnknown:
            return swarmkit::v1::EDGE_UNKNOWN;
    }
    return swarmkit::v1::EDGE_PHASE_UNSPECIFIED;
}

[[nodiscard]] bool Terminal(edge::Phase phase) {
    return phase == edge::Phase::kArrived || phase == edge::Phase::kFailed ||
           phase == edge::Phase::kCancelled;
}

[[nodiscard]] bool SafetyOverrideMode(const std::string& mode) {
    return mode == "SIM_RTL" || mode == "SIM_LAND" || mode == "SIM_TERMINATED" || mode == "RTL" ||
           mode == "LAND" || mode == "QRTL" || mode == "QLAND" || mode == "FAILSAFE";
}

// The ardupilot-sitl identity is supplied only by the isolated test adapter,
// never by the production MAVLink backend. Native MAVLink recovery stays disabled.
[[nodiscard]] bool SimulatorRecoveryReady(const AgentConfig& config,
                                          const core::BackendCapabilities& capabilities,
                                          const BackendHealth& health) {
    const bool simulator_mode =
        (capabilities.backend_name == "sim" &&
         (health.mode == "SIM_GOTO" || health.mode == "SIM_HOLD")) ||
        (capabilities.backend_name == "ardupilot-sitl" && health.mode == "GUIDED");
    return config.vecr_sim_recovery_enabled && simulator_mode && health.ready &&
           health.failsafe == false && health.gps_ok == true && health.ekf_ok == true &&
           health.armed == true;
}

[[nodiscard]] bool ValidGeo(const swarmkit::v1::GeoPoint& point) {
    return std::isfinite(point.lat_deg()) && std::isfinite(point.lon_deg()) &&
           std::isfinite(point.alt_m()) && point.lat_deg() >= -85.0 && point.lat_deg() <= 85.0 &&
           point.lon_deg() >= -180.0 && point.lon_deg() <= 180.0;
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
    return (std::filesystem::path(config.data.artifact_dir) / (safe_id + "-edge-attempts.journal"))
        .string();
}

[[nodiscard]] swarmkit::v1::EdgeReport ReservedReport(const swarmkit::v1::EdgeOperation& operation,
                                                      const core::ExecutionHandle& handle) {
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
    : supervisor(std::move(request)),
      operation(source),
      handle(std::move(execution)),
      context(std::move(command)),
      lease(std::move(lease_value)),
      last_sequence(lease.initial_sequence) {}

EdgeExecutionRuntime::EdgeExecutionRuntime(TelemetryManager* telemetry, ReportHub* reports,
                                           IDroneBackend* backend, CommandArbiter* arbiter,
                                           const AgentConfig* config, RuntimeProviders providers)
    : telemetry_(telemetry),
      reports_(reports),
      backend_(backend),
      arbiter_(arbiter),
      config_(config),
      providers_(std::move(providers)),
      journal_(JournalPath(*config), providers_.journal_fault_injector) {}

EdgeExecutionRuntime::~EdgeExecutionRuntime() {
    Shutdown();
}

std::optional<edge::EdgeRequest> EdgeExecutionRuntime::Convert(
    const swarmkit::v1::EdgeOperation& operation) {
    if (operation.route_size() != 2 ||
        operation.obligation() == swarmkit::v1::EDGE_OBLIGATION_UNSPECIFIED ||
        operation.obligation() > swarmkit::v1::EDGE_TRAVERSAL_AND_SURVEY ||
        !ValidGeo(operation.route(0)) || operation.commanded_speed_mps() <= 0.0F ||
        !std::isfinite(operation.commanded_speed_mps()))
        return std::nullopt;
    edge::EdgeRequest request;
    request.mission_id = operation.mission_id();
    request.mission_revision = operation.mission_revision();
    request.edge_id = operation.edge_id();
    request.attempt_id = operation.attempt_id();
    request.drone_id = operation.drone_id();
    request.obligation =
        operation.obligation() == swarmkit::v1::EDGE_ARRIVAL     ? edge::Obligation::kArrival
        : operation.obligation() == swarmkit::v1::EDGE_TRAVERSAL ? edge::Obligation::kTraversal
        : operation.obligation() == swarmkit::v1::EDGE_SURVEY
            ? edge::Obligation::kSurvey
            : edge::Obligation::kTraversalAndSurvey;
    request.maximum_capture_age_ms = operation.maximum_capture_age_ms();
    request.endpoint_evidence_wait_ms = operation.endpoint_evidence_wait_ms();
    request.corridor_half_width_m = operation.corridor_half_width_m();
    request.arrival_radius_m = operation.arrival_radius_m();
    request.arrival_max_speed_mps = operation.arrival_max_speed_mps();
    request.target_altitude_m = operation.route(operation.route_size() - 1).alt_m();
    request.maximum_missing_m = operation.maximum_missing_m();
    request.maximum_speed_mps = operation.maximum_speed_mps();
    request.max_recovery_attempts = operation.max_recovery_attempts();
    request.max_recovery_duration_ms = operation.max_recovery_duration_ms();
    request.max_recovery_travel_m = operation.max_recovery_travel_m();
    request.sustained_deviation_samples = operation.sustained_deviation_samples();
    request.recovery_cooldown_ms = operation.recovery_cooldown_ms();
    request.deviation_hysteresis_m = operation.deviation_hysteresis_m();
    request.timeout_ms = operation.timeout_ms();
    const auto& origin = operation.route(0);
    for (const auto& point : operation.route()) {
        if (!ValidGeo(point) || std::abs(point.alt_m() - origin.alt_m()) > 0.01 ||
            std::abs(point.lon_deg() - origin.lon_deg()) > 0.01 ||
            std::abs(point.lat_deg() - origin.lat_deg()) > 0.01)
            return std::nullopt;
        request.route.points.push_back(edge::Wgs84ToLocal(origin.lat_deg(), origin.lon_deg(),
                                                          point.lat_deg(), point.lon_deg()));
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
    const auto processed_ms = attempt.last_processed_position_ms.load(std::memory_order_acquire);
    if (processed_ms >= 0)
        report.set_last_position_monotonic_ms(processed_ms);
    report.set_at_b_verified(status.at_b_verified);
    report.set_traversal_verified(status.traversal_verified);
    report.set_survey_verified(status.survey_verified);
    report.set_traversal_quality(status.traversal_quality == edge::EvidenceQuality::kBounded
                                     ? swarmkit::v1::EDGE_EVIDENCE_BOUNDED
                                 : status.traversal_quality == edge::EvidenceQuality::kEstimated
                                     ? swarmkit::v1::EDGE_EVIDENCE_ESTIMATED
                                     : swarmkit::v1::EDGE_EVIDENCE_UNKNOWN);
    report.set_survey_covered_m(status.survey_covered_m);
    report.set_survey_prefix_m(status.survey_prefix_m);
    if (status.projected_progress_m)
        report.set_projected_progress_m(*status.projected_progress_m);
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
    report.set_recovery_travel_bound_m(status.recovery_travel_bound_m);
    report.set_reason(status.reason);
    if (status.pending_reentry) {
        const auto& origin = attempt.operation.route(0);
        const edge::GeoPoint geo =
            edge::LocalToWgs84(origin.lat_deg(), origin.lon_deg(), *status.pending_reentry);
        report.mutable_pending_reentry()->set_lat_deg(geo.lat_deg);
        report.mutable_pending_reentry()->set_lon_deg(geo.lon_deg);
        report.mutable_pending_reentry()->set_alt_m(origin.alt_m());
    }
    core::internal::PopulateExecutionHandle(attempt.handle, report.mutable_execution_handle());
    return report;
}

core::Result EdgeExecutionRuntime::Publish(Attempt& attempt, const edge::Status& status,
                                           swarmkit::v1::EdgeReport* published,
                                           std::string_view physical_unknown_reason,
                                           bool physical_hold_confirmed) {
    auto edge_report = ToReport(attempt, status);
    if (!physical_unknown_reason.empty()) {
        edge_report.set_phase(swarmkit::v1::EDGE_UNKNOWN);
        edge_report.set_at_b_verified(false);
        edge_report.set_traversal_verified(false);
        edge_report.set_survey_verified(false);
        edge_report.set_reason(std::string{physical_unknown_reason});
    }
    const bool terminal_report = edge_report.phase() == swarmkit::v1::EDGE_ARRIVED ||
                                 edge_report.phase() == swarmkit::v1::EDGE_FAILED ||
                                 edge_report.phase() == swarmkit::v1::EDGE_CANCELLED;
    if (terminal_report && attempt.terminal_published.exchange(true, std::memory_order_acq_rel)) {
        if (published)
            *published = edge_report;
        return core::Result::Success("terminal edge result already published");
    }
    const auto saved = journal_.Append(
        {.operation = attempt.operation,
         .client_id = attempt.handle.client_id,
         .report = edge_report,
         .dispatch_accepted = physical_unknown_reason.empty(),
         .physical_navigation_unresolved =
             !physical_hold_confirmed && edge_report.phase() != swarmkit::v1::EDGE_ARRIVED});
    if (!saved.IsOk()) {
        core::Logger::ErrorFmt("VECR attempt journal append failed: {}", saved.message);
        edge_report.set_phase(swarmkit::v1::EDGE_UNKNOWN);
        edge_report.set_at_b_verified(false);
        edge_report.set_traversal_verified(false);
        edge_report.set_survey_verified(false);
        edge_report.set_reason(
            "edge outcome not durably committed; physical state requires "
            "reconciliation: " +
            saved.message);
        std::lock_guard lock(attempt.mutex);
        attempt.physical_unknown_report = edge_report;
    }
    if (published)
        *published = edge_report;
    swarmkit::v1::AgentReport report;
    report.set_drone_id(attempt.handle.drone_id);
    report.set_correlation_id(attempt.handle.correlation_id);
    report.set_type(swarmkit::v1::EDGE_REPORT);
    const bool safety_critical = status.reason.find("failsafe") != std::string::npos ||
                                 status.reason.find("battery") != std::string::npos ||
                                 status.reason.find("safety mode") != std::string::npos;
    report.set_severity(
        !saved.IsOk() || !physical_unknown_reason.empty() ? swarmkit::v1::REPORT_CRITICAL
        : status.phase == edge::Phase::kFailed
            ? (safety_critical ? swarmkit::v1::REPORT_CRITICAL : swarmkit::v1::REPORT_WARNING)
            : swarmkit::v1::REPORT_INFO);
    report.set_message(edge_report.reason());
    core::internal::PopulateExecutionHandle(attempt.handle, report.mutable_execution_handle());
    *report.mutable_edge() = edge_report;
    reports_->Publish(std::move(report));
    return saved;
}

core::Result EdgeExecutionRuntime::Start(const swarmkit::v1::EdgeOperation& operation,
                                         const core::ExecutionHandle& handle,
                                         const commands::CommandContext& context,
                                         swarmkit::v1::EdgeReport* status,
                                         core::BackendCommandOutcome* dispatch) {
    std::lock_guard<std::mutex> start_lock(start_mutex_);
    auto request = Convert(operation);
    if (!request)
        return core::Result::Rejected("invalid edge operation or unsupported route frame");
    request->minimum_recovery_battery_percent = config_->vehicle_profile.min_battery_percent;
    if (operation.drone_id() != context.drone_id || handle.drone_id != operation.drone_id() ||
        handle.physical_attempt_id != operation.attempt_id())
        return core::Result::Rejected("edge/context/physical attempt identity mismatch");
    if (const auto ready = journal_.Ready(); !ready.IsOk())
        return ready;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto key = std::make_pair(operation.drone_id(), operation.attempt_id());
        if (const auto it = history_.find(key); it != history_.end()) {
            const auto& prior = it->second;
            if (prior->operation.SerializeAsString() != operation.SerializeAsString() ||
                prior->handle.client_id != handle.client_id)
                return core::Result::Rejected("edge attempt ID reused with different request");
            std::lock_guard<std::mutex> attempt_lock(prior->mutex);
            *status = prior->physical_unknown_report
                          ? *prior->physical_unknown_report
                          : ToReport(*prior, prior->supervisor.Snapshot());
            *dispatch = prior->initial_dispatch;
            if (prior->physical_unknown_report)
                return core::Result::Rejected("prior edge attempt physical state unresolved");
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
        if (prior->report.phase() == swarmkit::v1::EDGE_FAILED)
            return core::Result::Rejected("initial edge command was rejected; no redispatch");
        return core::Result::Rejected(
            "edge attempt previously reserved; physical dispatch outcome requires reconciliation");
    }
    if (const auto unresolved = journal_.Unresolved(operation.drone_id()))
        return core::Result::Rejected("prior edge attempt " + unresolved->operation.attempt_id() +
                                      " has unresolved physical navigation; reconcile it "
                                      "before dispatching another attempt");
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
    if (!ready.IsOk())
        return ready;
    std::shared_ptr<Attempt> old;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (const auto it = attempts_.find(operation.drone_id()); it != attempts_.end()) {
            old = it->second;
            std::lock_guard<std::mutex> attempt_lock(old->mutex);
            if (!Terminal(old->supervisor.Snapshot().phase) &&
                !(old->physical_unknown_report &&
                  old->physical_unknown_report->phase() == swarmkit::v1::EDGE_FAILED))
                return core::Result::Rejected("another edge attempt is active for this drone");
            attempts_.erase(it);
        }
    }
    if (old && old->worker.joinable())
        old->worker.join();
    const core::Result authority = arbiter_->CheckAndGrant(
        context, std::chrono::milliseconds{config_->default_authority_ttl_ms});
    if (!authority.IsOk())
        return authority;
    TelemetryLease lease;
    const core::Result lease_result = telemetry_->AcquireLease(
        operation.drone_id(), config_->default_telemetry_rate_hz, {}, &lease);
    if (!lease_result.IsOk())
        return lease_result;
    commands::CommandEnvelope envelope;
    envelope.context = context;
    const auto& target = operation.route(operation.route_size() - 1);
    envelope.command =
        commands::NavCmd{commands::CmdGoto{.lat_deg = target.lat_deg(),
                                           .lon_deg = target.lon_deg(),
                                           .alt_m = target.alt_m(),
                                           .speed_mps = operation.commanded_speed_mps()}};
    const auto reserved = ReservedReport(operation, handle);
    if (const auto saved = journal_.Append({.operation = operation,
                                            .client_id = handle.client_id,
                                            .report = reserved,
                                            .dispatch_accepted = false,
                                            .physical_navigation_unresolved = true});
        !saved.IsOk()) {
        telemetry_->ReleaseLease(lease);
        return saved;
    }
    *dispatch = backend_->Execute(envelope);
    if (!dispatch->result.IsOk()) {
        auto rejected = reserved;
        if (dispatch->dispatch_state == core::BackendDispatchState::kRejected) {
            rejected.set_phase(swarmkit::v1::EDGE_FAILED);
            rejected.set_reason("initial edge command rejected: " + dispatch->result.message);
        } else {
            rejected.set_reason("initial edge dispatch outcome uncertain: " +
                                dispatch->result.message);
        }
        if (const auto saved = journal_.Append(
                {.operation = operation,
                 .client_id = handle.client_id,
                 .report = rejected,
                 .dispatch_accepted = false,
                 .physical_navigation_unresolved =
                     dispatch->dispatch_state != core::BackendDispatchState::kRejected});
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
    if (const auto saved = Publish(*attempt, attempt->supervisor.Snapshot(), status);
        !saved.IsOk()) {
        RequestFailureHold(attempt);
        telemetry_->ReleaseLease(attempt->lease);
        return saved;
    }
    attempt->worker = std::thread([this, attempt] { Work(attempt); });
    return core::Result::Success("edge command accepted; physical result pending");
}

std::optional<swarmkit::v1::EdgeReport> EdgeExecutionRuntime::Get(
    const std::string& drone_id, const std::string& attempt_id) const {
    std::shared_ptr<Attempt> attempt;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = history_.find(std::make_pair(drone_id, attempt_id));
        if (it != history_.end())
            attempt = it->second;
    }
    if (!attempt) {
        const auto entry = journal_.Get(drone_id, attempt_id);
        return entry ? std::optional{entry->report} : std::nullopt;
    }
    std::lock_guard<std::mutex> lock(attempt->mutex);
    if (attempt->physical_unknown_report)
        return attempt->physical_unknown_report;
    return ToReport(*attempt, attempt->supervisor.Snapshot());
}

bool EdgeExecutionRuntime::Active(const std::string& drone_id) const {
    std::shared_ptr<Attempt> attempt;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = attempts_.find(drone_id);
        if (it == attempts_.end())
            return false;
        attempt = it->second;
    }
    std::lock_guard<std::mutex> lock(attempt->mutex);
    return !attempt->stop.load(std::memory_order_relaxed) && !attempt->physical_unknown_report &&
           !Terminal(attempt->supervisor.Snapshot().phase);
}

core::Result EdgeExecutionRuntime::SubmitCapture(
    const swarmkit::v1::SubmitEdgeCaptureRequest& capture, swarmkit::v1::EdgeReport* status) {
    if (capture.agent_monotonic_ms() > providers_.MonotonicTimeNs() / 1'000'000 + 100)
        return core::Result::Rejected("capture timestamp is in the future of agent clock");
    std::shared_ptr<Attempt> attempt;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = attempts_.find(capture.drone_id());
        if (it == attempts_.end())
            return core::Result::Rejected("edge attempt not found");
        attempt = it->second;
    }
    edge::Status updated;
    bool should_publish = false;
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        if (attempt->stop.load(std::memory_order_relaxed) || attempt->physical_unknown_report)
            return core::Result::Rejected(
                "edge attempt monitoring stopped or physical state unresolved");
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
            .attempt_id = capture.attempt_id(),
            .capture_id = capture.capture_id(),
            .calibration_id = capture.calibration_id(),
            .monotonic_ms = capture.agent_monotonic_ms(),
            .camera_center = {capture.camera_center_east_m(), capture.camera_center_north_m()},
            .altitude_above_ground_m = capture.altitude_above_ground_m(),
            .altitude_error_hard_bound_m = capture.altitude_error_hard_bound_m(),
            .horizontal_error_hard_bound_m = capture.horizontal_error_hard_bound_m(),
            .along_fov_deg = capture.along_fov_deg(),
            .cross_fov_deg = capture.cross_fov_deg(),
            .capture_succeeded = capture.capture_succeeded(),
            .image_quality_established = capture.image_quality_established(),
            .nadir_route_alignment_established = capture.nadir_route_alignment_established(),
            .flat_ground_established = capture.flat_ground_established()};
        const auto now_ms = providers_.MonotonicTimeNs() / 1'000'000;
        if (now_ms - attempt->started_agent_ms > requested.timeout_ms)
            return core::Result::Rejected("capture arrived after edge deadline");
        if (before.at_b_verified && (!before.last_position_monotonic_ms ||
                                     now_ms - *before.last_position_monotonic_ms > 1000))
            return core::Result::Rejected("endpoint pose is stale at capture delivery");
        const auto age = now_ms - input.monotonic_ms;
        if (age < 0 || age > requested.maximum_capture_age_ms)
            return core::Result::Rejected("capture outside declared acquisition-age limit");
        const edge::PositionObservation* nearby = nullptr;
        for (const auto& pose : attempt->pose_history) {
            if (std::abs(pose.monotonic_ms - input.monotonic_ms) <= 500 &&
                (!nearby || std::abs(pose.monotonic_ms - input.monotonic_ms) <=
                                std::abs(nearby->monotonic_ms - input.monotonic_ms)))
                nearby = &pose;
        }
        if (!nearby)
            return core::Result::Rejected("capture has no nearby bounded vehicle pose");
        const double elapsed_s = std::abs(input.monotonic_ms - nearby->monotonic_ms) / 1000.0;
        if (elapsed_s > 0 && requested.maximum_speed_mps <= 0)
            return core::Result::Rejected("capture alignment requires a hard speed bound");
        const double possible_displacement_m = *nearby->horizontal_error_hard_bound_m +
                                               input.horizontal_error_hard_bound_m +
                                               requested.maximum_speed_mps * elapsed_s;
        if (!std::isfinite(possible_displacement_m) ||
            std::hypot(input.camera_center.east_m - nearby->position.east_m,
                       input.camera_center.north_m - nearby->position.north_m) >
                possible_displacement_m + 1e-6)
            return core::Result::Rejected(
                "capture pose inconsistent with acquisition-time telemetry");
        const auto evidence =
            edge::BuildNadirStripEvidence(requested.route, requested.corridor_half_width_m, input);
        if (!evidence)
            return core::Result::Rejected("camera footprint cannot prove full strip");
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
        if (should_publish)
            attempt->last_reported_covered = updated.survey_covered_m;
    }
    if (should_publish) {
        if (const auto saved = Publish(*attempt, updated, status); !saved.IsOk()) {
            RequestFailureHold(attempt);
            attempt->stop.store(true, std::memory_order_relaxed);
            return saved;
        }
    }
    return core::Result::Success("capture evidence accepted");
}

core::Result EdgeExecutionRuntime::ConfirmSimulatorHold(const commands::CommandContext& context,
                                                        const std::string& drone_id) {
    const auto capabilities = backend_->GetCapabilities();
    const auto health = backend_->GetHealth();
    if (capabilities.backend_name != "sim" || !health.ready || health.failsafe != false ||
        health.ekf_ok != true || health.gps_ok != true || health.armed != true ||
        (health.mode != "SIM_GOTO" && health.mode != "SIM_HOLD"))
        return core::Result::Rejected("simulator hold takeover is unsafe or unavailable");
    TelemetryLease lease;
    if (const auto acquired =
            telemetry_->AcquireLease(drone_id, config_->default_telemetry_rate_hz, {}, &lease);
        !acquired.IsOk())
        return acquired;
    commands::CommandEnvelope envelope;
    envelope.context = context;
    envelope.command = commands::NavCmd{commands::CmdHoldPosition{}};
    const auto held = backend_->Execute(envelope);
    if (!held.result.IsOk() || held.dispatch_state != core::BackendDispatchState::kAccepted) {
        telemetry_->ReleaseLease(lease);
        return core::Result::Rejected("simulator hold command was not confirmed accepted");
    }
    std::uint64_t sequence = lease.initial_sequence;
    std::int64_t first_stationary_ms = -1;
    std::optional<std::pair<double, double>> first_position;
    bool confirmed = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{3};
    while (std::chrono::steady_clock::now() < deadline) {
        const auto read =
            TelemetryManager::WaitForFrame(lease, sequence, std::chrono::milliseconds{200});
        if (read.status != TelemetryReadStatus::kFrame)
            continue;
        sequence = read.frame.telemetry_sequence;
        const auto& frame = read.frame;
        const auto position_bound = frame.accuracy.horizontal_position;
        const auto velocity_bound = frame.accuracy.horizontal_velocity;
        const bool qualified =
            frame.HasPosition() && frame.HasVelocity() &&
            frame.position_frame == core::CoordinateFrame::kWgs84 &&
            frame.velocity_frame == core::CoordinateFrame::kLocalNed && frame.validity.mode &&
            frame.mode == "SIM_HOLD" && frame.validity.failsafe && !frame.failsafe &&
            frame.validity.estimator && frame.estimator_state == core::EstimatorState::kHealthy &&
            position_bound && velocity_bound &&
            position_bound->descriptor.semantics ==
                core::UncertaintySemantics::kDeterministicHardBound &&
            velocity_bound->descriptor.semantics ==
                core::UncertaintySemantics::kDeterministicHardBound &&
            std::isfinite(position_bound->value) && position_bound->value >= 0 &&
            std::isfinite(velocity_bound->value) && velocity_bound->value >= 0 &&
            std::hypot(frame.vx_mps, frame.vy_mps) + velocity_bound->value <= 0.5;
        if (!qualified) {
            first_stationary_ms = -1;
            first_position.reset();
            continue;
        }
        const std::int64_t time_ms = frame.agent_receive_monotonic_time_ns / 1'000'000;
        const edge::Point displacement =
            first_position ? edge::Wgs84ToLocal(first_position->first, first_position->second,
                                                frame.lat_deg, frame.lon_deg)
                           : edge::Point{};
        if (first_stationary_ms < 0 || time_ms <= first_stationary_ms || !first_position ||
            std::hypot(displacement.east_m, displacement.north_m) + 2 * position_bound->value >
                0.5) {
            first_stationary_ms = time_ms;
            first_position = std::make_pair(frame.lat_deg, frame.lon_deg);
            continue;
        }
        if (time_ms - first_stationary_ms >= 200) {
            confirmed = true;
            break;
        }
    }
    telemetry_->ReleaseLease(lease);
    return confirmed ? core::Result::Success("simulator hold physically confirmed")
                     : core::Result::Rejected(
                           "simulator hold accepted but physical stop "
                           "could not be confirmed from telemetry");
}

core::Result EdgeExecutionRuntime::Cancel(const core::ExecutionHandle& expected,
                                          const commands::CommandContext& context,
                                          swarmkit::v1::EdgeReport* status) {
    std::lock_guard<std::mutex> start_lock(start_mutex_);
    std::shared_ptr<Attempt> attempt;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = attempts_.find(expected.drone_id);
        if (it != attempts_.end())
            attempt = it->second;
    }
    if (attempt) {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        const auto prior = journal_.Get(expected.drone_id, expected.physical_attempt_id);
        if (attempt->handle != expected || attempt->physical_unknown_report ||
            (Terminal(attempt->supervisor.Snapshot().phase) && prior &&
             prior->physical_navigation_unresolved))
            attempt.reset();
    }
    if (!attempt) {
        if (const auto ready = journal_.Ready(); !ready.IsOk())
            return ready;
        const auto prior = journal_.Get(expected.drone_id, expected.physical_attempt_id);
        if (!prior || !prior->physical_navigation_unresolved ||
            !prior->report.has_execution_handle() ||
            core::internal::ToCoreExecutionHandle(prior->report.execution_handle()) != expected ||
            prior->client_id != context.client_id)
            return core::Result::Rejected("unresolved edge attempt/handle not found");
        *status = prior->report;
        if (const auto held = ConfirmSimulatorHold(context, expected.drone_id); !held.IsOk())
            return held;
        status->set_phase(swarmkit::v1::EDGE_FAILED);
        status->set_reason(
            "prior attempt reconciled after simulator hold and bounded "
            "stationary telemetry; survey obligation incomplete");
        if (const auto saved = journal_.Append({.operation = prior->operation,
                                                .client_id = prior->client_id,
                                                .report = *status,
                                                .dispatch_accepted = prior->dispatch_accepted,
                                                .physical_navigation_unresolved = false});
            !saved.IsOk()) {
            status->set_phase(swarmkit::v1::EDGE_UNKNOWN);
            status->set_reason("physical hold confirmed but reconciliation record was not durable");
            return saved;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (const auto it = history_.find({expected.drone_id, expected.physical_attempt_id});
                it != history_.end()) {
                std::lock_guard<std::mutex> attempt_lock(it->second->mutex);
                it->second->physical_unknown_report = *status;
            }
        }
        return core::Result::Success("prior attempt reconciled; simulator hold confirmed");
    }
    edge::Status before;
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        before = attempt->supervisor.Snapshot();
        if (Terminal(before.phase)) {
            *status = ToReport(*attempt, before);
            return core::Result::Success("edge attempt already terminal; no cancellation issued");
        }
        attempt->stop.store(true, std::memory_order_relaxed);
    }
    const auto held = ConfirmSimulatorHold(context, expected.drone_id);
    if (!held.IsOk()) {
        const auto saved = Publish(
            *attempt, before, status,
            "edge cancellation requested but physical hold unconfirmed; navigation unresolved");
        {
            std::lock_guard<std::mutex> lock(attempt->mutex);
            attempt->physical_unknown_report = *status;
        }
        if (attempt->worker.joinable())
            attempt->worker.join();
        return saved.IsOk() ? held : saved;
    }
    edge::Status cancelled;
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        cancelled = attempt->supervisor.Cancel(
            "edge cancelled after simulator hold and bounded stationary telemetry");
    }
    const auto saved = Publish(*attempt, cancelled, status, {}, true);
    if (attempt->worker.joinable())
        attempt->worker.join();
    if (!saved.IsOk())
        return saved;
    return core::Result::Success("edge cancelled; simulator hold physically confirmed");
}

void EdgeExecutionRuntime::Preempt(const std::string& drone_id,
                                   const std::string& affected_client_id) {
    std::shared_ptr<Attempt> attempt;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = attempts_.find(drone_id);
        if (it == attempts_.end())
            return;
        attempt = it->second;
    }
    edge::Status status;
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        if (attempt->handle.client_id != affected_client_id ||
            Terminal(attempt->supervisor.Snapshot().phase))
            return;
        status = attempt->supervisor.Snapshot();
        attempt->stop.store(true, std::memory_order_relaxed);
    }
    swarmkit::v1::EdgeReport reported;
    (void)Publish(*attempt, status, &reported,
                  "edge authority preempted; physical navigation unresolved");
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        attempt->physical_unknown_report = std::move(reported);
    }
}

void EdgeExecutionRuntime::DispatchRecovery(const std::shared_ptr<Attempt>& attempt,
                                            edge::Point target, std::int64_t now_ms) {
    const auto capabilities = backend_->GetCapabilities();
    const auto health = backend_->GetHealth();
    bool allowed = SimulatorRecoveryReady(*config_, capabilities, health);
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        allowed = allowed && !attempt->stop.load(std::memory_order_relaxed) &&
                  attempt->supervisor.Snapshot().phase == edge::Phase::kRecovering &&
                  attempt->last_battery_percent &&
                  *attempt->last_battery_percent >=
                      attempt->supervisor.Request().minimum_recovery_battery_percent;
    }
    if (allowed) {
        allowed = arbiter_
                      ->CheckAndGrant(attempt->context,
                                      std::chrono::milliseconds{config_->default_authority_ttl_ms})
                      .IsOk();
    }
    edge::Status updated;
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        if (attempt->stop.load(std::memory_order_relaxed) ||
            Terminal(attempt->supervisor.Snapshot().phase))
            return;
        allowed = allowed && attempt->supervisor.Snapshot().pending_reentry.has_value() &&
                  attempt->last_battery_percent &&
                  *attempt->last_battery_percent >=
                      attempt->supervisor.Request().minimum_recovery_battery_percent;
        if (allowed) {
            const auto& origin = attempt->operation.route(0);
            const edge::GeoPoint geo =
                edge::LocalToWgs84(origin.lat_deg(), origin.lon_deg(), target);
            commands::CommandEnvelope command;
            command.context = attempt->context;
            command.command = commands::NavCmd{
                commands::CmdGoto{.lat_deg = geo.lat_deg,
                                  .lon_deg = geo.lon_deg,
                                  .alt_m = origin.alt_m(),
                                  .speed_mps = attempt->operation.commanded_speed_mps()}};
            allowed = backend_->Execute(command).result.IsOk();
        }
        updated = attempt->supervisor.RecoveryDispatched(allowed, now_ms);
        if (allowed) {
            attempt->reentry_target = target;
            attempt->reentry_candidate_ms = -1;
        }
    }
    if (updated.phase == edge::Phase::kFailed)
        RequestFailureHold(attempt);
    (void)Publish(*attempt, updated);
}

void EdgeExecutionRuntime::DispatchReturnToB(const std::shared_ptr<Attempt>& attempt) {
    const auto health = backend_->GetHealth();
    bool allowed = SimulatorRecoveryReady(*config_, backend_->GetCapabilities(), health);
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        allowed = allowed && !attempt->stop.load(std::memory_order_relaxed) &&
                  !Terminal(attempt->supervisor.Snapshot().phase) &&
                  attempt->last_battery_percent &&
                  *attempt->last_battery_percent >=
                      attempt->supervisor.Request().minimum_recovery_battery_percent;
    }
    if (allowed)
        allowed = arbiter_
                      ->CheckAndGrant(attempt->context,
                                      std::chrono::milliseconds{config_->default_authority_ttl_ms})
                      .IsOk();
    std::optional<edge::Status> continuation;
    {
        std::lock_guard<std::mutex> lock(attempt->mutex);
        if (attempt->stop.load(std::memory_order_relaxed) ||
            Terminal(attempt->supervisor.Snapshot().phase))
            return;
        allowed = allowed && attempt->last_battery_percent &&
                  *attempt->last_battery_percent >=
                      attempt->supervisor.Request().minimum_recovery_battery_percent;
        if (allowed) {
            const auto& target = attempt->operation.route(attempt->operation.route_size() - 1);
            commands::CommandEnvelope command;
            command.context = attempt->context;
            command.command = commands::NavCmd{
                commands::CmdGoto{.lat_deg = target.lat_deg(),
                                  .lon_deg = target.lon_deg(),
                                  .alt_m = target.alt_m(),
                                  .speed_mps = attempt->operation.commanded_speed_mps()}};
            allowed = backend_->Execute(command).result.IsOk();
        }
        continuation = allowed ? attempt->supervisor.RecoveryReentryCompleted(
                                     providers_.MonotonicTimeNs() / 1'000'000)
                               : attempt->supervisor.Interrupt(
                                     "cannot safely continue from recovery re-entry to B");
    }
    if (continuation)
        (void)Publish(*attempt, *continuation);
}

void EdgeExecutionRuntime::RequestFailureHold(const std::shared_ptr<Attempt>& attempt) {
    const auto health = backend_->GetHealth();
    if (SafetyOverrideMode(health.mode) || attempt->stop.load(std::memory_order_relaxed))
        return;  // The flight controller's active safety mode has priority.
    const auto capabilities = backend_->GetCapabilities();
    if (std::find(capabilities.supported_commands.begin(), capabilities.supported_commands.end(),
                  "hold") == capabilities.supported_commands.end()) {
        core::Logger::ErrorFmt("VECR failed attempt {}: backend has no hold capability",
                               attempt->handle.physical_attempt_id);
        return;
    }
    if (!arbiter_
             ->CheckAndGrant(attempt->context,
                             std::chrono::milliseconds{config_->default_authority_ttl_ms})
             .IsOk()) {
        core::Logger::ErrorFmt("VECR failed attempt {}: hold authority unavailable",
                               attempt->handle.physical_attempt_id);
        return;
    }
    commands::CommandEnvelope command;
    command.context = attempt->context;
    command.command = commands::NavCmd{commands::CmdHoldPosition{}};
    const auto held = backend_->Execute(command);
    if (!held.result.IsOk() || held.dispatch_state != core::BackendDispatchState::kAccepted)
        core::Logger::ErrorFmt("VECR failed attempt {}: hold command not accepted",
                               attempt->handle.physical_attempt_id);
    // Command acceptance is not physical stop. The journal retains unresolved
    // physical navigation until a separate bounded stop is confirmed.
}

void EdgeExecutionRuntime::Work(const std::shared_ptr<Attempt>& attempt) {
    while (!attempt->stop.load(std::memory_order_relaxed)) {
        const auto read = TelemetryManager::WaitForFrame(attempt->lease, attempt->last_sequence,
                                                         std::chrono::milliseconds{200});
        if (read.status == TelemetryReadStatus::kShutdown)
            break;
        if (read.status != TelemetryReadStatus::kFrame) {
            const std::int64_t now_ms = providers_.MonotonicTimeNs() / 1'000'000;
            edge::Status timeout_status;
            bool publish_timeout = false;
            {
                std::lock_guard<std::mutex> lock(attempt->mutex);
                if (now_ms - attempt->started_agent_ms > attempt->supervisor.Request().timeout_ms) {
                    timeout_status =
                        attempt->supervisor.Interrupt("edge telemetry/attempt timeout");
                    publish_timeout = true;
                } else if (now_ms - attempt->last_frame_agent_ms > 1000 &&
                           attempt->supervisor.Snapshot().phase != edge::Phase::kUnknown) {
                    timeout_status = attempt->supervisor.ObservePosition(
                        {.monotonic_ms = now_ms, .valid = false});
                    publish_timeout = true;
                }
            }
            if (publish_timeout && !Publish(*attempt, timeout_status).IsOk())
                break;
            if (Terminal(timeout_status.phase))
                break;
            continue;
        }
        attempt->last_sequence = read.frame.telemetry_sequence;
        attempt->last_frame_agent_ms = providers_.MonotonicTimeNs() / 1'000'000;
        const auto& frame = read.frame;
        const auto& origin = attempt->operation.route(0);
        edge::PositionObservation observation;
        observation.monotonic_ms = frame.agent_receive_monotonic_time_ns / 1'000'000;
        observation.valid =
            frame.HasPosition() && frame.position_frame == core::CoordinateFrame::kWgs84 &&
            frame.validity.estimator && frame.estimator_state == core::EstimatorState::kHealthy;
        if (observation.valid) {
            observation.position = edge::Wgs84ToLocal(origin.lat_deg(), origin.lon_deg(),
                                                      frame.lat_deg, frame.lon_deg);
        }
        observation.failsafe = frame.validity.failsafe && frame.failsafe;
        observation.safe_to_navigate = observation.valid && !observation.failsafe &&
                                       frame.validity.failsafe && frame.validity.armed &&
                                       frame.armed;
        if (frame.HasBattery())
            observation.battery_percent = frame.battery_percent;
        if (frame.HasVelocity() && frame.velocity_frame == core::CoordinateFrame::kLocalNed)
            observation.speed_mps = std::hypot(frame.vx_mps, frame.vy_mps);
        if (frame.HasRelativeAltitude())
            observation.altitude_m = frame.rel_alt_m;
        if (frame.accuracy.horizontal_position &&
            frame.accuracy.horizontal_position->descriptor.semantics ==
                core::UncertaintySemantics::kDeterministicHardBound)
            observation.horizontal_error_hard_bound_m = frame.accuracy.horizontal_position->value;
        if (frame.accuracy.horizontal_velocity &&
            frame.accuracy.horizontal_velocity->descriptor.semantics ==
                core::UncertaintySemantics::kDeterministicHardBound)
            observation.speed_error_hard_bound_mps = frame.accuracy.horizontal_velocity->value;
        if (frame.accuracy.vertical_position &&
            frame.accuracy.vertical_position->descriptor.semantics ==
                core::UncertaintySemantics::kDeterministicHardBound)
            observation.vertical_error_hard_bound_m = frame.accuracy.vertical_position->value;
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
            if (observation.valid && std::isfinite(observation.position.east_m) &&
                std::isfinite(observation.position.north_m) &&
                observation.horizontal_error_hard_bound_m &&
                std::isfinite(*observation.horizontal_error_hard_bound_m) &&
                *observation.horizontal_error_hard_bound_m >= 0) {
                attempt->pose_history.push_back(observation);
            }
            // Resource exhaustion drops oldest evidence; it never invents a pose.
            while (!attempt->pose_history.empty() &&
                   (attempt->pose_history.size() > 2048 ||
                    observation.monotonic_ms - attempt->pose_history.front().monotonic_ms >
                        attempt->supervisor.Request().maximum_capture_age_ms + 500))
                attempt->pose_history.pop_front();
            publish = updated.phase != attempt->last_reported_phase ||
                      updated.at_b_verified != attempt->last_reported_arrival;
            if (publish) {
                attempt->last_reported_phase = updated.phase;
                attempt->last_reported_arrival = updated.at_b_verified;
            }
            reentry = updated.pending_reentry;
            if (attempt->reentry_target) {
                const auto& request = attempt->supervisor.Request();
                const bool at_reentry =
                    updated.phase == edge::Phase::kRecovering && observation.valid &&
                    observation.safe_to_navigate && observation.horizontal_error_hard_bound_m &&
                    observation.speed_mps && std::isfinite(*observation.speed_mps) &&
                    *observation.speed_mps >= 0 && observation.speed_error_hard_bound_mps &&
                    observation.altitude_m && observation.vertical_error_hard_bound_m &&
                    std::hypot(
                        std::hypot(
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
        if (updated.phase == edge::Phase::kFailed)
            RequestFailureHold(attempt);
        if (publish && !Publish(*attempt, updated).IsOk())
            break;
        if (reentry)
            DispatchRecovery(attempt, *reentry, observation.monotonic_ms);
        if (return_to_b)
            DispatchReturnToB(attempt);
        {
            std::lock_guard<std::mutex> lock(attempt->mutex);
            attempt->last_processed_position_ms.store(observation.monotonic_ms,
                                                      std::memory_order_release);
        }
        if (Terminal(updated.phase))
            break;
    }
    telemetry_->ReleaseLease(attempt->lease);
}

void EdgeExecutionRuntime::Shutdown() {
    std::vector<std::shared_ptr<Attempt>> attempts;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& [_, attempt] : attempts_)
            attempts.push_back(attempt);
        attempts_.clear();
        history_.clear();
    }
    for (auto& attempt : attempts)
        attempt->stop.store(true, std::memory_order_relaxed);
    for (auto& attempt : attempts)
        if (attempt->worker.joinable())
            attempt->worker.join();
}

}  // namespace swarmkit::agent::internal
