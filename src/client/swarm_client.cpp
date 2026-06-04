// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/client/swarm_client.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <expected>
#include <mutex>
#include <optional>
#include <ranges>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "swarmkit/core/logger.h"
#include "swarmkit/core/overloaded.h"

namespace swarmkit::client {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)

namespace {

constexpr std::size_t kFallbackParallelism = 8;
constexpr std::size_t kDefaultFanoutParallelism = 32;
constexpr std::size_t kMaxDefaultFanoutParallelism = 64;

[[nodiscard]] std::size_t ComputeParallelism(std::size_t task_count,
                                             const SwarmFanoutOptions& options) {
    if (task_count == 0) {
        return 0;
    }
    if (options.max_parallelism > 0) {
        return std::min(task_count, options.max_parallelism);
    }

    const std::size_t kHardwareThreads =
        std::max<std::size_t>(1, std::thread::hardware_concurrency());
    const std::size_t kUpperBound = std::min(
        kMaxDefaultFanoutParallelism,
        std::max({kFallbackParallelism, kDefaultFanoutParallelism, kHardwareThreads * 2U}));
    return std::min(task_count, kUpperBound);
}

class BoundedFanoutExecutor final {
   public:
    BoundedFanoutExecutor(const SwarmFanoutOptions& options, std::size_t task_count)
        : options_(options), parallelism_(ComputeParallelism(task_count, options)) {}

    [[nodiscard]] std::size_t Parallelism() const noexcept {
        return parallelism_;
    }

    [[nodiscard]] bool IsCancellationRequested() const noexcept {
        return options_.cancellation.stop_requested() || internal_stop_.stop_requested();
    }

    [[nodiscard]] bool CancelRemainingOnFailure() const noexcept {
        return options_.cancel_remaining_on_failure;
    }

    void RequestCancel() noexcept {
        internal_stop_.request_stop();
    }

   private:
    SwarmFanoutOptions options_;
    std::stop_source internal_stop_;
    std::size_t parallelism_{};
};

[[nodiscard]] RpcError MakeLocalError(core::ErrorDomain domain, RpcStatusCode code,
                                      std::string message) {
    if (code == RpcStatusCode::kOk) {
        RpcError error = core::SwarmError::Ok();
        error.user_message = std::move(message);
        return error;
    }

    core::ErrorRetryability retryability = core::ErrorRetryability::kAfterRemediation;
    if (code == RpcStatusCode::kUnavailable || code == RpcStatusCode::kDeadlineExceeded ||
        code == RpcStatusCode::kCancelled) {
        retryability = core::ErrorRetryability::kAfterBackoff;
    } else if (code == RpcStatusCode::kInternal || code == RpcStatusCode::kBackendFailure ||
               code == RpcStatusCode::kUnknown) {
        retryability = core::ErrorRetryability::kUnknown;
    }

    const core::ErrorSeverity severity =
        code == RpcStatusCode::kInternal || code == RpcStatusCode::kUnknown
            ? core::ErrorSeverity::kCritical
            : core::ErrorSeverity::kWarning;
    RpcError error =
        core::SwarmError::Make(domain, code, std::move(message), severity, retryability);
    error.debug_message = error.user_message;
    return error;
}

[[nodiscard]] ExecutionResult UnregisteredExecutionResult(const std::string& drone_id) {
    ExecutionResult out;
    out.ok = false;
    out.message = "drone '" + drone_id + "' not registered";
    out.error = MakeLocalError(core::ErrorDomain::kSwarm, RpcStatusCode::kNotFound, out.message);
    return out;
}

[[nodiscard]] GoalResult UnregisteredGoalResult(const std::string& drone_id) {
    GoalResult out;
    out.ok = false;
    out.message = "drone '" + drone_id + "' not registered";
    out.error = MakeLocalError(core::ErrorDomain::kSwarm, RpcStatusCode::kNotFound, out.message);
    return out;
}

[[nodiscard]] CommandResult LocalCommandResult(bool success, std::string message) {
    CommandResult out;
    out.ok = success;
    out.message = std::move(message);
    out.error = MakeLocalError(core::ErrorDomain::kSwarm,
                               success ? RpcStatusCode::kOk : RpcStatusCode::kRejected,
                               out.message);
    return out;
}

[[nodiscard]] CommandResult UnregisteredCommandResult(const std::string& drone_id) {
    CommandResult out;
    out.ok = false;
    out.message = "drone '" + drone_id + "' not registered";
    out.error = MakeLocalError(core::ErrorDomain::kSwarm, RpcStatusCode::kNotFound, out.message);
    return out;
}

[[nodiscard]] ReleaseAuthorityResult LocalReleaseAuthorityResult(bool success, std::string message,
                                                                 RpcStatusCode code) {
    ReleaseAuthorityResult out;
    out.ok = success;
    out.message = std::move(message);
    out.error = MakeLocalError(core::ErrorDomain::kSwarm, code, out.message);
    return out;
}

[[nodiscard]] ReleaseAuthorityResult UnregisteredReleaseAuthorityResult(
    const std::string& drone_id) {
    return LocalReleaseAuthorityResult(false, "drone '" + drone_id + "' not registered",
                                       RpcStatusCode::kNotFound);
}

[[nodiscard]] bool IsProtocolSchedulableCommand(const commands::Command& command) {
    return std::holds_alternative<commands::FlightCmd>(command) ||
           std::holds_alternative<commands::NavCmd>(command);
}

[[nodiscard]] std::int64_t NowUnixMs() {
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

[[nodiscard]] std::string MakeSwarmExecutionId(const commands::CommandContext& context) {
    if (!context.correlation_id.empty()) {
        return context.correlation_id;
    }
    return "swarm-sync-" + std::to_string(NowUnixMs());
}

[[nodiscard]] CommandResult CommandResultFromExecution(const ExecutionResult& execution,
                                                       const std::string& success_message) {
    CommandResult out;
    out.ok = execution.ok;
    out.message = execution.ok ? success_message : execution.message;
    out.correlation_id = execution.correlation_id;
    out.error = execution.ok
                    ? MakeLocalError(core::ErrorDomain::kSwarm, RpcStatusCode::kOk, out.message)
                    : execution.error;
    return out;
}

[[nodiscard]] CommandResult LocalSwarmError(RpcStatusCode code, std::string message) {
    CommandResult out;
    out.ok = false;
    out.message = std::move(message);
    out.error = MakeLocalError(core::ErrorDomain::kSwarm, code, out.message);
    return out;
}

[[nodiscard]] CommandResult CancelledCommandResult(const std::string& drone_id) {
    return LocalSwarmError(RpcStatusCode::kCancelled,
                           "swarm fanout cancelled before dispatch for drone '" + drone_id + "'");
}

[[nodiscard]] GoalResult CancelledGoalResult(const std::string& drone_id) {
    GoalResult out;
    out.ok = false;
    out.message = "swarm goal fanout cancelled before dispatch for drone '" + drone_id + "'";
    out.error = MakeLocalError(core::ErrorDomain::kSwarm, RpcStatusCode::kCancelled, out.message);
    return out;
}

[[nodiscard]] CommandResult ExceptionCommandResult(const std::string& drone_id,
                                                   const std::string& detail) {
    return LocalSwarmError(RpcStatusCode::kInternal,
                           "swarm fanout task failed for drone '" + drone_id + "': " + detail);
}

[[nodiscard]] GoalResult ExceptionGoalResult(const std::string& drone_id,
                                             const std::string& detail) {
    GoalResult out;
    out.ok = false;
    out.message = "swarm goal fanout task failed for drone '" + drone_id + "': " + detail;
    out.error = MakeLocalError(core::ErrorDomain::kSwarm, RpcStatusCode::kInternal, out.message);
    return out;
}

[[nodiscard]] ReleaseAuthorityResult CancelledReleaseAuthorityResult(const std::string& drone_id) {
    return LocalReleaseAuthorityResult(
        false, "swarm fanout cancelled before authority release for drone '" + drone_id + "'",
        RpcStatusCode::kCancelled);
}

[[nodiscard]] ReleaseAuthorityResult ExceptionReleaseAuthorityResult(const std::string& drone_id,
                                                                     const std::string& detail) {
    return LocalReleaseAuthorityResult(
        false, "swarm authority release task failed for drone '" + drone_id + "': " + detail,
        RpcStatusCode::kInternal);
}

[[nodiscard]] ExecutionResult CancelledExecutionResult(const std::string& drone_id) {
    ExecutionResult out;
    out.ok = false;
    out.message = "swarm fanout cancelled before dispatch for drone '" + drone_id + "'";
    out.error = MakeLocalError(core::ErrorDomain::kSwarm, RpcStatusCode::kCancelled, out.message);
    out.handle.drone_id = drone_id;
    return out;
}

[[nodiscard]] ExecutionResult ExceptionExecutionResult(const std::string& drone_id,
                                                       const std::string& detail) {
    ExecutionResult out;
    out.ok = false;
    out.message = "swarm fanout task failed for drone '" + drone_id + "': " + detail;
    out.error = MakeLocalError(core::ErrorDomain::kInternal, RpcStatusCode::kInternal, out.message);
    out.handle.drone_id = drone_id;
    return out;
}

template <typename Result, typename TaskFn, typename CancelledFn, typename ExceptionFn,
          typename ResultOkFn>
[[nodiscard]] std::unordered_map<std::string, Result> RunBoundedClientTasks(
    const std::vector<std::pair<std::string, std::shared_ptr<Client>>>& clients,
    const SwarmFanoutOptions& fanout_options, TaskFn&& task_fn, CancelledFn&& make_cancelled_result,
    ExceptionFn&& make_exception_result, ResultOkFn&& result_ok) {
    std::unordered_map<std::string, Result> results;
    results.reserve(clients.size());
    if (clients.empty()) {
        return results;
    }

    BoundedFanoutExecutor executor(fanout_options, clients.size());
    if (executor.Parallelism() == 0) {
        return results;
    }

    std::atomic<std::size_t> next_index{0};
    std::mutex results_mutex;
    std::vector<std::thread> workers;
    workers.reserve(executor.Parallelism());

    const auto store_result = [&results, &results_mutex](const std::string& drone_id,
                                                         Result result) {
        std::lock_guard<std::mutex> lock(results_mutex);
        results.emplace(drone_id, std::move(result));
    };

    for (std::size_t index = 0; index < executor.Parallelism(); ++index) {
        workers.emplace_back([&clients, &executor, &make_exception_result, &next_index, &result_ok,
                              &store_result, &task_fn]() {
            while (true) {
                if (executor.IsCancellationRequested()) {
                    return;
                }
                const std::size_t task_index = next_index.fetch_add(1, std::memory_order_relaxed);
                if (task_index >= clients.size()) {
                    return;
                }
                if (executor.IsCancellationRequested()) {
                    return;
                }

                const auto& [drone_id, client] = clients[task_index];
                Result result;
                try {
                    result = task_fn(drone_id, client);
                } catch (const std::exception& error) {
                    result = make_exception_result(drone_id, error.what());
                } catch (...) {
                    result = make_exception_result(drone_id, "unknown exception");
                }

                const bool task_ok = result_ok(result);
                store_result(drone_id, std::move(result));
                if (!task_ok && executor.CancelRemainingOnFailure()) {
                    executor.RequestCancel();
                }
            }
        });
    }

    for (auto& worker : workers) {
        worker.join();
    }

    for (const auto& [drone_id, client] : clients) {
        static_cast<void>(client);
        if (!results.contains(drone_id)) {
            results.emplace(drone_id, make_cancelled_result(drone_id));
        }
    }

    return results;
}

template <typename TaskFn>
[[nodiscard]] std::unordered_map<std::string, CommandResult> RunCommandTasks(
    const std::vector<std::pair<std::string, std::shared_ptr<Client>>>& clients,
    const SwarmFanoutOptions& fanout_options, TaskFn&& task_fn) {
    return RunBoundedClientTasks<CommandResult>(
        clients, fanout_options, std::forward<TaskFn>(task_fn), CancelledCommandResult,
        ExceptionCommandResult, [](const CommandResult& result) { return result.ok; });
}

template <typename TaskFn>
[[nodiscard]] std::unordered_map<std::string, ExecutionResult> RunExecutionTasks(
    const std::vector<std::pair<std::string, std::shared_ptr<Client>>>& clients,
    const SwarmFanoutOptions& fanout_options, TaskFn&& task_fn) {
    return RunBoundedClientTasks<ExecutionResult>(
        clients, fanout_options, std::forward<TaskFn>(task_fn), CancelledExecutionResult,
        ExceptionExecutionResult, [](const ExecutionResult& result) { return result.ok; });
}

template <typename TaskFn>
[[nodiscard]] std::unordered_map<std::string, GoalResult> RunGoalTasks(
    const std::vector<std::pair<std::string, std::shared_ptr<Client>>>& clients,
    const SwarmFanoutOptions& fanout_options, TaskFn&& task_fn) {
    return RunBoundedClientTasks<GoalResult>(
        clients, fanout_options, std::forward<TaskFn>(task_fn), CancelledGoalResult,
        ExceptionGoalResult, [](const GoalResult& result) { return result.ok; });
}

template <typename TaskFn>
[[nodiscard]] std::unordered_map<std::string, ReleaseAuthorityResult> RunReleaseAuthorityTasks(
    const std::vector<std::pair<std::string, std::shared_ptr<Client>>>& clients,
    const SwarmFanoutOptions& fanout_options, TaskFn&& task_fn) {
    return RunBoundedClientTasks<ReleaseAuthorityResult>(
        clients, fanout_options, std::forward<TaskFn>(task_fn), CancelledReleaseAuthorityResult,
        ExceptionReleaseAuthorityResult,
        [](const ReleaseAuthorityResult& result) { return result.ok; });
}

template <typename TaskFn>
[[nodiscard]] std::unordered_map<std::string, std::vector<ExecutionHandle>> RunExecutionListTasks(
    const std::vector<std::pair<std::string, std::shared_ptr<Client>>>& clients,
    const SwarmFanoutOptions& fanout_options, TaskFn&& task_fn) {
    return RunBoundedClientTasks<std::vector<ExecutionHandle>>(
        clients, fanout_options, std::forward<TaskFn>(task_fn),
        [](const std::string&) { return std::vector<ExecutionHandle>{}; },
        [](const std::string& drone_id, const std::string& detail) {
            core::Logger::WarnFmt("swarm fanout list task failed for drone '{}': {}", drone_id,
                                  detail);
            return std::vector<ExecutionHandle>{};
        },
        [](const std::vector<ExecutionHandle>&) { return true; });
}

[[nodiscard]] bool TimeSyncAcceptable(const TimeSyncState& state,
                                      const SwarmExecutionOptions& options, std::string* detail) {
    if (!options.require_time_sync) {
        if (detail != nullptr) {
            *detail = "time sync check disabled";
        }
        return true;
    }
    if (!state.synced) {
        if (detail != nullptr) {
            *detail = state.message.empty() ? "time sync is not established"
                                            : "time sync is not established: " + state.message;
        }
        return false;
    }
    if (state.stale) {
        if (detail != nullptr) {
            *detail = state.message.empty() ? "time sync state is stale"
                                            : "time sync state is stale: " + state.message;
        }
        return false;
    }
    if (state.sync_quality_percent < options.min_time_sync_quality_percent) {
        if (detail != nullptr) {
            *detail = "time sync quality " + std::to_string(state.sync_quality_percent) +
                      "% is below required " +
                      std::to_string(options.min_time_sync_quality_percent) + "%";
        }
        return false;
    }
    if (std::chrono::milliseconds{std::llabs(state.clock_offset_ms)} > options.max_clock_offset) {
        if (detail != nullptr) {
            *detail = "clock offset " + std::to_string(state.clock_offset_ms) +
                      "ms exceeds allowed " + std::to_string(options.max_clock_offset.count()) +
                      "ms";
        }
        return false;
    }
    if (detail != nullptr) {
        *detail = "time sync ready";
    }
    return true;
}

void PopulateTrajectoryPointFromCommand(const commands::Command& command, TrajectoryPoint* point) {
    if (point == nullptr) {
        return;
    }
    point->has_position = false;
    point->has_local_position = false;
    point->use_local_position = false;
    point->command = command;

    std::visit(core::Overloaded{
                   [](const commands::FlightCmd&) {},
                   [](const commands::MissionCmd&) {},
                   [](const commands::PayloadCmd&) {},
                   [](const commands::BackendCmd&) {},
                   [point](const commands::NavCmd& nav) {
                       std::visit(core::Overloaded{
                                      [point](const commands::CmdSetWaypoint& waypoint) {
                                          point->has_position = true;
                                          point->position = {
                                              .lat_deg = waypoint.lat_deg,
                                              .lon_deg = waypoint.lon_deg,
                                              .alt_m = waypoint.alt_m,
                                          };
                                      },
                                      [point](const commands::CmdGoto& go_to) {
                                          point->has_position = true;
                                          point->position = {
                                              .lat_deg = go_to.lat_deg,
                                              .lon_deg = go_to.lon_deg,
                                              .alt_m = go_to.alt_m,
                                          };
                                          point->yaw_deg = go_to.yaw_deg;
                                          point->has_yaw = go_to.use_yaw;
                                      },
                                      [](const auto&) {},
                                  },
                                  nav);
                   },
               },
               command);
}

[[nodiscard]] std::expected<TrajectoryPlan, CommandResult> BuildTimedCommandPlan(
    const std::string& drone_id, const commands::Command& command,
    const commands::CommandContext& context, const SwarmExecutionOptions& options,
    const std::string& execution_id) {
    if (drone_id.empty()) {
        return std::unexpected(LocalSwarmError(RpcStatusCode::kInvalidArgument,
                                               "synchronized execution requires drone_id"));
    }
    if (!IsProtocolSchedulableCommand(command)) {
        return std::unexpected(LocalSwarmError(
            RpcStatusCode::kUnsupported,
            "protocol synchronized execution currently supports flight and navigation commands"));
    }

    TrajectoryPlan plan;
    plan.execution_id = execution_id;
    plan.revision = 1;
    plan.drone_id = drone_id;
    plan.frame = "timed-command";
    plan.validation.tracking_tolerance_m = options.max_drift_m;
    plan.labels["swarmkit.execution_kind"] = "synchronized-command";
    plan.labels["swarmkit.context.client_id"] = context.client_id;
    plan.labels["swarmkit.context.priority"] = std::to_string(static_cast<int>(context.priority));
    if (!context.correlation_id.empty()) {
        plan.labels["swarmkit.context.correlation_id"] = context.correlation_id;
    }
    plan.labels["swarmkit.max_drift_m"] = std::to_string(options.max_drift_m);

    TrajectoryPoint point;
    point.time_offset_ms = 0;
    PopulateTrajectoryPointFromCommand(command, &point);
    plan.points.push_back(std::move(point));
    return plan;
}

[[nodiscard]] std::vector<std::pair<std::string, std::shared_ptr<Client>>> FilterClients(
    const std::vector<std::pair<std::string, std::shared_ptr<Client>>>& clients,
    const std::unordered_set<std::string>& drone_ids) {
    std::vector<std::pair<std::string, std::shared_ptr<Client>>> filtered;
    filtered.reserve(drone_ids.size());
    for (const auto& entry : clients) {
        if (drone_ids.contains(entry.first)) {
            filtered.push_back(entry);
        }
    }
    return filtered;
}

[[nodiscard]] std::size_t CountSucceeded(
    const std::unordered_map<std::string, CommandResult>& results) {
    std::size_t succeeded = 0;
    for (const auto& [drone_id, result] : results) {
        static_cast<void>(drone_id);
        if (result.ok) {
            ++succeeded;
        }
    }
    return succeeded;
}

}  // namespace

/// @brief Holds per-drone Client instances and synchronises fleet-wide access.
struct SwarmClient::Impl {
    ClientConfig default_config;

    mutable std::mutex mutex;
    std::unordered_map<std::string, std::shared_ptr<Client>> clients;

    /// @brief Returns a snapshot of all drone_id / Client pairs.
    ///
    /// Shared ownership means a concurrent RemoveDrone() cannot destroy a
    /// Client while a broadcast or StartAllTelemetry is still using it.
    [[nodiscard]] std::vector<std::pair<std::string, std::shared_ptr<Client>>> Snapshot() const {
        std::lock_guard<std::mutex> lock(mutex);
        std::vector<std::pair<std::string, std::shared_ptr<Client>>> out;
        out.reserve(clients.size());
        for (const auto& [drone_id, client] : clients) {
            out.emplace_back(drone_id, client);
        }
        return out;
    }
};

SwarmClient::SwarmClient(ClientConfig default_config) : impl_(std::make_unique<Impl>()) {
    impl_->default_config = std::move(default_config);
}

SwarmClient::~SwarmClient() {
    try {
        StopAllTelemetry();
        StopAllReports();
    } catch (const std::exception& exc) {
        core::Logger::WarnFmt("SwarmClient::~SwarmClient failed to stop telemetry: {}", exc.what());
    } catch (...) {
        core::Logger::Warn("SwarmClient::~SwarmClient failed to stop telemetry");
    }
}

void SwarmClient::AddDrone(const std::string& drone_id, const std::string& address) {
    ClientConfig cfg = impl_->default_config;
    cfg.address = address;
    auto new_client = std::make_shared<Client>(std::move(cfg));

    std::shared_ptr<Client> old_client;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(drone_id);
        if (iter != impl_->clients.end()) {
            old_client = std::move(iter->second);
            iter->second = new_client;
        } else {
            impl_->clients.emplace(drone_id, new_client);
        }
    }

    /// Stop any telemetry on the replaced client outside the lock.
    if (old_client) {
        old_client->StopTelemetry();
    }

    core::Logger::InfoFmt("SwarmClient: registered drone '{}' at {}", drone_id, address);
}

void SwarmClient::RemoveDrone(const std::string& drone_id) {
    std::shared_ptr<Client> removed;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(drone_id);
        if (iter == impl_->clients.end()) {
            return;
        }
        removed = std::move(iter->second);
        impl_->clients.erase(iter);
    }

    removed->StopTelemetry();
    core::Logger::InfoFmt("SwarmClient: removed drone '{}'", drone_id);
}

std::size_t SwarmClient::DroneCount() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->clients.size();
}

core::Result SwarmClient::ApplyConfig(const SwarmConfig& config,
                                      SwarmAddressPreference address_preference) {
    if (const core::Result kValidation = config.Validate(); !kValidation.IsOk()) {
        return kValidation;
    }

    std::unordered_map<std::string, std::shared_ptr<Client>> old_clients;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->default_config = config.default_client_config;
        old_clients.swap(impl_->clients);

        for (const auto& drone : config.drones) {
            ClientConfig client_config = impl_->default_config;
            const bool kUseLocalAddress =
                address_preference == SwarmAddressPreference::kPreferLocal &&
                !drone.local_address.empty();
            client_config.address = kUseLocalAddress ? drone.local_address : drone.address;
            impl_->clients.emplace(drone.drone_id,
                                   std::make_shared<Client>(std::move(client_config)));
        }
    }

    for (auto& [drone_id, client] : old_clients) {
        static_cast<void>(drone_id);
        client->StopTelemetry();
    }

    for (const auto& drone : config.drones) {
        const bool kUseLocalAddress = address_preference == SwarmAddressPreference::kPreferLocal &&
                                      !drone.local_address.empty();
        core::Logger::InfoFmt("SwarmClient: configured drone '{}' at {}", drone.drone_id,
                              kUseLocalAddress ? drone.local_address : drone.address);
    }
    return core::Result::Success();
}

CommandResult SwarmClient::SendCommand(const commands::CommandEnvelope& envelope) const {
    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(envelope.context.drone_id);
        if (iter == impl_->clients.end()) {
            return UnregisteredCommandResult(envelope.context.drone_id);
        }
        client = iter->second;
    }
    return client->SendCommand(envelope);
}

CommandResult SwarmClient::SendCommandAndWait(const commands::CommandEnvelope& envelope,
                                              const CommandWaitOptions& options) const {
    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(envelope.context.drone_id);
        if (iter == impl_->clients.end()) {
            return UnregisteredCommandResult(envelope.context.drone_id);
        }
        client = iter->second;
    }
    return client->SendCommandAndWait(envelope, options);
}

HealthStatus SwarmClient::GetHealth(const std::string& drone_id) const {
    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(drone_id);
        if (iter == impl_->clients.end()) {
            HealthStatus out;
            out.message = "drone '" + drone_id + "' not registered";
            out.error =
                MakeLocalError(core::ErrorDomain::kSwarm, RpcStatusCode::kNotFound, out.message);
            return out;
        }
        client = iter->second;
    }
    return client->GetHealth();
}

RuntimeStats SwarmClient::GetRuntimeStats(const std::string& drone_id) const {
    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(drone_id);
        if (iter == impl_->clients.end()) {
            RuntimeStats out;
            out.error = MakeLocalError(core::ErrorDomain::kSwarm, RpcStatusCode::kNotFound,
                                       "drone '" + drone_id + "' not registered");
            return out;
        }
        client = iter->second;
    }
    return client->GetRuntimeStats();
}

std::unordered_map<std::string, CommandResult> SwarmClient::BroadcastCommand(
    const commands::Command& command, const commands::CommandContext& context,
    const SwarmFanoutOptions& fanout_options) const {
    const auto kSnapshot = impl_->Snapshot();
    return RunCommandTasks(
        kSnapshot, fanout_options,
        [&command, &context](const std::string& drone_id, const std::shared_ptr<Client>& client) {
            commands::CommandEnvelope envelope;
            envelope.context = context;
            envelope.context.drone_id = drone_id;
            envelope.command = command;
            return client->SendCommand(envelope);
        });
}

std::unordered_map<std::string, CommandResult> SwarmClient::BroadcastCommandAndWait(
    const commands::Command& command, const commands::CommandContext& context,
    const CommandWaitOptions& options, const SwarmFanoutOptions& fanout_options) const {
    const auto kSnapshot = impl_->Snapshot();
    return RunCommandTasks(kSnapshot, fanout_options,
                           [&command, &context, &options](const std::string& drone_id,
                                                          const std::shared_ptr<Client>& client) {
                               commands::CommandEnvelope envelope;
                               envelope.context = context;
                               envelope.context.drone_id = drone_id;
                               envelope.command = command;
                               return client->SendCommandAndWait(envelope, options);
                           });
}

SwarmExecutionReport SwarmClient::ExecuteSynchronizedCommand(
    const commands::Command& command, const commands::CommandContext& context,
    const SwarmExecutionOptions& options) const {
    std::unordered_map<std::string, commands::Command> planned;
    const auto snapshot = impl_->Snapshot();
    planned.reserve(snapshot.size());
    for (const auto& [drone_id, client] : snapshot) {
        static_cast<void>(client);
        planned.emplace(drone_id, command);
    }
    return ExecutePlannedCommands(planned, context, options);
}

SwarmExecutionReport SwarmClient::ExecutePlannedCommands(
    const std::unordered_map<std::string, commands::Command>& planned_commands,
    const commands::CommandContext& context, const SwarmExecutionOptions& options) const {
    SwarmExecutionReport report;
    report.planned_commands = planned_commands;
    report.requested = planned_commands.size();
    if (planned_commands.empty()) {
        report.message = "no planned swarm commands";
        report.error = MakeLocalError(core::ErrorDomain::kSwarm, RpcStatusCode::kInvalidArgument,
                                      report.message);
        return report;
    }

    const auto snapshot = impl_->Snapshot();
    std::unordered_map<std::string, std::shared_ptr<Client>> clients_by_drone;
    clients_by_drone.reserve(snapshot.size());
    for (const auto& [drone_id, client] : snapshot) {
        clients_by_drone.emplace(drone_id, client);
    }

    std::vector<std::pair<std::string, std::shared_ptr<Client>>> target_clients;
    target_clients.reserve(planned_commands.size());
    for (const auto& [drone_id, command] : planned_commands) {
        static_cast<void>(command);
        const auto client_iter = clients_by_drone.find(drone_id);
        if (client_iter == clients_by_drone.end()) {
            report.results.emplace(
                drone_id, LocalCommandResult(false, "drone '" + drone_id + "' not registered"));
            continue;
        }
        target_clients.emplace_back(drone_id, client_iter->second);
    }

    const auto dispatch = [&planned_commands, &context, &options](
                              const std::string& drone_id,
                              const std::shared_ptr<Client>& client) -> CommandResult {
        const auto command_iter = planned_commands.find(drone_id);
        if (command_iter == planned_commands.end()) {
            return LocalCommandResult(false, "no planned command for drone");
        }
        commands::CommandEnvelope envelope;
        envelope.context = context;
        envelope.context.drone_id = drone_id;
        envelope.command = command_iter->second;
        return options.verify ? client->SendCommandAndWait(envelope, options.wait_options)
                              : client->SendCommand(envelope);
    };

    std::unordered_map<std::string, CommandResult> command_results;
    if (options.synchronization == SwarmExecutionSynchronization::kParallelFanout) {
        command_results = RunCommandTasks(target_clients, options.fanout, dispatch);
    } else {
        const std::string execution_id = MakeSwarmExecutionId(context);
        std::unordered_map<std::string, TrajectoryPlan> plans_by_drone;
        std::vector<std::pair<std::string, std::shared_ptr<Client>>> protocol_clients;
        plans_by_drone.reserve(target_clients.size());
        protocol_clients.reserve(target_clients.size());

        for (const auto& [drone_id, client] : target_clients) {
            const auto command_iter = planned_commands.find(drone_id);
            if (command_iter == planned_commands.end()) {
                command_results.emplace(drone_id,
                                        LocalCommandResult(false, "no planned command for drone"));
                continue;
            }
            auto plan = BuildTimedCommandPlan(drone_id, command_iter->second, context, options,
                                              execution_id);
            if (!plan.has_value()) {
                command_results.emplace(drone_id, std::move(plan.error()));
                continue;
            }
            SwarmDroneReadiness readiness;
            readiness.registered = true;
            readiness.message = "registered";
            report.readiness.emplace(drone_id, readiness);
            plans_by_drone.emplace(drone_id, std::move(*plan));
            protocol_clients.emplace_back(drone_id, client);
        }

        if (!protocol_clients.empty()) {
            report.upload_results = RunExecutionTasks(
                protocol_clients, options.fanout,
                [&plans_by_drone, &context](const std::string& drone_id,
                                            const std::shared_ptr<Client>& client) {
                    commands::CommandContext drone_context = context;
                    drone_context.drone_id = drone_id;
                    return client->UploadTrajectory(plans_by_drone.at(drone_id), drone_context);
                });

            std::unordered_set<std::string> uploaded_ids;
            for (const auto& [drone_id, upload] : report.upload_results) {
                auto& readiness = report.readiness[drone_id];
                readiness.registered = true;
                readiness.uploaded = upload.ok;
                readiness.handle = upload.handle;
                readiness.error = upload.error;
                if (!upload.ok) {
                    readiness.message = upload.message;
                    command_results.emplace(
                        drone_id,
                        CommandResultFromExecution(upload, "synchronized execution uploaded"));
                    continue;
                }
                readiness.message = "uploaded";
                uploaded_ids.insert(drone_id);
            }

            const auto uploaded_clients = FilterClients(snapshot, uploaded_ids);
            report.prepare_results = RunExecutionTasks(
                uploaded_clients, options.fanout,
                [&execution_id, &context](const std::string& drone_id,
                                          const std::shared_ptr<Client>& client) {
                    commands::CommandContext drone_context = context;
                    drone_context.drone_id = drone_id;
                    return client->PrepareTrajectory(drone_id, execution_id, drone_context);
                });

            std::unordered_set<std::string> prepared_ids;
            for (const auto& [drone_id, prepare] : report.prepare_results) {
                auto& readiness = report.readiness[drone_id];
                readiness.prepared = prepare.ok && prepare.handle.state == ExecutionState::kReady;
                readiness.handle = prepare.handle;
                readiness.error = prepare.error;
                if (!readiness.prepared) {
                    readiness.message = prepare.message.empty()
                                            ? "trajectory did not reach ready state"
                                            : prepare.message;
                    command_results.emplace(
                        drone_id,
                        CommandResultFromExecution(prepare, "synchronized execution prepared"));
                    continue;
                }
                readiness.message = "prepared";
                prepared_ids.insert(drone_id);
            }

            const auto prepared_clients = FilterClients(snapshot, prepared_ids);
            std::unordered_set<std::string> ready_ids;
            for (const auto& [drone_id, client] : prepared_clients) {
                TimeSyncState sync = client->GetTimeSyncState(drone_id);
                report.time_sync_states.emplace(drone_id, sync);
                auto& readiness = report.readiness[drone_id];
                readiness.time_sync = sync;
                std::string sync_detail;
                readiness.time_sync_ok = TimeSyncAcceptable(sync, options, &sync_detail);
                if (!readiness.time_sync_ok) {
                    readiness.message = sync_detail;
                    readiness.error = MakeLocalError(
                        core::ErrorDomain::kSwarm, RpcStatusCode::kFailedPrecondition, sync_detail);
                    command_results.emplace(
                        drone_id, LocalSwarmError(RpcStatusCode::kFailedPrecondition, sync_detail));
                    continue;
                }
                readiness.ready = true;
                readiness.message = "ready for synchronized start";
                ready_ids.insert(drone_id);
            }

            const std::size_t quorum =
                options.quorum > 0 ? options.quorum : (report.requested / 2U) + 1U;
            const bool existing_failure =
                std::ranges::any_of(command_results,
                                    [](const auto& entry) { return !entry.second.ok; }) ||
                std::ranges::any_of(report.results,
                                    [](const auto& entry) { return !entry.second.ok; });

            bool may_start = !ready_ids.empty();
            if (options.partial_failure_policy == SwarmPartialFailurePolicy::kAllOrAbort) {
                may_start = !existing_failure && ready_ids.size() == protocol_clients.size();
            } else if (options.partial_failure_policy == SwarmPartialFailurePolicy::kQuorum) {
                may_start = ready_ids.size() >= quorum;
            }

            if (!may_start) {
                const auto uploaded_for_cleanup = FilterClients(snapshot, uploaded_ids);
                report.abort_results =
                    RunExecutionTasks(uploaded_for_cleanup, options.fanout,
                                      [&execution_id](const std::string& drone_id,
                                                      const std::shared_ptr<Client>& client) {
                                          return client->AbortExecution(drone_id, execution_id);
                                      });
                for (const std::string& drone_id : ready_ids) {
                    command_results.emplace(
                        drone_id, LocalSwarmError(RpcStatusCode::kRejected,
                                                  "synchronized execution aborted before start "
                                                  "because readiness policy was not satisfied"));
                }
            } else {
                report.scheduled_start_unix_ms = NowUnixMs() + options.start_delay.count();
                const auto ready_clients = FilterClients(snapshot, ready_ids);
                report.start_results = RunExecutionTasks(
                    ready_clients, options.fanout,
                    [&execution_id, &context, &report](const std::string& drone_id,
                                                       const std::shared_ptr<Client>& client) {
                        commands::CommandContext drone_context = context;
                        drone_context.drone_id = drone_id;
                        return client->StartExecutionAt(
                            drone_id, execution_id, report.scheduled_start_unix_ms, drone_context);
                    });
                for (const auto& [drone_id, start] : report.start_results) {
                    auto& readiness = report.readiness[drone_id];
                    readiness.handle = start.handle;
                    readiness.error = start.error;
                    readiness.ready = readiness.ready && start.ok;
                    readiness.message = start.ok ? "scheduled synchronized start" : start.message;
                    command_results.emplace(
                        drone_id,
                        CommandResultFromExecution(start, "synchronized execution scheduled"));
                }
            }
        }
    }

    for (auto& [drone_id, result] : command_results) {
        report.results.emplace(drone_id, std::move(result));
    }

    std::unordered_set<std::string> failed_ids;
    std::unordered_set<std::string> succeeded_ids;
    for (const auto& [drone_id, result] : report.results) {
        if (result.ok) {
            succeeded_ids.insert(drone_id);
        } else {
            failed_ids.insert(drone_id);
        }
    }

    auto send_recovery = [&](const std::unordered_set<std::string>& target_ids,
                             const commands::Command& recovery_command) {
        const auto recovery_clients = FilterClients(snapshot, target_ids);
        auto recovery =
            RunCommandTasks(recovery_clients, options.fanout,
                            [&context, &recovery_command](const std::string& drone_id,
                                                          const std::shared_ptr<Client>& client) {
                                commands::CommandEnvelope envelope;
                                envelope.context = context;
                                envelope.context.drone_id = drone_id;
                                envelope.command = recovery_command;
                                return client->SendCommand(envelope);
                            });
        for (auto& [drone_id, result] : recovery) {
            report.recovery_results.emplace(drone_id, std::move(result));
        }
    };

    if (!failed_ids.empty()) {
        if (options.partial_failure_policy == SwarmPartialFailurePolicy::kAllOrAbort) {
            send_recovery(succeeded_ids, commands::NavCmd{commands::CmdHoldPosition{}});
        } else if (options.partial_failure_policy == SwarmPartialFailurePolicy::kLandFailed) {
            send_recovery(failed_ids, commands::FlightCmd{commands::CmdLand{}});
        } else if (options.partial_failure_policy == SwarmPartialFailurePolicy::kHoldFailed) {
            send_recovery(failed_ids, commands::NavCmd{commands::CmdHoldPosition{}});
        }
    }

    report.succeeded = CountSucceeded(report.results);
    report.failed = report.requested - report.succeeded;
    const std::size_t quorum = options.quorum > 0 ? options.quorum : (report.requested / 2U) + 1U;
    const auto recovery_succeeded = [&clients_by_drone,
                                     &report](const std::unordered_set<std::string>& target_ids) {
        for (const std::string& drone_id : target_ids) {
            if (!clients_by_drone.contains(drone_id)) {
                return false;
            }
            const auto recovery_iter = report.recovery_results.find(drone_id);
            if (recovery_iter == report.recovery_results.end() || !recovery_iter->second.ok) {
                return false;
            }
        }
        return true;
    };
    switch (options.partial_failure_policy) {
        case SwarmPartialFailurePolicy::kAllOrAbort:
            report.ok = report.failed == 0;
            break;
        case SwarmPartialFailurePolicy::kBestEffort:
            report.ok = report.succeeded > 0;
            break;
        case SwarmPartialFailurePolicy::kQuorum:
            report.ok = report.succeeded >= quorum;
            break;
        case SwarmPartialFailurePolicy::kLandFailed:
        case SwarmPartialFailurePolicy::kHoldFailed:
            report.ok =
                report.failed == 0 || (report.succeeded > 0 && recovery_succeeded(failed_ids));
            break;
    }

    std::ostringstream message;
    message << "swarm execution requested=" << report.requested << " succeeded=" << report.succeeded
            << " failed=" << report.failed;
    if (report.scheduled_start_unix_ms > 0) {
        message << " scheduled_start_unix_ms=" << report.scheduled_start_unix_ms;
    }
    if (options.partial_failure_policy == SwarmPartialFailurePolicy::kQuorum) {
        message << " quorum=" << quorum;
    }
    report.message = message.str();
    report.error =
        MakeLocalError(core::ErrorDomain::kSwarm,
                       report.ok ? RpcStatusCode::kOk : RpcStatusCode::kRejected, report.message);
    return report;
}

GoalResult SwarmClient::SetActiveGoal(ActiveGoal goal) const {
    if (goal.drone_id.empty()) {
        GoalResult out;
        out.ok = false;
        out.message = "goal.drone_id is required";
        out.error = MakeLocalError(core::ErrorDomain::kSwarm, RpcStatusCode::kInvalidArgument,
                                   out.message);
        return out;
    }

    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(goal.drone_id);
        if (iter == impl_->clients.end()) {
            return UnregisteredGoalResult(goal.drone_id);
        }
        client = iter->second;
    }
    return client->SetActiveGoal(goal);
}

std::unordered_map<std::string, GoalResult> SwarmClient::SetActiveGoals(
    const std::unordered_map<std::string, ActiveGoal>& goals,
    const SwarmFanoutOptions& fanout_options) const {
    if (goals.empty()) {
        return {};
    }

    const auto snapshot = impl_->Snapshot();
    std::unordered_map<std::string, std::shared_ptr<Client>> clients_by_drone;
    clients_by_drone.reserve(snapshot.size());
    for (const auto& [drone_id, client] : snapshot) {
        clients_by_drone.emplace(drone_id, client);
    }

    std::unordered_map<std::string, GoalResult> results;
    std::vector<std::pair<std::string, std::shared_ptr<Client>>> target_clients;
    target_clients.reserve(goals.size());
    for (const auto& [drone_id, goal] : goals) {
        static_cast<void>(goal);
        const auto client_iter = clients_by_drone.find(drone_id);
        if (client_iter == clients_by_drone.end()) {
            results.emplace(drone_id, UnregisteredGoalResult(drone_id));
            continue;
        }
        target_clients.emplace_back(drone_id, client_iter->second);
    }

    auto goal_results = RunGoalTasks(
        target_clients, fanout_options,
        [&goals](const std::string& drone_id, const std::shared_ptr<Client>& client) {
            ActiveGoal goal = goals.at(drone_id);
            goal.drone_id = drone_id;
            return client->SetActiveGoal(goal);
        });
    for (auto& [drone_id, result] : goal_results) {
        results.emplace(drone_id, std::move(result));
    }
    return results;
}

ExecutionResult SwarmClient::UploadTrajectory(const TrajectoryPlan& plan) const {
    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(plan.drone_id);
        if (iter == impl_->clients.end()) {
            return UnregisteredExecutionResult(plan.drone_id);
        }
        client = iter->second;
    }
    return client->UploadTrajectory(plan);
}

std::unordered_map<std::string, ExecutionResult> SwarmClient::UploadTrajectories(
    const std::vector<TrajectoryPlan>& plans, const SwarmFanoutOptions& fanout_options) const {
    std::unordered_map<std::string, TrajectoryPlan> by_drone;
    for (const auto& plan : plans) {
        by_drone.emplace(plan.drone_id, plan);
    }
    const auto snapshot = impl_->Snapshot();
    return RunExecutionTasks(
        snapshot, fanout_options,
        [&by_drone](const std::string& drone_id, const std::shared_ptr<Client>& client) {
            const auto iter = by_drone.find(drone_id);
            if (iter == by_drone.end()) {
                ExecutionResult out;
                out.ok = false;
                out.message = "no trajectory plan supplied for drone";
                out.error = MakeLocalError(core::ErrorDomain::kValidation,
                                           RpcStatusCode::kInvalidArgument, out.message);
                return out;
            }
            return client->UploadTrajectory(iter->second);
        });
}

std::unordered_map<std::string, ExecutionResult> SwarmClient::ValidateTrajectories(
    const std::vector<TrajectoryPlan>& plans, const SwarmFanoutOptions& fanout_options) const {
    std::unordered_map<std::string, TrajectoryPlan> by_drone;
    for (const auto& plan : plans) {
        by_drone.emplace(plan.drone_id, plan);
    }
    const auto snapshot = impl_->Snapshot();
    return RunExecutionTasks(
        snapshot, fanout_options,
        [&by_drone](const std::string& drone_id, const std::shared_ptr<Client>& client) {
            const auto iter = by_drone.find(drone_id);
            if (iter == by_drone.end()) {
                ExecutionResult out;
                out.ok = false;
                out.message = "no trajectory plan supplied for drone";
                out.error = MakeLocalError(core::ErrorDomain::kValidation,
                                           RpcStatusCode::kInvalidArgument, out.message);
                return out;
            }
            return client->ValidateTrajectory(iter->second);
        });
}

std::unordered_map<std::string, ExecutionResult> SwarmClient::PrepareAll(
    const std::string& execution_id, const SwarmFanoutOptions& fanout_options) const {
    return RunExecutionTasks(
        impl_->Snapshot(), fanout_options,
        [&execution_id](const std::string& drone_id, const std::shared_ptr<Client>& client) {
            return client->PrepareTrajectory(drone_id, execution_id);
        });
}

std::unordered_map<std::string, ExecutionResult> SwarmClient::StartAllAt(
    const std::string& execution_id, std::int64_t unix_time_ms,
    const SwarmFanoutOptions& fanout_options) const {
    return RunExecutionTasks(impl_->Snapshot(), fanout_options,
                             [&execution_id, unix_time_ms](const std::string& drone_id,
                                                           const std::shared_ptr<Client>& client) {
                                 return client->StartExecutionAt(drone_id, execution_id,
                                                                 unix_time_ms);
                             });
}

std::unordered_map<std::string, ExecutionResult> SwarmClient::AbortAll(
    const std::string& execution_id, const SwarmFanoutOptions& fanout_options) const {
    return RunExecutionTasks(
        impl_->Snapshot(), fanout_options,
        [&execution_id](const std::string& drone_id, const std::shared_ptr<Client>& client) {
            return client->AbortExecution(drone_id, execution_id);
        });
}

std::unordered_map<std::string, std::vector<ExecutionHandle>> SwarmClient::ListAllExecutions(
    const SwarmFanoutOptions& fanout_options) const {
    return RunExecutionListTasks(
        impl_->Snapshot(), fanout_options,
        [](const std::string& drone_id, const std::shared_ptr<Client>& client) {
            return client->ListExecutions(drone_id);
        });
}

SubscriptionResult SwarmClient::StartTelemetry(TelemetrySubscription subscription,
                                               TelemetryHandler on_frame,
                                               TelemetryErrorHandler on_error,
                                               SubscriptionEventHandler on_event,
                                               SubscriptionOptions options) {
    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(subscription.drone_id);
        if (iter == impl_->clients.end()) {
            return std::unexpected(
                MakeLocalError(core::ErrorDomain::kSwarm, RpcStatusCode::kNotFound,
                               "drone '" + subscription.drone_id + "' is not registered"));
        }
        client = iter->second;
    }
    return client->StartTelemetry(std::move(subscription), std::move(on_frame), std::move(on_error),
                                  std::move(on_event), options);
}

void SwarmClient::StopTelemetry(const std::string& drone_id) {
    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(drone_id);
        if (iter == impl_->clients.end()) {
            return;
        }
        client = iter->second;
    }
    client->StopTelemetry();
}

SwarmSubscriptionResults SwarmClient::StartAllTelemetry(int rate_hertz,
                                                        const TelemetryHandler& on_frame,
                                                        const TelemetryErrorHandler& on_error,
                                                        const SubscriptionEventHandler& on_event,
                                                        SubscriptionOptions options) {
    SwarmSubscriptionResults results;
    const auto clients = impl_->Snapshot();
    results.reserve(clients.size());
    for (const auto& [drone_id, client] : clients) {
        results.emplace(drone_id,
                        client->StartTelemetry({.drone_id = drone_id, .rate_hertz = rate_hertz},
                                               on_frame, on_error, on_event, options));
    }
    return results;
}

void SwarmClient::StopAllTelemetry() {
    for (const auto& [drone_id, client] : impl_->Snapshot()) {
        client->StopTelemetry();
    }
}

SwarmSubscriptionResults SwarmClient::StartAllReports(const AgentReportHandler& on_report,
                                                      const TelemetryErrorHandler& on_error,
                                                      std::uint64_t after_sequence,
                                                      const SubscriptionEventHandler& on_event,
                                                      SubscriptionOptions options) {
    SwarmSubscriptionResults results;
    const auto clients = impl_->Snapshot();
    results.reserve(clients.size());
    for (const auto& [drone_id, client] : clients) {
        results.emplace(
            drone_id, client->StartReports({.drone_id = drone_id, .after_sequence = after_sequence},
                                           on_report, on_error, on_event, options));
    }
    return results;
}

void SwarmClient::StopAllReports() {
    for (const auto& [drone_id, client] : impl_->Snapshot()) {
        client->StopReports();
    }
}

CommandResult SwarmClient::LockDrone(const std::string& drone_id, std::int64_t ttl_ms) const {
    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(drone_id);
        if (iter == impl_->clients.end()) {
            return UnregisteredCommandResult(drone_id);
        }
        client = iter->second;
    }
    return client->LockAuthority(drone_id, ttl_ms);
}

ReleaseAuthorityResult SwarmClient::UnlockDrone(const std::string& drone_id) const {
    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(drone_id);
        if (iter == impl_->clients.end()) {
            return UnregisteredReleaseAuthorityResult(drone_id);
        }
        client = iter->second;
    }
    return client->ReleaseAuthority(drone_id);
}

std::unordered_map<std::string, CommandResult> SwarmClient::LockAll(
    std::int64_t ttl_ms, const SwarmFanoutOptions& fanout_options) const {
    const auto kSnapshot = impl_->Snapshot();
    return RunCommandTasks(
        kSnapshot, fanout_options,
        [ttl_ms](const std::string& drone_id, const std::shared_ptr<Client>& client) {
            return client->LockAuthority(drone_id, ttl_ms);
        });
}

std::unordered_map<std::string, ReleaseAuthorityResult> SwarmClient::UnlockAll(
    const SwarmFanoutOptions& fanout_options) const {
    const auto kSnapshot = impl_->Snapshot();
    return RunReleaseAuthorityTasks(
        kSnapshot, fanout_options,
        [](const std::string& drone_id, const std::shared_ptr<Client>& client) {
            return client->ReleaseAuthority(drone_id);
        });
}

}  // namespace swarmkit::client
