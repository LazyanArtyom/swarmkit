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

[[nodiscard]] GoalResult UnregisteredGoalResult(const std::string& drone_id) {
    GoalResult out;
    out.ok = false;
    out.message = "drone '" + drone_id + "' not registered";
    out.error = MakeLocalError(core::ErrorDomain::kSwarm, RpcStatusCode::kNotFound, out.message);
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
[[nodiscard]] std::unordered_map<std::string, GoalResult> RunGoalTasks(
    const std::vector<std::pair<std::string, std::shared_ptr<Client>>>& clients,
    const SwarmFanoutOptions& fanout_options, TaskFn&& task_fn) {
    return RunBoundedClientTasks<GoalResult>(clients, fanout_options, std::forward<TaskFn>(task_fn),
                                             CancelledGoalResult, ExceptionGoalResult,
                                             [](const GoalResult& result) { return result.ok; });
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

GoalResult SwarmClient::SetActiveGoal(const ActiveGoal& goal) const {
    if (goal.drone_id.empty()) {
        GoalResult out;
        out.ok = false;
        out.message = "goal.drone_id is required";
        out.error =
            MakeLocalError(core::ErrorDomain::kSwarm, RpcStatusCode::kInvalidArgument, out.message);
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

    auto goal_results =
        RunGoalTasks(target_clients, fanout_options,
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
