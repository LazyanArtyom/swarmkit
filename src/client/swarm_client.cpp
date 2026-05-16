// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/client/swarm_client.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "swarmkit/core/logger.h"

namespace swarmkit::client {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)

namespace {

constexpr std::size_t kFallbackParallelism = 4;
constexpr double kEarthRadiusMeters = 6378137.0;
constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] std::size_t ComputeParallelism(std::size_t task_count) {
    if (task_count == 0) {
        return 0;
    }

    const std::size_t kHardwareThreads =
        std::max<std::size_t>(1, std::thread::hardware_concurrency());
    const std::size_t kUpperBound = std::max(kFallbackParallelism, kHardwareThreads);
    return std::min(task_count, kUpperBound);
}

template <typename TaskFn>
[[nodiscard]] std::unordered_map<std::string, CommandResult> RunCommandTasks(
    const std::vector<std::pair<std::string, std::shared_ptr<Client>>>& clients, TaskFn&& task_fn) {
    std::unordered_map<std::string, CommandResult> results;
    results.reserve(clients.size());
    if (clients.empty()) {
        return results;
    }

    const std::size_t kParallelism = ComputeParallelism(clients.size());
    std::atomic<std::size_t> next_index{0};
    std::mutex results_mutex;
    std::vector<std::thread> workers;
    workers.reserve(kParallelism);

    for (std::size_t index = 0; index < kParallelism; ++index) {
        workers.emplace_back([&clients, &next_index, &results, &results_mutex, &task_fn]() {
            while (true) {
                const std::size_t kTaskIndex = next_index.fetch_add(1, std::memory_order_relaxed);
                if (kTaskIndex >= clients.size()) {
                    return;
                }

                const auto& [drone_id, client] = clients[kTaskIndex];
                CommandResult result = task_fn(drone_id, client);

                std::lock_guard<std::mutex> lock(results_mutex);
                results.emplace(drone_id, std::move(result));
            }
        });
    }

    for (auto& worker : workers) {
        worker.join();
    }

    return results;
}

template <typename Result, typename TaskFn>
[[nodiscard]] std::unordered_map<std::string, Result> RunClientTasks(
    const std::vector<std::pair<std::string, std::shared_ptr<Client>>>& clients, TaskFn&& task_fn) {
    std::unordered_map<std::string, Result> results;
    results.reserve(clients.size());
    if (clients.empty()) {
        return results;
    }

    const std::size_t kParallelism = ComputeParallelism(clients.size());
    std::atomic<std::size_t> next_index{0};
    std::mutex results_mutex;
    std::vector<std::thread> workers;
    workers.reserve(kParallelism);

    for (std::size_t index = 0; index < kParallelism; ++index) {
        workers.emplace_back([&clients, &next_index, &results, &results_mutex, &task_fn]() {
            while (true) {
                const std::size_t task_index = next_index.fetch_add(1, std::memory_order_relaxed);
                if (task_index >= clients.size()) {
                    return;
                }
                const auto& [drone_id, client] = clients[task_index];
                Result result = task_fn(drone_id, client);
                std::lock_guard<std::mutex> lock(results_mutex);
                results.emplace(drone_id, std::move(result));
            }
        });
    }

    for (auto& worker : workers) {
        worker.join();
    }

    return results;
}

[[nodiscard]] ExecutionResult UnregisteredExecutionResult(const std::string& drone_id) {
    ExecutionResult out;
    out.ok = false;
    out.message = "drone '" + drone_id + "' not registered";
    out.error.code = RpcStatusCode::kInvalidArgument;
    out.error.user_message = out.message;
    return out;
}

[[nodiscard]] CommandResult LocalCommandResult(bool ok, std::string message) {
    CommandResult out;
    out.ok = ok;
    out.message = std::move(message);
    out.error.code = ok ? RpcStatusCode::kOk : RpcStatusCode::kRejected;
    out.error.user_message = out.message;
    return out;
}

[[nodiscard]] bool IsSwarmCommand(const commands::Command& command) {
    return std::holds_alternative<commands::SwarmCmd>(command);
}

[[nodiscard]] double DegreesToRadians(double degrees) {
    return degrees * kPi / 180.0;
}

[[nodiscard]] double RadiansToDegrees(double radians) {
    return radians * 180.0 / kPi;
}

[[nodiscard]] GeoPoint OffsetGlobalPosition(const SwarmFormationAnchor& anchor, double north_m,
                                            double east_m, double up_m) {
    const double lat_rad = DegreesToRadians(anchor.lat_deg);
    const double cos_lat = std::max(0.000001, std::cos(lat_rad));
    GeoPoint point;
    point.lat_deg = anchor.lat_deg + RadiansToDegrees(north_m / kEarthRadiusMeters);
    point.lon_deg = anchor.lon_deg + RadiansToDegrees(east_m / (kEarthRadiusMeters * cos_lat));
    point.alt_m = anchor.alt_m + up_m;
    return point;
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
    std::unordered_map<std::string, std::string> roles;
    std::unordered_map<std::string, SwarmFormationAssignment> formation_assignments;

    /// @brief Returns a snapshot of all drone_id / Client pairs.
    ///
    /// Shared ownership means a concurrent RemoveDrone() cannot destroy a
    /// Client while a broadcast or SubscribeAllTelemetry is still using it.
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
    if (IsSwarmCommand(envelope.command)) {
        return HandleSwarmCommand(envelope);
    }

    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(envelope.context.drone_id);
        if (iter == impl_->clients.end()) {
            return {.ok = false,
                    .message = "drone '" + envelope.context.drone_id + "' not registered"};
        }
        client = iter->second;
    }
    return client->SendCommand(envelope);
}

CommandResult SwarmClient::SendCommandAndWait(const commands::CommandEnvelope& envelope,
                                              const CommandWaitOptions& options) const {
    if (IsSwarmCommand(envelope.command)) {
        static_cast<void>(options);
        return HandleSwarmCommand(envelope);
    }

    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(envelope.context.drone_id);
        if (iter == impl_->clients.end()) {
            return {.ok = false,
                    .message = "drone '" + envelope.context.drone_id + "' not registered"};
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
            out.error.code = RpcStatusCode::kInvalidArgument;
            out.error.user_message = out.message;
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
            out.error.code = RpcStatusCode::kInvalidArgument;
            out.error.user_message = "drone '" + drone_id + "' not registered";
            return out;
        }
        client = iter->second;
    }
    return client->GetRuntimeStats();
}

std::unordered_map<std::string, CommandResult> SwarmClient::BroadcastCommand(
    const commands::Command& command, const commands::CommandContext& context) const {
    if (IsSwarmCommand(command)) {
        const auto kSnapshot = impl_->Snapshot();
        return RunCommandTasks(kSnapshot,
                               [this, &command, &context](const std::string& drone_id,
                                                          const std::shared_ptr<Client>& client) {
                                   static_cast<void>(client);
                                   commands::CommandEnvelope envelope;
                                   envelope.context = context;
                                   envelope.context.drone_id = drone_id;
                                   envelope.command = command;
                                   return HandleSwarmCommand(envelope);
                               });
    }

    const auto kSnapshot = impl_->Snapshot();
    return RunCommandTasks(kSnapshot, [&command, &context](const std::string& drone_id,
                                                           const std::shared_ptr<Client>& client) {
        commands::CommandEnvelope envelope;
        envelope.context = context;
        envelope.context.drone_id = drone_id;
        envelope.command = command;
        return client->SendCommand(envelope);
    });
}

std::unordered_map<std::string, CommandResult> SwarmClient::BroadcastCommandAndWait(
    const commands::Command& command, const commands::CommandContext& context,
    const CommandWaitOptions& options) const {
    if (IsSwarmCommand(command)) {
        const auto kSnapshot = impl_->Snapshot();
        return RunCommandTasks(kSnapshot,
                               [this, &command, &context](const std::string& drone_id,
                                                          const std::shared_ptr<Client>& client) {
                                   static_cast<void>(client);
                                   commands::CommandEnvelope envelope;
                                   envelope.context = context;
                                   envelope.context.drone_id = drone_id;
                                   envelope.command = command;
                                   return HandleSwarmCommand(envelope);
                               });
    }

    const auto kSnapshot = impl_->Snapshot();
    return RunCommandTasks(kSnapshot,
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

SwarmExecutionReport SwarmClient::ApplyFormation(const SwarmFormationPlan& plan,
                                                 const commands::CommandContext& context,
                                                 const SwarmExecutionOptions& options) const {
    SwarmExecutionReport report;
    if (plan.formation_id.empty()) {
        report.message = "formation_id is required";
        return report;
    }
    if (plan.slots.empty()) {
        report.message = "formation requires at least one slot";
        return report;
    }

    std::unordered_map<std::string, commands::Command> planned;
    planned.reserve(plan.slots.size());
    for (std::size_t index = 0; index < plan.slots.size(); ++index) {
        const SwarmFormationSlot& slot = plan.slots[index];
        if (slot.drone_id.empty()) {
            report.results.emplace(
                "", LocalCommandResult(false, "formation slot drone_id is required"));
            continue;
        }
        const GeoPoint target =
            OffsetGlobalPosition(plan.anchor, slot.north_m, slot.east_m, slot.up_m);
        const float speed = slot.speed_mps > 0.0F ? slot.speed_mps : plan.speed_mps;
        planned[slot.drone_id] = commands::NavCmd{commands::CmdGoto{
            .lat_deg = target.lat_deg,
            .lon_deg = target.lon_deg,
            .alt_m = target.alt_m,
            .speed_mps = speed,
            .yaw_deg = slot.yaw_deg,
            .use_yaw = slot.use_yaw,
        }};
        static_cast<void>(
            AssignFormationSlot(slot.drone_id, plan.formation_id, static_cast<int>(index)));
        if (!slot.role.empty()) {
            static_cast<void>(AssignRole(slot.drone_id, slot.role));
        }
    }

    SwarmExecutionReport execution_report = ExecutePlannedCommands(planned, context, options);
    for (auto& [drone_id, result] : report.results) {
        execution_report.results.emplace(std::move(drone_id), std::move(result));
    }
    execution_report.requested = execution_report.results.size();
    execution_report.succeeded = CountSucceeded(execution_report.results);
    execution_report.failed = execution_report.requested - execution_report.succeeded;
    if (!execution_report.ok && execution_report.message.empty()) {
        execution_report.message = "formation execution failed";
    }
    return execution_report;
}

CommandResult SwarmClient::AssignRole(const std::string& drone_id, std::string role) const {
    if (drone_id.empty()) {
        return LocalCommandResult(false, "drone_id is required");
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->clients.contains(drone_id)) {
        return LocalCommandResult(false, "drone '" + drone_id + "' not registered");
    }
    impl_->roles[drone_id] = std::move(role);
    return LocalCommandResult(true, "swarm role assigned");
}

CommandResult SwarmClient::AssignFormationSlot(const std::string& drone_id,
                                               std::string formation_id, int slot_index) const {
    if (drone_id.empty()) {
        return LocalCommandResult(false, "drone_id is required");
    }
    if (formation_id.empty()) {
        return LocalCommandResult(false, "formation_id is required");
    }
    if (slot_index < 0) {
        return LocalCommandResult(false, "slot_index must be non-negative");
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->clients.contains(drone_id)) {
        return LocalCommandResult(false, "drone '" + drone_id + "' not registered");
    }
    impl_->formation_assignments[drone_id] = {
        .formation_id = std::move(formation_id),
        .slot_index = slot_index,
    };
    return LocalCommandResult(true, "swarm formation slot assigned");
}

std::optional<std::string> SwarmClient::GetDroneRole(const std::string& drone_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto iter = impl_->roles.find(drone_id);
    if (iter == impl_->roles.end()) {
        return std::nullopt;
    }
    return iter->second;
}

std::optional<SwarmFormationAssignment> SwarmClient::GetFormationAssignment(
    const std::string& drone_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto iter = impl_->formation_assignments.find(drone_id);
    if (iter == impl_->formation_assignments.end()) {
        return std::nullopt;
    }
    return iter->second;
}

CommandResult SwarmClient::HandleSwarmCommand(const commands::CommandEnvelope& envelope) const {
    if (envelope.context.drone_id.empty()) {
        return LocalCommandResult(false, "swarm command requires context.drone_id");
    }
    const auto& swarm = std::get<commands::SwarmCmd>(envelope.command);
    if (const auto* role = std::get_if<commands::CmdSetRole>(&swarm); role != nullptr) {
        return AssignRole(envelope.context.drone_id, role->role);
    }
    if (const auto* formation = std::get_if<commands::CmdSetFormation>(&swarm);
        formation != nullptr) {
        return AssignFormationSlot(envelope.context.drone_id, formation->formation_id,
                                   formation->slot_index);
    }
    return LocalCommandResult(false, "unknown swarm command");
}

SwarmExecutionReport SwarmClient::ExecutePlannedCommands(
    const std::unordered_map<std::string, commands::Command>& planned_commands,
    const commands::CommandContext& context, const SwarmExecutionOptions& options) const {
    SwarmExecutionReport report;
    report.planned_commands = planned_commands;
    report.requested = planned_commands.size();
    if (planned_commands.empty()) {
        report.message = "no planned swarm commands";
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

    const auto dispatch = [this, &planned_commands, &context, &options](
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
        if (IsSwarmCommand(envelope.command)) {
            return HandleSwarmCommand(envelope);
        }
        return options.verify ? client->SendCommandAndWait(envelope, options.wait_options)
                              : client->SendCommand(envelope);
    };

    std::unordered_map<std::string, CommandResult> command_results;
    if (options.synchronization == SwarmExecutionSynchronization::kParallelFanout ||
        target_clients.size() <= 1) {
        command_results = RunCommandTasks(target_clients, dispatch);
    } else {
        command_results.reserve(target_clients.size());
        std::mutex result_mutex;
        std::mutex barrier_mutex;
        std::condition_variable barrier_cv;
        std::size_t ready_count = 0;
        bool release = false;
        const auto start_at = std::chrono::steady_clock::now() + options.start_delay;

        std::vector<std::thread> workers;
        workers.reserve(target_clients.size());
        for (const auto& [drone_id, client] : target_clients) {
            workers.emplace_back([&, drone_id, client]() {
                {
                    std::unique_lock<std::mutex> lock(barrier_mutex);
                    ++ready_count;
                    if (ready_count == target_clients.size()) {
                        release = true;
                        barrier_cv.notify_all();
                    } else {
                        barrier_cv.wait(lock, [&release]() { return release; });
                    }
                }
                if (options.start_delay.count() > 0) {
                    std::this_thread::sleep_until(start_at);
                }
                CommandResult result = dispatch(drone_id, client);
                std::lock_guard<std::mutex> lock(result_mutex);
                command_results.emplace(drone_id, std::move(result));
            });
        }
        for (auto& worker : workers) {
            worker.join();
        }
    }

    for (auto& [drone_id, result] : command_results) {
        report.results.emplace(std::move(drone_id), std::move(result));
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
        auto recovery = RunCommandTasks(
            recovery_clients, [&context, &recovery_command](const std::string& drone_id,
                                                            const std::shared_ptr<Client>& client) {
                commands::CommandEnvelope envelope;
                envelope.context = context;
                envelope.context.drone_id = drone_id;
                envelope.command = recovery_command;
                return client->SendCommand(envelope);
            });
        for (auto& [drone_id, result] : recovery) {
            report.recovery_results.emplace(std::move(drone_id), std::move(result));
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
    if (options.partial_failure_policy == SwarmPartialFailurePolicy::kQuorum) {
        message << " quorum=" << quorum;
    }
    report.message = message.str();
    return report;
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
    const std::vector<TrajectoryPlan>& plans) const {
    std::unordered_map<std::string, TrajectoryPlan> by_drone;
    for (const auto& plan : plans) {
        by_drone.emplace(plan.drone_id, plan);
    }
    const auto snapshot = impl_->Snapshot();
    return RunClientTasks<ExecutionResult>(
        snapshot, [&by_drone](const std::string& drone_id, const std::shared_ptr<Client>& client) {
            const auto iter = by_drone.find(drone_id);
            if (iter == by_drone.end()) {
                ExecutionResult out;
                out.ok = false;
                out.message = "no trajectory plan supplied for drone";
                out.error.code = RpcStatusCode::kInvalidArgument;
                out.error.user_message = out.message;
                return out;
            }
            return client->UploadTrajectory(iter->second);
        });
}

std::unordered_map<std::string, ExecutionResult> SwarmClient::ValidateTrajectories(
    const std::vector<TrajectoryPlan>& plans) const {
    std::unordered_map<std::string, TrajectoryPlan> by_drone;
    for (const auto& plan : plans) {
        by_drone.emplace(plan.drone_id, plan);
    }
    const auto snapshot = impl_->Snapshot();
    return RunClientTasks<ExecutionResult>(
        snapshot, [&by_drone](const std::string& drone_id, const std::shared_ptr<Client>& client) {
            const auto iter = by_drone.find(drone_id);
            if (iter == by_drone.end()) {
                ExecutionResult out;
                out.ok = false;
                out.message = "no trajectory plan supplied for drone";
                out.error.code = RpcStatusCode::kInvalidArgument;
                out.error.user_message = out.message;
                return out;
            }
            return client->ValidateTrajectory(iter->second);
        });
}

std::unordered_map<std::string, ExecutionResult> SwarmClient::PrepareAll(
    const std::string& execution_id) const {
    return RunClientTasks<ExecutionResult>(
        impl_->Snapshot(),
        [&execution_id](const std::string& drone_id, const std::shared_ptr<Client>& client) {
            return client->PrepareTrajectory(drone_id, execution_id);
        });
}

std::unordered_map<std::string, ExecutionResult> SwarmClient::StartAllAt(
    const std::string& execution_id, std::int64_t unix_time_ms) const {
    return RunClientTasks<ExecutionResult>(
        impl_->Snapshot(), [&execution_id, unix_time_ms](const std::string& drone_id,
                                                         const std::shared_ptr<Client>& client) {
            return client->StartExecutionAt(drone_id, execution_id, unix_time_ms);
        });
}

std::unordered_map<std::string, ExecutionResult> SwarmClient::AbortAll(
    const std::string& execution_id) const {
    return RunClientTasks<ExecutionResult>(
        impl_->Snapshot(),
        [&execution_id](const std::string& drone_id, const std::shared_ptr<Client>& client) {
            return client->AbortExecution(drone_id, execution_id);
        });
}

std::unordered_map<std::string, std::vector<ExecutionHandle>> SwarmClient::ListAllExecutions()
    const {
    return RunClientTasks<std::vector<ExecutionHandle>>(
        impl_->Snapshot(), [](const std::string& drone_id, const std::shared_ptr<Client>& client) {
            return client->ListExecutions(drone_id);
        });
}

void SwarmClient::SubscribeTelemetry(TelemetrySubscription subscription, TelemetryHandler on_frame,
                                     TelemetryErrorHandler on_error) {
    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(subscription.drone_id);
        if (iter == impl_->clients.end()) {
            core::Logger::WarnFmt("SwarmClient::SubscribeTelemetry: drone '{}' not registered",
                                  subscription.drone_id);
            return;
        }
        client = iter->second;
    }
    client->SubscribeTelemetry(std::move(subscription), std::move(on_frame), std::move(on_error));
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

void SwarmClient::SubscribeAllTelemetry(int rate_hertz, const TelemetryHandler& on_frame,
                                        const TelemetryErrorHandler& on_error) {
    for (const auto& [drone_id, client] : impl_->Snapshot()) {
        client->SubscribeTelemetry({.drone_id = drone_id, .rate_hertz = rate_hertz}, on_frame,
                                   on_error);
    }
}

void SwarmClient::StopAllTelemetry() {
    for (const auto& [drone_id, client] : impl_->Snapshot()) {
        client->StopTelemetry();
    }
}

void SwarmClient::SubscribeAllReports(const AgentReportHandler& on_report,
                                      const TelemetryErrorHandler& on_error,
                                      std::uint64_t after_sequence) {
    for (const auto& [drone_id, client] : impl_->Snapshot()) {
        client->SubscribeReports({.drone_id = drone_id, .after_sequence = after_sequence},
                                 on_report, on_error);
    }
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
            return {.ok = false, .message = "drone '" + drone_id + "' not registered"};
        }
        client = iter->second;
    }
    return client->LockAuthority(drone_id, ttl_ms);
}

void SwarmClient::UnlockDrone(const std::string& drone_id) const {
    std::shared_ptr<Client> client;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto iter = impl_->clients.find(drone_id);
        if (iter == impl_->clients.end()) {
            return;
        }
        client = iter->second;
    }
    client->ReleaseAuthority(drone_id);
}

std::unordered_map<std::string, CommandResult> SwarmClient::LockAll(std::int64_t ttl_ms) const {
    const auto kSnapshot = impl_->Snapshot();
    return RunCommandTasks(
        kSnapshot, [ttl_ms](const std::string& drone_id, const std::shared_ptr<Client>& client) {
            return client->LockAuthority(drone_id, ttl_ms);
        });
}

void SwarmClient::UnlockAll() const {
    for (const auto& [drone_id, client] : impl_->Snapshot()) {
        client->ReleaseAuthority(drone_id);
    }
}

}  // namespace swarmkit::client
