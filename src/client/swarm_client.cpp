// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/client/swarm_client.h"

#include <exception>
#include <mutex>
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

}  // namespace

/// @brief Holds per-drone Client instances and synchronises fleet-wide access.
struct SwarmClient::Impl {
    ClientConfig default_config;

    mutable std::mutex mutex;
    std::unordered_map<std::string, std::shared_ptr<Client>> clients;

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
            return {.ok = false,
                    .message = "drone '" + envelope.context.drone_id + "' not registered"};
        }
        client = iter->second;
    }
    return client->SendCommand(envelope);
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
