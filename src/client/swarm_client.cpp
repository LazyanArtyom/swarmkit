// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/client/swarm_client.h"

#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "swarmkit/core/logger.h"

namespace swarmkit::client {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)

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
    StopAllTelemetry();
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

std::unordered_map<std::string, CommandResult> SwarmClient::BroadcastCommand(
    const commands::Command& command, const commands::CommandContext& context) const {
    const auto kSnapshot = impl_->Snapshot();

    std::unordered_map<std::string, CommandResult> results;
    std::mutex results_mutex;
    std::vector<std::thread> threads;
    threads.reserve(kSnapshot.size());

    for (const auto& [entry_id, entry_client] : kSnapshot) {
        threads.emplace_back(
            [entry_id, entry_client, command, context, &results, &results_mutex]() {
                commands::CommandEnvelope envelope;
                envelope.context = context;
                envelope.context.drone_id = entry_id;
                envelope.command = command;

                CommandResult result = entry_client->SendCommand(envelope);

                std::lock_guard<std::mutex> lock(results_mutex);
                results[entry_id] = std::move(result);
            });
    }

    for (auto& thread : threads) {
        thread.join();
    }

    return results;
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

    std::unordered_map<std::string, CommandResult> results;
    std::mutex results_mutex;
    std::vector<std::thread> threads;
    threads.reserve(kSnapshot.size());

    for (const auto& [entry_id, entry_client] : kSnapshot) {
        threads.emplace_back([entry_id, entry_client, ttl_ms, &results, &results_mutex]() {
            CommandResult result = entry_client->LockAuthority(entry_id, ttl_ms);
            std::lock_guard<std::mutex> lock(results_mutex);
            results[entry_id] = std::move(result);
        });
    }

    for (auto& thread : threads) {
        thread.join();
    }

    return results;
}

void SwarmClient::UnlockAll() const {
    for (const auto& [drone_id, client] : impl_->Snapshot()) {
        client->ReleaseAuthority(drone_id);
    }
}

}  // namespace swarmkit::client
