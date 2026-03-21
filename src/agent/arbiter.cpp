// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/agent/arbiter.h"

#include <utility>

#include "swarmkit/core/logger.h"

namespace swarmkit::agent {

using swarmkit::commands::CommandContext;
using swarmkit::commands::CommandPriority;

namespace {

[[nodiscard]] bool HasExpiry(const std::chrono::system_clock::time_point& expiry) {
    return expiry != std::chrono::system_clock::time_point{};
}

[[nodiscard]] bool IsExpired(const std::chrono::system_clock::time_point& expiry) {
    return HasExpiry(expiry) && std::chrono::system_clock::now() >= expiry;
}

}  // namespace

/// @name EventQueue
/// @{

void EventQueue::Push(AuthorityEvent event) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (shutdown_) {
            return;
        }
        queue_.push(std::move(event));
    }
    cv_.notify_one();
}

bool EventQueue::Pop(AuthorityEvent& out, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool kReady =
        cv_.wait_for(lock, timeout, [this] { return !queue_.empty() || shutdown_; });
    if (!kReady || queue_.empty()) {
        return false;
    }
    out = std::move(queue_.front());
    queue_.pop();
    return true;
}

void EventQueue::Shutdown() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
    }
    cv_.notify_all();
}

/// @}

/// @name CommandArbiter — helpers
/// @{

void CommandArbiter::NotifyWatchers(const std::vector<WatcherEntry>& watchers,
                                    const PendingNotification& notification) {
    for (const auto& entry : watchers) {
        if (entry.client_id != notification.target_client_id) {
            continue;
        }

        auto queue = entry.queue.lock();
        if (!queue) {
            continue;
        }

        AuthorityEvent event;
        event.kind = notification.kind;
        event.drone_id = notification.drone_id;
        event.holder_client_id = notification.holder_client_id;
        event.holder_priority = notification.holder_priority;
        queue->Push(std::move(event));
    }
}

void CommandArbiter::NotifyPending(const std::vector<WatcherEntry>& watchers,
                                   const std::vector<PendingNotification>& notifications) {
    for (const auto& notification : notifications) {
        NotifyWatchers(watchers, notification);
    }
}

void CommandArbiter::ResumeSuspendedHolder(DroneState& state, std::string_view drone_id,
                                           std::vector<PendingNotification>* notifications) {
    while (!state.suspended_holders.empty()) {
        DroneState::Holder resumed_holder = std::move(state.suspended_holders.back());
        state.suspended_holders.pop_back();

        if (IsExpired(resumed_holder.expiry)) {
            notifications->push_back(PendingNotification{
                .target_client_id = resumed_holder.client_id,
                .kind = AuthorityEvent::Kind::kExpired,
                .drone_id = std::string(drone_id),
                .holder_client_id = "",
                .holder_priority = resumed_holder.priority,
            });
            continue;
        }

        state.holder = resumed_holder;
        notifications->push_back(PendingNotification{
            .target_client_id = resumed_holder.client_id,
            .kind = AuthorityEvent::Kind::kResumed,
            .drone_id = std::string(drone_id),
            .holder_client_id = resumed_holder.client_id,
            .holder_priority = resumed_holder.priority,
        });
        return;
    }
}

void CommandArbiter::EvictExpiredHolder(DroneState& state, std::string_view drone_id,
                                        std::vector<PendingNotification>* notifications) {
    while (state.holder.has_value() && IsExpired(state.holder->expiry)) {
        const DroneState::Holder kExpiredHolder = *state.holder;
        core::Logger::WarnFmt("CommandArbiter: authority of '{}' on drone '{}' expired",
                              kExpiredHolder.client_id, drone_id);

        state.holder.reset();
        notifications->push_back(PendingNotification{
            .target_client_id = kExpiredHolder.client_id,
            .kind = AuthorityEvent::Kind::kExpired,
            .drone_id = std::string(drone_id),
            .holder_client_id = "",
            .holder_priority = kExpiredHolder.priority,
        });

        ResumeSuspendedHolder(state, drone_id, notifications);
    }

    std::erase_if(state.suspended_holders, [&](const DroneState::Holder& holder) {
        if (!IsExpired(holder.expiry)) {
            return false;
        }

        notifications->push_back(PendingNotification{
            .target_client_id = holder.client_id,
            .kind = AuthorityEvent::Kind::kExpired,
            .drone_id = std::string(drone_id),
            .holder_client_id = "",
            .holder_priority = holder.priority,
        });
        return true;
    });
}

/// @}

/// @name CommandArbiter — CheckAndGrant
/// @{

core::Result CommandArbiter::CheckAndGrant(const CommandContext& context,
                                           std::chrono::milliseconds ttl) {
    if (context.priority >= CommandPriority::kEmergency) {
        return core::Result::Success();
    }

    std::vector<WatcherEntry> watchers_to_notify;
    std::vector<PendingNotification> notifications;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& state = drone_states_[context.drone_id];

        EvictExpiredHolder(state, context.drone_id, &notifications);

        if (!state.holder.has_value()) {
            DroneState::Holder new_holder;
            new_holder.client_id = context.client_id;
            new_holder.priority = context.priority;
            if (ttl.count() > 0) {
                new_holder.expiry = std::chrono::system_clock::now() + ttl;
            }
            state.holder = std::move(new_holder);

            notifications.push_back(PendingNotification{
                .target_client_id = context.client_id,
                .kind = AuthorityEvent::Kind::kGranted,
                .drone_id = context.drone_id,
                .holder_client_id = context.client_id,
                .holder_priority = context.priority,
            });

            core::Logger::DebugFmt(
                "CommandArbiter: '{}' granted authority on drone '{}' (priority={})",
                context.client_id, context.drone_id, static_cast<int>(context.priority));
        } else if (state.holder->client_id == context.client_id) {
            const auto kZeroPoint = std::chrono::system_clock::time_point{};
            if (ttl.count() > 0 && state.holder->expiry != kZeroPoint) {
                state.holder->expiry = std::chrono::system_clock::now() + ttl;
            }
        } else if (context.priority > state.holder->priority) {
            const DroneState::Holder kPreviousHolder = *state.holder;

            DroneState::Holder new_holder;
            new_holder.client_id = context.client_id;
            new_holder.priority = context.priority;
            if (ttl.count() > 0) {
                new_holder.expiry = std::chrono::system_clock::now() + ttl;
            }

            state.suspended_holders.push_back(kPreviousHolder);
            state.holder = std::move(new_holder);

            notifications.push_back(PendingNotification{
                .target_client_id = kPreviousHolder.client_id,
                .kind = AuthorityEvent::Kind::kPreempted,
                .drone_id = context.drone_id,
                .holder_client_id = context.client_id,
                .holder_priority = context.priority,
            });
            notifications.push_back(PendingNotification{
                .target_client_id = context.client_id,
                .kind = AuthorityEvent::Kind::kGranted,
                .drone_id = context.drone_id,
                .holder_client_id = context.client_id,
                .holder_priority = context.priority,
            });

            core::Logger::InfoFmt(
                "CommandArbiter: '{}' (priority={}) preempted '{}' (priority={}) on drone '{}'",
                context.client_id, static_cast<int>(context.priority), kPreviousHolder.client_id,
                static_cast<int>(kPreviousHolder.priority), context.drone_id);
        } else {
            return core::Result::Rejected("command authority held by '" + state.holder->client_id +
                                          "' at priority " +
                                          std::to_string(static_cast<int>(state.holder->priority)));
        }

        watchers_to_notify = state.watchers;
    }

    NotifyPending(watchers_to_notify, notifications);
    return core::Result::Success();
}

/// @}

/// @name CommandArbiter — Release
/// @{

void CommandArbiter::Release(const std::string& drone_id, const std::string& client_id) {
    std::vector<WatcherEntry> watchers_to_notify;
    std::vector<PendingNotification> notifications;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto iter = drone_states_.find(drone_id);
        if (iter == drone_states_.end()) {
            return;
        }

        auto& state = iter->second;
        EvictExpiredHolder(state, drone_id, &notifications);

        if (state.holder.has_value() && state.holder->client_id == client_id) {
            core::Logger::InfoFmt("CommandArbiter: '{}' released authority on drone '{}'",
                                  client_id, drone_id);
            state.holder.reset();
            ResumeSuspendedHolder(state, drone_id, &notifications);
            watchers_to_notify = state.watchers;
        } else {
            const std::size_t kRemovedCount = std::erase_if(
                state.suspended_holders,
                [&](const DroneState::Holder& holder) { return holder.client_id == client_id; });
            if (kRemovedCount == 0U && notifications.empty()) {
                return;
            }
            watchers_to_notify = state.watchers;
        }
    }

    NotifyPending(watchers_to_notify, notifications);
}

/// @}

/// @name CommandArbiter — Watch / Unwatch
/// @{

WatchToken CommandArbiter::Watch(const std::string& drone_id, const std::string& client_id,
                                 CommandPriority priority,
                                 const std::shared_ptr<EventQueue>& queue) {
    const std::uint64_t kWatchId = next_watch_id_.fetch_add(1, std::memory_order_relaxed);

    WatcherEntry entry;
    entry.watch_id = kWatchId;
    entry.client_id = client_id;
    entry.priority = priority;
    entry.queue = queue;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        drone_states_[drone_id].watchers.push_back(std::move(entry));
    }

    core::Logger::DebugFmt("CommandArbiter: '{}' watching drone '{}' (watch_id={})", client_id,
                           drone_id, kWatchId);

    return WatchToken{kWatchId};
}

void CommandArbiter::Unwatch(WatchToken token) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [drone_id, state] : drone_states_) {
        std::erase_if(state.watchers,
                      [&](const WatcherEntry& entry) { return entry.watch_id == token.watch_id; });
    }
}

/// @}

}  // namespace swarmkit::agent
