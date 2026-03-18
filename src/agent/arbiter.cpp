#include "swarmkit/agent/arbiter.h"

#include <algorithm>
#include <chrono>
#include <utility>

#include "swarmkit/core/logger.h"

namespace swarmkit::agent {

// ---------------------------------------------------------------------------
// EventQueue
// ---------------------------------------------------------------------------

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
    const bool kReady = cv_.wait_for(lock, timeout, [this] {
        return !queue_.empty() || shutdown_;
    });
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

// ---------------------------------------------------------------------------
// CommandArbiter — helpers
// ---------------------------------------------------------------------------

void CommandArbiter::EvictExpiredHolder(DroneState&        state,
                                        const std::string& drone_id) {
    if (!state.holder.has_value()) {
        return;
    }
    const auto& expiry = state.holder->expiry;
    const bool  kHasExpiry =
        expiry != std::chrono::system_clock::time_point{};
    if (kHasExpiry && std::chrono::system_clock::now() >= expiry) {
        core::Logger::WarnFmt(
            "CommandArbiter: authority of '{}' on drone '{}' expired",
            state.holder->client_id, drone_id);

        const auto kExpiredClient   = state.holder->client_id;
        const auto kExpiredPriority = state.holder->priority;
        const auto kWatchers        = state.watchers;
        state.holder.reset();

        // Notify outside lock — release lock before calling NotifyWatchers.
        // (EvictExpiredHolder is always called while the lock is held by the
        //  caller, so we copy the watcher list and notify after returning.)
        // We store watchers in a local and the caller will notify.
        // This helper just mutates state; see CheckAndGrant / Release for the
        // notification path after acquiring watchers.
        (void)kExpiredClient;
        (void)kExpiredPriority;
        (void)kWatchers;
    }
}

void CommandArbiter::NotifyWatchers(std::vector<WatcherEntry> watchers,
                                    const std::string&         drone_id,
                                    AuthorityEvent::Kind        kind,
                                    const std::string&          holder_client_id,
                                    CommandPriority             holder_priority) {
    for (const auto& entry : watchers) {
        auto queue = entry.queue.lock();
        if (!queue) {
            continue;  // Queue destroyed; watcher already disconnected.
        }
        AuthorityEvent event;
        event.kind              = kind;
        event.drone_id          = drone_id;
        event.holder_client_id  = holder_client_id;
        event.holder_priority   = holder_priority;
        queue->Push(std::move(event));
    }
}

// ---------------------------------------------------------------------------
// CommandArbiter — CheckAndGrant
// ---------------------------------------------------------------------------

core::Result CommandArbiter::CheckAndGrant(const CommandContext&     context,
                                           std::chrono::milliseconds ttl) {
    // Emergency commands bypass the arbiter entirely.
    if (context.priority >= CommandPriority::kEmergency) {
        return core::Result::Ok();
    }

    std::vector<WatcherEntry> watchers_to_notify;
    AuthorityEvent::Kind      notify_kind{};
    bool                      should_notify = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& state = drone_states_[context.drone_id];

        // Evict expired holder before evaluating.
        if (state.holder.has_value()) {
            const auto& expiry = state.holder->expiry;
            const bool  kHasExpiry =
                expiry != std::chrono::system_clock::time_point{};
            if (kHasExpiry && std::chrono::system_clock::now() >= expiry) {
                core::Logger::WarnFmt(
                    "CommandArbiter: authority of '{}' on drone '{}' expired",
                    state.holder->client_id, context.drone_id);
                state.holder.reset();
                // Notify kExpired outside the lock below.
                watchers_to_notify = state.watchers;
                notify_kind        = AuthorityEvent::Kind::kExpired;
                should_notify      = true;
            }
        }

        if (!state.holder.has_value()) {
            // No current holder — grant authority.
            DroneState::Holder new_holder;
            new_holder.client_id = context.client_id;
            new_holder.priority  = context.priority;
            if (ttl.count() > 0) {
                new_holder.expiry =
                    std::chrono::system_clock::now() + ttl;
            }
            state.holder = std::move(new_holder);

            if (!should_notify) {
                watchers_to_notify = state.watchers;
                notify_kind        = AuthorityEvent::Kind::kGranted;
                should_notify      = true;
            }

            core::Logger::DebugFmt(
                "CommandArbiter: '{}' granted authority on drone '{}' (priority={})",
                context.client_id, context.drone_id,
                static_cast<int>(context.priority));
        } else if (state.holder->client_id == context.client_id) {
            // Same client — refresh TTL if requested.
            if (ttl.count() > 0) {
                state.holder->expiry =
                    std::chrono::system_clock::now() + ttl;
            }
        } else if (context.priority > state.holder->priority) {
            // Higher priority — preempt the current holder.
            const std::string kPrevClient   = state.holder->client_id;
            const CommandPriority kPrevPrio = state.holder->priority;

            DroneState::Holder new_holder;
            new_holder.client_id = context.client_id;
            new_holder.priority  = context.priority;
            if (ttl.count() > 0) {
                new_holder.expiry =
                    std::chrono::system_clock::now() + ttl;
            }
            state.holder = std::move(new_holder);

            core::Logger::InfoFmt(
                "CommandArbiter: '{}' (priority={}) preempted '{}' (priority={}) "
                "on drone '{}'",
                context.client_id, static_cast<int>(context.priority),
                kPrevClient, static_cast<int>(kPrevPrio),
                context.drone_id);

            watchers_to_notify = state.watchers;
            notify_kind        = AuthorityEvent::Kind::kPreempted;
            should_notify      = true;
        } else {
            // Lower or equal priority from a different client — reject.
            return core::Result::Rejected(
                "command authority held by '" + state.holder->client_id +
                "' at priority " +
                std::to_string(static_cast<int>(state.holder->priority)));
        }
    }  // lock released

    if (should_notify) {
        NotifyWatchers(std::move(watchers_to_notify), context.drone_id,
                       notify_kind, context.client_id, context.priority);
    }

    return core::Result::Ok();
}

// ---------------------------------------------------------------------------
// CommandArbiter — Release
// ---------------------------------------------------------------------------

void CommandArbiter::Release(const std::string& drone_id,
                             const std::string& client_id) {
    std::vector<WatcherEntry> watchers_to_notify;
    bool                      should_notify = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto iter = drone_states_.find(drone_id);
        if (iter == drone_states_.end()) {
            return;
        }
        auto& state = iter->second;
        if (!state.holder.has_value() ||
            state.holder->client_id != client_id) {
            return;
        }

        core::Logger::InfoFmt(
            "CommandArbiter: '{}' released authority on drone '{}'",
            client_id, drone_id);

        state.holder.reset();
        watchers_to_notify = state.watchers;
        should_notify      = true;
    }

    if (should_notify) {
        NotifyWatchers(std::move(watchers_to_notify), drone_id,
                       AuthorityEvent::Kind::kResumed, /*holder=*/"",
                       CommandPriority::kOperator);
    }
}

// ---------------------------------------------------------------------------
// CommandArbiter — Watch / Unwatch
// ---------------------------------------------------------------------------

WatchToken CommandArbiter::Watch(const std::string&          drone_id,
                                 const std::string&          client_id,
                                 CommandPriority             priority,
                                 std::shared_ptr<EventQueue> queue) {
    const std::uint64_t kWatchId =
        next_watch_id_.fetch_add(1, std::memory_order_relaxed);

    WatcherEntry entry;
    entry.watch_id  = kWatchId;
    entry.client_id = client_id;
    entry.priority  = priority;
    entry.queue     = queue;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        drone_states_[drone_id].watchers.push_back(std::move(entry));
    }

    core::Logger::DebugFmt(
        "CommandArbiter: '{}' watching drone '{}' (watch_id={})",
        client_id, drone_id, kWatchId);

    return WatchToken{kWatchId};
}

void CommandArbiter::Unwatch(WatchToken token) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [drone_id, state] : drone_states_) {
        auto& watchers = state.watchers;
        watchers.erase(
            std::remove_if(watchers.begin(), watchers.end(),
                           [&](const WatcherEntry& entry) {
                               return entry.watch_id == token.watch_id;
                           }),
            watchers.end());
    }
}

}  // namespace swarmkit::agent
