#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "swarmkit/commands.h"
#include "swarmkit/core/result.h"

namespace swarmkit::agent {

using swarmkit::commands::CommandContext;
using swarmkit::commands::CommandPriority;

/// ---------------------------------------------------------------------------
/// AuthorityEvent -- delivered to watchers when authority changes on a drone.
/// ---------------------------------------------------------------------------

/**
 * @brief Notification delivered to clients watching command authority.
 *
 * Clients that registered via CommandArbiter::Watch() receive targeted events
 * describing changes that affect their own authority state on a drone.
 * Use the events to pause/resume a command loop without polling.
 */
struct AuthorityEvent {
    /// @brief Describes what happened to the authority state.
    enum class Kind : std::uint8_t {
        kGranted,    ///< This client was granted command authority.
        kPreempted,  ///< A higher-priority client took authority from this client.
        kResumed,    ///< A previously preempted client regained authority.
        kExpired,    ///< This client's TTL elapsed; authority was revoked.
    };

    Kind kind;
    std::string drone_id;
    std::string holder_client_id;  ///< Client that now holds authority.
    CommandPriority holder_priority;
};

/// ---------------------------------------------------------------------------
/// WatchToken -- opaque handle returned by CommandArbiter::Watch().
/// Pass to CommandArbiter::Unwatch() when the watcher disconnects.
/// ---------------------------------------------------------------------------

/// @brief Opaque handle for a registered authority watcher.
struct WatchToken {
    std::uint64_t watch_id{0};
};

/// ---------------------------------------------------------------------------
/// EventQueue -- thread-safe bounded queue used by Watch() subscribers.
///
/// The WatchAuthority RPC handler owns one queue per stream. The arbiter
/// pushes targeted events from whichever thread calls CheckAndGrant()
/// or Release(). The handler pops and writes to the gRPC stream without
/// holding any arbiter-internal lock, avoiding deadlock with the gRPC stream
/// mutex.
/// ---------------------------------------------------------------------------

/**
 * @brief Thread-safe, blocking event queue for authority notifications.
 *
 * Created by the caller and passed to CommandArbiter::Watch(). The arbiter
 * holds a @c std::weak_ptr so that destruction of the queue (when the RPC
 * stream ends) automatically removes the watcher.
 */
class EventQueue {
   public:
    /// @brief Push an event from any thread. Never blocks.
    void Push(AuthorityEvent event);

    /**
     * @brief Pop the next event, blocking up to @p timeout.
     * @returns true and writes to @p out if an event was available,
     *          false on timeout or after Shutdown().
     */
    [[nodiscard]] bool Pop(AuthorityEvent& out, std::chrono::milliseconds timeout);

    /// @brief Unblock any pending Pop() call permanently.
    void Shutdown();

   private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<AuthorityEvent> queue_;
    bool shutdown_{false};
};

/// ---------------------------------------------------------------------------
/// CommandArbiter -- priority-based command authority arbiter.
/// ---------------------------------------------------------------------------

/**
 * @brief Controls which client may issue commands to a drone at any given time.
 *
 * Every call to an IDroneBackend::Execute() passes through CheckAndGrant()
 * first. The arbiter grants authority to the client with the highest active
 * priority level and rejects all others. When a higher-priority client
 * preempts the current holder, the affected clients are notified
 * asynchronously so they can pause or resume their command loops.
 *
 * @par Priority levels
 * Defined in CommandPriority:
 *   - kOperator   (0)   — CLI, SDK users
 *   - kSupervisor (10)  — automated local systems
 *   - kOverride   (20)  — deviation-correction server
 *   - kEmergency  (100) — always executes, bypasses arbitration
 *
 * @par Threading
 * All public methods are thread-safe. Event callbacks are invoked without
 * holding the internal lock, so they may safely call back into the arbiter.
 *
 * @par TTL / automatic release
 * If @p ttl passed to CheckAndGrant() is non-zero, authority is
 * automatically released after that many milliseconds even if the client
 * never calls Release(). Expiry is evaluated lazily on the next arbiter
 * interaction for the drone.
 */
class CommandArbiter {
   public:
    CommandArbiter() = default;

    /**
     * @brief Check whether a command from @p context may execute and grant
     *        authority if so.
     *
     * - If no client holds authority for the drone, authority is granted.
     * - If the caller already holds authority, it is refreshed (TTL reset).
     * - If the caller's priority is strictly greater than the current holder's,
     *   the holder is preempted and the affected clients are notified.
     * - Otherwise the command is rejected.
     *
     * Commands at CommandPriority::kEmergency always succeed regardless of
     * who currently holds authority.
     *
     * @param context  Command metadata including client_id, drone_id, priority.
     * @param ttl      Optional authority TTL in milliseconds (0 = no expiry).
     * @returns Ok if the command may proceed, Rejected otherwise.
     */
    [[nodiscard]] core::Result CheckAndGrant(
        const CommandContext& context,
        std::chrono::milliseconds ttl = std::chrono::milliseconds{0});

    /**
     * @brief Explicitly release command authority held by @p client_id.
     *
     * If @p client_id does not currently hold authority for @p drone_id,
     * the suspended entry for that client is removed if present.
     * On success, the most recently preempted live client is resumed
     * and notified.
     *
     * @param drone_id  Target drone identifier.
     * @param client_id Client identifier that wishes to release authority.
     */
    void Release(const std::string& drone_id, const std::string& client_id);

    /**
     * @brief Register a watcher to receive authority events for @p drone_id.
     *
     * The arbiter stores a @c weak_ptr to @p queue; when the queue is
     * destroyed (e.g. when the RPC stream ends) the watcher is removed
     * automatically on the next notification cycle.
     *
     * @param drone_id   Drone to watch.
     * @param client_id  Identity of the registering client.
     * @param priority   Client's authority priority level.
     * @param queue      Thread-safe queue that receives events.
     * @returns WatchToken that must be passed to Unwatch() on disconnect.
     */
    [[nodiscard]] WatchToken Watch(const std::string& drone_id, const std::string& client_id,
                                   CommandPriority priority, std::shared_ptr<EventQueue> queue);

    /**
     * @brief Unregister a watcher identified by @p token.
     *
     * Safe to call from any thread, including from within an event callback.
     */
    void Unwatch(WatchToken token);

   private:
    struct PendingNotification {
        std::string target_client_id;
        AuthorityEvent::Kind kind;
        std::string drone_id;
        std::string holder_client_id;
        CommandPriority holder_priority;
    };

    struct WatcherEntry {
        std::uint64_t watch_id;
        std::string client_id;
        CommandPriority priority;
        std::weak_ptr<EventQueue> queue;
    };

    struct DroneState {
        /// @brief Current or suspended holder state.
        struct Holder {
            std::string client_id;
            CommandPriority priority;
            /// Absolute time after which authority expires; zero = no expiry.
            std::chrono::system_clock::time_point expiry;
        };

        std::optional<Holder> holder;
        std::vector<Holder> suspended_holders;
        std::vector<WatcherEntry> watchers;
    };

    /// @brief Notify matching watchers without holding the lock.
    void NotifyWatchers(std::vector<WatcherEntry> watchers,
                        const PendingNotification& notification);

    /// @brief Notify all pending notifications without holding the lock.
    void NotifyPending(std::vector<WatcherEntry> watchers,
                       const std::vector<PendingNotification>& notifications);

    /// @brief Evict expired authority holders and resume suspended holders.
    void EvictExpiredHolder(DroneState& state, std::string_view drone_id,
                            std::vector<PendingNotification>* notifications);

    /// @brief Resume the most recently preempted live holder if one exists.
    void ResumeSuspendedHolder(DroneState& state, std::string_view drone_id,
                               std::vector<PendingNotification>* notifications);

    std::mutex mutex_;
    std::unordered_map<std::string, DroneState> drone_states_;
    std::atomic<std::uint64_t> next_watch_id_{1};
};

}  // namespace swarmkit::agent
