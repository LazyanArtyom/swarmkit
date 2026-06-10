// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "runtime_authority.h"

#include "runtime_common.h"

namespace swarmkit::apps::cli::internal {

int RunLock(Client& client, std::string_view drone_id, int argc, char** argv) {
    const auto ttl_ms = ParseTtlMs(argc, argv);
    if (!ttl_ms.has_value()) {
        std::cerr << ttl_ms.error() << "\n";
        return EXIT_FAILURE;
    }
    const auto result = client.LockAuthority(std::string(drone_id), *ttl_ms);
    if (!result.ok) {
        std::cerr << "Lock FAILED: " << result.message << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "Lock OK" << (result.message.empty() ? "" : ": " + result.message) << "\n";
    return EXIT_SUCCESS;
}

int RunUnlock(Client& client, std::string_view drone_id) {
    const auto result = client.ReleaseAuthority(std::string(drone_id));
    return PrintReleaseAuthorityResult("Unlock", result) ? EXIT_SUCCESS : EXIT_FAILURE;
}

int RunWatchAuthority(Client& client, std::string_view drone_id, CommandPriority priority) {
    ResetStopRequested();
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    swarmkit::client::AuthoritySubscription subscription;
    subscription.drone_id = std::string(drone_id);
    subscription.priority = priority;

    auto authority_watch = client.StartAuthorityWatch(
        subscription,
        [](const swarmkit::client::AuthorityEventInfo& event) {
            std::cout << "authority event drone=" << event.drone_id
                      << " holder=" << event.holder_client_id
                      << " priority=" << static_cast<int>(event.holder_priority)
                      << " kind=" << static_cast<int>(event.kind) << "\n";
        },
        [](const std::string& error_msg) {
            std::cerr << "Authority stream error: " << error_msg << "\n";
        });
    if (!authority_watch.has_value()) {
        std::cerr << "Failed to start authority watch: " << authority_watch.error().user_message
                  << "\n";
        return EXIT_FAILURE;
    }

    while (!IsStopRequested()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kTelemetryPollIntervalMs));
    }
    authority_watch->Stop();
    std::cout << "\nStopped.\n";
    return EXIT_SUCCESS;
}

}  // namespace swarmkit::apps::cli::internal
