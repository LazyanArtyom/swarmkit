// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "swarmkit/agent/arbiter.h"

namespace swarmkit::agent {
namespace {

constexpr auto kEventTimeout = std::chrono::milliseconds{300};
constexpr auto kNoEventTimeout = std::chrono::milliseconds{50};

[[nodiscard]] commands::CommandContext MakeContext(std::string drone_id, std::string client_id,
                                                   commands::CommandPriority priority) {
    commands::CommandContext context;
    context.drone_id = std::move(drone_id);
    context.client_id = std::move(client_id);
    context.priority = priority;
    return context;
}

}  // namespace

TEST_CASE("CommandArbiter sends targeted preempt and resume notifications", "[agent][arbiter]") {
    CommandArbiter arbiter;

    auto operator_queue = std::make_shared<EventQueue>();
    auto override_queue = std::make_shared<EventQueue>();

    const WatchToken kOperatorToken = arbiter.Watch(
        "drone-1", "operator-client", commands::CommandPriority::kOperator, operator_queue);
    const WatchToken kOverrideToken = arbiter.Watch(
        "drone-1", "override-client", commands::CommandPriority::kOverride, override_queue);

    REQUIRE(arbiter
                .CheckAndGrant(
                    MakeContext("drone-1", "operator-client", commands::CommandPriority::kOperator),
                    std::chrono::seconds{5})
                .IsOk());

    AuthorityEvent event;
    REQUIRE(operator_queue->Pop(event, kEventTimeout));
    CHECK(event.kind == AuthorityEvent::Kind::kGranted);
    CHECK(event.holder_client_id == "operator-client");
    CHECK_FALSE(override_queue->Pop(event, kNoEventTimeout));

    REQUIRE(arbiter
                .CheckAndGrant(
                    MakeContext("drone-1", "override-client", commands::CommandPriority::kOverride),
                    std::chrono::seconds{5})
                .IsOk());

    REQUIRE(operator_queue->Pop(event, kEventTimeout));
    CHECK(event.kind == AuthorityEvent::Kind::kPreempted);
    CHECK(event.holder_client_id == "override-client");

    REQUIRE(override_queue->Pop(event, kEventTimeout));
    CHECK(event.kind == AuthorityEvent::Kind::kGranted);
    CHECK(event.holder_client_id == "override-client");

    arbiter.Release("drone-1", "override-client");

    REQUIRE(operator_queue->Pop(event, kEventTimeout));
    CHECK(event.kind == AuthorityEvent::Kind::kResumed);
    CHECK(event.holder_client_id == "operator-client");
    CHECK_FALSE(override_queue->Pop(event, kNoEventTimeout));

    arbiter.Unwatch(kOperatorToken);
    arbiter.Unwatch(kOverrideToken);
    operator_queue->Shutdown();
    override_queue->Shutdown();
}

TEST_CASE("CommandArbiter rejects lower priority clients", "[agent][arbiter]") {
    CommandArbiter arbiter;
    REQUIRE(arbiter
                .CheckAndGrant(
                    MakeContext("drone-1", "override-client", commands::CommandPriority::kOverride),
                    std::chrono::seconds{5})
                .IsOk());

    const core::Result kResult = arbiter.CheckAndGrant(
        MakeContext("drone-1", "operator-client", commands::CommandPriority::kOperator),
        std::chrono::seconds{5});
    CHECK_FALSE(kResult.IsOk());
    CHECK(kResult.message.find("override-client") != std::string::npos);
}

TEST_CASE("CommandArbiter expires holders and resumes suspended clients", "[agent][arbiter]") {
    CommandArbiter arbiter;

    auto operator_queue = std::make_shared<EventQueue>();
    auto override_queue = std::make_shared<EventQueue>();

    const WatchToken kOperatorToken = arbiter.Watch(
        "drone-1", "operator-client", commands::CommandPriority::kOperator, operator_queue);
    const WatchToken kOverrideToken = arbiter.Watch(
        "drone-1", "override-client", commands::CommandPriority::kOverride, override_queue);

    REQUIRE(arbiter
                .CheckAndGrant(
                    MakeContext("drone-1", "operator-client", commands::CommandPriority::kOperator),
                    std::chrono::milliseconds{500})
                .IsOk());
    REQUIRE(arbiter
                .CheckAndGrant(
                    MakeContext("drone-1", "override-client", commands::CommandPriority::kOverride),
                    std::chrono::milliseconds{50})
                .IsOk());

    std::this_thread::sleep_for(std::chrono::milliseconds{80});

    REQUIRE(arbiter
                .CheckAndGrant(MakeContext("drone-1", "supervisor-client",
                                           commands::CommandPriority::kSupervisor),
                               std::chrono::milliseconds{500})
                .IsOk());

    AuthorityEvent event;
    REQUIRE(operator_queue->Pop(event, kEventTimeout));
    CHECK(event.kind == AuthorityEvent::Kind::kGranted);
    REQUIRE(operator_queue->Pop(event, kEventTimeout));
    CHECK(event.kind == AuthorityEvent::Kind::kPreempted);
    REQUIRE(operator_queue->Pop(event, kEventTimeout));
    CHECK(event.kind == AuthorityEvent::Kind::kResumed);

    REQUIRE(override_queue->Pop(event, kEventTimeout));
    CHECK(event.kind == AuthorityEvent::Kind::kGranted);
    REQUIRE(override_queue->Pop(event, kEventTimeout));
    CHECK(event.kind == AuthorityEvent::Kind::kExpired);

    arbiter.Unwatch(kOperatorToken);
    arbiter.Unwatch(kOverrideToken);
    operator_queue->Shutdown();
    override_queue->Shutdown();
}

TEST_CASE("CommandArbiter expires holders without requiring another arbiter call",
          "[agent][arbiter][expiry]") {
    CommandArbiter arbiter;
    auto queue = std::make_shared<EventQueue>();

    const WatchToken kToken = arbiter.Watch(
        "drone-1", "operator-client", commands::CommandPriority::kOperator, queue);

    REQUIRE(arbiter
                .CheckAndGrant(
                    MakeContext("drone-1", "operator-client", commands::CommandPriority::kOperator),
                    std::chrono::milliseconds{80})
                .IsOk());

    AuthorityEvent event;
    REQUIRE(queue->Pop(event, kEventTimeout));
    CHECK(event.kind == AuthorityEvent::Kind::kGranted);

    REQUIRE(queue->Pop(event, std::chrono::milliseconds{250}));
    CHECK(event.kind == AuthorityEvent::Kind::kExpired);
    CHECK(event.holder_client_id.empty());

    arbiter.Unwatch(kToken);
    queue->Shutdown();
}

}  // namespace swarmkit::agent
