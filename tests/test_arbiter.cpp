#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <memory>
#include <string>

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

    const WatchToken operator_token = arbiter.Watch(
        "drone-1", "operator-client", commands::CommandPriority::kOperator, operator_queue);
    const WatchToken override_token = arbiter.Watch(
        "drone-1", "override-client", commands::CommandPriority::kOverride, override_queue);

    REQUIRE(arbiter
                .CheckAndGrant(
                    MakeContext("drone-1", "operator-client", commands::CommandPriority::kOperator),
                    std::chrono::seconds{5})
                .ok());

    AuthorityEvent event;
    REQUIRE(operator_queue->Pop(event, kEventTimeout));
    CHECK(event.kind == AuthorityEvent::Kind::kGranted);
    CHECK(event.holder_client_id == "operator-client");
    CHECK_FALSE(override_queue->Pop(event, kNoEventTimeout));

    REQUIRE(arbiter
                .CheckAndGrant(
                    MakeContext("drone-1", "override-client", commands::CommandPriority::kOverride),
                    std::chrono::seconds{5})
                .ok());

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

    arbiter.Unwatch(operator_token);
    arbiter.Unwatch(override_token);
    operator_queue->Shutdown();
    override_queue->Shutdown();
}

}  // namespace swarmkit::agent
