// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include "runtime_edge.h"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>

#include "common/arg_utils.h"
#include "options.h"
#include "output.h"
#include "runtime_common.h"

namespace swarmkit::apps::cli::internal {
namespace {

std::string_view PhaseName(edge::Phase phase) {
    switch (phase) {
        case edge::Phase::kMoving: return "moving";
        case edge::Phase::kDeviating: return "deviating";
        case edge::Phase::kRecovering: return "recovering";
        case edge::Phase::kArrived: return "arrived";
        case edge::Phase::kFailed: return "failed";
        case edge::Phase::kCancelled: return "cancelled";
        case edge::Phase::kUnknown: return "unknown";
    }
    return "unknown";
}

void PrintEdge(const client::AgentReport::EdgeReport& report) {
    std::cout << "edge=" << report.edge_id << " attempt=" << report.attempt_id
              << " phase=" << PhaseName(report.status.phase)
              << " at_b=" << report.status.at_b_verified
              << " traversal=" << report.status.traversal_verified
              << " survey=" << report.status.survey_verified
              << " covered_m=" << report.status.survey_covered_m;
    if (report.status.projected_progress_m)
        std::cout << " progress_m=" << *report.status.projected_progress_m;
    std::cout << " recovery_travel_bound_m=" << report.status.recovery_travel_bound_m
              << " missing=";
    for (const auto& gap : report.status.survey_missing)
        std::cout << '[' << gap.begin_m << ',' << gap.end_m << ']';
    std::cout << " reason=" << report.status.reason << '\n';
}

}  // namespace

int RunEdge(client::Client& client, int argc, char** argv) {
    const auto actions = FindActionsAfterCommand(argc, argv, "edge");
    if (actions.empty()) {
        std::cerr << "edge requires execute, status, or watch\n";
        return EXIT_FAILURE;
    }
    const std::string drone = common::GetOptionValue(argc, argv, "--drone", "default");
    const std::string attempt = common::GetOptionValue(argc, argv, "--attempt-id");
    if (actions.front() == "status") {
        if (attempt.empty()) {
            std::cerr << "edge status requires --attempt-id\n";
            return EXIT_FAILURE;
        }
        const auto result = client.GetEdgeStatus(drone, attempt);
        if (!result.ok || !result.status) {
            std::cerr << result.message << '\n';
            return EXIT_FAILURE;
        }
        PrintEdge(*result.status);
        return EXIT_SUCCESS;
    }
    if (actions.front() == "watch") {
        ResetStopRequested();
        std::signal(SIGINT, OnSignal);
        std::signal(SIGTERM, OnSignal);
        client::ReportSubscription subscription;
        subscription.drone_id = drone;
        auto stream =
            client.StartReports(subscription, [attempt](const client::AgentReport& report) {
                if (report.edge && (attempt.empty() || report.edge->attempt_id == attempt))
                    PrintReportText(report, std::cout);
            });
        if (!stream) {
            std::cerr << stream.error().user_message << '\n';
            return EXIT_FAILURE;
        }
        WaitForStop(ParseDurationMs(argc, argv).value_or(0));
        stream->Stop();
        return EXIT_SUCCESS;
    }
    if (actions.front() != "execute") {
        std::cerr << "unknown edge action\n";
        return EXIT_FAILURE;
    }
    const std::string mission = common::GetOptionValue(argc, argv, "--mission-id");
    const std::string edge_id = common::GetOptionValue(argc, argv, "--edge-id");
    if (mission.empty() || edge_id.empty() || attempt.empty()) {
        std::cerr << "edge execute requires --mission-id, --edge-id, --attempt-id\n";
        return EXIT_FAILURE;
    }
    const auto from_lat =
        ParseDoubleArg(common::GetOptionValue(argc, argv, "--from-lat"), "--from-lat");
    const auto from_lon =
        ParseDoubleArg(common::GetOptionValue(argc, argv, "--from-lon"), "--from-lon");
    const auto to_lat = ParseDoubleArg(common::GetOptionValue(argc, argv, "--to-lat"), "--to-lat");
    const auto to_lon = ParseDoubleArg(common::GetOptionValue(argc, argv, "--to-lon"), "--to-lon");
    const auto altitude = ParseDoubleArg(common::GetOptionValue(argc, argv, "--alt"), "--alt");
    const auto corridor =
        ParseDoubleArg(common::GetOptionValue(argc, argv, "--corridor-m", "4"), "--corridor-m");
    const auto arrival = ParseDoubleArg(common::GetOptionValue(argc, argv, "--accept-radius", "2"),
                                        "--accept-radius");
    const auto speed = ParseFloatArg(common::GetOptionValue(argc, argv, "--speed", "2"), "--speed");
    const auto revision =
        ParseIntArg(common::GetOptionValue(argc, argv, "--revision", "1"), "--revision");
    const auto timeout =
        ParseIntArg(common::GetOptionValue(argc, argv, "--timeout-ms", "120000"), "--timeout-ms");
    if (!from_lat || !from_lon || !to_lat || !to_lon || !altitude || !corridor || !arrival ||
        !speed || !revision || !timeout || *revision <= 0 || *timeout <= 0) {
        std::cerr << "invalid edge coordinates or numeric policy\n";
        return EXIT_FAILURE;
    }
    client::EdgeOperationRequest request;
    request.edge.mission_id = mission;
    request.edge.mission_revision = static_cast<std::uint64_t>(*revision);
    request.edge.edge_id = edge_id;
    request.edge.attempt_id = attempt;
    request.edge.drone_id = drone;
    request.edge.route.points = {{0, 0},
                                 edge::Wgs84ToLocal(*from_lat, *from_lon, *to_lat, *to_lon)};
    const std::string mode = common::GetOptionValue(argc, argv, "--obligation", "survey");
    if (mode == "arrival")
        request.edge.obligation = edge::Obligation::kArrival;
    else if (mode == "traversal")
        request.edge.obligation = edge::Obligation::kTraversal;
    else if (mode == "survey")
        request.edge.obligation = edge::Obligation::kSurvey;
    else if (mode == "traversal-survey")
        request.edge.obligation = edge::Obligation::kTraversalAndSurvey;
    else {
        std::cerr << "--obligation must be arrival|traversal|survey|traversal-survey\n";
        return EXIT_FAILURE;
    }
    request.edge.corridor_half_width_m = *corridor;
    request.edge.arrival_radius_m = *arrival;
    request.edge.arrival_max_speed_mps = 0.5;
    request.edge.maximum_missing_m = 0;
    request.edge.maximum_speed_mps = 0;  // Unknown until a hard physical bound is established.
    const auto hard_speed = ParseDoubleArg(
        common::GetOptionValue(argc, argv, "--hard-speed-bound", "0"), "--hard-speed-bound");
    const auto age = ParseIntArg(common::GetOptionValue(argc, argv, "--capture-age-ms", "500"),
                                 "--capture-age-ms");
    const auto wait = ParseIntArg(common::GetOptionValue(argc, argv, "--evidence-wait-ms", "500"),
                                  "--evidence-wait-ms");
    const auto retries = ParseIntArg(common::GetOptionValue(argc, argv, "--recovery-attempts", "0"),
                                     "--recovery-attempts");
    const auto recovery_duration = ParseIntArg(
        common::GetOptionValue(argc, argv, "--recovery-duration-ms", "30000"),
        "--recovery-duration-ms");
    const auto recovery_travel = ParseDoubleArg(
        common::GetOptionValue(argc, argv, "--recovery-travel-m", "200"),
        "--recovery-travel-m");
    if (!hard_speed || !age || !wait || !retries || *retries < 0 || !recovery_duration ||
        *recovery_duration <= 0 || !recovery_travel || *recovery_travel <= 0)
        return EXIT_FAILURE;
    request.edge.maximum_speed_mps = *hard_speed;
    request.edge.maximum_capture_age_ms = *age;
    request.edge.endpoint_evidence_wait_ms = *wait;
    request.edge.max_recovery_attempts = static_cast<std::uint32_t>(*retries);
    request.edge.max_recovery_duration_ms = *recovery_duration;
    request.edge.max_recovery_travel_m = *recovery_travel;
    request.edge.sustained_deviation_samples = 3;
    request.edge.recovery_cooldown_ms = 1000;
    request.edge.timeout_ms = *timeout;
    request.wgs84_origin = {*from_lat, *from_lon};
    request.altitude_m = *altitude;
    request.commanded_speed_mps = *speed;
    const auto result = client.ExecuteEdge(request);
    if (!result.ok || !result.status) {
        std::cerr << result.message << '\n';
        return EXIT_FAILURE;
    }
    PrintEdge(*result.status);
    return EXIT_SUCCESS;
}

}  // namespace swarmkit::apps::cli::internal
