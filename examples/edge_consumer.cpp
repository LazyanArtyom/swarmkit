// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
// Example of an external graph application consuming one edge obligation.
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "swarmkit/client/client.h"

int main(int argc, char** argv) {
    if (argc != 8) {
        std::cerr << "usage: edge-consumer 127.0.0.1:PORT DRONE FROM_LAT FROM_LON "
                     "TO_LAT TO_LON ALT_M\n";
        return EXIT_FAILURE;
    }
    swarmkit::client::ClientConfig config;
    config.address = argv[1];
    config.client_id = "edge-example";
    config.security.transport_security = swarmkit::core::TransportSecurityMode::kInsecure;
    swarmkit::client::Client client(config);

    swarmkit::client::EdgeOperationRequest request;
    request.edge.mission_id = "example-mission";
    request.edge.mission_revision = 1;
    request.edge.edge_id = "A-B";
    request.edge.attempt_id = "example-attempt-1";
    request.edge.drone_id = argv[2];
    const double from_lat = std::stod(argv[3]);
    const double from_lon = std::stod(argv[4]);
    const double to_lat = std::stod(argv[5]);
    const double to_lon = std::stod(argv[6]);
    request.edge.route.points = {{0, 0},
                                 swarmkit::edge::Wgs84ToLocal(from_lat, from_lon, to_lat, to_lon)};
    request.edge.obligation = swarmkit::edge::Obligation::kSurvey;
    request.edge.corridor_half_width_m = 4;
    request.edge.arrival_radius_m = 2;
    request.edge.arrival_max_speed_mps = 0.5;
    request.edge.maximum_speed_mps = 0; // Unknown physical bound is explicit.
    request.edge.sustained_deviation_samples = 3;
    request.edge.recovery_cooldown_ms = 1000;
    request.edge.timeout_ms = 120000;
    request.wgs84_origin = {from_lat, from_lon};
    request.altitude_m = std::stod(argv[7]);
    request.commanded_speed_mps = 2;

    auto reports = client.StartReports(
        {.drone_id = request.edge.drone_id},
        [](const swarmkit::client::AgentReport& event) {
            if (event.edge && event.edge->attempt_id == "example-attempt-1")
                std::cout << "edge event sequence=" << event.sequence
                          << " at_b=" << event.edge->status.at_b_verified
                          << " survey=" << event.edge->status.survey_verified
                          << " missing_intervals=" << event.edge->status.survey_missing.size()
                          << '\n';
        });
    if (!reports) {
        std::cerr << reports.error().user_message << '\n';
        return EXIT_FAILURE;
    }
    const auto started = client.ExecuteEdge(request);
    if (!started.ok) {
        std::cerr << "edge rejected: " << started.message << '\n';
        reports->Stop();
        return EXIT_FAILURE;
    }
    for (int poll = 0; poll < 240; ++poll) {
        const auto result = client.GetEdgeStatus(request.edge.drone_id, request.edge.attempt_id);
        if (result.status) {
            const auto phase = result.status->status.phase;
            if (phase == swarmkit::edge::Phase::kFulfilled) {
                std::cout << "EDGE_FULFILLED: external graph algorithm may now consume this fact\n";
                reports->Stop();
                return EXIT_SUCCESS;
            }
            if (phase == swarmkit::edge::Phase::kInterrupted) {
                std::cerr << "edge incomplete: " << result.status->status.reason << '\n';
                reports->Stop();
                return EXIT_FAILURE;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{500});
    }
    std::cerr << "edge outcome unknown after client wait; reconcile by attempt ID\n";
    reports->Stop();
    return EXIT_FAILURE;
}
