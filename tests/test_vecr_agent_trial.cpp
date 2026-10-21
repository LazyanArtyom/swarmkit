// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include <catch2/catch_test_macros.hpp>

#include "vecr_agent_trial.h"

TEST_CASE("B2 and VECR execute the same camera-gap trial through agent and SimBackend",
          "[vecr][agent-campaign]") {
    swarmkit::testsupport::VecrTrialConfig config;
    config.scenario_id = "S04";
    config.seed = 1;
    config.fault = swarmkit::testsupport::TrialFault::kTransientCameraGap;
    config.policy = swarmkit::edge::RepairPolicy::kTargetedGap;
    const auto targeted = swarmkit::testsupport::RunVecrAgentTrial(config);
    config.policy = swarmkit::edge::RepairPolicy::kFullEdgeRestart;
    const auto full_restart = swarmkit::testsupport::RunVecrAgentTrial(config);
    INFO("P phase=" << static_cast<int>(targeted.reported_phase)
                    << " truth=" << targeted.truth_survey_complete
                    << " distance=" << targeted.physical_distance_m
                    << " B2 phase=" << static_cast<int>(full_restart.reported_phase)
                    << " truth=" << full_restart.truth_survey_complete
                    << " distance=" << full_restart.physical_distance_m);
    CHECK(targeted.requested_fault_realized);
    CHECK(full_restart.requested_fault_realized);
    CHECK(targeted.reported_phase == swarmkit::edge::Phase::kFulfilled);
    CHECK(full_restart.reported_phase == swarmkit::edge::Phase::kFulfilled);
    CHECK(targeted.truth_survey_complete);
    CHECK(full_restart.truth_survey_complete);
    CHECK(targeted.backend_goto_accepted >= 3);
    CHECK(full_restart.backend_goto_accepted >= 3);
    CHECK(full_restart.physical_distance_m > targeted.physical_distance_m);
}

TEST_CASE("Dishonest camera metadata is detected only by the private truth oracle",
          "[vecr][agent-campaign][oracle]") {
    swarmkit::testsupport::VecrTrialConfig config;
    config.scenario_id = "S11";
    config.seed = 1;
    config.fault = swarmkit::testsupport::TrialFault::kDishonestCamera;
    const auto observed = swarmkit::testsupport::RunVecrAgentTrial(config);
    CHECK(observed.requested_fault_realized);
    CHECK(observed.reported_phase == swarmkit::edge::Phase::kFulfilled);
    CHECK_FALSE(observed.truth_survey_complete);
    CHECK(observed.truth_missing_fraction > 0);
}

TEST_CASE("Detectably inconsistent camera metadata is rejected before repair",
          "[vecr][agent-campaign][oracle]") {
    swarmkit::testsupport::VecrTrialConfig config;
    config.scenario_id = "S11a";
    config.seed = 1;
    config.fault = swarmkit::testsupport::TrialFault::kDetectableBadMetadata;
    const auto observed = swarmkit::testsupport::RunVecrAgentTrial(config);
    CHECK(observed.requested_fault_realized);
    CHECK(observed.capture_submitted > observed.capture_accepted);
    CHECK(observed.recovery_attempts == 1);
    CHECK(observed.truth_survey_complete);
    CHECK(observed.reported_phase == swarmkit::edge::Phase::kFulfilled);
}

TEST_CASE("Persistent motor failure interrupts both agent-backed recovery policies",
          "[vecr][agent-campaign]") {
    swarmkit::testsupport::VecrTrialConfig config;
    config.scenario_id = "S16";
    config.fault = swarmkit::testsupport::TrialFault::kMotorFailureOnRepair;
    for (const int seed : {1, 10}) {
        config.seed = seed;
        if (seed == 10) {
            // The v4.0 camera-center trigger left this exact preregistered
            // gap fully covered and the intended motor failure never fired.
            config.gap_begin_m = 8.2072414973192238;
            config.gap_end_m = 12.069877598199628;
        }
        for (const auto policy : {swarmkit::edge::RepairPolicy::kTargetedGap,
                                  swarmkit::edge::RepairPolicy::kFullEdgeRestart}) {
            config.policy = policy;
            const auto observed = swarmkit::testsupport::RunVecrAgentTrial(config);
            INFO("seed=" << seed << " policy=" << static_cast<int>(policy)
                          << " phase=" << static_cast<int>(observed.reported_phase));
            CHECK(observed.requested_fault_realized);
            CHECK(observed.recovery_attempts == 1);
            CHECK(observed.reported_phase == swarmkit::edge::Phase::kInterrupted);
            CHECK_FALSE(observed.truth_survey_complete);
        }
    }
}

TEST_CASE("Rejected recovery command is a realized failure for both policies",
          "[vecr][agent-campaign]") {
    swarmkit::testsupport::VecrTrialConfig config;
    config.scenario_id = "S19";
    config.seed = 10;
    config.gap_begin_m = 8.2072414973192238;
    config.gap_end_m = 12.069877598199628;
    config.fault = swarmkit::testsupport::TrialFault::kRecoveryCommandRejected;
    for (const auto policy : {swarmkit::edge::RepairPolicy::kTargetedGap,
                              swarmkit::edge::RepairPolicy::kFullEdgeRestart}) {
        config.policy = policy;
        const auto observed = swarmkit::testsupport::RunVecrAgentTrial(config);
        CHECK(observed.requested_fault_realized);
        CHECK(observed.backend_goto_rejected > 0);
        CHECK(observed.reported_phase == swarmkit::edge::Phase::kInterrupted);
        CHECK_FALSE(observed.truth_survey_complete);
    }
}

TEST_CASE("Physical detour and wide off-center sensing have different survey outcomes",
          "[vecr][agent-campaign]") {
    swarmkit::testsupport::VecrTrialConfig config;
    config.seed = 1;
    config.gap_begin_m = 6;
    config.gap_end_m = 14;
    config.wind_north_mps = 2;
    config.scenario_id = "S03";
    config.fault = swarmkit::testsupport::TrialFault::kDetourWind;
    const auto detour = swarmkit::testsupport::RunVecrAgentTrial(config);
    CHECK(detour.requested_fault_realized);
    CHECK(detour.max_truth_cross_track_m > 4);
    CHECK(detour.recovery_attempts == 1);
    CHECK(detour.truth_survey_complete);
    config.scenario_id = "S06";
    config.fault = swarmkit::testsupport::TrialFault::kOffcenterWideCamera;
    const auto wide = swarmkit::testsupport::RunVecrAgentTrial(config);
    CHECK(wide.requested_fault_realized);
    CHECK(wide.max_truth_cross_track_m > 4);
    CHECK(wide.recovery_attempts == 0);
    CHECK(wide.truth_survey_complete);
}
