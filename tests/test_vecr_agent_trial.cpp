// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include <catch2/catch_test_macros.hpp>

#include "vecr_agent_trial.h"

TEST_CASE("VECR reports a camera gap at B without a return command", "[vecr][agent-campaign]") {
    swarmkit::testsupport::VecrTrialConfig config;
    config.scenario_id = "camera-gap";
    config.seed = 1;
    config.fault = swarmkit::testsupport::TrialFault::kTransientCameraGap;
    const auto result = swarmkit::testsupport::RunVecrAgentTrial(config);
    CHECK(result.requested_fault_realized);
    CHECK(result.reported_phase == swarmkit::edge::Phase::kArrived);
    CHECK_FALSE(result.truth_survey_complete);
    CHECK(result.backend_goto_accepted == 1);
    CHECK(result.recovery_attempts == 0);
}

TEST_CASE("Dishonest camera metadata is detected only by the private truth oracle",
          "[vecr][agent-campaign][oracle]") {
    swarmkit::testsupport::VecrTrialConfig config;
    config.scenario_id = "S11";
    config.seed = 1;
    config.fault = swarmkit::testsupport::TrialFault::kDishonestCamera;
    const auto observed = swarmkit::testsupport::RunVecrAgentTrial(config);
    CHECK(observed.requested_fault_realized);
    CHECK(observed.reported_phase == swarmkit::edge::Phase::kArrived);
    CHECK_FALSE(observed.truth_survey_complete);
    CHECK(observed.truth_missing_fraction > 0);
}

TEST_CASE("Detectably inconsistent camera metadata remains unverified at B",
          "[vecr][agent-campaign][oracle]") {
    swarmkit::testsupport::VecrTrialConfig config;
    config.scenario_id = "S11a";
    config.seed = 1;
    config.fault = swarmkit::testsupport::TrialFault::kDetectableBadMetadata;
    const auto observed = swarmkit::testsupport::RunVecrAgentTrial(config);
    CHECK(observed.requested_fault_realized);
    CHECK(observed.capture_submitted > observed.capture_accepted);
    CHECK(observed.recovery_attempts == 0);
    CHECK_FALSE(observed.truth_survey_complete);
    CHECK(observed.reported_phase == swarmkit::edge::Phase::kArrived);
}

TEST_CASE("Persistent motor failure interrupts agent-backed recovery", "[vecr][agent-campaign]") {
    swarmkit::testsupport::VecrTrialConfig config;
    config.scenario_id = "S16";
    config.fault = swarmkit::testsupport::TrialFault::kMotorFailureOnRepair;
    config.wind_north_mps = 3.0;
    for (const int seed : {1, 10}) {
        config.seed = seed;
        if (seed == 10) {
            // The v4.0 camera-center trigger left this exact preregistered
            // gap fully covered and the intended motor failure never fired.
            config.gap_begin_m = 8.2072414973192238;
            config.gap_end_m = 12.069877598199628;
        }
        {
            const auto observed = swarmkit::testsupport::RunVecrAgentTrial(config);
            INFO("seed=" << seed << " phase=" << static_cast<int>(observed.reported_phase));
            CHECK(observed.requested_fault_realized);
            CHECK(observed.recovery_attempts == 1);
            CHECK(observed.reported_phase == swarmkit::edge::Phase::kFailed);
            CHECK_FALSE(observed.truth_survey_complete);
        }
    }
}

TEST_CASE("Rejected recovery command is a realized failure for VECR", "[vecr][agent-campaign]") {
    swarmkit::testsupport::VecrTrialConfig config;
    config.scenario_id = "S19";
    config.seed = 10;
    config.gap_begin_m = 8.2072414973192238;
    config.gap_end_m = 12.069877598199628;
    config.fault = swarmkit::testsupport::TrialFault::kRecoveryCommandRejected;
    config.wind_north_mps = 3.0;
    {
        const auto observed = swarmkit::testsupport::RunVecrAgentTrial(config);
        CHECK(observed.requested_fault_realized);
        CHECK(observed.backend_goto_rejected > 0);
        CHECK(observed.reported_phase == swarmkit::edge::Phase::kFailed);
        CHECK_FALSE(observed.truth_survey_complete);
    }
}

TEST_CASE(
    "Physical detour and wide off-center sensing have different survey "
    "outcomes",
    "[vecr][agent-campaign]") {
    swarmkit::testsupport::VecrTrialConfig config;
    config.seed = 1;
    config.gap_begin_m = 6;
    config.gap_end_m = 14;
    config.wind_north_mps = 2;
    config.scenario_id = "S03";
    config.fault = swarmkit::testsupport::TrialFault::kDetourWind;
    config.camera_cross_half_m = 5;
    const auto detour = swarmkit::testsupport::RunVecrAgentTrial(config);
    CHECK(detour.requested_fault_realized);
    CHECK(detour.max_truth_cross_track_m > 4);
    CHECK(detour.recovery_attempts == 1);
    CHECK(detour.truth_survey_complete);
    config.scenario_id = "S06";
    config.fault = swarmkit::testsupport::TrialFault::kOffcenterWideCamera;
    config.camera_cross_half_m = 14;
    const auto wide = swarmkit::testsupport::RunVecrAgentTrial(config);
    CHECK(wide.requested_fault_realized);
    CHECK(wide.max_truth_cross_track_m > 4);
    CHECK(wide.recovery_attempts == 1);
    CHECK(wide.truth_survey_complete);
}

TEST_CASE("S06 short wind zone crosses the corridor while wide sensing covers it",
          "[vecr][agent-campaign]") {
    swarmkit::testsupport::VecrTrialConfig config;
    config.scenario_id = "S06";
    config.seed = 1;
    config.fault = swarmkit::testsupport::TrialFault::kOffcenterWideCamera;
    config.gap_begin_m = 8.262826256419121;
    config.gap_end_m = 12.158663513898638;
    config.wind_north_mps = 2.9506691517960726;
    config.camera_cross_half_m = 14;
    {
        const auto observed = swarmkit::testsupport::RunVecrAgentTrial(config);
        CHECK(observed.requested_fault_realized);
        CHECK(observed.max_truth_cross_track_m > 4);
        CHECK(observed.truth_survey_complete);
        CHECK(observed.reported_phase == swarmkit::edge::Phase::kArrived);
    }
}
