// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "swarmkit/core/evidence_store.h"

using namespace swarmkit::core;
using Catch::Matchers::WithinAbs;

TEST_CASE("DecomposeToEvidence extracts valid per-field evidence records", "[evidence_store]") {
    TelemetryFrame frame;
    frame.drone_id = "drone-alpha";
    frame.agent_session_id = "session-1234";
    frame.telemetry_sequence = 42;
    frame.agent_receive_unix_time_ms = 1700000000000;

    frame.lat_deg = 37.7749;
    frame.lon_deg = -122.4194;
    frame.rel_alt_m = 15.5F;
    frame.position_frame = CoordinateFrame::kWgs84;
    frame.validity.position = true;
    frame.provenance.position.source_time.timestamp_ms = 1699999999900;
    frame.provenance.position.source = "ekf3_pos";
    frame.accuracy.horizontal_position = UncertaintyEstimate{
        .value = 0.3F,
        .descriptor = {.semantics = UncertaintySemantics::kDeterministicHardBound},
    };

    frame.vx_mps = 1.2F;
    frame.vy_mps = -0.5F;
    frame.vz_mps = 0.1F;
    frame.velocity_frame = CoordinateFrame::kLocalNed;
    frame.validity.velocity = true;
    frame.provenance.velocity.source_time.timestamp_ms = 1699999999910;
    frame.provenance.velocity.source = "ekf3_vel";
    frame.accuracy.horizontal_velocity = UncertaintyEstimate{
        .value = 0.1F,
        .descriptor = {.semantics = UncertaintySemantics::kDeterministicHardBound},
    };

    frame.roll_deg = 1.0F;
    frame.pitch_deg = -2.0F;
    frame.yaw_deg = 90.0F;
    frame.validity.attitude = true;

    frame.battery_percent = 88.5F;
    frame.validity.battery = true;
    frame.provenance.vehicle_state.source_time.timestamp_ms = 1699999999800;
    frame.provenance.vehicle_state.source = "mavlink_sys_status";

    frame.estimator_state = EstimatorState::kHealthy;
    frame.estimator_position_ok = true;
    frame.estimator_velocity_ok = true;
    frame.validity.estimator = true;
    frame.provenance.estimator.source_time.timestamp_ms = 1699999999900;
    frame.provenance.estimator.source = "ekf_flags";

    frame.gps_quality = GpsQuality::kRtkFixed;
    frame.validity.gps = true;

    frame.armed = true;
    frame.validity.armed = true;

    frame.landed = false;
    frame.validity.landed = true;

    frame.failsafe = false;
    frame.validity.failsafe = true;

    auto records = DecomposeToEvidence(frame);
    // Position, Velocity, Attitude, Battery, Estimator, GPS, Armed, Landed, Failsafe = 9
    REQUIRE(records.size() == 9);

    SECTION("Position record validation") {
        auto it = std::find_if(records.begin(), records.end(), [](const EvidenceRecord& r) {
            return r.identity.field_id == EvidenceFieldId::kPosition;
        });
        REQUIRE(it != records.end());
        REQUIRE(it->identity.agent_id == "drone-alpha");
        REQUIRE(it->identity.agent_session_id == "session-1234");
        REQUIRE(it->identity.sequence == 42);
        REQUIRE(it->identity.coordinate_frame == CoordinateFrame::kWgs84);
        REQUIRE(it->identity.source_component == "ekf3_pos");
        REQUIRE(it->source_time.timestamp_ms == 1699999999900);
        REQUIRE(it->receive_time_ms == 1700000000000);
        REQUIRE(it->quality.estimator_healthy);
        REQUIRE(it->quality.estimator_position_ok);
        REQUIRE(it->quality.uncertainty.has_value());
        REQUIRE_THAT(it->quality.uncertainty->value, WithinAbs(0.3F, 1e-6));

        auto pos_val = std::get<std::array<double, 3>>(it->value);
        REQUIRE_THAT(pos_val[0], WithinAbs(37.7749, 1e-6));
        REQUIRE_THAT(pos_val[1], WithinAbs(-122.4194, 1e-6));
        REQUIRE_THAT(pos_val[2], WithinAbs(15.5, 1e-6));
    }

    SECTION("Velocity record validation") {
        auto it = std::find_if(records.begin(), records.end(), [](const EvidenceRecord& r) {
            return r.identity.field_id == EvidenceFieldId::kVelocity;
        });
        REQUIRE(it != records.end());
        REQUIRE(it->identity.coordinate_frame == CoordinateFrame::kLocalNed);
        REQUIRE(it->identity.source_component == "ekf3_vel");
        auto vel_val = std::get<std::array<float, 3>>(it->value);
        REQUIRE_THAT(vel_val[0], WithinAbs(1.2F, 1e-6));
        REQUIRE_THAT(vel_val[1], WithinAbs(-0.5F, 1e-6));
        REQUIRE_THAT(vel_val[2], WithinAbs(0.1F, 1e-6));
    }

    SECTION("Battery record validation") {
        auto it = std::find_if(records.begin(), records.end(), [](const EvidenceRecord& r) {
            return r.identity.field_id == EvidenceFieldId::kBattery;
        });
        REQUIRE(it != records.end());
        REQUIRE_THAT(std::get<float>(it->value), WithinAbs(88.5F, 1e-6));
    }

    SECTION("Estimator health record validation") {
        auto it = std::find_if(records.begin(), records.end(), [](const EvidenceRecord& r) {
            return r.identity.field_id == EvidenceFieldId::kEstimatorHealth;
        });
        REQUIRE(it != records.end());
        REQUIRE(std::get<EstimatorState>(it->value) == EstimatorState::kHealthy);
    }

    SECTION("GPS quality record validation") {
        auto it = std::find_if(records.begin(), records.end(), [](const EvidenceRecord& r) {
            return r.identity.field_id == EvidenceFieldId::kGpsQuality;
        });
        REQUIRE(it != records.end());
        REQUIRE(std::get<GpsQuality>(it->value) == GpsQuality::kRtkFixed);
    }

    SECTION("Boolean records validation") {
        auto it_armed = std::find_if(records.begin(), records.end(), [](const EvidenceRecord& r) {
            return r.identity.field_id == EvidenceFieldId::kArmedState;
        });
        REQUIRE(it_armed != records.end());
        REQUIRE(std::get<bool>(it_armed->value) == true);

        auto it_landed = std::find_if(records.begin(), records.end(), [](const EvidenceRecord& r) {
            return r.identity.field_id == EvidenceFieldId::kLandedState;
        });
        REQUIRE(it_landed != records.end());
        REQUIRE(std::get<bool>(it_landed->value) == false);
    }
}

TEST_CASE("DecomposeToEvidence ignores invalid fields", "[evidence_store]") {
    TelemetryFrame frame;
    frame.drone_id = "drone-beta";
    frame.validity.position = true;
    frame.lat_deg = 10.0;
    frame.validity.velocity = false;
    frame.validity.attitude = false;
    frame.validity.battery = false;
    frame.validity.estimator = false;
    frame.validity.gps = false;
    frame.validity.armed = false;
    frame.validity.landed = false;
    frame.validity.failsafe = false;

    auto records = DecomposeToEvidence(frame);
    REQUIRE(records.size() == 1);
    REQUIRE(records[0].identity.field_id == EvidenceFieldId::kPosition);
}

TEST_CASE("AgentEvidenceBuffer ring buffer behavior", "[evidence_store]") {
    AgentEvidenceBuffer buf(3);  // Capacity 3

    REQUIRE(buf.TotalRecords() == 0);
    REQUIRE_FALSE(buf.CurrentSessionId().has_value());

    EvidenceRecord r1{
        .value = 10.0F,
        .source_time = {.timestamp_ms = 100},
        .identity = {.agent_session_id = "sess-1", .field_id = EvidenceFieldId::kBattery, .sequence = 1},
    };
    EvidenceRecord r2{
        .value = 20.0F,
        .source_time = {.timestamp_ms = 200},
        .identity = {.agent_session_id = "sess-1", .field_id = EvidenceFieldId::kBattery, .sequence = 2},
    };
    EvidenceRecord r3{
        .value = 30.0F,
        .source_time = {.timestamp_ms = 300},
        .identity = {.agent_session_id = "sess-1", .field_id = EvidenceFieldId::kBattery, .sequence = 3},
    };
    EvidenceRecord r4{
        .value = 40.0F,
        .source_time = {.timestamp_ms = 400},
        .identity = {.agent_session_id = "sess-2", .field_id = EvidenceFieldId::kBattery, .sequence = 4},
    };

    buf.Insert(r1);
    buf.Insert(r2);
    REQUIRE(buf.TotalRecords() == 2);
    REQUIRE(buf.CurrentSessionId() == "sess-1");

    auto recent2 = buf.Recent(EvidenceFieldId::kBattery, 2);
    REQUIRE(recent2.size() == 2);
    REQUIRE(recent2[0].identity.sequence == 2);  // Newest first
    REQUIRE(recent2[1].identity.sequence == 1);

    buf.Insert(r3);
    REQUIRE(buf.TotalRecords() == 3);

    // Now insert 4th item, exceeding capacity 3. Oldest (r1) should be dropped.
    buf.Insert(r4);
    buf.SetCurrentSession("sess-2");
    REQUIRE(buf.TotalRecords() == 3);
    REQUIRE(buf.CurrentSessionId() == "sess-2");

    auto all = buf.All(EvidenceFieldId::kBattery);
    REQUIRE(all.size() == 3);
    // All returns oldest-first: r2 (seq 2), r3 (seq 3), r4 (seq 4)
    REQUIRE(all[0].identity.sequence == 2);
    REQUIRE(all[1].identity.sequence == 3);
    REQUIRE(all[2].identity.sequence == 4);

    auto recent3 = buf.Recent(EvidenceFieldId::kBattery, 3);
    REQUIRE(recent3.size() == 3);
    REQUIRE(recent3[0].identity.sequence == 4);
    REQUIRE(recent3[1].identity.sequence == 3);
    REQUIRE(recent3[2].identity.sequence == 2);
}

TEST_CASE("EvidenceStore multi-agent operations", "[evidence_store]") {
    EvidenceStore store(EvidenceStoreConfig{.max_records_per_field = 10});

    TelemetryFrame f1;
    f1.drone_id = "uav-1";
    f1.agent_session_id = "sess-uav1";
    f1.telemetry_sequence = 101;
    f1.validity.position = true;
    f1.lat_deg = 1.0;
    f1.validity.battery = true;
    f1.battery_percent = 90.0F;

    TelemetryFrame f2;
    f2.drone_id = "uav-2";
    f2.agent_session_id = "sess-uav2";
    f2.telemetry_sequence = 201;
    f2.validity.position = true;
    f2.lat_deg = 2.0;

    store.InsertFrame(f1);
    store.InsertFrame(f2);

    REQUIRE(store.TotalRecords() == 3);  // 2 from uav-1, 1 from uav-2

    auto agents = store.AgentIds();
    REQUIRE(agents.size() == 2);
    REQUIRE(std::find(agents.begin(), agents.end(), "uav-1") != agents.end());
    REQUIRE(std::find(agents.begin(), agents.end(), "uav-2") != agents.end());

    REQUIRE(store.CurrentSessionId("uav-1") == "sess-uav1");
    REQUIRE(store.CurrentSessionId("uav-2") == "sess-uav2");
    REQUIRE_FALSE(store.CurrentSessionId("nonexistent").has_value());

    auto uav1_pos = store.Recent("uav-1", EvidenceFieldId::kPosition, 1);
    REQUIRE(uav1_pos.size() == 1);
    REQUIRE(uav1_pos[0].identity.agent_id == "uav-1");

    auto uav2_pos = store.Recent("uav-2", EvidenceFieldId::kPosition, 1);
    REQUIRE(uav2_pos.size() == 1);
    REQUIRE(uav2_pos[0].identity.agent_id == "uav-2");

    auto uav2_bat = store.Recent("uav-2", EvidenceFieldId::kBattery, 1);
    REQUIRE(uav2_bat.empty());
}

TEST_CASE("EvidenceStore authoritative session management and delayed packet isolation", "[evidence_store]") {
    EvidenceStore store(EvidenceStoreConfig{.max_records_per_field = 10});

    // 1. Initial old session active
    store.SetCurrentSession("uav-1", "session-old");
    REQUIRE(store.CurrentSessionId("uav-1") == "session-old");

    EvidenceRecord old_rec1{
        .value = 10.0F,
        .source_time = {.timestamp_ms = 100},
        .identity = {.agent_id = "uav-1", .agent_session_id = "session-old", .field_id = EvidenceFieldId::kBattery, .sequence = 1},
    };
    store.Insert("uav-1", old_rec1);
    REQUIRE(store.CurrentSessionId("uav-1") == "session-old");

    // 2. Transition old -> new authoritative session
    store.SetCurrentSession("uav-1", "session-new");
    REQUIRE(store.CurrentSessionId("uav-1") == "session-new");

    // 3. Delayed old-session packet arrives after transition
    EvidenceRecord delayed_old_rec{
        .value = 15.0F,
        .source_time = {.timestamp_ms = 110},
        .identity = {.agent_id = "uav-1", .agent_session_id = "session-old", .field_id = EvidenceFieldId::kBattery, .sequence = 2},
    };
    store.Insert("uav-1", delayed_old_rec);

    // 4. Authoritative current session MUST remain new (never moves backwards)
    REQUIRE(store.CurrentSessionId("uav-1") == "session-new");

    // 5. Insert new session packet
    EvidenceRecord new_rec{
        .value = 95.0F,
        .source_time = {.timestamp_ms = 200},
        .identity = {.agent_id = "uav-1", .agent_session_id = "session-new", .field_id = EvidenceFieldId::kBattery, .sequence = 3},
    };
    store.Insert("uav-1", new_rec);
    REQUIRE(store.CurrentSessionId("uav-1") == "session-new");

    // 6. Another delayed old packet arrives
    EvidenceRecord delayed_old_rec2{
        .value = 12.0F,
        .source_time = {.timestamp_ms = 105},
        .identity = {.agent_id = "uav-1", .agent_session_id = "session-old", .field_id = EvidenceFieldId::kBattery, .sequence = 4},
    };
    store.Insert("uav-1", delayed_old_rec2);
    REQUIRE(store.CurrentSessionId("uav-1") == "session-new");

    // All records are retained for replay
    auto all_records = store.All("uav-1", EvidenceFieldId::kBattery);
    REQUIRE(all_records.size() == 4);
}
