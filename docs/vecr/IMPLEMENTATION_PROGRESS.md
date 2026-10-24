# VECR implementation status — 2026-10-24

The v5 digital-simulation campaign is complete: 5,280 paired-policy attempts
across 21 primary scenarios and 18 sensitivity conditions, plus 80 separate
serial host benchmarks. Independent reconstruction checked 453,524 delivered
captures and 4,246 completed re-entry dwells. Release, UBSan, new regression,
installed-SDK and artifact verification gates passed. Experimental source is
`7896f20`; final analysis source is `62a3363`. See
[FINAL_VALIDATION_V5.md](FINAL_VALIDATION_V5.md) for every gate, measured results,
uncertainty, negative controls and unfinished scope. No SITL was run.

VECR reduced travel by 25–28% in three equally successful midflight-recovery
conditions. Both policies had zero false fulfillment across 4,880 honest-sensor
attempts, but all 400 dishonest-sensor controls falsely fulfilled. Persistent
missing evidence still prevented certification. These are agent-backed simulator
results under a restricted full-strip camera model, not physical-flight claims.

## Earlier frozen studies

The v4.2 simulator validation is complete: 2,400 confirmatory B2/P agent
attempts across 12 comparative scenarios and 100 paired seeds, 72 separate
held-out sensitivity attempts, seven rendered figures, 107 verified final
artifact hashes, and release/UBSan/focused regression gates passing. See
`FINAL_VALIDATION_V4_2.md` for measured effects, negative outcomes and
remaining limits. The trial source is `5adb687`; corrected analysis source is
`60f79e8`. SITL, real camera calibration and physical flight remain untested.

The v4.1 campaign was stopped after 1,452 attempts because S06 crosswind
never drove the true path outside the configured corridor. V4.2 increases
only that scenario's wind, records its actual wide camera footprint explicitly
in configuration and raw data, and asserts the physical excursion before
counting an S06 trial. See `SIMULATION_PROTOCOL_V4_2.md`.

The first v4.0 full campaign stopped after 2,018 completed attempts when
S16 seed 10 did not enter recovery and its scheduled motor failure did not
occur. The partial output is preserved with `INCOMPLETE` status. V4.1 changes
the synthetic camera-gap trigger to suppress scheduled captures whose true
footprints overlap the missing strip; fault realization now requires an
affected capture, actual motor failure or actual command rejection. All 100
S16 and S19 seeds were preflighted under both policies before the v4.1 final
run. See `SIMULATION_PROTOCOL_V4_1.md`.

V4 adds a shared agent-side repair policy: targeted-gap VECR or coverage-aware
full-edge restart B2. Both use the same verification state, sensor evidence,
arrival and safety contracts; only the repair re-entry point changes. The
policy is present in protobuf, public C++ API, SDK, agent and CLI. The
simulator supports optional finite horizontal acceleration and controllable
wind. A private 0.25 m raster oracle and trial runner measure actual physical
trajectories through SDK → AgentService → SimBackend. See
`SIMULATION_PROTOCOL_V4.md` and `results/vecr/agent_campaign_v4/README.md`.

P0 persistence and cancellation changes are detailed in
`V4_IMPLEMENTATION_AUDIT.md`. The journal format is `VECRJ02`; pre-release
`VECRJ01` files fail closed until reviewed/migrated. Interrupted software
obligation does not assert physical stop. Simulator hold reconciliation
requires authorized command acceptance and bounded stationary telemetry.
Live MAVLink recovery remains unvalidated.

The 21-case v4 inventory has 12 B2/P comparative scenarios, five
diagnostic/safety cases, one route capability boundary and three unsupported
communication/multi-UAV scenarios (S13, S14, S17). The v4 manifest states
actual completed run counts. It does not establish SITL or physical-flight
performance.

The pre-release library has a canonical `ExecuteEdge` operation with distinct arrival, bounded traversal, qualified strip survey and combined traversal-and-survey obligations. The agent owns the attempt state machine, telemetry consumption, safety arbitration and typed reports. The SDK, CLI and example consume this API. The agent accepts only straight two-point routes, computes survey intervals from bounded nadir capture metadata, and restricts capture submission to a configured mTLS sensor client and calibration ID. These inputs are still trusted assertions from the sensor producer, not independently validated camera evidence. The simulator-only recovery option performs bounded re-entry and return-to-B dispatch; it is disabled by default and has no live MAVLink recovery qualification. A durable, fsynced attempt journal now preserves terminal reports and prevents ambiguous attempt IDs from redispatching after restart; ambiguous outcomes remain `UNKNOWN` and require reconciliation.

The old state-acceptance research simulator, results and isolated public core were removed from the active tree after SHA-256-verified archives. Product backends, telemetry, authority, data transport, logging and associated tests remain. See `CLEANUP_INVENTORY.md`.

The deterministic VECR strip experiment has frozen v1 outputs and amended v2 outputs. V2 models eight of the requested 21 scenario IDs, 100 seeds per ID, and four paired methods. Private raster truth is separate from the verifier's observed metadata. The model uses geometric replay rather than the production agent/backend; its figures do not establish flight or live sensor performance. See `results/vecr/reports/CORE_RESULTS_V2_100_SEEDS.md` and its manifest.

Regression tests exercise the SDK → Agent → SimBackend path with independent simulator truth, successful gap repair and a motor failure during repair. The versioned [agent-backed v3 pilot](../../results/vecr/agent_pilot_v3/README.md) records those two attempts, raw traces, a configuration and hash manifest, and a figure. That pilot remains an integration check. No current VECR SITL campaign, Raspberry Pi timing study, peer/server goodput campaign or live camera evidence source has been executed. Independent SITL truth and the unsupported v4 cases remain open.

## v5 follow-up (2026-10-24)

The independent review drove a new simulator-only study. Recovery now remains
RECOVERING until physical re-entry and continuation, prevents overlapping repairs,
and exposes bounded departure with hysteresis. A noisy telemetry-only camera
estimator and independent exact-strip oracle replace ideal metadata in the new
harness. Full measurements/acquisitions/rejections/command/trajectory traces,
21 primary cases, 18 sensitivity conditions, paired inference, lossless raw outputs
and serial process performance measurement are implemented. See
`SIMULATION_PROTOCOL_V5.md` and `FINAL_VALIDATION_V5.md` for execution status.
Previous v4.2 results remain frozen and retain their narrower interpretation.
