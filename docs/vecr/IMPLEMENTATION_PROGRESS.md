# VECR implementation status — 2026-10-21

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
