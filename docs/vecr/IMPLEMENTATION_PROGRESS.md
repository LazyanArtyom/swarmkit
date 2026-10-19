# VECR implementation status — 2026-10-19

The pre-release library now has a canonical `ExecuteEdge` operation with distinct arrival, bounded traversal and qualified strip-survey obligations. The agent owns the attempt state machine, telemetry consumption, safety arbitration and typed reports. The SDK, CLI and example consume this API. Survey capture submission is limited to a configured mTLS sensor client and is disabled by default. A restricted nadir footprint builder is available for a future calibrated producer. The simulator-only recovery option performs bounded re-entry and return-to-B dispatch; it is disabled by default and has no live MAVLink recovery qualification.

The old state-acceptance research simulator, results and isolated public core were removed from the active tree after SHA-256-verified archives. Product backends, telemetry, authority, data transport, logging and associated tests remain. See `CLEANUP_INVENTORY.md`.

The deterministic VECR strip experiment has frozen v1 outputs and amended v2 outputs. V2 models eight of the requested 21 scenario IDs, 100 seeds per ID, and four paired methods. Private raster truth is separate from the verifier's observed metadata. The model uses geometric replay rather than the production agent/backend; its figures do not establish flight or live sensor performance. See `results/vecr/reports/CORE_RESULTS_V2_100_SEEDS.md` and its manifest.

The full CTest suite and VECR-specific integration tests pass on the local build when loopback sockets are permitted. No current VECR SITL campaign, Raspberry Pi timing study, or peer/server goodput campaign has been executed. A live camera evidence source, independent SITL truth, full 21-scenario matrix and publication-grade confidence intervals remain open.
