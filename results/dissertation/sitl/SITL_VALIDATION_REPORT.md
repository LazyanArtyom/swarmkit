# Three-UAV ArduPilot SITL Integration Validation

## SECTION A — Repository and environment

- Experiment base commit: `e375c24d23ee5ee10d7740435029fa0196e96e18`
- Captured git state: dirty (`true`), limited to the new SITL integration harness, its two SITL-only configurations, CMake target, and the separate `results/dissertation/sitl/` tree enumerated in Section O. The manifest records this state rather than claiming a clean revision.
- Host: Darwin 25.5.0, arm64, Apple M3 Pro
- Compiler/build: AppleClang 17.0.0.17000319, Release
- ArduPilot: `AION-Orca-Alpha2-26682-g958493b474`, commit `958493b47428a8f5e4ee5ed3f23d5ac193c24299`
- MAVLink Router: `v4-16-g2362c62`; inputs `0.0.0.0:24561`–`24563`, outputs to QGroundControl `192.168.123.36:14550` and agents `192.168.123.36:14601`–`14603`
- QGroundControl: UDP `*:14550` was bound by the running QGroundControl process during validation.
- SITL launch: `~/swarm_sitl_direct.sh start 3 192.168.123.36` on Ubuntu 24.04.1 LTS at `192.168.123.43`
- Build/test result: full Release build succeeded; `ctest --preset mac-release --output-on-failure` passed 2/2 tests, 0 failures.

## SECTION B — Three-UAV architecture

| UAV | SYSID | SITL instance | SITL endpoint | Router endpoint | SwarmKit agent ID | SwarmKit endpoint |
|---|---:|---:|---|---|---|---|
| drone-1 | 1 | 0 | `udpclient:127.0.0.1:24561` | input `0.0.0.0:24561` → Mac UDP `14601` | `sitl-agent-1` | MAVLink `0.0.0.0:14601`; gRPC `127.0.0.1:50061` |
| drone-2 | 2 | 1 | `udpclient:127.0.0.1:24562` | input `0.0.0.0:24562` → Mac UDP `14602` | `sitl-agent-2` | MAVLink `0.0.0.0:14602`; gRPC `127.0.0.1:50062` |
| drone-3 | 3 | 2 | `udpclient:127.0.0.1:24563` | input `0.0.0.0:24563` → Mac UDP `14603` | `sitl-agent-3` | MAVLink `0.0.0.0:14603`; gRPC `127.0.0.1:50063` |

All three vehicles simultaneously reported the intended unique SYSID, RTK-fixed GPS (fix 6, 10 satellites, HDOP 1.21), healthy EKF, and fresh heartbeat/telemetry through their respective production agents.

## SECTION C — Flight validation

| UAV | Armed observed / final | Takeoff | Distance moved | Max observed horizontal speed | Campaign mode / shutdown mode |
|---|---|---|---:|---:|---|
| drone-1 | yes / no | verified at about 5 m | 34.583 m | 5.740 m/s | GUIDED / LAND |
| drone-2 | yes in E1 and E2 / no | verified at about 5 m | 35.145 m | 6.440 m/s | GUIDED / LAND |
| drone-3 | yes / no | verified at about 5 m | 38.405 m | 6.804 m/s | GUIDED / LAND |

Mode, arm, takeoff, velocity, and land commands went through `SwarmClient`/Agent/MAVLinkBackend. The distances are changes in ArduPilot-reported simulated position and establish motion in simulation; they are not reused as an independent truth channel. After the campaign, all three LAND commands were acknowledged and telemetry-verified. Final health reported `armed=false`, `landed=true`, fresh telemetry, and `failsafe=false`; local agents and Ubuntu ArduCopter/MAVLink Router processes were then stopped.

## SECTION D — Evidence pipeline

The exercised path was:

`ArduPilot SITL → MAVLink Router UDP → MavlinkBackend → TelemetryFrame → DecomposeToEvidence → EvidenceStore → StateAcceptanceEngine`

Concrete implementation points:

1. `src/agent/backends/mavlink_udp_transport.cpp:75` receives the UDP datagram, and `src/agent/backends/mavlink_backend.cpp:253` parses MAVLink bytes in `ReceiverLoop`.
2. `src/agent/backends/mavlink_backend.cpp:318` invokes `MavlinkTelemetryDecoder::Decode`; `src/agent/backends/mavlink_telemetry_decoder.cpp:261` maps `GLOBAL_POSITION_INT` into WGS84 position, LocalNED velocity, and vehicle-boot source time.
3. `src/agent/backends/mavlink_backend.cpp:337` publishes backend telemetry. `src/agent/telemetry_manager.cpp:54` stamps agent receive times, binds the process incarnation, normalizes per-group provenance, and assigns the telemetry sequence.
4. The SDK stream supplies that production-normalized `TelemetryFrame` to `apps/experiment/sitl_validation.cpp:546`. At `:564`, the harness calls the canonical `DecomposeToEvidence` function and inserts every resulting record into `EvidenceStore`. This direct decomposition call is deliberate: S1/S3 delay occurs after SDK delivery but before evidence-store delivery. Source timestamps, sequence, session, frame, health, and numeric measurements are not rewritten.
5. `src/core/evidence_store.cpp:251` is the canonical decomposition used by `EvidenceStore::InsertFrame` (`:138`). No alternate evidence schema or fabricated primary evidence path was introduced.
6. `apps/experiment/sitl_validation.cpp:630` invokes `StateAcceptanceEngine::RequestSnapshot` for the three-agent participant set, then builds and serializes certificates at `:633`–`:634`.

The evaluated position uncertainty originates from `GPS_RAW_INT.h_acc` but is explicitly tagged backend-specific, not a deterministic hard bound. Accordingly, this SITL integration contract sets `require_deterministic_bounds=false`; it does not turn backend accuracy into theorem-grade evidence.

## SECTION E — Clock semantics

| UAV/incarnation | Clock domain | theta_hat (ms) | Base rho (ms) | Effective rho (ms) | Model | Incarnation |
|---|---|---:|---:|---:|---|---|
| drone-1 E1 | vehicle boot | -1787256925112 | 113 | 113 | `sitl-observed-receive-envelope-v1` | `agent-session-1787255276918-19508172377872517948717267141108568655-1` |
| drone-2 E1 | vehicle boot | -1787256930125 | 120 | 120 | `sitl-observed-receive-envelope-v1` | `agent-session-1787255276922-1412525859096521706715300069754288583200-1` |
| drone-2 E2 | vehicle boot | -1787256930172 | 95 | 95 | `sitl-observed-receive-envelope-v1` | `agent-session-1787257288113-175144100154114150108422047801532694376-1` |
| drone-3 E1 | vehicle boot | -1787256935163.5 | 116.5 | 116.5 | `sitl-observed-receive-envelope-v1` | `agent-session-1787255276918-1790892970145920185013645599045014863377-1` |

A deterministic clock mapping was **not** established, and this report does not claim one. For each incarnation, five genuine `GLOBAL_POSITION_INT` observations yielded candidates for `source_boot_ms - agent_receive_unix_ms`; theta_hat is the observed envelope midpoint and rho is half the observed range plus 1 ms, with a 2 ms floor. Synchronization is `Estimated`, `deterministic_bound=false`, and drift is declared 0 ppm, so effective rho equals base rho. Each model is bound to the exact agent incarnation; E2 acceptance remained blocked until a new E2-bound model existed.

## SECTION F — Independent truth determination

**INDEPENDENT SITL TRUTH NOT USED**

The evaluated MAVLink estimator stream was not reused as simulator truth. No separate ArduPilot ground-truth channel with independently documented semantics and timing provenance was established.

## SECTION G — Scenario results

| Scenario | Trials | Requests | Accepted | Availability | Primary rejection reasons | Replay agreements |
|---|---:|---:|---:|---:|---|---:|
| S0 normal moving flight | 10 | 200 | 200 | 100.0% | none | 200/200 |
| S1 delayed telemetry (300 ms configured) | 10 | 200 | 191 | 95.5% | 9 requests had age failures; the same rejected requests also carried other evidence-availability failures | 200/200 |
| S2 agent restart + genuine old epoch | 10 | 200 | 180 | 90.0% | 20 pre-clock rejects; 11 carried an explicit clock category and all 20 carried other missing/current-evidence failures | 200/200 |
| S3 high-speed delayed motion | 10 | 200 | 197 | 98.5% | 3 requests had age failures and accompanying other evidence-availability failures | 200/200 |

Total: 40 trials, 800 snapshot requests, 768 accepted certificates, and 800/800 persisted replay agreements. Availability was not forced to 100%; the injected delay and restart produced the intended rejection behavior.

## SECTION H — Restart invariant

| Measure | Result |
|---|---:|
| Restart events | 1 |
| Genuine E1 frames injected after E2 became authoritative | 5 |
| Old E1 frames accepted by Proposed | **0** |
| E2 requests before new clock | 20 |
| E2 accepts before new clock | **0** |
| E2 clock re-establishments | 1 |
| E2 accepts after clock re-establishment | 180 |

The old agent did not release UDP 14602 promptly after SIGTERM, so the restart was completed with process termination and a fresh E2 process while ArduPilot remained airborne. The E1 execution log is therefore retained and explicitly named `agent2-e1-interrupted.evidence`; it is not represented as a clean-complete run. The persisted replay trace itself is complete and records the E1→E2 session transition, old-frame injection, pre-clock rejection interval, E2 clock update, and resumed acceptance.

## SECTION I — High-speed behavior

- Configured S3 horizontal speeds: drone-1 6.000 m/s; drone-2 6.801 m/s; drone-3 7.433 m/s. All are below declared `Vh_max=8 m/s`.
- Maximum observed horizontal speed: 6.804 m/s (per-vehicle maxima: 5.740, 6.440, 6.804 m/s).
- Configured delay: 300 ms after normalized SDK telemetry and before decomposition/store delivery.
- Realized delay over 66 delayed frames: mean 312.482 ms, p95 321.497 ms, max 327.244 ms.
- Accepted position Delta+ distribution (`n=591`): mean 539.350 ms, p50 535 ms, p95 773 ms, max 981 ms.
- Propagated position uncertainty distribution (`n=591`): mean 4.908 m, p50 4.871 m, p95 6.905 m, max 8.682 m.
- Exact-comparability monotonicity check: 537 age-increasing pairs sharing the same agent, incarnation, and telemetry sequence; zero cases in which larger age produced smaller propagated uncertainty. Across all scenarios, 2,079 comparable pairs passed.

The preserved model is `epsilon_p = e_p + V3Dmax * Delta+`, with `V3Dmax = sqrt(8^2 + 3^2) ≈ 8.544 m/s`. The bounded-speed spherical enclosure may be conservative during aggressive motion. The acceptance architecture is model-versioned and can support tighter ellipsoidal, polytopic, or acceleration/dynamics-aware reachable sets if their soundness assumptions and replay semantics are explicitly defined.

## SECTION J — Optional truth/containment

Not reported because an independent SITL truth channel was not established.

No false-valid, containment-failure, or enclosure-ratio statistic is inferred from the evaluated MAVLink telemetry.

## SECTION K — Persisted replay

- Persisted SITL decisions: 800
- Agreements: 800
- Disagreements: 0
- Agreement: 100%
- State reconstructed from disk: yes

Each scenario trace was loaded from its JSONL file into a fresh `EvidenceStore`, session state, and clock-state map. The independent `StateAcceptanceVerifier` verified stored certificates against the separately supplied canonical contract and reconstructed request context.

## SECTION L — SITL certificate semantic spot-check

- Real SITL certificates selected: 10
- Mutation classes: `r_star`, session/incarnation, `theta_hat`, clock model version, coordinate frame, participant set, propagated uncertainty
- Outer certificate hash recomputed after each mutation: yes
- Cases tested: 70
- Cases rejected by verifier: 70

This is a backend-portability spot-check, not a replacement for the frozen large SimBackend mutation campaign.

## SECTION M — SITL integration latency

Scope: `RequestSnapshot + BuildCertificate + SerializeCertificate`, labeled **SITL integration latency on this host**.

| Count | Mean | Stddev | p50 | p95 | p99 | Max |
|---:|---:|---:|---:|---:|---:|---:|
| 800 | 2118.606 us | 2572.266 us | 758.959 us | 6676.725 us | 6775.002 us | 6851.000 us |

These macOS integration measurements are not compared with hard real-time flight-control-loop deadlines.

## SECTION N — Generated artifact paths

Machine-readable JSON:

- `results/dissertation/sitl/sitl_manifest.json`
- `results/dissertation/sitl/sitl_contract.json`
- `results/dissertation/sitl/sitl_scenario_summary.json`
- `results/dissertation/sitl/sitl_replay_results.json`
- `results/dissertation/sitl/sitl_latency_summary.json`
- `results/dissertation/sitl/sitl_fault_realization.json`
- `results/dissertation/sitl/sitl_certificate_spotcheck.json`
- `results/dissertation/sitl/sitl_flight_diagnostics.json`
- `results/dissertation/sitl/sitl_clock_models.json`
- `results/dissertation/sitl/sitl_shutdown_verification.json`
- `results/dissertation/sitl/sitl_execution_evidence_verification.json`

CSV:

- `results/dissertation/sitl/sitl_trials.csv`
- `results/dissertation/sitl/sitl_latency_samples.csv`
- `results/dissertation/sitl/telemetry/drone-1.csv`, `drone-2.csv`, `drone-3.csv`
- `results/dissertation/sitl/telemetry/pre_motion/drone-1.csv`, `drone-2.csv`, `drone-3.csv`
- `results/dissertation/sitl/telemetry/motion/drone-1.csv`, `drone-2.csv`, `drone-3.csv`
- `results/dissertation/sitl/telemetry/post_motion/drone-1.csv`, `drone-2.csv`, `drone-3.csv`
- `results/dissertation/sitl/telemetry/motion_verified_window/drone-1.csv`, `drone-2.csv`, `drone-3.csv`

Persisted traces and certificates:

- `results/dissertation/sitl/traces/s0.jsonl`
- `results/dissertation/sitl/traces/s1.jsonl`
- `results/dissertation/sitl/traces/s2.jsonl`
- `results/dissertation/sitl/traces/s3.jsonl`
- `results/dissertation/sitl/certificates/sitl-certificate-0.cert` through `sitl-certificate-9.cert`
- `results/dissertation/sitl/execution_evidence/agent1-final.evidence`
- `results/dissertation/sitl/execution_evidence/agent2-e1-interrupted.evidence`
- `results/dissertation/sitl/execution_evidence/agent2-e2b-final.evidence`
- `results/dissertation/sitl/execution_evidence/agent3-final.evidence`

Configuration/harness/report:

- `testdata/agent_mavlink_sitl_validation.yaml`
- `testdata/sitl_takeoff_sequence.yaml`
- `apps/experiment/sitl_validation.cpp`
- `apps/CMakeLists.txt`
- `results/dissertation/sitl/SITL_VALIDATION_REPORT.md`

## SECTION O — Source changes

Backend/integration changes:

- `apps/CMakeLists.txt`: adds the Release-buildable `swarmkit-sitl-validation` executable.
- `apps/experiment/sitl_validation.cpp`: adds the three-agent production-SDK campaign, deterministic delivery delay, genuine E1 frame retention/injection, E2 clock barrier, trace persistence/replay, certificate spot-checks, latency capture, monotonicity evaluation, and machine-readable exporters.
- `testdata/agent_mavlink_sitl_validation.yaml`: adds a SITL-only MAVLink profile. Its 3.5 s ACK wait remains below the SDK's 5 s unary deadline while accommodating observed cross-host ACK latency; production defaults were not weakened.
- `testdata/sitl_takeoff_sequence.yaml`: explicitly verifies ARM before TAKEOFF to avoid conflating command acknowledgement with observed arm state or physical ascent.

Core semantic changes:

- **None.** No file under `src/core`, no certificate semantics, no acceptance predicate, and no reachability model was changed.
- No MAVLink backend source change was required; integration behavior was exercised as implemented.
- Existing frozen SimBackend result files were not overwritten or regenerated.

Generated results are isolated under `results/dissertation/sitl/`. No main paper/dissertation text was edited.

## SECTION P — Limitations

- SITL is software-in-the-loop, not physical flight.
- Main theorem-conformance statistics remain from SimBackend.
- SITL does not replace independent hardware validation.
- Because no independent SITL truth was used, this campaign provides no new false-valid or containment claim.
- The current spherical bounded-speed reachability model is intentionally conservative.
- macOS host latency is not target embedded real-time evidence.
- The live clock envelope is estimated and non-deterministic; the SITL contract therefore demonstrates integration/replay behavior, not a theorem-grade deterministic clock bound.
- Drone-2 E1 required forced termination after graceful signals did not promptly release the MAVLink socket. This is preserved as an interrupted execution-evidence log and should not be described as a graceful restart.
- Position motion and observed speed come from evaluated ArduPilot MAVLink telemetry. They prove activity through the integration path but are not independent truth.

## SECTION Q — FINAL VERDICT

**THREE-UAV ARDUPILOT SITL INTEGRATION VALIDATION PASSED**

Three simultaneous moving ArduCopter SITL vehicles were controlled through the production MAVLinkBackend and canonical evidence decomposition into the unchanged StateAcceptanceEngine. All 800 persisted decisions replayed with 100% agreement, and all 70 rehashed semantic certificate mutations were rejected. The critical restart invariants held: zero old-E1 acceptances, zero E2 acceptances before an E2-bound clock model, and 180 acceptances after clock re-establishment. Delayed and high-speed scenarios triggered measurable age/rejection/uncertainty behavior with no monotonicity violation, while no independent-truth or hardware-validation claim is made.
