# SwarmKit TCRR Readiness and Empirical Evaluation Task List

Status: repository and paper audit completed on 2026-08-11. Gate 0 and Phase 1 were implemented on 2026-08-11. Phase 2 was implemented on 2026-08-12. No rotor-router, TCRR certificate, or commit logic has been added to SwarmKit.

## Objective

Prepare SwarmKit to provide trustworthy, typed, ordered, and replayable physical-execution evidence, then build TCRR as a separate consumer and use the combined system to fill the paper's empirical result placeholders.

The implementation order is driven by two rules:

1. Evidence identity and provenance must be correct before recording, replay, or fault injection can be scientifically useful.
2. Experimental definitions and ground-truth rules must be frozen before collecting result data.

## Baseline verified during this audit

- The repository configures with `cmake --preset mac-debug`.
- The repository builds with `cmake --build --preset mac-debug`.
- The current test target passes with `ctest --preset mac-debug` (1/1 CTest targets, 0 failures).
- The protobuf and public C++ APIs now expose one canonical execution-handle and telemetry-evidence model.
- SwarmKit is pre-release: compatibility-only fields, overloads, fixtures, and aliases are intentionally absent.
- The public telemetry model has producer identity, exact attempt binding, typed measurement provenance,
  explicit uncertainty semantics, GPS/estimator state, and transport-local delivery metadata.
- Telemetry and reports have bounded in-memory replay; telemetry has an atomic replay/live boundary and explicit history/session status.
- One deterministic, checksummed protobuf recorder is the only durable execution-evidence path; report-only JSONL was removed.

## Paper result contract

The work is complete only when it can produce auditable inputs for all of the following paper fields.

### Table I: held-out enclosure calibration

- Regime: hover, cruise, turn, disturbed.
- Number of independent samples and runs.
- Empirical enclosure coverage with a 95% confidence interval.
- 99th-percentile position error.
- Calibration profile/version, airframe, autopilot, estimator, telemetry rate, ground-truth source, and time-alignment method.

### Table II: safety and operational results

Methods:

- command-ACK commit;
- one-sample fixed radius;
- radius plus dwell;
- existing SwarmKit `GOAL_REACHED`;
- position-only TCRR ablation;
- full TCRR.

Outcomes:

- false commits / transition opportunities, with denominator and interval;
- invalid committed trace steps;
- stale or duplicate commits;
- added certification latency, median and IQR;
- end-to-end completion time;
- CPU time, peak memory, and network bytes per commit/run;
- rejected or missed certificates, timeouts, and retries as liveness/availability outcomes.

### Final paper statements

- independent run count and transition-opportunity count;
- best-baseline and full-TCRR false-commit rates with intervals;
- absolute risk difference and risk ratio;
- trace-correctness result;
- stale/duplicate adversarial trial count and corrupt-commit count;
- one-sided 95% upper bound when zero corrupt commits are observed;
- certification latency and operational-overhead trade-off.

## Remaining audit findings

1. The simulator is not yet a command-responsive experimental dynamics model and has no separate
   truth channel or seeded fault scheduler.
2. The external TCRR controller, calibration pipeline, experimental runner, statistical analysis,
   and paper result replacement remain future phases.

## Gap analysis

| Requirement | Current support | Remaining gap | Next change | Required tests |
| --- | --- | --- | --- | --- |
| Execution identity and Agent session | Complete optional intent context, exact execution handle, and global evidence binding | Phase 3 reader/replay tools remain | Preserve the envelope unchanged in normalized replay | Restart, retry, supersession, failure, timeout, exact cancel |
| Goal lineage | Goal/revision/physical-attempt/session/client/correlation are recorded under one exact handle | Phase 3 scripted lifecycle coverage remains | Exercise every lifecycle through the scripted backend | Retry, replacement, stale report, and guarded cancellation |
| Telemetry order | Session/drone sequence, replay cursor, live boundary, and explicit loss/session events are complete | Phase 3 exact sequence injection remains | Drive canonical observations from the scripted backend | Duplicate, gap, reorder, reconnect, restart, eviction |
| Measurement provenance | Per-group provenance is preserved unchanged in normalized frames and evidence envelopes | Calibration profile generation remains external | Replay envelopes without renumbering or restamping | Cached-position freshness and deterministic replay |
| Timing uncertainty | Agent ingress and per-group source evidence survive stream and recorder paths | Empirical clock calibration remains external | Validate timing profiles in the experiment layer | Unknown remains absent, never zero; replay equality tests |
| Accuracy semantics | Optional typed XY/Z position/velocity and generic speed estimates with descriptors | Calibration profiles do not yet supply empirical bounds | Add external calibration artifacts and profile loading at the experiment layer | Round-trip, absent values, MAVLink backend-specific labels |
| Estimator/failsafe evidence | Typed state, validity, raw quality, and typed capability support | Fault harness cannot yet script all degradations | Add scripted state changes and seeded faults | Unknown versus healthy and degradation sequences |
| Motion bounds | Cohesive value+semantics+source/profile type | Validated physical bounds are not calibrated | Load validated profiles only after empirical validation | Unknown/default and semantic round trips |
| Telemetry replay and gap status | Complete bounded replay/live stream with session, range, boundary, and eviction events | Deterministic scripted producer belongs to Phase 3 | Exercise the canonical stream through the scripted backend | Eviction, replay ordering, reconnect, bounded memory |
| Deterministic execution record | Complete global protobuf envelope with deterministic serialization, checksums, clean-close marker, metadata, and loss policy | Reader/replay tooling belongs to Phase 3 | Promote a strict log reader and normalized replay backend | Byte-stable replay, rotation, truncation, and corruption |
| Replay/fault testing | `RecordingBackend` can manually emit telemetry; simulator and integration harness exist | No public scripted/replay backend, manual time/session injection, seeded faults, command-responsive motion, or separate truth | Add a scripted backend, deterministic backend decorator/scheduler, and command-responsive kinematic simulator with a truth-only channel | Production MAVLink path remains free of random test behavior. Fixed-seed tests required |
| SDK ergonomics and conversions | Exact handles and typed stream/frame observations expose cursor, gaps, replay/live, sessions, and local callback drops | Phase 3 end-to-end matrix remains | Exercise all observations through deterministic scripts | Conversion tests for every correctness field |
| Documentation boundary | The scientific, canonical API, and readiness contracts state guarantees and non-guarantees | Fault/simulator guarantees await Phase 3 | Update the contract alongside each completed phase | Review all guarantees and non-guarantees |

## Ordered implementation backlog

Priority meanings:

- **P0**: blocks trustworthy evidence or invalidates a paper safety claim.
- **P1**: blocks deterministic reconstruction or controlled experiments.
- **P2**: builds the external TCRR system and paper baselines.
- **P3**: produces calibration, simulation, SITL, and physical-flight data.
- **P4**: performs the locked statistical analysis and paper integration.

Tasks within each priority are ordered by dependency. Do not begin a task whose listed dependencies are incomplete.

### Gate 0 - freeze the scientific and protocol contracts

- [x] **TCRR-000 (P0): Write the experimental estimand and ground-truth specification.**
  - Define a transition opportunity, attempted transition, eligible sample, accepted certificate, commit, false commit, invalid trace step, stale/duplicate commit, missed certificate, timeout, and retry.
  - Define the exact occupancy time used to label a commit and require ground-truth occupancy at that same time.
  - Define the independent experimental unit as a complete seeded run; transitions are repeated observations clustered within runs.
  - Predeclare primary outcome, confidence level, target interval precision/upper bound, paired-seed policy, exclusions, and corrupted/incomplete-run handling.
  - Separate offline same-log method comparison from closed-loop end-to-end comparison. The former isolates decision logic; the latter measures operational effects when methods change future commands.
  - Output: `docs/experiments/tcrr-experimental-protocol.md`.
  - Paper unlock: denominators, sample-size calculation, Tables I/II interpretation.

- [x] **TCRR-001 (P0): Approve the algorithm-neutral identity and evidence API RFC.**
  - Define the canonical greenfield protobuf and public C++ model.
  - Specify `ExecutionContext`, Agent session, goal handle, physical attempt, telemetry identity, timestamp evidence, uncertainty semantics, motion-limit semantics, replay status, and execution-record envelope.
  - Decide equality and lifecycle rules, including exactly when a retry obtains a new attempt.
  - Require that Agent-generated IDs and receive times can use injected providers in tests.
  - Output: `docs/design/execution-evidence-api.md`.
  - Dependencies: TCRR-000.

- [x] **TCRR-002 (P0): Remove compatibility-only APIs and freeze one canonical pre-release model.**
  - Remove weak overloads, parallel old/new fields, aliases, and wire-compatibility fixtures.
  - Require complete optional execution context and exact-handle cancellation.
  - Cover all canonical proto-to-C++ and C++-to-proto paths.
  - Dependencies: TCRR-001.

### Phase 1 - trustworthy execution identity and telemetry evidence

- [x] **TCRR-010 (P0): Add injectable clock and ID/session providers.**
  - Introduce small internal abstractions for wall time, monotonic time, Agent session IDs, goal-attempt IDs, and recorder/run IDs.
  - Generate one unique `agent_session_id` at Agent service startup; never imply it is a persisted numeric epoch.
  - Expose the session in health/ping/capabilities or a dedicated identity response, telemetry, reports, and records.
  - Primary files: `src/agent/server.cpp`, new small core/internal headers, test service factory.
  - Tests: deterministic IDs/times and different session after service restart.
  - Dependencies: TCRR-002.

- [x] **TCRR-011 (P0): Implement generic execution context and goal-attempt lineage.**
  - Add client-supplied mission ID/revision, model hash, operation ID, operation-attempt revision, origin client, and correlation context without graph/rotor fields.
  - On each accepted physical execution, assign an Agent-generated physical-attempt ID and an Agent-session-scoped physical-attempt sequence/revision. Keep this distinct from the controller's operation-attempt revision.
  - Return the exact finalized handle from `SetActiveGoal`; include it in `GetActiveGoal`, all goal reports, command/backend result records, and normalized telemetry.
  - Define and test `ACTIVE`, `REACHED`, `CANCELLED`, `SUPERSEDED`, `FAILED`, `TIMEOUT`, retry, and replacement semantics.
  - Bind cancellation to one complete handle; provide no weaker goal-ID-only overload.
  - Primary files: `proto/swarmkit.proto`, `include/swarmkit/commands.h`, `include/swarmkit/client/client.h`, `src/agent/active_goal_supervisor.*`, `src/agent/server.cpp`, `src/client/client.cpp`, `src/client/swarm_client.cpp`.
  - Tests: same goal/new revision, same revision/new attempt, supersession, delayed old report, and failed backend execution.
  - Dependencies: TCRR-010.

- [x] **TCRR-012 (P0): Move execution binding and sequencing into the normalized telemetry ingress.**
  - Assign `agent_session_id + drone_id + telemetry_sequence` exactly once when a backend frame enters `TelemetryManager`.
  - Keep sequence state for the Agent session even when no SDK subscriber is connected; never reset it on stream replacement.
  - Snapshot the current execution/goal handle at normalization time, not later in each subscriber loop.
  - Keep `stream_id` as separate per-RPC transport metadata and preserve it in SDK stream metadata rather than confusing it with producer identity.
  - Primary files: `include/swarmkit/core/telemetry.h`, `src/agent/telemetry_manager.*`, `src/agent/server.cpp`, `src/client/client.cpp`.
  - Tests: monotonic sequence, two subscribers see identical producer IDs, reconnect continuity, and session reset only after Agent restart.
  - Dependencies: TCRR-011.

- [x] **TCRR-013 (P0): Add per-measurement timing, update, and freshness provenance.**
  - Distinguish frame normalization time, Agent receive wall/monotonic time, position source time, velocity source time, accuracy source time, vehicle boot time, and optional SDK receive time.
  - Add source clock domain, timestamp validity, synchronization state, and optional clock uncertainty.
  - Add update flags/sample generations for at least position, velocity, accuracy, estimator, and failsafe so repeated cached values cannot count as new physical samples.
  - Remove ambiguous aggregate timestamps; retain canonical Agent times and per-group source evidence.
  - Ensure MAVLink timestamps remain associated with the measurement group that supplied them.
  - Tests: attitude/heartbeat updates cannot refresh position provenance; unknown clock uncertainty remains absent; source and Agent receive times survive all conversions.
  - Dependencies: TCRR-012.

- [x] **TCRR-014 (P0): Add explicit uncertainty semantics and provenance.**
  - Add semantics such as unknown, standard deviation, confidence bound, empirically calibrated bound, deterministic hard bound, and backend-specific.
  - Represent confidence level and calibration profile/version only when meaningful.
  - Split velocity uncertainty into horizontal and vertical components; keep generic speed uncertainty only when it is the truthful backend quantity.
  - Associate each uncertainty with its backend field/source and relevant measurement generation.
  - Label current MAVLink GPS accuracy as backend-specific/unknown statistical semantics unless authoritative documentation and measurement association justify something stronger. Never mark it as a hard bound.
  - Tests: all semantic values round-trip, unknown stays unknown, invalid values remain absent, and MAVLink is never mislabeled.
  - Dependencies: TCRR-013.

- [x] **TCRR-015 (P0): Type evidence capabilities and motion-limit provenance.**
  - Add typed support declarations for source time, position, velocity, XY/Z uncertainty, uncertainty semantics, estimator health, failsafe, goal lineage, telemetry sequencing, and replay.
  - Add motion-limit source/semantics: configured command limit, platform assumption, observed limit, validated bound, or unknown.
  - Replace telemetry-field strings and split numeric/provenance structures with typed canonical capabilities.
  - Mark simulator limits according to what the simulator actually enforces; keep MAVLink limits unknown until a vehicle profile or validated source is explicit.
  - Tests: simulator, MAVLink, recording backend, proto conversions, and absent/unknown semantics.
  - Dependencies: TCRR-014.

### Phase 2 - replayable and reconstructable SwarmKit evidence

Implementation complete. Per project direction, Phase 2 added no new tests; its deferred correctness matrix remains consolidated under TCRR-033.

- [x] **TCRR-020 (P1): Add bounded telemetry retention and the canonical replay/live evidence stream.**
  - Add a new stream shape that can carry normalized frames plus stream-start/session metadata, replay range, live boundary, session mismatch, and history-evicted status.
  - Support `after_sequence`, `expected_agent_session_id`, and bounded replay configured per Agent/drone.
  - Never silently bridge an unavailable range; emit an explicit history-loss event.
  - Bound ring memory by configured frame count and/or bytes.
  - Tests: exact replay, live handoff without a race, eviction, old-session request, and multiple subscribers.
  - Dependencies: TCRR-015.

- [x] **TCRR-021 (P1): Add SDK cursor, gap, duplicate, reorder, and reconnect handling.**
  - Expose producer identity and a typed stream observation/event callback.
  - Detect next, duplicate, older/reordered, gap, session change, replay/live boundary, and history loss.
  - Advance reconnect cursors from the last accepted producer sequence rather than replaying from the original static cursor.
  - Distinguish producer gaps from SDK callback-queue drops; preserve existing backpressure policy and dropped-callback counters.
  - Evolve `StartTelemetry` to expose the canonical observation contract; do not retain a weaker wrapper.
  - Tests: scripted 10, 11, 13, delayed 12, duplicate 13, reconnect, local callback drop, and new session.
  - Dependencies: TCRR-020.

- [x] **TCRR-022 (P1): Implement a generic deterministic execution-event recorder.**
  - Use a versioned protobuf envelope with one Agent-global event sequence and Agent session.
  - Record session start, normalized telemetry, command request, backend result/ACK detail, goal lifecycle/attempt, reports, health/failsafe changes, and authority changes.
  - Record run/scenario ID, seed, software version, configuration hash, backend identity, and calibration profile reference.
  - Use deterministic protobuf serialization or define semantic canonicalization; include record length/checksum and clean-truncation detection.
  - Configure bounded rotation/retention and an explicit loss policy. Scientific mode must fail or mark the run invalid if evidence is dropped.
  - Retire report-only persistence when the generic recorder fully supersedes it; console logs remain diagnostic.
  - Tests: stable global order under concurrent publishers, replay equality, rotation, truncated/corrupt record detection, and recorder-overflow policy.
  - Dependencies: TCRR-020.

- [x] **TCRR-023 (P1): Surface typed command/backend outcomes for the recorder.**
  - Record command kind and parameters, execution context, dispatch time, transport/backend response, MAVLink ACK code/status text where available, and commands with no autopilot ACK.
  - Do not infer movement or arrival from ACK.
  - Change `IDroneBackend::Execute` if needed to make backend outcomes explicit and typed.
  - Tests: accepted, rejected, failed, ACKed, ACK timeout, setpoint-without-ACK, and ACK-without-motion.
  - Dependencies: TCRR-022.

### Phase 3 - deterministic test and experiment infrastructure

- [ ] **TCRR-030 (P1): Promote a scripted/replay backend for exact evidence tests.**
  - Feed explicitly ordered telemetry and health changes using a manual clock and ID source.
  - Script backend command outcomes and ACK details.
  - Use the scripted backend to test normalization. Separately replay already-normalized records through the same controller-facing evidence interface without silently renumbering or restamping them.
  - Do not use sleep to express event order.
  - Tests: the complete stale/reorder/duplicate/supersession/restart sequence from the readiness prompt.
  - Dependencies: TCRR-023.

- [ ] **TCRR-031 (P1): Add a seeded deterministic fault-injection decorator/scheduler.**
  - Support loss, delay, duplication, reordering, bias, noise, spikes, clock offset/uncertainty, estimator degradation, invalid accuracy, ACK without motion, target crossing, backend failure, supersession, restart/session change, interruption, stale rejoin, and duplicate requests where the interface permits.
  - Log the seed, fault configuration, and realized fault decisions as evidence.
  - Keep test randomness outside the production MAVLink execution path.
  - Tests: same scenario/config/seed yields the same logical fault sequence; a changed seed changes at least one randomized decision.
  - Dependencies: TCRR-030.

- [ ] **TCRR-032 (P1): Replace/extend the demo simulator with command-responsive deterministic motion.**
  - Replace demo dynamics where necessary; keep one simulator contract rather than parallel legacy/experimental factories.
  - Model goal-directed position/velocity, hold, high-speed crossing, failure, and configurable rate/geometry sufficiently for controlled experiments.
  - Provide independent simulation truth through an experiment-only channel/file that cannot be mistaken for `TelemetryFrame`.
  - Enforce or accurately classify advertised motion limits.
  - Tests: deterministic trajectory, command response, truth/estimate separation, and limit semantics.
  - Dependencies: TCRR-031.

- [ ] **TCRR-033 (P1): Complete the SwarmKit correctness test matrix.**
  - Add unit tests for every conversion and pure state rule.
  - Add Agent/SDK integration tests for identity, sequencing, replay, gaps, lineage, sessions, recorder, and backpressure.
  - Add sanitizer/release CI coverage and schema linting for the canonical protocol.
  - Remove avoidable sleep-based waits from the new correctness tests.
  - Dependencies: TCRR-032.

- [ ] **TCRR-034 (P1): Publish the SwarmKit evidence contract and correct existing docs.**
  - Add `docs/tcrr-readiness.md` describing guarantees, non-guarantees, identity, session, sequencing, timing, uncertainty, replay, recording, and capability semantics.
  - Correct `docs/SCIENTIFIC_CONTEXT.md`: `GOAL_REACHED` is an execution-layer baseline; rotor state may be updated only by the external TCRR guarded commit.
  - Document `Command ACK != physical arrival`, `accuracy != hard bound unless explicitly declared`, and `SwarmKit != rotor consensus`.
  - Add a public SDK example that sets a goal, receives its finalized execution handle, consumes ordered evidence, and inspects timing/uncertainty/health.
  - Dependencies: TCRR-033.

### Gate 1 - SwarmKit evidence-ready acceptance

Do not start `tcrr-core` integration until all are true:

- [ ] One normalized frame has an immutable Agent session, drone ID, producer sequence, execution handle snapshot, and measurement provenance.
- [ ] A reconnect can replay or explicitly report the missing interval.
- [ ] Old-session and old-attempt evidence are mechanically distinguishable.
- [ ] Unknown timing/accuracy/bound semantics remain unknown.
- [ ] A complete run can be reconstructed from the machine-readable log with no silent evidence loss.
- [ ] The scripted/fault layer reproduces the same event sequence for a fixed seed.
- [ ] All repository consumers use only the canonical API; no compatibility wrappers or fixtures remain.
- [ ] No graph, rotor, reservation, certificate, or commit logic exists in SwarmKit core.

### Phase 4 - external TCRR and baseline implementation

These tasks belong in a separate library/application that depends only on SwarmKit's public SDK/protobuf API. It may live in a separate repository or a clearly separate top-level project, but it must not include SwarmKit Agent internals.

- [ ] **TCRR-100 (P2): Create `tcrr-core` domain types and deterministic configuration.**
  - Graph model and ordered neighbor lists.
  - Versioned rotor/logical-position state.
  - Pending transition and reservation/state-provider interfaces.
  - Target geometry, certificate configuration, run configuration, and canonical hashing.
  - Dependencies: Gate 1.

- [ ] **TCRR-101 (P2): Implement evidence-window eligibility and arrival certificate.**
  - Require current execution handle/session, new position measurements, freshness, estimator/failsafe validity, acceptable uncertainty semantics, containment, velocity, K samples, and hold span.
  - Calculate enclosure propagation and margins outside SwarmKit.
  - Store sample/time ranges, target geometry, margins, decision/reason, and evidence hash.
  - Explicitly distinguish deterministic and probabilistic certificate modes.
  - Dependencies: TCRR-100.

- [ ] **TCRR-102 (P2): Implement linearizable guarded commit and recovery semantics.**
  - Provide an in-process transactional state provider for experiments and keep the distributed provider as an interface/assumption.
  - Enforce current, state matches, not previously committed, and exactly-once transition ID.
  - Implement abort, timeout, supersession, orphan/session change, retry, and duplicate commit behavior as no logical change.
  - Dependencies: TCRR-101.

- [ ] **TCRR-103 (P2): Implement ideal trace projector and validator.**
  - Project only successful commits.
  - Verify every projected step against the ideal rotate-then-move relation.
  - Emit exact invalid-step diagnostics and trace hashes.
  - Dependencies: TCRR-102.

- [ ] **TCRR-104 (P2): Implement all paper baselines behind one evaluator interface.**
  - Command ACK, fixed radius, radius+dwell, SwarmKit `GOAL_REACHED`, position-only TCRR, and full TCRR.
  - Record each method's claimed occupancy/commit time and decision reasons.
  - Run offline baselines on the same normalized evidence log where method behavior does not alter the physical trajectory.
  - Dependencies: TCRR-103.

- [ ] **TCRR-105 (P2): Build the experiment runner and immutable run manifest.**
  - Scenario ID/version/hash, method, seed, software commits, configs, calibration profile, node geometry, airframe/backend, expected faults, and output checksums.
  - Produce normalized long-form result records suitable for statistical scripts.
  - Measure latency, CPU, memory, and bytes at defined boundaries.
  - Reject/flag incomplete logs rather than silently analyzing them.
  - Dependencies: TCRR-104.

- [ ] **TCRR-106 (P2): Prove replay determinism and complete adversarial tests.**
  - Same log plus same TCRR config yields the same eligibility, certificate, commit, and projected trace decisions.
  - Cover stale, duplicate, superseded, failed, old-session, duplicate-commit, target-crossing, and changed-source-state cases.
  - Dependencies: TCRR-105.

- [ ] **TCRR-107 (P2): Run a pilot and lock the primary sample size/scenario matrix.**
  - Use pilot-only runs to estimate run-to-run variability, transition clustering, event prevalence, and feasible opportunities per run.
  - Calculate the independent run count and transition-opportunity target from the predeclared false-commit precision/one-sided upper-bound objective in TCRR-000.
  - Freeze the paired-seed list, scenario strata, stopping rule, and primary analysis before primary data collection.
  - Keep pilot/tuning runs out of the primary result dataset.
  - Dependencies: TCRR-106.

### Gate 2 - experiment-ready acceptance

- [ ] Every method consumes a declared evidence set and emits a declared commit time.
- [ ] Ground-truth labeling is independent of estimator telemetry.
- [ ] Trace validation runs automatically for every completed run.
- [ ] Fixed seeds and manifests reproduce scenarios and decisions.
- [ ] All required outcomes and denominators are generated without manual log interpretation.
- [ ] Incomplete or recorder-loss runs are marked invalid according to TCRR-000.
- [ ] Primary run count, opportunity target, scenario matrix, paired seeds, and stopping rule are locked by TCRR-107.

### Phase 5 - empirical data collection

- [ ] **TCRR-200 (P3): Select and validate independent ground truth and time alignment.**
  - Simulation truth for controlled simulation.
  - SITL truth from a source not fed into the evaluated telemetry path.
  - Physical truth from motion capture, RTK reference, surveyed fiducials/vision, or another justified independent source.
  - Quantify ground-truth uncertainty and time-alignment error; include them in labels or limitations.
  - Dependencies: Gate 2.

- [ ] **TCRR-201 (P3): Run enclosure calibration and held-out validation.**
  - Collect hover, cruise, turn, and disturbed regimes over predeclared independent runs.
  - Split calibration and held-out validation by run, not by randomly mixing samples from the same flight.
  - Version the resulting calibration profile and lock it before primary evaluation.
  - Determine whether evidence supports a hard-bound claim or only an empirical/probabilistic claim; do not upgrade the claim based on desired results.
  - Output Table I source data and plots.
  - Dependencies: TCRR-200.

- [ ] **TCRR-202 (P3): Run controlled simulation experiments.**
  - Vary delay, loss, reorder, duplicate, noise, bias, spikes, clock error, estimator state, ACK/no-progress, target speed/crossing, node geometry, sample rate, supersession, failure, and session change.
  - Use paired seeds and a frozen scenario matrix.
  - Run offline same-log comparisons and closed-loop comparisons as separate analyses.
  - Dependencies: TCRR-201.

- [ ] **TCRR-203 (P3): Run multi-UAV ArduPilot SITL experiments.**
  - Seeded missions, concurrent transitions, goal supersession, Agent restart, stale rejoin, network interruption/partition where the harness permits, and duplicate commit requests.
  - Capture Agent logs, TCRR records, SITL truth, configs, versions, and resource/network measurements.
  - Dependencies: TCRR-202.

- [ ] **TCRR-204 (P3): Run a safe physical-flight subset.**
  - Obtain flight-test approvals/checklists and define abort conditions.
  - Repeat calibration and selected safety/overhead scenarios with independent truth where available.
  - Treat airframe/autopilot/estimator configuration as part of the experimental condition.
  - Dependencies: TCRR-203 and successful held-out calibration for the physical configuration.

### Phase 6 - statistical analysis and paper completion

- [ ] **TCRR-300 (P4): Lock and validate the analysis dataset.**
  - Verify checksums, schemas, session continuity, replay completeness, expected seed pairs, exclusions, and denominators.
  - Produce an immutable data dictionary and CONSORT-like run-accounting table: scheduled, started, completed, excluded, and reason.
  - Dependencies: TCRR-202; update after TCRR-203/204.

- [ ] **TCRR-301 (P4): Execute the predeclared statistical analysis.**
  - Counts and denominators for all binary outcomes.
  - Wilson or Clopper-Pearson intervals as predeclared.
  - Run-clustered bootstrap intervals or mixed-effects models for repeated transitions.
  - Median/IQR for skewed timing and overhead.
  - Absolute risk difference and risk ratio versus full TCRR.
  - Holm correction for multiple baseline comparisons.
  - One-sided upper bound for zero corrupt commits.
  - Sensitivity analyses for calibration profile, missing runs, ground-truth uncertainty, and alternative occupancy-time definitions.
  - Dependencies: TCRR-300.

- [ ] **TCRR-302 (P4): Generate publication artifacts reproducibly.**
  - Table I and Table II from scripts, never manual transcription.
  - Figures for false-commit rate, latency/overhead, calibration coverage, and fault-class rejection.
  - Machine-readable CSV/JSON plus rendered paper tables with counts, denominators, intervals, units, and configuration footnotes.
  - Dependencies: TCRR-301.

- [ ] **TCRR-303 (P4): Fill and audit the paper.**
  - Replace every `TBD`/`[RESULT: ...]` placeholder.
  - Align claims with deterministic versus probabilistic calibration evidence.
  - State exactly which stages/backends/airframes were tested.
  - Add the two placeholder figures, verified funding/facility acknowledgment, limitations, and artifact availability.
  - Run an automated placeholder search and an independent numbers-to-source-data audit.
  - Dependencies: TCRR-302.

## Recommended first implementation slice

The completed Gate 0/Phase 1 slice established the identity spine. The next implementation slice is:

1. bounded telemetry retention and replay/live metadata;
2. SDK gap/session/history observations;
3. the generic execution recorder;
4. typed backend outcomes;
5. scripted replay and seeded fault injection.

These tasks make the canonical evidence model reconstructable before TCRR consumes it.

## Explicitly deferred or prohibited

- No rotor configuration, graph model, reservation, arrival certificate, commit engine, trace theorem logic, or coverage algorithm in SwarmKit core.
- No claim that `GOAL_REACHED` certifies TCRR arrival.
- No claim that MAVLink accuracy is a deterministic bound without explicit supported semantics and calibration.
- No fabricated persisted Agent epoch; use a unique process-lifetime session identity.
- No distributed rotor-state consensus in this work.
- No physical safety claim before held-out calibration and independent ground-truth validation.
- No unbounded telemetry, event, callback, or fault-scheduler queue.

## Completion definition

The overall program is complete when:

```text
same normalized execution log
+ same TCRR configuration
-> same certificate and commit decisions
-> same projected rotor-router trace
-> reproducible statistical result rows
```

and the implementation boundary remains:

```text
SwarmKit: typed physical execution evidence
tcrr-core: evidence interpretation and guarded logical commit
experiment layer: independent truth, seeded scenarios, metrics, and statistics
```
