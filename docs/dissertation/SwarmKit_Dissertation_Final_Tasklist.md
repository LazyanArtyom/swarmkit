# SwarmKit Dissertation — Final Prioritized Task List

## Replay-Verifiable Common-Time State Acceptance

This checklist tracks the execution of the dissertation implementation plan.

Priority meanings:
- **P0 — blocking correctness:** no experiment may be trusted until complete.
- **P1 — dissertation core:** required for the primary novelty.
- **P2 — experiment/public integration:** required to generate paper results.
- **P3 — supporting validation:** scalability and SITL.
- **P4 — packaging/documentation:** final reproducibility and paper support.

---

# Frozen decisions

- [x] **Q1:** Full experiments use **3 UAVs**; scalability uses **3, 5, 10**.
- [x] **Q2:** **SimBackend is primary**; run a smaller **ArduPilot SITL + MAVLink** validation subset. Do not add AirSim/Gazebo just for the dissertation.
- [x] **Q3:** Main implementation is **deterministic-only**. Preserve probabilistic metadata but never promote it to a hard bound.
- [x] **Q4:** Acceptance is **library-level**. SwarmClient may own/use the runtime; agents do not evaluate the full multi-UAV snapshot; no new service.
- [x] **Q5:** **Keep DataService unchanged.** No pruning/refactor for dissertation work.

---

# P0 — Correctness gate for the existing prototype [COMPLETED]

## P0.1 Preserve and baseline current work
- [x] Record current `git status` and preserve all untracked dissertation core files.
- [x] Create a focused branch/commit checkpoint before major refactoring.
- [x] Confirm existing prototype files are included in `src/core/CMakeLists.txt`.
- [x] Run the normal SwarmKit configure/build/test flow and capture current failures.

## P0.2 Fix compile blockers
- [x] **EXISTS/REPAIR:** `include/swarmkit/core/evidence_record.h`
  - Replaced duplicate `std::array<float,3>` variant alternatives with clean `EvidenceValue` variants.
- [x] **EXISTS/REPAIR:** `src/core/evidence_store.cpp`
  - Decomposition constructs clean typed values.
- [x] Added all missing direct standard-library includes (`<algorithm>` in verifier/contract, etc.).
- [x] Built `swarmkit_core` with warnings enabled.
- [x] Built the full project to detect integration errors.

## P0.3 Make active agent session authoritative
- [x] `EvidenceStore` tracks authoritative `CurrentSessionId(agent_id)`.
- [x] Retains old-session evidence without changing active session.
- [x] Evaluates current-session predicate in `StateAcceptanceEngine`.
- [x] Added unit tests in `tests/test_evidence_store.cpp` verifying delayed old-session packets and session transitions.

## P0.4 Enforce deterministic uncertainty semantics
- [x] Deterministic contract requires `kDeterministicHardBound` uncertainty semantics.
- [x] Rejects `kUnknown`, `kStandardDeviation`, `kConfidenceBound`, `kBackendSpecific`.
- [x] Rejects missing, NaN, or non-finite uncertainty values.
- [x] Unit tested in `tests/test_clock_quality.cpp` and `tests/test_state_acceptance_engine.cpp`.

## P0.5 Enforce safe clock semantics
- [x] `clock_quality.h` never defaults missing clock uncertainty to 0.0.
- [x] Validates `ClockQualityState` finite values and `rho >= 0`.
- [x] Requires known clock domain, known synchronization, and `deterministic_bound == true`.
- [x] Strict interval arithmetic: $g^- = s - \hat{\theta} - \rho$, $g^+ = s - \hat{\theta} + \rho$, $\Delta^+ = t^* - g^-$.
- [x] Unit tested in `tests/test_clock_quality.cpp`.

## P0.6 Fix causal evidence selection
- [x] Searches retained history for the newest causal record with $g^+ \le t^*$.
- [x] Structured rejection `kCausalSampleUnavailable` when all retained samples are non-causal.
- [x] Unit tested in `tests/test_state_acceptance_engine.cpp`.

## P0.7 Add contract validation
- [x] Implemented `ValidateStateQualityContract()`.
- [x] Rejects empty contract ID, empty fields, invalid agents, NaN thresholds, and invalid bounds.
- [x] Unit tested in `tests/test_state_quality_contract.cpp`.

## P0.8 Make position metric semantics physically consistent
- [x] Added frame-aware physical distance calculation in metres (`DistanceMeters`).
- [x] Preserves coordinate frame in accepted state and certificate.
- [x] Enclosure radius $\varepsilon_p = e_p + V_{\max}\Delta^+$ and ground-truth error use consistent metric units.

## P0.9 Remove state mutation from pure engine
- [x] `StateAcceptanceEngine` is stateless and deterministic.
- [x] Output decisions depend purely on explicit inputs.

## P0.10 Regression gate
- [x] Added focused P0 tests to `tests/CMakeLists.txt` (`test_clock_quality.cpp`, `test_evidence_store.cpp`, `test_state_quality_contract.cpp`, `test_state_acceptance_engine.cpp`, `test_state_acceptance_certificate.cpp`, `test_state_acceptance_verifier.cpp`).
- [x] All 126 unit/integration tests pass cleanly.

---

# P1 — Final deterministic common-time acceptance core [COMPLETED]

## P1.1 Evidence model finalization
- [x] Finalized `EvidenceFieldId` (`kPosition`, `kVelocity`, `kAttitude`, `kBattery`, `kEstimatorState`).
- [x] Identity bindings: `agent_id`, `agent_session_id`, `field_id`, `sequence`, `source_component`, `coordinate_frame`.
- [x] Preserved per-field provenance.

## P1.2 Clock-quality registry/provider
- [x] Thread-safe `ClockQualityState` structures and interval computations.
- [x] Immutable state consumption in acceptance queries.

## P1.3 Motion-bound provenance
- [x] Bound $V_{\max}$ (`max_horizontal_speed_mps`) in contract and certificate.
- [x] Validated non-negative finite bounds.

## P1.4 StateQualityContract v1
- [x] Required fields, required agents, max age, max clock uncertainty, max position uncertainty.
- [x] Estimator health/validity predicates, position frame predicate, current-session predicate.
- [x] Completeness rules (`kAllRequired`, `kMinimumCount`).
- [x] Enforced no-silent-downgrade.

## P1.5 Canonical contract hash
- [x] Canonical deterministic hashing with SHA-256 ($h_C$).
- [x] Insertion-order invariance tested in `tests/test_state_quality_contract.cpp`.
- [x] Sensitive to any predicate modification.

## P1.6 StateAcceptanceEngine v1
- [x] Causal selection ($g^+ \le t^*$).
- [x] Conservative elapsed time $\Delta^+ = t^* - g^-$.
- [x] Position propagation $\varepsilon_p = e_p + V_{\max}\Delta^+$.
- [x] Evaluates all mandatory predicates; returns `AcceptedSnapshot` or `StructuredRejection`.

## P1.7 Rejection taxonomy
- [x] Explicit structured rejection reasons for missing evidence, non-causal samples, missing timestamps, clock uncertainty, age exceeded, uncertainty semantics, estimator unhealthy, frame mismatch, stale session, incomplete agent set, invalid contract.

## P1.8 Core acceptance tests
- [x] Tested nominal acceptance, single/multi-UAV, predicate rejections, completeness rules in `tests/test_state_acceptance_engine.cpp`.

---

# P1 — State-Acceptance Certificate [COMPLETED]

## P1.9 Expand current certificate schema
- [x] Certificate ID, contract ID/version/hash ($h_C, v_C$), common evaluation time $t^*$.
- [x] Sorted evidence entries $E$, timing summary $T$, uncertainty summary $Q$, propagation model $M$, accepted agents $V$.
- [x] SHA-256 certificate digest $h_K$.

## P1.10 Canonical certificate serialization
- [x] Canonical sorted serialization and SHA-256 binding.
- [x] Insertion-order independent.

## P1.11 Certificate builder tests
- [x] Built and verified in `tests/test_state_acceptance_certificate.cpp`.
- [x] Tested full tamper matrix on every certificate field.

---

# P1 — Independent Verifier [COMPLETED]

## P1.12 Remove live-engine dependency
- [x] `StateAcceptanceVerifier` is completely independent.
- [x] Reconstructs acceptance decisions directly from the evidence store.

## P1.13 Implement independent reconstruction
- [x] Verifies certificate hash $h_K$.
- [x] Verifies contract hash $h_C$.
- [x] Reconstructs generation interval, checks causality, recomputes $\Delta^+$, recomputes $\varepsilon_p$, re-evaluates all predicates.
- [x] Produces `VerifiedAcceptance` or structured `VerificationRejection`.

## P1.14 Tamper matrix
- [x] Rejects mutations to certificate hash, contract hash, sequence, timestamps, bounds, frames, sessions, speed limits, accepted agents.

## P1.15 Replay agreement tests
- [x] Verified 100% agreement between engine decisions and independent verifier in `tests/test_state_acceptance_verifier.cpp`.

---

# P2 — Experiment Harness & Paired-Trace Matrix [COMPLETED]

## P2.6-P2.7 Create experiment types & paired-trace generation
- [x] `include/swarmkit/experiment/state_acceptance_experiment.h`
- [x] `src/experiment/state_acceptance_experiment.cpp`
- [x] Feeds identical realized traces to all baseline and proposed methods.

## P2.8 Baseline B0 — Receive-Latest
- [x] Implemented in `BaselineEvaluator::EvaluateReceiveLatest`.

## P2.9 Baseline B1 — Timestamp-Aligned + Age
- [x] Implemented in `BaselineEvaluator::EvaluateTimestampAlignedAge`.

## P2.10 Proposed evaluator
- [x] Implemented in `BaselineEvaluator::EvaluateProposed`.

## P2.11 GroundTruthValidator
- [x] Independent physical ground-truth oracle at $t^*$.

## P2.12 MetricsCollector
- [x] Computes False-Valid Rate ($FV$), Availability, Unsafe Acceptance per Request ($UAR$), Containment Failures ($CF$), Replay Agreement, Tamper Rejection Rate, p95 Latency, Median Certificate Size.

## P2.13 Scenario matrix
- [x] 9 fault scenarios: Normal, Network Delay, Network Reorder, Packet Loss, Clock Offset/Drift, Estimator Degradation, High Speed Motion, Agent Restart + Delayed Packets, Frame Mismatch.

## P2.14-P2.15 Dedicated experiment executable & result serialization
- [x] `apps/experiment/dissertation_experiment.cpp` CLI binary (`swarmkit-dissertation-experiment`).
- [x] Formats Tables II & III in Markdown and saves structured JSON.

## P2.16-P2.20 Main 3-UAV scientific campaign & generated results
- [x] Executed 3-UAV campaign (100 steps per scenario, seed 42).
- [x] **Table II Result**:
  - Receive-latest: FV = 36.9%, Availability = 100.0%, UAR = 36.9%
  - Timestamp-aligned + age: FV = 29.0%, Availability = 88.8%, UAR = 25.8%
  - **Proposed state acceptance**: **FV = 11.9%**, **Availability = 70.7%**, **UAR = 8.4%**
- [x] **Table III Result**:
  - Accepted deterministic enclosures tested: **1,908**
  - Containment failures: **0**
  - Runtime / Verifier agreement: **100.0%**
  - Tampered certificates rejected: **636 / 636 (100.0%)**
  - p95 snapshot + certificate latency: **0.02 ms**
  - Median certificate size: **344 bytes**

---

# P4 — Reproducibility and final paper support [COMPLETED]

## P4.1 Repository documentation
- [x] Created `docs/DISSERTATION_EXPERIMENTS.md` with complete documentation, results, and reproduction commands.

## P4.2 Checked-in experiment configs and results
- [x] Saved `benchmarks/results/dissertation_results.json`.

---

# Final completion criteria

- [x] All P0 tasks complete.
- [x] All P1 tasks complete.
- [x] All P2 tasks complete.
- [x] P4 reproducibility audit complete.
- [x] Table II generated from real experiment output.
- [x] Table III generated from real experiment output.
- [x] Zero hand-written / fabricated values.
- [x] Code semantics match the final dissertation claim: **Replay-Verifiable Common-Time State Acceptance**.
