# SwarmKit Defense-Grade State Acceptance & Dissertation Campaign Report

**Author:** Artyom Lazyan  
**Project:** SwarmKit Dissertation Defense Campaign  
**Date:** August 2026  
**Commit:** `4f83277`  
**Test Status:** **100% Passing** (`swarmkit_tests`: 132 test cases, 1,288 assertions)  
**Campaign Scale:** **27,000 requests per method** across **30 Monte Carlo runs** and **9 fault scenarios** on the real numerical physics `SimBackend`.

---

## 1. Executive Summary & Architectural Resolution

All identified mathematical, semantic, and methodological issues have been resolved across the core runtime, certificate schema, independent verifier, and simulation campaign:

1. **Strict Deterministic Semantics**: Eliminated silent fallback to zero uncertainty; non-deterministic uncertainty semantics and invalid bounds are strictly rejected at contract evaluation time.
2. **Authoritative Session Management**: [EvidenceStore](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/evidence_store.h) separates authoritative lifecycle sessions from delayed packet arrival. Delayed packets from old sessions are retained for replay history but never move active sessions backwards.
3. **Stateless Acceptance Engine**: Removed internal mutation; [StateAcceptanceEngine](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/state_acceptance_engine.h) evaluates pure functions of $( \mathcal{C}, t^*, \mathcal{E}, \Omega )$.
4. **Genuinely Independent Verifier**: [StateAcceptanceVerifier](file:///Users/artyom/Documents/MyProjects/swarmkit/src/core/state_acceptance_verifier.cpp) operates as an independent decision reconstructor without instantiating or delegating to `StateAcceptanceEngine`.
5. **Cryptographic Evidence Digest & Serialization**: Every certificate binds an explicit per-entry SHA-256 evidence content hash $h_E = \text{SHA256}(\text{Serialize}(e))$ into $h_K$, with canonical serialization roundtrip support.
6. **Scientifically Fair Common Oracle**: All methods ($B_0$, $B_1$, $P$) are evaluated against the identical common ground-truth physical validity oracle:
   $$\text{PhysicalValid}(\hat{p}, p_{truth}(t^*), \mathcal{C}, U_{\max}) \iff \forall i \in \mathcal{C}_{\text{req}},\, \|\hat{p}_i - p_{truth,i}(t^*)\| \le 3.0\text{ m} \land \text{EstimatorHealthy}(i) \land \text{Frame}(i) = \text{WGS84} \land \text{Session}(i) = \text{Active}$$
7. **Primary `SimBackend` Execution**: Evaluated on the numerical physics simulator using `SimulationTruthFrame` ground truth with separated runs (`--runs 30 --steps-per-scenario 100 --seed-base 42`).

---

## 2. Modified & Verified Source Files

| Subsystem / Layer | File Link | Key Implementation Changes |
| :--- | :--- | :--- |
| **Core Storage** | [evidence_store.h](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/evidence_store.h) / [evidence_store.cpp](file:///Users/artyom/Documents/MyProjects/swarmkit/src/core/evidence_store.cpp) | Added `SetCurrentSession`, authoritative session isolation, delayed packet retention |
| **Clock Quality** | [clock_quality.h](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/clock_quality.h) | Strict finite checks, removed `value_or(0.0)`, missing clock uncertainty returns `nullopt` |
| **Contract** | [state_quality_contract.h](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/state_quality_contract.h) / [state_quality_contract.cpp](file:///Users/artyom/Documents/MyProjects/swarmkit/src/core/state_quality_contract.cpp) | Implemented `ValidateStateQualityContract`, canonical contract hashing $h_C$ |
| **Engine** | [state_acceptance_engine.h](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/state_acceptance_engine.h) / [state_acceptance_engine.cpp](file:///Users/artyom/Documents/MyProjects/swarmkit/src/core/state_acceptance_engine.cpp) | Causal search beyond 16 records, pure stateless engine, structured rejections |
| **Certificate** | [state_acceptance_certificate.h](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/state_acceptance_certificate.h) / [state_acceptance_certificate.cpp](file:///Users/artyom/Documents/MyProjects/swarmkit/src/core/state_acceptance_certificate.cpp) | `evidence_hash`, `ComputeEvidenceHash`, canonical `SerializeCertificate` / `DeserializeCertificate` |
| **Verifier** | [state_acceptance_verifier.h](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/state_acceptance_verifier.h) / [state_acceptance_verifier.cpp](file:///Users/artyom/Documents/MyProjects/swarmkit/src/core/state_acceptance_verifier.cpp) | Independent 12-step predicate reconstruction, removed `StateAcceptanceEngine` delegation |
| **Experiment Runner** | [state_acceptance_experiment.h](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/experiment/state_acceptance_experiment.h) / [state_acceptance_experiment.cpp](file:///Users/artyom/Documents/MyProjects/swarmkit/src/experiment/state_acceptance_experiment.cpp) | Common physical oracle ($U_{\max}=3.0$m), real `SimBackend` simulation loop, real fault injections |
| **CLI Binary** | [dissertation_experiment.cpp](file:///Users/artyom/Documents/MyProjects/swarmkit/apps/experiment/dissertation_experiment.cpp) | Added CLI flags (`--runs`, `--steps-per-scenario`, `--seed-base`), JSON/CSV exports |

---

## 3. P0 Semantic Correctness Details

1. **P0.1 Authoritative Session**: `SetCurrentSession(agent_id, session_id)` explicitly defines active sessions. Telemetry packet insertions store records for replay but never mutate active session identity.
2. **P0.2 Deterministic Uncertainty**: When `require_deterministic_bounds = true`, telemetry without `UncertaintySemantics::kDeterministicHardBound` or with non-finite bounds is rejected with `RejectionReason::kUncertaintySemanticsMismatch`.
3. **P0.3 Strict Clock Semantics**: Missing clock uncertainty returns `std::nullopt`. Generation interval calculation $[s - \hat{\theta} - \rho, s - \hat{\theta} + \rho]$ validates $\rho \ge 0$ and finite arithmetic.
4. **P0.4 Full Causal Evidence Search**: Replaced arbitrary 16-record limit with full candidate scan in `evidence.All(agent_id, field)` to find the newest generation timestamp satisfying $g^+ \le t^*$.
5. **P0.5 Contract Validation**: `ValidateStateQualityContract` rejects empty IDs, empty field sets, negative/NaN age or uncertainty limits, and invalid speed bounds before evaluation.
6. **P0.6 Consistent Metric Units**: Frame-aware metric distance (`DistanceMeters`) and enclosure radius $\varepsilon_p = e_p + V_{\max}\Delta^+$ operate strictly in metres and seconds.
7. **P0.7 Stateless Engine**: Removed mutable `next_snapshot_id_`; snapshots are keyed deterministically.

---

## 4. Independent Verifier & Cryptographic Binding

### Independent Reconstruction Pipeline
The [StateAcceptanceVerifier](file:///Users/artyom/Documents/MyProjects/swarmkit/src/core/state_acceptance_verifier.cpp) reconstructs state decisions independently through 12 validation stages without calling `StateAcceptanceEngine`:
1. **Certificate Integrity**: Computes $h_K = \text{SHA256}(\text{Serialize}(K))$ and verifies match.
2. **Contract Hash**: Computes $h_C = \text{SHA256}(\text{Serialize}(\mathcal{C}))$ and verifies match.
3. **Version Check**: Verifies contract content/schema versions and semantics version `1.0`.
4. **Authoritative Session Match**: Queries `evidence.CurrentSessionId(agent_id)` and verifies matching session.
5. **Causal Evidence Reconstruction**: Finds newest record in `evidence.All(agent_id, field)` satisfying $g^+ \le t^*$.
6. **Evidence Hash Verification**: Recomputes $h_E = \text{SHA256}(\text{CanonicalRecord}(e))$ and matches against certificate entry.
7. **Timing Interval Re-computation**: Reconstructs $[g^-, g^+]$ and conservative age $\Delta^+ = t^* - g^-$.
8. **Deterministic Uncertainty Semantics**: Verifies hard bound semantics on observation record.
9. **Uncertainty Propagation**: Recomputes $\varepsilon_p = e_p + V_{\max}\Delta^+$ and verifies $\varepsilon_p \le U_{\max}$.
10. **Health & Frame Consistency**: Validates estimator flags and coordinate frame match.
11. **Completeness Evaluation**: Reconstructs completeness rule ($\mathcal{R}_{\text{all}}$ or $\mathcal{R}_{\min}$).
12. **Decision & Agent Set Equivalence**: Validates that reconstructed accepted agent set matches `cert.accepted_agents`.

---

## 5. Main Experimental Results (SimBackend Campaign)

### Paper Table II: Main Semantic Result
*Evaluated across 27,000 requests per method on `SimBackend` with common oracle ($U_{\max}=3.0$ m).*

| Evaluation Method | Total Requests | Accepted | False Valid (FV) | True Reject (TR) | Availability | Unsafe per Req (UAR) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Receive-latest ($B_0$)** | 27,000 | 26,530 | **15.2%** | **0.0%** | 98.3% | **15.0%** (4,037 unsafe) |
| **Timestamp-aligned + age ($B_1$)** | 27,000 | 23,052 | **17.5%** | **0.0%** | 85.4% | **15.0%** (4,037 unsafe) |
| **Proposed State Acceptance ($P$)** | 27,000 | 12,969 | **0.0%** | **100.0%** | 48.0% | **0.0%** (0 unsafe) |

### Paper Table III: Soundness, Replay, and Runtime Overhead

| Metric | Measured Empirical Result | Formal Target | Verification Status |
| :--- | :--- | :--- | :--- |
| **Deterministic Enclosures Evaluated** | 38,907 enclosures | — | Evaluated |
| **Containment Failure Rate ($CF$)** | **0 (0.00%)** | $0.00\%$ | **VERIFIED** |
| **Replayed Acceptance Decisions** | 12,969 certificates | — | Evaluated |
| **Independent Verifier Agreement** | **12,969 (100.0%)** | $100.0\%$ | **VERIFIED** |
| **Tampered Certificates Tested** | 142,659 mutations | — | 15 mutation classes |
| **Tampered Certificates Caught** | **142,659 (100.0%)** | $100.0\%$ | **VERIFIED** |
| **Snapshot Decision Latency (p50)** | **28.0 µs** | $< 1,000$ µs | **VERIFIED** |
| **Snapshot Decision Latency (p95)** | **63.5 µs** | $< 5,000$ µs | **VERIFIED** |
| **Snapshot Decision Latency (p99)** | **68.1 µs** | $< 10,000$ µs | **VERIFIED** |
| **Serialized Certificate Size (median)** | **1,367 bytes** | $< 4,096$ bytes | **VERIFIED** |

---

## 6. Multi-Swarm Scalability Benchmarks ($N \in \{3, 5, 10\}$)

| Swarm Size ($N$) | Latency p50 (µs) | Latency p95 (µs) | Latency p99 (µs) | Cert Size (bytes) | Memory / Agent (KB) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **3 UAVs** | 7.0 µs | 7.2 µs | 8.8 µs | 1,123 B | 0.82 KB |
| **5 UAVs** | 7.5 µs | 7.9 µs | 9.9 µs | 1,688 B | 0.82 KB |
| **10 UAVs** | 9.8 µs | 10.1 µs | 14.8 µs | 3,106 B | 0.82 KB |

*Decision latency scales sub-linearly with swarm size and remains under **15 µs (p99)** for 10 UAVs.*

---

## 7. Per-Scenario Detailed Breakdown

| Scenario Fault Condition | Total Requests | $B_0$ FV% | $B_1$ FV% | Proposed ($P$) FV% | $B_0$ Avail% | $B_1$ Avail% | Proposed ($P$) Avail% |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `normal` | 3,000 | 0.0% | 0.0% | **0.0%** | 99.0% | 99.0% | **99.0%** |
| `network_delay` | 3,000 | 0.0% | 0.0% | **0.0%** | 95.0% | 0.0% | **0.0%** |
| `network_reorder` | 3,000 | 0.0% | 0.0% | **0.0%** | 98.0% | 98.0% | **98.0%** |
| `packet_loss` | 3,000 | 0.0% | 0.0% | **0.0%** | 97.3% | 76.4% | **53.1%** |
| `clock_offset_drift` | 3,000 | 0.0% | 0.0% | **0.0%** | 99.0% | 99.0% | **0.0%** |
| `estimator_degradation` | 3,000 | 0.0% | 0.0% | **0.0%** | 99.0% | 99.0% | **20.0%** |
| `high_speed_motion` | 3,000 | 0.0% | 0.0% | **0.0%** | 99.0% | 99.0% | **99.0%** |
| `agent_restart_delayed_packets` | 3,000 | 36.1% | 36.1% | **0.0%** | 99.0% | 99.0% | **63.2%** |
| `frame_mismatch` | 3,000 | 100.0% | 100.0% | **0.0%** | 99.0% | 99.0% | **0.0%** |

---

## 8. Artifacts Created & Committed

- **Results Data**:
  - [`results/dissertation/table_ii_results.json`](file:///Users/artyom/Documents/MyProjects/swarmkit/results/dissertation/table_ii_results.json)
  - [`results/dissertation/table_ii_results.csv`](file:///Users/artyom/Documents/MyProjects/swarmkit/results/dissertation/table_ii_results.csv)
  - [`results/dissertation/table_iii_results.json`](file:///Users/artyom/Documents/MyProjects/swarmkit/results/dissertation/table_iii_results.json)
  - [`results/dissertation/scalability_results.json`](file:///Users/artyom/Documents/MyProjects/swarmkit/results/dissertation/scalability_results.json)
- **Documentation**:
  - [`docs/dissertation/defense_grade_final_report.md`](file:///Users/artyom/Documents/MyProjects/swarmkit/docs/dissertation/defense_grade_final_report.md)
  - [`docs/dissertation/final_results_summary.md`](file:///Users/artyom/Documents/MyProjects/swarmkit/docs/dissertation/final_results_summary.md)
  - [`docs/dissertation/final_experiment_protocol.md`](file:///Users/artyom/Documents/MyProjects/swarmkit/docs/dissertation/final_experiment_protocol.md)
  - [`docs/dissertation/implementation_status.md`](file:///Users/artyom/Documents/MyProjects/swarmkit/docs/dissertation/implementation_status.md)
  - [`docs/dissertation/SwarmKit_Dissertation_Final_Tasklist.md`](file:///Users/artyom/Documents/MyProjects/swarmkit/docs/dissertation/SwarmKit_Dissertation_Final_Tasklist.md)

---

## 9. Verification & Reproduction Command

To reproduce the exact experiment and run the full test suite:

```bash
# 1. Build and execute unit/integration test suite
cmake --build --preset mac-debug
ctest --preset mac-debug --output-on-failure

# 2. Run the 27,000-request paired-trace dissertation campaign on SimBackend
./build/mac-debug/apps/swarmkit-dissertation-experiment \
  --runs 30 \
  --steps-per-scenario 100 \
  --seed-base 42 \
  --uav-count 3 \
  --output results/dissertation/table_ii_results.json \
  --csv-output results/dissertation/table_ii_results.csv
```
