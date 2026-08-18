# SwarmKit Defense-Grade State Acceptance & Dissertation Campaign Report

**Author:** Artyom Lazyan  
**Project:** SwarmKit Dissertation Defense Campaign  
**Date:** August 2026  
**Test Status:** **100% Passing** (`swarmkit_tests`: 137 test cases, 1,335 assertions)  
**Campaign Scale:** **27,000 requests per method** across **30 Monte Carlo runs** and **9 fault scenarios** on the real numerical physics `SimBackend`.

---

## 1. Executive Summary & Architectural Resolution

All identified mathematical, semantic, and methodological issues have been resolved across the core runtime, certificate schema, independent verifier, and simulation campaign:

1. **Moving UAVs & Real Ingress Pipeline**: Primary campaign exercises physically moving UAVs (takeoff to 5m, continuous horizontal flight vectors up to 8.86 m/s) with real `TelemetryFrame` evidence ingress into `EvidenceStore::InsertFrame()`.
2. **Strict Deterministic Semantics**: Eliminated silent fallback to zero uncertainty; non-deterministic uncertainty semantics and invalid bounds are strictly rejected at contract evaluation time.
3. **Authoritative Session Management & Authentic Restart**: [EvidenceStore](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/evidence_store.h) separates authoritative lifecycle sessions from delayed packet arrival. Authentic restart tests capture pre-restart frames and re-inject them after authoritative restart, verifying that delayed packets never move active sessions backwards.
4. **Stateless Acceptance Engine**: Removed internal mutation; [StateAcceptanceEngine](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/state_acceptance_engine.h) evaluates pure functions of $( \mathcal{C}, t^*, \mathcal{E}, \Omega )$.
5. **Genuinely Independent Verifier & Offline Replay**: [StateAcceptanceVerifier](file:///Users/artyom/Documents/MyProjects/swarmkit/src/core/state_acceptance_verifier.cpp) operates as an independent decision reconstructor without instantiating or delegating to `StateAcceptanceEngine`. Fresh-store offline replay verifies deserialized wire certificates in an isolated store.
6. **Cryptographic Evidence Digest & Serialization**: Every certificate binds an explicit per-entry SHA-256 evidence content hash $h_E = \text{SHA256}(\text{Serialize}(e))$ into $h_K$, with canonical serialization roundtrip support.
7. **Scientifically Fair Common Oracle**: All methods ($B_0$, $B_1$, $P$) are evaluated against the identical common ground-truth physical validity oracle:
   $$\text{PhysicalValid}(\hat{p}, p_{truth}(t^*), \mathcal{C}, U_{\max}) \iff \forall i \in \mathcal{C}_{\text{req}},\, \|\hat{p}_i - p_{truth,i}(t^*)\| \le 3.0\text{ m} \land \text{EstimatorHealthy}(i) \land \text{Frame}(i) = \text{WGS84} \land \text{Session}(i) = \text{Active}$$
8. **Scalability Measurement Hardening**: Scalability benchmarks measure end-to-end `RequestSnapshot + BuildCertificate + SerializeCertificate` across 500 timed iterations without fabricated memory metrics.

---

## 2. Modified & Verified Source Files

| Subsystem / Layer | File Link | Key Implementation Changes |
| :--- | :--- | :--- |
| **Core Storage** | [evidence_store.h](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/evidence_store.h) / [evidence_store.cpp](file:///Users/artyom/Documents/MyProjects/swarmkit/src/core/evidence_store.cpp) | Added `SetCurrentSession`, authoritative session isolation, delayed packet retention |
| **Clock Quality** | [clock_quality.h](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/clock_quality.h) | Strict finite checks, removed `value_or(0.0)`, missing clock uncertainty returns `nullopt` |
| **Contract** | [state_quality_contract.h](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/state_quality_contract.h) / [state_quality_contract.cpp](file:///Users/artyom/Documents/MyProjects/swarmkit/src/core/state_quality_contract.cpp) | Implemented `ValidateStateQualityContract`, canonical contract hashing $h_C$ |
| **Engine** | [state_acceptance_engine.h](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/state_acceptance_engine.h) / [state_acceptance_engine.cpp](file:///Users/artyom/Documents/MyProjects/swarmkit/src/core/state_acceptance_engine.cpp) | Causal search beyond 16 records, pure stateless engine, structured rejections |
| **Certificate** | [state_acceptance_certificate.h](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/state_acceptance_certificate.h) / [state_acceptance_certificate.cpp](file:///Users/artyom/Documents/MyProjects/swarmkit/src/core/state_acceptance_certificate.cpp) | `CERT_V2`, `evidence_hash`, expanded evidence fields, canonical `SerializeCertificate` / `DeserializeCertificate` |
| **Verifier** | [state_acceptance_verifier.h](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/core/state_acceptance_verifier.h) / [state_acceptance_verifier.cpp](file:///Users/artyom/Documents/MyProjects/swarmkit/src/core/state_acceptance_verifier.cpp) | Independent 12-step predicate reconstruction, explicit comparison across all certified fields |
| **Experiment Runner** | [state_acceptance_experiment.h](file:///Users/artyom/Documents/MyProjects/swarmkit/include/swarmkit/experiment/state_acceptance_experiment.h) / [state_acceptance_experiment.cpp](file:///Users/artyom/Documents/MyProjects/swarmkit/src/experiment/state_acceptance_experiment.cpp) | Moving UAVs, real `FaultInjectingBackend` pipeline, fresh offline replay, 15-class tamper matrix, end-to-end timing |
| **CLI Binary** | [dissertation_experiment.cpp](file:///Users/artyom/Documents/MyProjects/swarmkit/apps/experiment/dissertation_experiment.cpp) | Compact Table II output, multi-table generation, JSON/CSV exports |

---

## 3. Independent Verifier & Cryptographic Binding

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

## 4. Main Experimental Results (SimBackend Campaign)

### Paper Table II: Main Semantic Result
*Evaluated across 27,000 requests per method on `SimBackend` with common oracle ($U_{\max}=3.0$ m).*

| Evaluation Method | Requests | Accepted | False Valid (FV) | Availability | Unsafe per Req (UAR) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Receive-latest ($B_0$)** | 27,000 | 26,866 | **22.1%** | 99.5% | **22.0%** |
| **Timestamp-aligned + age ($B_1$)** | 27,000 | 26,541 | **30.5%** | 98.3% | **30.0%** |
| **Proposed state acceptance ($P$)** | 27,000 | 16,684 | **1.8%** | 61.8% | **1.1%** |

### Paper Table III: Soundness, Replay, and Runtime Overhead

| Metric | Measured Empirical Result | Formal Target | Verification Status |
| :--- | :--- | :--- | :--- |
| **Deterministic Enclosures Evaluated** | 50,052 | — | Evaluated |
| **Containment Failure Rate ($CF$)** | **0 (0.00%)** | $0.00\%$ | **VERIFIED** |
| **Replayed Acceptance Decisions** | 16,684 | — | Evaluated |
| **Independent Verifier Agreement** | **16,684 (100.0%)** | $100.0\%$ | **VERIFIED** |
| **Tampered Certificates Tested** | 250,260 | — | 15 mutation classes |
| **Tampered Certificates Caught** | **250,260 (100.0%)** | $100.0\%$ | **VERIFIED** |
| **Snapshot Decision Latency (p50)** | **60.7 µs** | $< 1,000$ µs | **VERIFIED** |
| **Snapshot Decision Latency (p95)** | **89.8 µs** | $< 5,000$ µs | **VERIFIED** |
| **Snapshot Decision Latency (p99)** | **93.3 µs** | $< 10,000$ µs | **VERIFIED** |
| **Serialized Certificate Size (median)** | **1,461 bytes** | $< 4,096$ bytes | **VERIFIED** |

---

## 5. Multi-Swarm Scalability Benchmarks ($N \in \{3, 5, 10\}$)

| Swarm Size ($N$) | Latency p50 (µs) | Latency p95 (µs) | Latency p99 (µs) | Cert Size (bytes) |
| :--- | :--- | :--- | :--- | :--- |
| **3 UAVs** | 65.3 µs | 65.9 µs | 70.5 µs | 1,545 B |
| **5 UAVs** | 97.2 µs | 101.8 µs | 111.7 µs | 2,360 B |
| **10 UAVs** | 184.5 µs | 188.3 µs | 194.2 µs | 4,401 B |

*Decision latency scales linearly with swarm size and remains under **200 µs (p99)** for 10 UAVs.*

---

## 6. Per-Scenario Detailed Breakdown

| Scenario Fault Condition | Requests | $B_0$ FV% | $B_1$ FV% | Proposed ($P$) FV% | $B_0$ Avail% | $B_1$ Avail% | Proposed ($P$) Avail% |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `normal` | 3,000 | 0.0% | 0.0% | **0.0%** | 100.0% | 100.0% | **99.0%** |
| `network_delay` | 3,000 | 0.0% | 0.0% | **0.0%** | 98.0% | 98.0% | **98.0%** |
| `network_reorder` | 3,000 | 0.0% | 0.0% | **0.0%** | 99.6% | 99.6% | **99.0%** |
| `packet_loss` | 3,000 | 0.3% | 0.0% | **0.0%** | 97.9% | 88.1% | **50.7%** |
| `clock_offset_drift` | 3,000 | 0.0% | 0.0% | **0.0%** | 100.0% | 99.0% | **0.0%** |
| `estimator_degradation` | 3,000 | 97.5% | 99.9% | **80.2%** | 100.0% | 100.0% | **12.4%** |
| `high_speed_motion` | 3,000 | 0.0% | 0.0% | **0.0%** | 100.0% | 100.0% | **99.0%** |
| `agent_restart_delayed_packets` | 3,000 | 0.0% | 70.0% | **0.0%** | 100.0% | 100.0% | **98.0%** |
| `frame_mismatch` | 3,000 | 100.0% | 100.0% | **0.0%** | 100.0% | 100.0% | **0.0%** |

---

## 7. Artifacts Created & Committed

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

## 8. Verification & Reproduction Command

To reproduce the exact experiment and run the full test suite:

```bash
# 1. Build and execute unit/integration test suite
cmake --build --preset mac-debug
ctest --preset mac-debug --output-on-failure

# 2. Run defense-grade dissertation campaign binary
./build/mac-debug/apps/swarmkit-dissertation-experiment \
    --runs 30 \
    --steps-per-scenario 100 \
    --seed-base 42 \
    --uav-count 3 \
    --output results/dissertation/table_ii_results.json \
    --csv-output results/dissertation/table_ii_results.csv
```
