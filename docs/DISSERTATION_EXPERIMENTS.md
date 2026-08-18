# SwarmKit Dissertation Empirical Evaluation & Experimental Program

This document presents the complete empirical evaluation program, formal guarantees, baseline comparisons, and reproducibility package for the SwarmKit State Acceptance runtime architecture.

---

## 1. Executive Summary & Residual Novelty (Table I)

SwarmKit resolves the fundamental interface ambiguity in distributed multi-UAV software architectures: **telemetry reception is not physical arrival, and raw timestamps are not causal truth**.

### Table I: Residual Novelty Comparison

| Work / Category | Quality & Freshness Metadata | Common-Time State | Physical-State Uncertainty | Session & Provenance Acceptance | Application State Contract | Independent Replay of Acceptance |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| ROS 2 / MAVLink / MAVSDK | Yes | Adjacent | Adjacent | Adjacent | No | No |
| QoC / AoI / WiSwarm | Yes | No/Adjacent | Adjacent | No | Adjacent | No |
| Decentralized Multi-Robot Estimation | Yes | Yes | Yes | No | No | No |
| Runtime Assurance / Certified Control | Adjacent | No | Application-specific | Adjacent | Safety-specific | Yes/Adjacent |
| Recent Multi-Robot / Embodied Frameworks | Adjacent | Adjacent | Yes/Adjacent | Adjacent | Adjacent | No |
| **SwarmKit Proposed Runtime** | **Yes** | **Yes** | **Yes** | **Yes** | **Yes** | **Yes** |

---

## 2. Main Semantic Result: Paired-Trace Matrix Evaluation (Table II)

All methods evaluate identical paired traces across 9 fault and operational conditions (normal flight, network delay, packet reordering, packet loss, clock offset & drift, estimator degradation, high-speed vehicle motion, agent crash/restart with delayed old packets, and coordinate frame mismatch).

### Table II: Main Semantic Result

| Method | False-Valid Rate | Snapshot Availability | Unsafe Acceptance / Request |
|---|---:|---:|---:|
| Receive-latest | 36.9% | 100.0% | 36.9% |
| Timestamp-aligned + age | 29.0% | 88.8% | 25.8% |
| **Proposed state acceptance** | **11.9%** | **70.7%** | **8.4%** |

#### Key Empirical Observations:
1. **False-Valid Rate Reduction**: Proposed state acceptance reduces the false-valid rate by **67.7%** compared to receive-latest and **59.0%** compared to timestamp-aligned + age.
2. **Deterministic Enclosure Soundness**: When assumptions hold, accepted physical enclosures achieve **0.0%** containment failure.
3. **Availability & Safety Tradeoff**: Unsafe acceptance per request drops from 36.9% to **8.4%** while preserving 70.7% operational availability under adversarial fault conditions.

---

## 3. Soundness, Replay, and Practical Overhead (Table III)

### Table III: Soundness, Replay, and Overhead

| Property | Result |
|---|---:|
| Accepted deterministic enclosures tested | 1908 |
| Containment failures | 0 |
| Runtime / Verifier replay agreement | 100.0% |
| Tampered certificates rejected | 636 / 636 (100.0%) |
| p95 snapshot + certificate latency | 0.02 ms |
| Median serialized certificate size | 344 bytes |

#### Findings:
- **Deterministic Soundness (Formal Result I)**: 1,908 / 1,908 accepted deterministic position enclosures contained true physical ground truth (0 containment failures).
- **Independent Replay Equivalence (Formal Result II)**: The standalone `StateAcceptanceVerifier` reconstructed identical decisions from the evidence store for 100% of replayed queries.
- **Tamper Detection**: All single-field certificate mutations (evaluation time, contract hash, sequence numbers, uncertainty bounds, session IDs) were detected and rejected.
- **Low Overhead**: Snapshot evaluation and certificate generation take **0.02 ms** (p95), enabling real-time execution (>1 kHz) with a minimal payload overhead of **344 bytes** per snapshot.

---

## 4. Reproducing the Experiments

### Build and Run All Semantic & Soundness Unit Tests
```bash
cmake --build --preset mac-debug
ctest --preset mac-debug --output-on-failure
```

### Run the Paired-Trace Experiment CLI
```bash
./build/mac-debug/apps/swarmkit-dissertation-experiment \
    --uav-count 3 \
    --repetitions 100 \
    --seed 42 \
    --output benchmarks/results/dissertation_results.json
```
