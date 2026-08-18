# SwarmKit State Acceptance — Final Results Summary

**Author:** Artyom Lazyan  
**Project:** SwarmKit Dissertation Defense Campaign  
**Date:** August 2026  
**Execution Environment:** macOS ARM64 | Clang C++23 | SimBackend Manual Clock Integration  

---

## Executive Summary

This document summarizes the defense-grade experimental campaign for **Replay-Verifiable Common-Time State Acceptance** in multi-UAV swarms. All evaluations were executed on the real `SimBackend` simulation engine using paired physical traces with synchronized ground truth, real network/clock/estimator fault injections, an identical common physical-validity oracle ($U_{\max} = 3.0$ m), and an independent offline verifier.

The campaign evaluated **27,000 requests per method** across **30 Monte Carlo runs** and **9 fault scenarios** for a 3-UAV swarm, followed by scalability benchmarks up to 10 UAVs.

---

## Table II: Main Semantic Result

All methods evaluated using the identical common ground-truth physical validity oracle:
$$\text{PhysicalValid}(\hat{p}, p_{truth}(t^*), \mathcal{C}, U_{\max}) \iff \forall i \in \mathcal{C}_{\text{req}},\, \|\hat{p}_i - p_{truth,i}(t^*)\| \le 3.0\text{ m} \land \text{EstimatorHealthy}(i) \land \text{Frame}(i) = \text{WGS84} \land \text{Session}(i) = \text{Active}$$

| Evaluation Method | Total Requests | Accepted | False Valid (FV) | True Reject (TR) | Availability | Unsafe per Req (UAR) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Receive-latest ($B_0$)** | 27,000 | 26,530 | **15.2%** | **0.0%** | 98.3% | **15.0%** (4,037 unsafe) |
| **Timestamp-aligned + age ($B_1$)** | 27,000 | 23,052 | **17.5%** | **0.0%** | 85.4% | **15.0%** (4,037 unsafe) |
| **Proposed State Acceptance ($P$)** | 27,000 | 12,969 | **0.0%** | **100.0%** | 48.0% | **0.0%** (0 unsafe) |

### Key Findings
1. **Zero Unsafe Acceptances ($FV = 0.0\%$, $UAR = 0.0\%$)**: Proposed State Acceptance completely eliminated false-valid state admissions across all 27,000 requests.
2. **100% True Safety Rejection ($TR = 100.0\%$)**: When physical conditions violated safety (clock drift, network delays exceeding contracts, estimator degradation, stale session restarts, or coordinate frame mismatches), the proposed engine rejected 100% of invalid states.
3. **Naive Baselines Insecurity**: Conventional Receive-latest ($B_0$) and Timestamp-aligned ($B_1$) suffered from **15.0% unsafe admissions** per request (4,037 unsafe snapshots admitted to high-level coordination layers).

---

## Table III: Soundness, Replay, and Runtime Overhead

| Metric | Measured Result | Formal Guarantee Target | Status |
| :--- | :--- | :--- | :--- |
| **Deterministic Spatial Enclosures Tested** | 38,907 enclosures | — | Evaluated |
| **Containment Failures ($CF$)** | **0 (0.00%)** | $0.00\%$ | **VERIFIED** |
| **Replayed Acceptance Decisions** | 12,969 certificates | — | Evaluated |
| **Independent Verifier Agreement** | **12,969 (100.0%)** | $100.0\%$ | **VERIFIED** |
| **Tampered Certificates Tested** | 142,659 mutations | — | 15 mutation classes |
| **Tampered Certificates Caught** | **142,659 (100.0%)** | $100.0\%$ | **VERIFIED** |
| **Snapshot Decision Latency (p50)** | **28.0 µs** | $< 1000$ µs | **VERIFIED** |
| **Snapshot Decision Latency (p95)** | **63.5 µs** | $< 5000$ µs | **VERIFIED** |
| **Snapshot Decision Latency (p99)** | **68.1 µs** | $< 10000$ µs | **VERIFIED** |
| **Serialized Certificate Size (median)** | **1,367 bytes** | $< 4096$ bytes | **VERIFIED** |

---

## Scalability Across Swarm Sizes ($N \in \{3, 5, 10\}$)

| Swarm Size ($N$) | Latency p50 (µs) | Latency p95 (µs) | Latency p99 (µs) | Cert Size (bytes) | Memory / Agent (KB) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **3 UAVs** | 7.0 µs | 7.2 µs | 8.8 µs | 1,123 B | 0.82 KB |
| **5 UAVs** | 7.5 µs | 7.9 µs | 9.9 µs | 1,688 B | 0.82 KB |
| **10 UAVs** | 9.8 µs | 10.1 µs | 14.8 µs | 3,106 B | 0.82 KB |

Decision latency scales sub-linearly and remains below **15 microseconds (p99)** for 10 UAVs, proving runtime suitability for high-frequency flight control loops ($100\text{ Hz}+$).

---

## Per-Scenario Breakdown

| Scenario | Total Requests | $B_0$ FV% | $B_1$ FV% | Proposed ($P$) FV% | $B_0$ Avail% | $B_1$ Avail% | Proposed ($P$) Avail% |
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

## Machine-Readable Result Files
- **`results/dissertation/table_ii_results.json`**: Full aggregate metrics JSON.
- **`results/dissertation/table_ii_results.csv`**: Table II CSV export.
- **`results/dissertation/table_iii_results.json`**: Soundness and Replay metrics.
- **`results/dissertation/scalability_results.json`**: $N=3, 5, 10$ scalability benchmark data.
