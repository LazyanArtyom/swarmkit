# SwarmKit State Acceptance — Final Results Summary

**Author:** Artyom Lazyan  
**Project:** SwarmKit Dissertation Defense Campaign  
**Date:** August 2026  
**Execution Environment:** macOS ARM64 | Clang C++23 | SimBackend Manual Clock Integration  

---

## Executive Summary

This document summarizes the defense-grade experimental campaign for **Replay-Verifiable Common-Time State Acceptance** in multi-UAV swarms. All evaluations were executed on the real `SimBackend` simulation engine using moving UAV physics, real `TelemetryFrame` evidence ingestion, synchronized ground truth via `SimBackendControl::Truth()`, real network/clock/estimator fault injections, an identical common physical-validity oracle ($U_{\max} = 3.0$ m), and an independent offline verifier.

The campaign evaluated **27,000 requests per method** across **30 Monte Carlo runs** and **9 fault scenarios** for a 3-UAV swarm, followed by end-to-end scalability benchmarks up to 10 UAVs.

---

## Table II: Main Semantic Result

All methods evaluated using the compact, standardized schema and identical common ground-truth physical validity oracle:
$$\text{PhysicalValid}(\hat{p}, p_{truth}(t^*), \mathcal{C}, U_{\max}) \iff \forall i \in \mathcal{C}_{\text{req}},\, \|\hat{p}_i - p_{truth,i}(t^*)\| \le 3.0\text{ m} \land \text{EstimatorHealthy}(i) \land \text{Frame}(i) = \text{WGS84} \land \text{Session}(i) = \text{Active}$$

| Evaluation Method | Requests | Accepted | False Valid (FV) | Availability | Unsafe per Req (UAR) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Receive-latest ($B_0$)** | 27,000 | 26,866 | **22.1%** | 99.5% | **22.0%** |
| **Timestamp-aligned + age ($B_1$)** | 27,000 | 26,541 | **30.5%** | 98.3% | **30.0%** |
| **Proposed state acceptance ($P$)** | 27,000 | 16,684 | **1.8%** | 61.8% | **1.1%** |

### Key Findings
1. **Unsafe Acceptance Rate Reduction**: Proposed State Acceptance reduces the unsafe acceptance rate per request from **22.0% ($B_0$)** and **30.0% ($B_1$)** down to **1.1% ($P$)**, an order-of-magnitude improvement in mission safety.
2. **Zero Spatial Enclosure Violations ($CF = 0.00\%$)**: Across all 50,052 evaluated spatial enclosures, zero containment failures were observed.
3. **Authentic Restart & Frame Protection**: When historical frames were re-injected after agent restarts, Timestamp-aligned + age ($B_1$) admitted **70.0% false-valid states**, whereas Proposed State Acceptance achieved **0.0% false-valid admissions** through strict epoch verification.
4. **Coordinate Frame Mismatch Protection**: Under coordinate frame mutations, both naive baselines admitted **100.0% false-valid states**, whereas Proposed State Acceptance rejected 100% of mismatched frames ($0.0\%\text{ Availability}, 0.0\%\text{ FV}$).

---

## Table III: Soundness, Replay, and Runtime Overhead

| Metric | Measured Value | Formal Guarantee Target | Status |
| :--- | :--- | :--- | :--- |
| **Deterministic Enclosures Tested** | 50,052 | — | Evaluated |
| **Containment Failures ($CF$)** | **0 (0.00%)** | $0.00\%$ | **VERIFIED** |
| **Replayed Acceptance Decisions** | 16,684 | — | Evaluated |
| **Independent Verifier Agreement** | **16,684 (100.0%)** | $100.0\%$ | **VERIFIED** |
| **Tampered Certificates Tested** | 250,260 | — | 15 mutation classes |
| **Tampered Certificates Caught** | **250,260 (100.0%)** | $100.0\%$ | **VERIFIED** |
| **Snapshot Decision Latency (p50)** | **60.7 µs** | $< 1000$ µs | **VERIFIED** |
| **Snapshot Decision Latency (p95)** | **89.8 µs** | $< 5000$ µs | **VERIFIED** |
| **Snapshot Decision Latency (p99)** | **93.3 µs** | $< 10000$ µs | **VERIFIED** |
| **Serialized Certificate Size (median)** | **1,461 bytes** | $< 4096$ bytes | **VERIFIED** |

---

## Scalability Across Swarm Sizes ($N \in \{3, 5, 10\}$)

Measured end-to-end across 500 timed iterations of `RequestSnapshot + BuildCertificate + SerializeCertificate`:

| Swarm Size ($N$) | Latency p50 (µs) | Latency p95 (µs) | Latency p99 (µs) | Cert Size (bytes) |
| :--- | :--- | :--- | :--- | :--- |
| **3 UAVs** | 65.3 µs | 65.9 µs | 70.5 µs | 1,545 B |
| **5 UAVs** | 97.2 µs | 101.8 µs | 111.7 µs | 2,360 B |
| **10 UAVs** | 184.5 µs | 188.3 µs | 194.2 µs | 4,401 B |

Decision latency scales linearly ($O(N)$) and remains below **200 microseconds (p99)** for 10 UAVs, proving runtime suitability for high-frequency embedded flight control loops ($100\text{ Hz}+$).

---

## Per-Scenario Detailed Breakdown

| Scenario | Total Requests | $B_0$ FV% | $B_1$ FV% | Proposed ($P$) FV% | $B_0$ Avail% | $B_1$ Avail% | Proposed ($P$) Avail% |
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

## Machine-Readable Result Files
- **`results/dissertation/table_ii_results.json`**: Full aggregate metrics JSON.
- **`results/dissertation/table_ii_results.csv`**: Table II CSV export.
- **`results/dissertation/table_iii_results.json`**: Soundness and Replay metrics.
- **`results/dissertation/scalability_results.json`**: $N=3, 5, 10$ scalability benchmark data.
