# SwarmKit State Acceptance — Final Experiment Protocol

**Author:** Artyom Lazyan  
**Project:** SwarmKit Dissertation Defense Campaign  
**Date:** August 2026  

---

## 1. Experimental Objectives

The goal of this evaluation is to empirically validate the mathematical and runtime properties of **Replay-Verifiable Common-Time State Acceptance** across dynamic, distributed UAV swarms operating over imperfect wireless communication links and heterogeneous sensor channels:

1. **Semantic Soundness**: Spatial containment $CF = 0.0\%$ (all physical vehicle positions are strictly enclosed within computed uncertainty bounds $\varepsilon(t^*)$).
2. **Safety Enforcement**: False-valid rate $FV = 0.0\%$ under all fault conditions when judged by a baseline-independent physical validity oracle.
3. **Replay Verifiability**: Independent verifier reproduces identical acceptance decisions and catches 100% of certificate tampering without re-invoking runtime engines.
4. **Sub-millisecond Latency**: State acceptance decisions evaluate in under 100 µs (p99).

---

## 2. Benchmark Architecture & Environment

### Simulation Engine (`SimBackend`)
- Deterministic numerical physics simulator advancing multi-UAV dynamics using manual clock stepping (`SimulationClockMode::kManual`, $dt = 20$ ms).
- Physical truth extracted directly from `SimBackendControl::Truth(drone_id)` producing `SimulationTruthFrame` (independent of emitted telemetry).

### Telemetry Pipeline
- Sensor noise: Gaussian noise $\sigma = 0.15$ m on GNSS positions.
- Evidence Decomposition: Telemetry frames decomposed into typed `EvidenceRecord` elements with provenance and timestamp intervals.
- Evidence Store: Ring-buffered multi-agent evidence store retaining historical evidence without mutating authoritative agent session state on packet arrival.

### Common Physical Validity Oracle
To guarantee scientific fairness, all methods ($B_0$, $B_1$, $P$) are evaluated against the same ground-truth condition:
$$\text{PhysicalValid}(\hat{p}, p_{truth}(t^*), \mathcal{C}, U_{\max}) \iff \forall i \in \mathcal{C}_{\text{req}},\, \|\hat{p}_i - p_{truth,i}(t^*)\| \le 3.0\text{ m} \land \text{EstimatorHealthy}(i) \land \text{Frame}(i) = \text{WGS84} \land \text{Session}(i) = \text{Active}$$

---

## 3. Evaluated Methods

1. **Receive-Latest ($B_0$)**: Naively selects the most recently received packet for each agent, ignoring timestamps, clock offsets, and uncertainty.
2. **Timestamp-Aligned + Age ($B_1$)**: Selects the newest candidate with source timestamp $s \le t^*$ and simple age threshold $(t^* - s) \le \text{max\_age\_ms}$, assuming perfect clocks and neglecting velocity-based spatial expansion.
3. **Proposed State Acceptance ($P$)**: Evaluates formal State-Quality Contracts $\mathcal{C}$ using clock-uncertainty interval arithmetic $[g^-, g^+]$, causal selection ($g^+ \le t^*$), conservative elapsed time $\Delta^+ = t^* - g^-$, deterministic spatial uncertainty propagation $\varepsilon_p = e_p + V_{\max}\Delta^+$, and emits tamper-evident cryptographic certificates $K$.

---

## 4. Fault Scenarios Evaluated

1. **`normal`**: Nominal jitter (10-30 ms), synchronized clocks, healthy estimators.
2. **`network_delay`**: Packet transport delay 450-500 ms (> contract limit 300 ms).
3. **`network_reorder`**: Alternating delivery latencies causing out-of-order packet arrival.
4. **`packet_loss`**: 45% stochastic message drop rate.
5. **`clock_offset_drift`**: Dynamic clock drift $\theta(t) = 15.0 + 0.08 \cdot t$, testing clock uncertainty bounds.
6. **`estimator_degradation`**: EKF estimator failure / covariance inflation at step $\ge 20$.
7. **`high_speed_motion`**: High-velocity vehicle maneuvers testing spatial uncertainty expansion.
8. **`agent_restart_delayed_packets`**: Agent reboots with new session ID while stale packets arrive from old session.
9. **`frame_mismatch`**: Agent transmits positions in local NED rather than required WGS-84 frame.

---

## 5. Execution Command

```bash
cmake --build --preset mac-debug
./build/mac-debug/apps/swarmkit-dissertation-experiment \
  --runs 30 \
  --steps-per-scenario 100 \
  --seed-base 42 \
  --uav-count 3 \
  --output results/dissertation/table_ii_results.json \
  --csv-output results/dissertation/table_ii_results.csv
```
