# SwarmKit state-acceptance final experiment protocol

Protocol frozen before the final campaign on 20 August 2026. The parameters below are the source configuration; they are not post-result targets.

## Study design

- Primary environment: deterministic `SimBackend` with manual 20 ms physics integration.
- Primary swarm: 3 UAVs.
- Independent unit: replicate ID.
- Replicates: 50.
- Scenario families: 9.
- Scored requests: 100 per replicate and scenario, at 100 ms intervals.
- Requests per method: 50 × 9 × 100 = 45,000.
- Methods: receive-latest (`B0`), source-timestamp aligned plus raw-age threshold (`B1`), and contract state acceptance (`P`).
- Base seed: 42. Motion and fault seeds are independently derived from the base seed, replicate ID, and respectively the literal `motion` label or canonical scenario name.

No Gaussian position noise or nominal network jitter is configured. `SimBackend` emits exact deterministic position and velocity estimates with zero observation-time hard bounds. The experiment converts geodetic simulator telemetry back to metric LocalNED before evidence ingestion and reads physical truth independently through `SimBackendControl::Truth()`.

## Frozen contract and common oracle

The fixed primary contract requires position and velocity evidence for every participant, all three agents, current incarnation, deterministic bounds, estimator healthy, and LocalNED position. Its thresholds and assumptions are:

- maximum conservative physical age: 300 ms;
- maximum clock uncertainty: 10 ms;
- maximum propagated position uncertainty and common-oracle `Umax`: 3.0 m;
- maximum horizontal speed: 10.0 m/s;
- maximum vertical speed: 3.0 m/s;
- 3D propagation speed: `hypot(10, 3) = 10.4403065089 m/s`;
- propagation model: `linear_bounded_vmax`, version `1.0`.

Every method is judged by one accepted-output oracle. For each required agent it checks 3D metric position error at `t*`, required LocalNED frame, current selected-evidence incarnation, selected-evidence health predicates declared by the contract, provenance, and any declared mission or GPS predicate. Current simulator estimator health at `t*` is not an additional undeclared predicate. Baselines may select evidence that fails this oracle; an accepted invalid output is counted as false-valid.

`B0` chooses the highest receive-time position record available at `r*`. `B1` chooses the highest raw source timestamp satisfying `s <= t*` and raw age `t* - s <= 300 ms`. `P` uses the deterministic clock mapping, requires `receive_time <= r*` and `g+ <= t*`, and ranks eligible evidence by maximum `g-`, then maximum sequence, then the canonical content-bound EvidenceId.

## Frozen scenarios

1. `normal`: no injected transport, clock, estimator, or frame fault.
2. `network_delay`: every telemetry frame is delayed by 5 emission frames, i.e. 500 ms at 10 Hz.
3. `network_reorder`: independent deterministic-seed Bernoulli probability 0.5 per per-UAV source frame; a selected frame is held and released after the following frame, producing within-UAV sequence inversions.
4. `packet_loss`: independent deterministic-seed Bernoulli drop probability 0.45 per per-UAV source frame.
5. `clock_offset_drift`: source timestamps receive `15 + 0.08 × source_frame_index` ms offset; per-sample clock uncertainty and the runtime model base radius are 15 ms, above the 10 ms contract threshold.
6. `estimator_degradation`: from scored step 20 onward, emitted evidence is degraded with position/velocity health flags false.
7. `high_speed_delayed_motion`: commanded velocity is `(8.5, 1.0, 0.0) m/s` (3D magnitude about 8.559 m/s), and every frame is delayed by 2 emission frames, i.e. 200 ms.
8. `agent_restart_delayed_packets`: E1 evidence is captured at step 29; session changes to E2 and the E1 clock is invalidated at step 30; authentic delayed E1 evidence is injected at step 31; the E2 clock model, version `clock-v2`, is established at step 33. Telemetry for the restarted agent is withheld during steps 30–32.
9. `frame_mismatch`: the first UAV's metric LocalNED position values are deliberately labelled WGS84 before ingestion on every scored step.

Fault injection owns separate delay/reorder queues and source counters for each UAV. The campaign exports realized delay, loss, reorder, inversion, degradation, restart, obsolete-epoch, frame, and clock counts per paired replicate/scenario trace. A smoke or final campaign is invalid if any intended mechanism is absent.

## Time, enclosure, and replay semantics

Each request binds physical evaluation time `t*`, evidence-freeze frontier `r*`, the exact participant set, and membership revision. Evidence arriving after `r*` remains in the persisted trace but is ineligible for that request.

For source timestamp `s`, clock offset estimate `theta_hat`, base radius `rho`, drift-rate bound `nu`, and reference-domain clock-update time `t_update`:

```text
g_hat  = s - theta_hat
rho_eff = rho + nu * 1e-6 * abs(g_hat - t_update)
g- = g_hat - rho_eff
g+ = g_hat + rho_eff
Delta+ = t* - g-
epsilon_position = observation_bound + hypot(Vhorizontal, Vvertical) * Delta+ / 1000
```

For every accepted deterministic position component, containment is computed from `SimBackend` truth at `t*`; no zero is hardcoded. The campaign exports the failure count and truth-error/enclosure-radius distribution.

Accepted decisions are persisted as JSON-lines events. Offline replay starts with fresh evidence, session, clock, and membership state; reloads request `t*`, request `r*`, and the serialized certificate; verifies the request and certificate canonical contract hashes against the separately supplied fixed contract; and invokes the independent verifier. Therefore the precise claim is: persisted offline replay reconstructs evidence, session, clock, membership, request-frontier state, and certificates from disk and verifies them against the fixed canonical StateQualityContract.

## Statistical analysis

The cluster bootstrap uses 10,000 draws and analysis seed 123456789. Each draw samples 50 replicate IDs with replacement and includes all scenario/method counts for each sampled replicate. The same sampled replicate IDs are used for `P` and `B1`.

Main rates are ratios of summed counts within each draw:

- `FV = sum(false_accepts) / sum(accepted)`;
- `availability = sum(accepted) / sum(requests)`;
- `UAR = sum(false_accepts) / sum(requests)`;
- paired `Delta_FV = FV_P - FV_B1`.

The reported interval is the 2.5th–97.5th percentile cluster-bootstrap interval. No p-value is reported. Replicate-level distributions are descriptive and do not replace the ratio-of-summed-count bootstrap.

## Replay mutation and performance gates

Every persisted accepted `P` decision is independently replayed. Each certificate is then exercised against 44 mutation classes. Except for the outer consistency-hash case, the semantic field is changed and the outer SHA-256 consistency hash is recomputed; verification must still reject the inconsistent certificate. Any replay disagreement or accepted mutation invalidates the campaign.

The performance workload is exactly `RequestSnapshot + BuildCertificate + SerializeCertificate` for `N = 3, 5, 10`, measured with `std::chrono::steady_clock` in Release. Each size receives 100 untimed warmups followed by 1,000 timed iterations, with every latency and serialized certificate size saved. Host results characterize only the tested macOS ARM64 system; target embedded or real-time suitability requires target-hardware benchmarking.

## Execution

```bash
./build/mac-release/apps/swarmkit-dissertation-experiment \
  --runs 50 \
  --steps-per-scenario 100 \
  --seed-base 42 \
  --uav-count 3 \
  --output-dir results/dissertation
```
