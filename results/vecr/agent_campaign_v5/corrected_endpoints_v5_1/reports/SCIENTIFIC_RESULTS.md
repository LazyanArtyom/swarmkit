# VECR v5.1 digital-simulation results

Source revision: `7da3db33e70114503dfc7c9a45671da2b9335541`. Binary SHA-256: `6d607d7369ef1faa05ee919da935f9710ff2ab10a2cdf1895a8d1e884264c901`.
Executed **5280** assigned attempts: 21 primary scenarios and 18 sensitivity conditions; exact paired seed counts are in the manifest and per-scenario table. Every attempted run remains in the denominator. Evidence: SDK → local mTLS AgentService → production runtime → SimBackend. No SITL or hardware trials.

## Primary honest-sensor outcomes

| Policy | N | True survey | True survey + stopped B | Reported | False completion | True complete but unaccepted | Recall |
|---|---:|---:|---:|---:|---:|---:|---:|
| B2 | 1900 | 1300 | 800 | 700 | 0 | 100 | 87.500% |
| P | 1900 | 1300 | 1094 | 794 | 0 | 300 | 72.578% |

The honest primary aggregate excludes only the two preregistered dishonest-sensor controls (M14/M15). Missing bounds, stale captures, safety failures and timeouts remain included. Zero observed false completions is conditional on sensor assumptions and the modeled strip; it is not proof of zero risk or a verification advantage.

## Midflight comparison

| Scenario | Matched successful pairs | B2 mean m | VECR mean m | VECR − B2 m [95% CI] |
|---|---:|---:|---:|---|
| M03 | 100 | 79.561 | 57.764 | -21.798 [-21.821, -21.773] |
| M04 | 100 | 79.561 | 57.418 | -22.143 [-22.185, -22.101] |
| M05 | 100 | 79.561 | 59.440 | -20.121 [-20.153, -20.092] |

These costs are conditional on both methods actually completing and reporting fulfillment. Full per-scenario results, all-assigned costs, p50/p95 and paired intervals are in `tables/`. B2 and VECR share detection, evidence, dynamics, faults, cooldown, safety and timeout; only the re-entry policy differs. B2 restarts at A; VECR re-enters before the earliest unverified gap, capped at current progress during departure. Off-corridor motion can still observe a wide strip; neither algorithm discards those accepted images.

## Failures and uncertainty

* M02 Sustained wind: B2/P reported 0/0; true survey 0/0; true survey + stopped B 0/0; false reports 0/0.
* M08 GNSS outage: B2/P reported 0/0; true survey 100/100; true survey + stopped B 0/100; false reports 0/0.
* M09 Invalid estimator: B2/P reported 0/0; true survey 100/100; true survey + stopped B 0/100; false reports 0/0.
* M10 Unbounded pose: B2/P reported 0/0; true survey 100/100; true survey + stopped B 100/100; false reports 0/0.
* M11 Stale captures: B2/P reported 0/0; true survey 100/100; true survey + stopped B 0/0; false reports 0/0.
* M12 Wrong calibration: B2/P reported 0/0; true survey 100/100; true survey + stopped B 0/0; false reports 0/0.
* M13 Declared yaw offset: B2/P reported 0/0; true survey 0/0; true survey + stopped B 0/0; false reports 0/0.
* M14 False frame success*: B2/P reported 100/100; true survey 0/0; true survey + stopped B 0/0; false reports 100/100.
* M15 Undeclared yaw offset*: B2/P reported 100/100; true survey 0/0; true survey + stopped B 0/0; false reports 100/100.
* M16 Command rejected: B2/P reported 0/0; true survey 0/0; true survey + stopped B 0/0; false reports 0/0.
* M17 Motor failure: B2/P reported 0/0; true survey 0/0; true survey + stopped B 0/0; false reports 0/0.
* M18 No recovery budget: B2/P reported 0/0; true survey 0/0; true survey + stopped B 0/0; false reports 0/0.
* M19 Low battery: B2/P reported 0/0; true survey 0/0; true survey + stopped B 0/0; false reports 0/0.
* M20 Sparse captures: B2/P reported 0/94; true survey 100/100; true survey + stopped B 0/94; false reports 0/0.

Every unsuccessful attempt is listed with final reason, missing intervals and repair count. Every acquisition and metadata rejection is preserved. Repeated GNSS/frame invalidity prevents the estimator from generating bounded camera poses in the fault region; neither extra motion nor a command ACK restores that evidence. A shared 90 s deadline may stop B2 during its second full restart while VECR has already returned to B. These are policy costs under a fixed time budget, not infrastructure errors. Camera data that arrive after termination remain explicitly pending.

Confidence intervals use 10,000 paired seed bootstrap resamples, fixed RNG 20261024; per-scenario binomial intervals use Wilson 95%. Aggregate tables are descriptive: pooled iid binomial bounds are omitted because the same seeds recur across conditions. Fixed-mixture aggregate paired intervals resample whole seed clusters (100 clusters, retaining all 19 honest conditions); see `paired_primary_cluster_effects.csv`. Conditional comparisons select pairs successful under both policies and can conceal differences in availability; the all-assigned table must accompany them. Seeds vary bounded pose/velocity/altitude errors and calibrated optical width. They do not represent real airframes, terrains or weather distributions. No multiple-comparison significance claims are made.

## Oracle and sensor limits

The camera estimator receives noisy telemetry and fixed calibration; it cannot access private simulator poses. Metadata carry acquisition timestamps, nonzero hard bounds and a conservative optical envelope. Independent private truth uses actual acquisition geometry and physical 20 ms trajectories. A second Python implementation reconstructs geometry from rectangle half-space inequalities and verifies every accepted footprint, final truth outcome and completed physical re-entry dwell.

The oracle is a one-dimensional union of complete strip cross-sections under individual rectangular nadir footprints. It does not mosaic partial-width views, model terrain, real pixels, pitch/roll or image-quality classification. The angle controls use rotated ground rectangles (yaw), not a calibrated real camera. Cell-center raster estimates can conceal subcell holes; exact union is primary. Geometric simulation v1/v2, agent-backed v3/v4.2 and v5 remain separate evidence levels.

## Performance

* cpu_s: median 0.091; p95 0.195; 80 isolated serial processes.
* peak_rss_MiB: median 22.312; p95 23.438; 80 isolated serial processes.
* capture_latency_p95_us: median 117.354; p95 155.531; 80 isolated serial processes.

CPU/RSS cover the whole in-process SDK + agent + backend + trace writer, including startup for CPU/RSS. Capture latency covers the mTLS capture RPC. Wall-time rates measure accelerated host execution, not radio/network capacity. This is not an agent-only profiler, Raspberry Pi measurement, energy result or hard real-time bound. Benchmark assignments and source/binary hashes are stored separately.

## Validation and remaining scope

Independent reconstruction passed for 5280 attempts, 453524 delivered captures and 4246 physical re-entry dwells. Release/UBSan/install gates are recorded separately in `docs/vecr/FINAL_VALIDATION_V5_1.md`.

The original v4.2 300 matched localized-repair pairs and approximately 21.5% distance reduction remain frozen at their original revisions. The v5 availability inventory enumerates all 330 v4.2 missed completions; old logs cannot retrospectively identify every missing rejection reason.

Remaining research: live calibrated camera evidence, obstacles/geofences and realistic aerodynamics, remote replay/peer transport/multi-UAV experiments, process/power-loss recovery and agent-only resource profiling. No ArduPilot SITL, hardware, rotor-router or gossip experiment was run.
