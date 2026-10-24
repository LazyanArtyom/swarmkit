# VECR v5 digital-simulation results

Source revision: `5063dfe492da1cf68790fb579837d7b07ac96178`. Binary SHA-256: `61ae7bfe6306fe1673ceffd8039c8ad727eb19b83820c08500178dce70fa8491`.
Executed **234** assigned attempts: 21 primary scenarios and 18 sensitivity conditions; exact paired seed counts are in the manifest and per-scenario table. Every attempted run remains in the denominator. Evidence: SDK → local mTLS AgentService → production runtime → SimBackend. No SITL or hardware trials.

## Primary honest-sensor outcomes

| Policy | N | True survey | True survey + stopped B | Reported | False completion | True complete but unaccepted | Recall |
|---|---:|---:|---:|---:|---:|---:|---:|
| B2 | 57 | 39 | 24 | 21 | 0 | 3 | 87.500% |
| P | 57 | 39 | 33 | 24 | 0 | 9 | 72.727% |

The honest primary aggregate excludes only the two preregistered dishonest-sensor controls (M14/M15). Missing bounds, stale captures, safety failures and timeouts remain included. Zero observed false completions is conditional on sensor assumptions and the modeled strip; it is not proof of zero risk or a verification advantage.

## Midflight comparison

| Scenario | Matched successful pairs | B2 mean m | VECR mean m | VECR − B2 m [95% CI] |
|---|---:|---:|---:|---|
| M03 | 3 | 79.650 | 57.721 | -21.929 [-22.047, -21.747] |
| M04 | 3 | 79.650 | 57.463 | -22.188 [-22.242, -22.118] |
| M05 | 3 | 79.650 | 59.469 | -20.181 [-20.315, -20.040] |

These costs are conditional on both methods actually completing and reporting fulfillment. Full per-scenario results, all-assigned costs, p50/p95 and paired intervals are in `tables/`. B2 and VECR share detection, evidence, dynamics, faults, cooldown, safety and timeout; only the re-entry policy differs. B2 restarts at A; VECR re-enters before the earliest unverified gap, capped at current progress during departure. Off-corridor motion can still observe a wide strip; neither algorithm discards those accepted images.

## Failures and uncertainty

* M02 Sustained wind: B2/P reported 0/0; true survey 0/0; true survey + stopped B 0/0; false reports 0/0.
* M08 GNSS outage: B2/P reported 0/0; true survey 3/3; true survey + stopped B 0/3; false reports 0/0.
* M09 Invalid estimator: B2/P reported 0/0; true survey 3/3; true survey + stopped B 0/3; false reports 0/0.
* M10 Unbounded pose: B2/P reported 0/0; true survey 3/3; true survey + stopped B 3/3; false reports 0/0.
* M11 Stale captures: B2/P reported 0/0; true survey 3/3; true survey + stopped B 0/0; false reports 0/0.
* M12 Wrong calibration: B2/P reported 0/0; true survey 3/3; true survey + stopped B 0/0; false reports 0/0.
* M13 Declared yaw offset: B2/P reported 0/0; true survey 0/0; true survey + stopped B 0/0; false reports 0/0.
* M14 False frame success*: B2/P reported 3/3; true survey 0/0; true survey + stopped B 0/0; false reports 3/3.
* M15 Undeclared yaw offset*: B2/P reported 3/3; true survey 0/0; true survey + stopped B 0/0; false reports 3/3.
* M16 Command rejected: B2/P reported 0/0; true survey 0/0; true survey + stopped B 0/0; false reports 0/0.
* M17 Motor failure: B2/P reported 0/0; true survey 0/0; true survey + stopped B 0/0; false reports 0/0.
* M18 No recovery budget: B2/P reported 0/0; true survey 0/0; true survey + stopped B 0/0; false reports 0/0.
* M19 Low battery: B2/P reported 0/0; true survey 0/0; true survey + stopped B 0/0; false reports 0/0.
* M20 Sparse captures: B2/P reported 0/3; true survey 3/3; true survey + stopped B 0/3; false reports 0/0.

Every unsuccessful attempt is listed with final reason, missing intervals and repair count. Every acquisition and metadata rejection is preserved. Repeated GNSS/frame invalidity prevents the estimator from generating bounded camera poses in the fault region; neither extra motion nor a command ACK restores that evidence. A shared 90 s deadline may stop B2 during its second full restart while VECR has already returned to B. These are policy costs under a fixed time budget, not infrastructure errors. Camera data that arrive after termination remain explicitly pending.

Confidence intervals use 10,000 paired seed bootstrap resamples, fixed RNG 20261024; binomial intervals use Wilson 95%. Conditional comparisons select pairs successful under both policies and can conceal differences in availability; the all-assigned table must accompany them. Seeds vary bounded pose/velocity/altitude errors and calibrated optical width. They do not represent real airframes, terrains or weather distributions. No multiple-comparison significance claims are made.

## Oracle and sensor limits

The camera estimator receives noisy telemetry and fixed calibration; it cannot access private simulator poses. Metadata carry acquisition timestamps, nonzero hard bounds and a conservative optical envelope. Independent private truth uses actual acquisition geometry and physical 20 ms trajectories. A second Python implementation reconstructs geometry from rectangle half-space inequalities and verifies every accepted footprint, final truth outcome and completed physical re-entry dwell.

The oracle is a one-dimensional union of complete strip cross-sections under individual rectangular nadir footprints. It does not mosaic partial-width views, model terrain, real pixels, pitch/roll or image-quality classification. The angle controls use rotated ground rectangles (yaw), not a calibrated real camera. Cell-center raster estimates can conceal subcell holes; exact union is primary. Geometric simulation v1/v2, agent-backed v3/v4.2 and v5 remain separate evidence levels.

## Performance


## Validation and remaining scope

Independent reconstruction passed for 234 attempts, 20302 delivered captures and 207 physical re-entry dwells. Release/UBSan/install gates are recorded separately in `docs/vecr/FINAL_VALIDATION_V5.md`.

The original v4.2 300 matched localized-repair pairs and approximately 21.5% distance reduction remain frozen at their original revisions. The v5 availability inventory enumerates all 330 v4.2 missed completions; old logs cannot retrospectively identify every missing rejection reason.

Remaining research: live calibrated camera evidence, obstacles/geofences and realistic aerodynamics, remote replay/peer transport/multi-UAV experiments, process/power-loss recovery and agent-only resource profiling. No ArduPilot SITL, hardware, rotor-router or gossip experiment was run.
