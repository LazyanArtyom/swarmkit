# VECR v5 preregistration — digital simulation only

Written before v5 implementation and pilot trials. Baseline `5063dfe` and all
previous research outputs remain frozen. Review:
`VECR_v4_2_Independent_Audit_And_Next_Codex_Tasks.md`.

## Questions and criteria

1. Can the production agent detect sustained departure and dispatch/complete
   re-entry before B with attainable thresholds, bounded observations and cooldown?
2. Does conservative accepted strip coverage remain a subset of independent
   actual coverage under declared sensor assumptions? What happens when those
   assumptions are absent, stale, or deliberately violated?
3. What are verification availability, false completion, failures, time and distance
   for paired coverage-aware B2 full restart and VECR earliest-gap re-entry?
4. What CPU, RSS, capture/report throughput and RPC latency does this host exhibit?

No required direction of comparative effect. No SITL, hardware, energy, networking,
image-quality or full two-dimensional terrain claims. Primary truth is an independent
exact union of acquired rectangular full-strip intersections on a straight flat route;
secondary all-cell and cell-center rasters at 0.1/0.25/0.5/1 m measure discretization.
Arrival truth requires actual stopped pose near B for 200 ms, independently of ACK.

## Reliability work identified before implementation

Recovery ACK currently changes RECOVERING to MOVING before physical re-entry and
permits duplicate repairs during that motion. Add a physically completed re-entry
transition and prevent overlapping recovery. Add explicit DEVIATING, spatial reset
hysteresis and observable processed telemetry time. Do not treat missing bounds as
zero. Preserve arrival, full-strip, calibration and safety checks. Tests cover ACK,
physical completion, invalid evidence, cooldown, repeated samples and safety override.

## Experimental design

The exact executable matrix is frozen in `scripts/vecr/run_agent_campaign_v5.py`
with a source commit and binary hash before confirmatory execution. Development
pilots use seeds 10001–10003 and are retained separately. Confirmatory seeds 1–100
per primary condition, 1–30 per sensitivity condition; both policies on every seed,
order balanced by seed. No deletion/replacement of unsuccessful attempts. Infrastructure
errors remain assigned rows and invalidate a completion gate; fixes require a new run
version. Confidence intervals are paired seed bootstrap (10,000 resamples, fixed RNG),
with precision, recall, false completion, Wilson binomial intervals, p50/p95 and
conditional costs on pairs with both true **and** reported completion. Overall costs
are descriptive fixed-mixture values, not fleet savings. No significance filtering.

Primary conditions: nominal bounded sensing; short gust; sustained crosswind;
transient departure before B; departure with wide visible strip; departure with
hidden coverage gap; bounded biased/delayed sensing; metadata outage; GNSS outage;
estimator-invalid region; unbounded pose accuracy; stale captures; inconsistent
calibration; declared camera tilt; dishonest frame success; undeclared camera tilt;
recovery command rejection; motor failure during recovery; exhausted recovery budget;
low-battery safety; sparse captures. Dishonest cases are explicit assumption-violation
negative controls, excluded from honest-sensor soundness aggregates but fully reported.

Sensitivity varies one factor at a time around the departure/gap reference: route
length (20/40/80 m), along footprint, sampling period, position-error budget,
gap position/length, wind strength and recovery-attempt budget. Exact levels will be
frozen after engineering pilots; any changes are documented as preregistration
amendments, not concealed as confirmatory choices.

Shared default: 100 ms simulation steps, 20 ms private dynamics integration,
10 Hz telemetry, 3 consecutive departure samples, 0.5 m hysteresis, 1 s cooldown,
2 m/s commanded speed, 2 m/s² acceleration, 4 m corridor/strip half-width,
200 ms stationary arrival and re-entry dwell, identical safety/timeout/attempt limits.
Faults use predetermined simulation-time schedules or fixed spatial regions, never
policy-specific success criteria. Realized faults and exposure differences are logged.

## Observation separation

A seeded measurement adapter corrupts backend telemetry and declares explicit
bounds. The camera estimator accepts only those measurements and fixed calibration;
it has no SimulationTruthFrame or simulator control input. Delay preserves acquisition
time. Capture quality/alignment assertions reflect simulated sensor status. Actual
footprints, acquisitions, tilt and stopped truth feed only the independent oracle.
Bounded valid FOV uses a conservative calibration envelope. Declared tilt is rejected
by the restricted nadir adapter; undeclared tilt demonstrates its trust boundary.

## Required artifacts and gates

Archive every trial summary, per-step state/reason/gaps, every capture outcome with
rejection detail and claimed/actual footprints, physical trajectory, command targets,
phase transitions and timing. Freeze environment, source revision, configuration,
assigned seeds, binary SHA-256 before final runs; hash all outputs afterward. Retain
failed pilots and amendments. Run release/UBSan suites and SDK installation smoke test.
Benchmark serial isolated processes separately from parallel science runs, same release
binary; CPU/RSS cover the in-process SDK+agent+backend harness and are explicitly not
agent-only embedded measurements. Record RPC latency and observed report/capture rates.

## Frozen engineering decisions before confirmatory execution

The executable matrix contains M00–M20 (21 × 100 pairs), X00–X17
(18 × 30 pairs): **5,280 attempts**. Default route is 40 m; default deadline
90 s (X01, 80 m, uses 180 s). The same deadline can stop the slower policy
mid-repair; true survey alone and true survey plus stopped arrival are reported
separately. The 21 M identifiers are a new v5 matrix; they do not rebrand the
unsupported v4 remote/peer/multi-UAV scenarios as completed.

Spatial hysteresis retains the departure count in the 3.5–4 m reset band;
three outside-envelope observations without a definite re-entry trigger recovery.
The reset is at `cross_track + error <= 3.5 m`. Missing/invalid bounds reset
that count and cannot establish departure or arrival. Cooldown starts after
physical re-entry completion, and an active re-entry cannot be overwritten by
another recovery. Command rejection and motor failure remain real negative
outcomes, and timeout does not assert a physical hold.

Sensor errors use deterministic seeded, bounded radial horizontal error with
nonzero east bias, plus bounded altitude and velocity errors. Actual optical width
is 0–2% larger than the fixed conservative calibration envelope. Noise is a function
of seed and absolute simulation tick, never policy. Delay defaults to 200 ms;
M06 uses 400 ms and 0.9 m horizontal bound. M11 delays metadata 1.5 s and expects
rejection. Camera-angle controls rotate an actual ground rectangle by 45° yaw;
they are not pitch/roll photogrammetry. M14 lies about frame acquisition; M15 lies
about alignment. These controls explicitly violate assumptions.

Pilots 01–03 are retained. Pilot 02 had one infrastructure startup failure out of
234 assigned attempts; its old generic error lost the rejection reason. This root
cause cannot be proven retrospectively. Per-trial journal/artifact isolation and
complete dispatch errors were added. Pilot 03 completed 234/234 and its independent
validator reconstructed every acquisition, trajectory and 207 re-entry dwells.
Pilot 04 checks the final instrumentation changes (fault event ordering, atomic
processed-observation timestamps, explicit unavailable latency).

The isolated serial performance experiment uses 10 paired seeds 20001–20010 in
M00/M03/M05/M08 (80 attempts); scientific runs use four processes. Physical and
verification results remain independent of host timing; CPU/RSS and rates describe
the whole instrumented process. No agent-only resource-isolation claim is made.
