# SwarmKit VECR agent campaign v4: frozen simulation protocol

This protocol was written before the v4 full campaign. Its machine-readable
parameters and 21-case inventory are in
`results/vecr/agent_campaign_v4/config/protocol.json`. Historical v1/v2/v3
research is unchanged. Only the `COMPARATIVE` cases enter the B2–P primary
contrast. `UNSUPPORTED` cases are declared missing, not silently counted.

## Questions and hypotheses

- **H1:** Under accurate, bounded telemetry and honest camera metadata,
  accepted edge fulfillment agrees with an independent modeled coverage
  oracle. The dishonest-camera S11 variant intentionally challenges that
  assumption and can falsify a broader soundness claim.
- **H2:** For local gaps, P targeted-gap recovery reduces actual simulated
  travel and time relative to B2 full-edge restart while preserving true task
  completion. Persistent disturbances and failures may reverse the effect.
- **H3:** Journal and hold reconciliation protect attempt identity and prevent
  contradictory new navigation after uncertain restart or cancellation.
- **H4:** Local SDK callers receive typed edge outcomes without a server-side
  deviation controller. Subscriber replay under transport loss is outside the
  current instrumented campaign and cannot be claimed from it.

## System and comparator

Each comparative attempt calls the production SDK through a local TLS gRPC
AgentService, production edge runtime and SimBackend. B2 and P differ only
in the edge supervisor's repair re-entry policy. Both retain accepted capture
evidence, use the same verifier, sensor adapter, arrival/corridor bounds,
recovery limit, safety checks, 60 s attempt timeout, 65 s observation horizon,
20 ms physical integration, 100 ms campaign tick, 2 m/s default command speed,
2 m/s² horizontal acceleration and nominal 0.5 s camera interval. B2 goes
back to A for a full-edge repair; P goes to the earliest missing span with 2 m
overlap. No intervention teleports the simulated vehicle. The graph fixture
requests the single directed A→B edge; it is not rotor-router or gossip.

The camera adapter uses private simulator state to generate synthetic capture
messages. The verifier receives only those messages and normalized telemetry.
An independent compilation unit scores the actual camera footprint on a
0.25 m raster across the required strip, separately integrates true vehicle
positions and computes true arrival. It cannot validate real photo quality,
occlusion, camera calibration, estimator error tails or actual vehicle dynamics.
The simulator uses exact baseline telemetry unless a scenario explicitly
degrades it. Wind, battery, motor and communication faults are model inputs.

## Assignment and analysis

The full campaign assigns seeds 1–100 to every `COMPARATIVE` case and executes
both B2 and P. Each pair shares initial state and deterministic environment
parameters. The runner derives a gap-position stream and a wind stream from
SHA-256 of `(scenario, seed, stream)` before either policy runs. GPS jitter
uses a separate deterministic sequence/seed stream inside the telemetry
adapter. Faults tied to a location apply to the trajectory each policy
actually flies; realized effects may differ. No attempts are excluded from
policy denominators, including timeouts, interrupted runs and false
completions. Infrastructure failures are recorded separately and render the
corresponding pair invalid; they are never reclassified as policy failures.

The primary outcomes are false completion per assigned attempt, true
completion, reported completion and P−B2 paired differences in actual travel
and mission-clock elapsed time. Secondary outcomes include observed and true
coverage, recovery attempts, accepted/rejected navigation commands, camera
acquisitions/submissions, interruption and requested-fault realization.
Distance and time are measured for all assigned attempts; a shorter failed
attempt is not interpreted as a successful efficiency gain. A false complete
requires an EDGE_FULFILLED report while at least one required truth cell is
uncovered. Zero fulfilled reports gives an undefined false-complete fraction
among fulfilled reports.

Wilson 95% intervals are used for binomial rates. P−B2 mean differences use
10,000 paired bootstrap resamples of independent seed pairs, with fixed RNG
seed 4062026 and percentile 95% interval. Scenario effects are not treated
as independent replicates in an overall pooled seed calculation; a separate
macro average over scenario means is descriptive. Medians and p95 use the
linear percentile convention. No tuning is permitted after seeing the full
data. Any protocol or source correction requires a new result version.

## Scenarios and limitations declared before the run

S00–S09, S16 and S19 are the 12 comparative cases (2,400 full attempts).
S10, S11, S12, S15 and S18 are diagnostics/safety checks. S20 explicitly
rejects unsupported polylines. S13, S14 and S17 are not instrumented in this
campaign and remain `UNSUPPORTED`; they must not appear in the comparative
denominator. S07 drops telemetry but the current model does not guarantee a
physical excursion between samples, so the stronger claim about detecting
such an excursion is not tested. S10's flight model can eventually settle
near B, so a brief-radius crossing without later arrival is not isolated.

No B0/B1 intervention arms are included. Arrival-only or sampled-corridor
diagnostic rates must not be mixed into B2/P performance comparisons. No SITL,
HIL, real camera or physical-flight data are acquired here. Simulator battery
percentage is a model quantity, not measured electrical energy.

## Gates

Run a release build, install check, focused VECR regressions, full CTest and
available sanitizer tests before publishing final data. Run a three-seed
smoke campaign first and inspect paired traces and fault realization. The
full command and exact binary/source/config hashes are saved in the final
manifest; output directories refuse overwrite. Analysis reads only the
saved raw rows and fails if pair identity, fault realization or artifact
counts disagree with the protocol. All figures and prose are generated from
those rows. A failed gate leaves the study marked `INCOMPLETE`.
