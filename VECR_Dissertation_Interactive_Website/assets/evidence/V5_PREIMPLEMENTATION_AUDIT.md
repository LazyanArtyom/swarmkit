# v5 preimplementation audit

Baseline: `5063dfe`; read AGENTS.md, METHOD_AND_CORRECTNESS.md, v4.2 final report,
implementation progress and independent review before changing source.

* Confirmed v4 sets deviation threshold 1000 with 650 tick horizon. It isolates
  post-arrival repair and cannot validate early correction.
* Confirmed v4 CameraMessage consumes SimulationTruthFrame with zero pose errors.
* Confirmed RecoveryDispatched clears pending target and returns MOVING on ACK.
  Core has no active physical re-entry guard; multi-attempt early recovery can
  overwrite the actual navigation target. Runtime separately waits for stationary
  re-entry but does not expose it as the execution phase.
* No explicit departure phase/hysteresis. Unknown horizontal bounds become zero
  for the sample corridor predicate (although traversal/recovery need bounds).
* Frozen false negatives: S08 B2 82/P 85; S09 B2 78/P 85. Every row is INTERRUPTED
  at 60.4 s, with one recovery and residual unverified coverage. Final uncovered
  range is approximately 0.0704–1 m despite complete raster truth. Frozen traces
  only cover seeds 1–3 and omit per-capture rejection reasons. Exact per-rejection
  attribution for all 330 is unrecoverable; do not invent it. Source establishes
  that position/frame invalidity clears the capture's bounded-pose prerequisite
  on every traversal of the fault region. Repeated blind repair is not guaranteed
  to produce new accepted evidence. Sampling/repair phase also affects small holes.
* v4 uses a cell-center oracle; v5 will score exact full-strip union and separately
  report raster sensitivity, plus independent stopped-arrival dwell.
* Keep general SDK/control/telemetry/transport APIs and all prior evidence intact.
