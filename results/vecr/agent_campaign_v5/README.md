# VECR v5 — agent-backed digital simulation

Protocol: `docs/vecr/SIMULATION_PROTOCOL_V5.md`. B2 and VECR share the production
agent verifier and safety rules, with different recovery targets. No SITL/hardware.

* `audit/`: all 330 frozen v4.2 missed completions and explicit attribution limits.
* `pilot_01/`: 42 first engineering trials; early instrumentation.
* `pilot_02/`: 234 assigned trials, one startup infrastructure failure retained.
* `pilot_03/`: 234 completed engineering trials, independently reconstructed.
* `pilot_04/`: final instrumentation pilot, retained separately from confirmatory seeds.
* `final_v5/`: preregistered 5,280 confirmatory attempts and scientific analysis.
* `benchmark_v5/`: separate 80 serial instrumented process benchmarks.

Prior v1/v2/v3/v4.2 results are untouched. Exact full-strip union is the v5 primary
truth; the older cell-center raster is a different definition. Each trial includes
all 20 ms private poses, noisy measurements, actual acquisitions, every delivered
or absent/pending metadata outcome, phase/reason/gaps and command targets. Gzip is
lossless. No failed attempt is discarded or silently rerun.

Reproduce with the recorded release build and `run_agent_campaign_v5.py --mode
final --output <new-dir> --workers 4`, then run serial `--mode benchmark`, then
`analyze_agent_campaign_v5.py --output <new-dir> --benchmark <benchmark-dir>` in an
environment matching `scripts/vecr/requirements-v5.txt`. The validator independently
reconstructs strip geometry, arrival, distance, containment and re-entry dwell.
