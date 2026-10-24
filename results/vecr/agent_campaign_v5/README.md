# VECR v5 — agent-backed digital simulation

Protocol: `docs/vecr/SIMULATION_PROTOCOL_V5.md`. B2 and VECR share the production
agent verifier and safety rules, with different recovery targets. No SITL/hardware.

**Complete:** 5,280 scientific attempts (21 × 100 paired seeds and 18 × 30
paired seeds), plus 80 separate serial benchmarks; zero infrastructure errors
in these final runs. The final independent validator reconstructed 453,524
delivered captures and 4,246 physical re-entry dwells.

* [Scientific report](final_v5/reports/SCIENTIFIC_RESULTS.md)
* [Every validation gate, measured findings and unfinished scope](../../../docs/vecr/FINAL_VALIDATION_V5.md)
* [Scenario matrix](final_v5/tables/scenario_matrix.csv), [raw per-attempt results](final_v5/per_attempt.csv), [comparison tables](final_v5/tables)
* [Figures: SVG, PDF and PNG](final_v5/figures), [full execution traces](final_v5/trials)
* [Frozen assignments and source/binary provenance](final_v5/manifest.json), [analysis provenance](final_v5/analysis_manifest.json)
* [Scientific hashes](final_v5/SHA256SUMS), [benchmark hashes](benchmark_v5/SHA256SUMS), [analysis correction notes](../../../docs/vecr/V5_ANALYSIS_NOTES.md)

The three equally successful midflight conditions reduced modeled travel by
25–28%. Honest-sensor attempts had zero false fulfillment (4,880 attempts),
while all 400 deliberate dishonest-sensor controls falsely fulfilled. Persistent
missing or overly uncertain evidence prevented certification in several cases.
Conditional cost results and all-assigned availability must be read together.

## Preserved executions

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
