# v5 analysis provenance

Experimental source was frozen at `7896f200ec7fa41bd4f9ca1f293616dbd3896d36`.
The binary SHA-256 is recorded before execution in the campaign manifest.

While the confirmatory campaign ran, a code review found that the paired-effects
CSV labeled simulation time `time_ms` after scaling its values to seconds. The
analysis label is corrected to `time_s` before final analysis. This changes no
measurements, assignments, exclusions, bootstrap operations, values or criteria.
The analysis manifest records the actual analysis revision and script hash,
separately from the frozen experimental revision. Earlier engineering-pilot tables
remain unchanged; their paired time rows are in seconds despite the old label.

Before final analysis, the report generator also gained a tidy scenario/configuration
matrix exported from the already frozen assignment manifest and assertions that
the benchmark uses the same executable hash and one worker. These are metadata
and validation additions; the experimental protocol and inference calculations
remain those preregistered at `7896f20`.

## Statistical audit before publication

The first complete analysis exported Wilson intervals for both individual scenarios
and pooled totals. Reused seed IDs make the pooled trials dependent across scenarios;
those iid pooled bounds were inappropriate. The final analysis removes them, retains
per-scenario Wilson bounds, and adds paired aggregate intervals resampling the 100
whole seed clusters with all 19 honest primary conditions retained. Per-scenario
bootstrap results and every measured value are unchanged. Precision and acceptance
Wilson intervals are now explicit in the per-scenario table. The original prepublication
analysis is retained in `final_v5/analysis_prepublication_revision_1.tar.gz`; it is
superseded and must not be used for pooled uncertainty claims. No raw trial was changed,
excluded or rerun. This is an analysis correction, not a new experimental protocol.
