# VECR agent campaign v4.2 — corridor-fidelity correction

This addendum was written before the v4.2 final campaign. It retains the
v4.1 hypotheses, 12 comparative and nine noncomparative scenario assignments,
100 paired seeds, outcomes and 10,000-resample analysis plan. The complete
machine-readable protocol is
`results/vecr/agent_campaign_v4/config/protocol_v4_2.json`.

The v4.1 campaign was stopped after 1,452 counted attempts. All 100 S06
pairs had crossed wind, but none actually left the required ±4 m corridor;
maximum cross-track distance was 3.654 m. Consequently S06 did not test its
specified outside-corridor/wide-camera condition. Its partial raw data and
diagnostics are preserved as `partial_aborted_s06_fidelity_v4_1/`, marked
`INCOMPLETE` and excluded from confirmatory statistics.

V4.2 increases the S06 configured crosswind from 2.0 to 3.0 m/s. A
preflight across all 100 seeds and both policies must show true cross-track
distance greater than 4 m; the runner now treats failure to do so as an
incomplete campaign. B2 and P receive the same wind, vehicle, camera and
verification rules. The actual camera cross-footprint parameter is now
explicit in the per-scenario configuration and raw row: 5 m for S03 and
14 m for S06. These are the same modeled footprints used by the earlier
binary; this removes hidden scenario constants and records the actual sensor
condition. Other scenario parameters and the outcome plan are unchanged.

The analysis records true edge completion only when the independent oracle
finds both full modeled strip survey and a true terminal arrival at B. It
continues to report the separately defined false *survey* completion metric.
The charts now show re-entry over simulated time and keep uncertainty bars
inside the plot. All analysis code is frozen with the v4.2 source before the
final campaign. Neither v4.0 nor v4.1 partial rows enter v4.2 statistics.
