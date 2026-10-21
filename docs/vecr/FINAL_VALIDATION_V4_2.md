# VECR v4.2 simulator validation record

The confirmatory agent-backed campaign is complete under the frozen v4.2
trial source `5adb6872f4b593cdd21ab2ce5c63063814a03b7f`. Analysis code
revision `60f79e8abfcb6bb02ed90407d46799a0b079611f` contains the
documented post-collection arithmetic correction. The 2,400 raw attempts
were not rerun or edited for that correction. The complete source, binary,
protocol and artifact hashes are in
`results/vecr/agent_campaign_v4/final_v4_2/manifest.json` (SHA-256
`bd49772a668736fbd1d30373366b050641c4b714a1eda1a643e6e55f92e1dac8`).

## Gates and actual execution

| Gate | Evidence and outcome |
|---|---|
| Pre-change audit | `V4_IMPLEMENTATION_AUDIT.md` records baseline commit, compiler, CMake/Conan, tests and archive hashes. |
| P0 reliability | Journal commit-before-publication, same-drone reconciliation gate and physical-hold cancellation regressions pass. Focused VECR: 39 cases, 1,818 assertions. |
| Agent-backed method | Both B2 and P execute SDK → local mTLS AgentService → edge runtime → SimBackend. S03/S04 traces show distinct real backend re-entry targets and continuous true motion. |
| Scenario smoke | v4.2 protocol smoke: 12 scenarios × 3 seeds × 2 arms = 72 attempts. S06 crosses ±4 m in both arms. |
| Full raw campaign | S00–S09, S16, S19: 12 scenarios × 100 seeds × 2 arms = 2,400 attempts. All 2,400 rows are present; zero infrastructure-invalid rows and zero unrealized scheduled faults. No assigned task was excluded. |
| Noncomparative diagnostics | S10 one inconclusive fast-fly-through; S11 two sensor trust cases; S12 one safety interruption; S15/S18/S20 regressions passed. S13/S14/S17 are unsupported. |
| Independent analysis | 10,000 paired bootstrap resamples and Wilson intervals; 107 SHA-256 artifacts validated, including raw rows, event logs, 72 selected full traces, tables, seven rendered SVG figures and reports. All 72 overlapping smoke/full row values match except run ID and host wall time. |
| Held-out sensitivity | Seeds 101–103: 72 additional attempts, analyzed separately and validated with 15 hashes. The runner did not retain their detailed traces, so no held-out trajectory figures are asserted. |
| Final product checks | Release build; SDK install; release CTest 2/2; UBSan CTest 2/2; focused VECR 39 cases/1,818 assertions; Python compile; SVG XML parsing and PNG rendering; `git diff --check` pass. The unprivileged sanitizer attempt failed because the workspace blocked loopback gRPC; the permitted rerun passed. |
| Historical integrity | Archived core and research tarballs still hash to `a949230659ec5fcfd78443530adf2b46d304ae7771e6dc8096e6b4b7ff1bd109` and `65d484a9ee0705b7450b29f9699b3070e205736bcc177b1f0fc7700ef5c25bc3`. Frozen v1/v2/v3 results were not edited. |

The first v4.0 full run stopped after 2,018 attempts when its scheduled S16
motor fault did not occur; v4.1 stopped after 1,452 attempts when S06 never
left the corridor. Both partial datasets are retained as `INCOMPLETE` and
excluded from v4.2 statistics. See the v4.1/v4.2 protocol addenda and abort
records. These are unsuccessful experimental attempts, not negative policy
outcomes.

## Measured results

The independent true edge definition requires both complete modeled strip
survey and true terminal arrival. Each arm had 900/1,200 true edge completions.
B2 reported 740/1,200 fulfilled; P reported 730/1,200. Both recorded 0/1,200
false survey completions and 0/1,200 false edge completions under the honest
comparative camera model. A zero-event scenario at n=100 still has a 3.699%
Wilson 95% upper bound. S11's separate dishonest-camera diagnostic did
produce one false acceptance, establishing the trust boundary.

| Scenario | B2 / P true edge | B2 / P reported | Mean travel B2 / P (m) | Paired P−B2 travel (m; 95% CI) | Paired P−B2 time (s; 95% CI) |
|---|---:|---:|---:|---:|---:|
| S03 physical detour | 100 / 100 | 100 / 100 | 62.046 / 44.687 | −17.359 [−17.559, −17.159] | −8.680 [−8.780, −8.580] |
| S04 camera outage | 100 / 100 | 100 / 100 | 60.198 / 49.279 | −10.920 [−11.099, −10.760] | −5.460 [−5.550, −5.370] |
| S05 metadata omission | 100 / 100 | 100 / 100 | 60.198 / 49.179 | −11.020 [−11.199, −10.840] | −5.510 [−5.600, −5.420] |
| S07 sparse telemetry | 100 / 100 | 100 / 100 | 60.500 / 55.380 | −5.120 [−5.120, −5.120] | −2.000 [−2.000, −2.000] |
| S08 GNSS outage | 100 / 100 | 18 / 15 | 57.389 / 43.557 | −13.832 [−14.532, −13.008] | +0.230 [−1.346, +1.832] |
| S09 estimator frame | 100 / 100 | 22 / 15 | 56.184 / 42.824 | −13.360 [−14.196, −12.416] | +1.534 [+0.226, +3.100] |

S00/S01/S06 had 100/100 true and reported completion per arm with no travel
or time difference. S02 had 0/100 true or reported completion per arm under
persistent wind. S16/S19 had 0/100 true or reported completion per arm after
realized physical recovery faults. Across all fixed scenarios, mean assigned
travel was 42.330 m for B2 and 36.363 m for P; medians were 50.931 m and
43.800 m. Mean simulated time was 28.909 s for B2 and 27.252 s for P; medians
were 24.2 s and 22.2 s. Each arm recorded an accepted recovery on 683/1,200 assigned
tasks. The fixed-scenario macro paired travel difference was −5.968 m
(seed-cluster 95% CI [−6.082, −5.834]), a descriptive 14.097% reduction
relative to B2 mean travel. The macro reported-completion difference was
−0.833 percentage points (95% CI [−1.5, −0.25] points). This aggregate cost
includes failed and unaccepted tasks; it is not an unconditional efficiency
advantage. S09's P−B2 accepted-completion difference was −7 percentage points
(paired 95% CI [−12, −2] points).

The full per-scenario medians, p95, IQR, Wilson intervals, capture and
recovery counts, truth gap measures and all paired intervals are in the
versioned CSV tables. The raw attempt table SHA-256 is
`a285af4a07ea073bb37ed73f04e5e2142fc1633d51a3ea3623d09d4cf7a2f46e`.

## Claims and limits

- H1 is supported only as observed agreement under this honest synthetic
  camera model. Zero measured false completions cannot establish zero risk;
  an authenticated dishonest adapter caused false acceptance in S11.
- H2 is supported for the S03/S04/S05 localized repair cases, with equal
  measured completion and less travel/time for P. S07 has the same measured
  direction but does not create a guaranteed hidden excursion. S02 shows no
  completion by either arm; S08/S09 show lower P acceptance despite shorter
  travel, with a statistically negative acceptance difference in S09.
- H3 has simulator/service-restart and reconciliation regression evidence.
  It does not cover an OS process power loss or real MAVLink takeover.
- H4 has local SDK/agent execution evidence. Remote replay, peer transport
  behavior and multi-UAV throughput are not measured.

The campaign uses a one-dimensional full-strip truth raster, synthetic
camera metadata, simplified acceleration/wind and terminal-sample true
arrival. It sets a 1,000-sample deviation threshold to isolate post-arrival
repair, so it does not experimentally quantify active mid-flight re-entry.
No electrical energy, CPU/RSS, real camera calibration, SITL or physical
flight measurements were taken. Only simulator evidence has been obtained.
SITL, real camera calibration, and physical flights are not validated.
