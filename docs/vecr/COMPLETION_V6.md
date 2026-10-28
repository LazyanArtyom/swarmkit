# VECR v6 completion record — 28 October 2026

## Validation gates

| Gate | Outcome | Evidence |
|---|---|---|
| Audit before implementation | PASS | V6_WORK_LOG.md and frozen prior studies |
| Canonical core / protobuf / SDK / CLI / agent policies | PASS | v6 source, four comparison mechanisms and waiting ablations |
| Conditional recovery rule and derivation | COMPLETE, conditional | RECOVERY_MODEL_V6.md; no global optimality or completion theorem |
| V6 regression cases | PASS | 104 assertions, five cases, including native recovery isolation and MAVLink speed reset |
| Complete release suite | PASS | 2/2 CTest targets after final runtime changes |
| Complete UBSan suite | PASS | 2/2 CTest targets after final runtime changes |
| Installed SDK consumer | PASS | Recorded install, configure and compile logs |
| Frozen digital primary assignments | PASS | 5,940/5,940; 22 conditions × 30 seeds × nine policies; zero infrastructure failures |
| Independent digital reconstruction | PASS | 516,751 delivered captures; 3,984 physical re-entry dwells |
| Strong baselines and negative findings | COMPLETE | B2, D, R2, R8, P, G, GW, E0, E; E does not improve completion over GW |
| Statistical artifacts | COMPLETE | Raw results, 198 condition/policy rows, paired bootstrap intervals, Wilson intervals, SVG/PDF/PNG figures |
| External client reuse | PASS within stated scope | 200 trials, 100 direct/routed matched pairs; no metric differences |
| Independent reuse reconstruction | PASS | 11,120 captures; 80 re-entry dwells |
| ArduPilot corrected pilot | PASS | 12/12 checks; 525 accepted footprints and two sampled dwells |
| Frozen ArduPilot final qualification | PASS within synthetic-observation tier | 24/24 checks; 12 fulfilled, 12 intended negative outcomes; zero infrastructure failures |
| Independent SITL reconstruction | PASS with timing/sampling limits | 1,058 accepted footprints; four sampled re-entry dwells; no commands after LAND |
| Reader static integrity | PASS | 16 chapters, 198 current and 78 historical policy rows, 171 copied artifacts, no remote dependencies |
| Reader browser checks | PASS | Widths 390/720/1024/1440; filters, replay, dialogs, search, theme, navigation, print; no JavaScript errors or remote requests |
| Digital artifact SHA-256 verification | PASS | 99,893 recorded files, excluding inventories themselves |
| SITL artifact SHA-256 verification | PASS | 1,382 recorded files, excluding inventories themselves |
| Historical research preservation | PASS | No change to frozen previous campaigns; previous complete reader additionally archived |

The full scientific results are in FINAL_VALIDATION_V6.md; separate autopilot
results are in SITL_VALIDATION_V6.md. The reader includes both and preserves
Artyom Lazyan's institutional/department profile.

## Scientific interpretation

Gap-aware repair can avoid missing earlier work that departure/rewind policies
fail to revisit. Waiting helps metadata latency but costs time for missing images.
The derived E target is conditionally sufficient and conservative; GW matches its
completion and often uses less travel. In digital multiple gaps, E−GW distance is
+6.24008 m (95% paired interval [+6.23995, +6.24020], 30 joint successes).
SITL multiple gaps average 60.626 m / 39.248 s for E versus 55.490 m / 36.520 s
for GW, only two flights per policy. No broad SITL efficacy interval is inferred.
There are zero false fulfillments among 5,400 honest digital attempts and 540/540
in deliberately dishonest controls. The trusted sensing boundary remains explicit.

## Remaining limitations, not completed deployment features

* Native MAVLink accuracy is not certified as a hard bound; its recovery remains
  disabled. The positive SITL tier uses an explicit synthetic observation adapter.
* SIM_STATE lacks acquisition timestamps; <=50 ms upstream age is an assumption,
  not a proven bound. Sampled dwell checks do not prove continuous dwell.
* Calibrated real imagery, physical flight, complex terrain, obstacle/geofence-aware
  recovery and operational fleet reliability remain unvalidated.
* Reuse establishes direct/routed SDK entry paths, not arbitrary external planners.
* No empirical dominance of E, worldwide priority for local recovery, or guaranteed
  dissertation acceptance is established. The supported contribution is a precise
  implementation, conditional result and reproducible systems evaluation.

All requested digital comparisons, formalization, client-reuse tests and the
specified small SITL qualification have been executed. Failed pilots, the digital
20-seed protocol deviation and all raw outcomes remain available and separate.
