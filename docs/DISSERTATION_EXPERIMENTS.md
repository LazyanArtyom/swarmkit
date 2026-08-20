# SwarmKit dissertation empirical evaluation

This page is a compact entry point to the final, frozen state-acceptance campaign. Detailed
methodology is in `dissertation/final_experiment_protocol.md`; generated values are in
`results/dissertation/` and summarized in `dissertation/final_results_summary.md`.

## Study design

- Deterministic `SimBackend`, 3 UAVs.
- 50 independent replicate clusters.
- 9 scenarios and 100 scored requests per replicate/scenario.
- 45,000 paired requests per method.
- Fixed base seed 42.
- 10,000-draw replicate-cluster bootstrap with analysis seed 123456789.
- Common accepted-output oracle with 3D `Umax = 3.0 m`.

## Final Table II

| Method | Requests | Accepted | FV [95% CI] | Availability [95% CI] | UAR [95% CI] |
| :--- | ---: | ---: | ---: | ---: | ---: |
| Receive-latest (`B0`) | 45,000 | 44,514 | 20.591% [20.570%, 20.616%] | 98.920% [98.871%, 98.967%] | 20.369% [20.349%, 20.391%] |
| Timestamp-aligned + age (`B1`) | 45,000 | 39,125 | 23.387% [23.349%, 23.426%] | 86.944% [86.798%, 87.084%] | 20.333% [20.333%, 20.333%] |
| Proposed (`P`) | 45,000 | 22,981 | 0.000% [0.000%, 0.000%] | 51.069% [50.891%, 51.236%] | 0.000% [0.000%, 0.000%] |

The paired `P-B1` FV difference was -23.387 percentage points, with 95% cluster-bootstrap CI
[-23.426, -23.349] percentage points. No p-value was computed.

No false-valid events were observed for `P` in this finite campaign.

## Final Table III

| Property | Generated result |
| :--- | ---: |
| Deterministic position enclosures evaluated | 68,943 |
| Observed containment failures | 0 (0.000%) |
| Persisted replay agreements | 22,981 / 22,981 |
| Replay disagreements | 0 |
| Mutation classes rejected | 44 / 44 |
| Mutated/inconsistent certificate cases rejected | 1,011,164 / 1,011,164 |
| N=10 end-to-end Release p95 latency | 426.000 µs |
| N=10 median serialized certificate | 7,744 bytes |

No containment failures were observed among the evaluated enclosures under the configured
deterministic assumptions. Finite experimental conformance does not prove universal zero failure
probability.

## Release scalability result

The timed operation is exactly
`RequestSnapshot + BuildCertificate + SerializeCertificate`, with 100 warmups and 1,000 saved
timed samples for each N.

| N | p50 (µs) | p95 (µs) | p99 (µs) | Median certificate bytes |
| ---: | ---: | ---: | ---: | ---: |
| 3 | 138.916 | 140.625 | 145.209 | 2,563 |
| 5 | 220.833 | 225.458 | 233.125 | 4,040 |
| 10 | 402.625 | 426.000 | 431.667 | 7,744 |

Latency increased approximately linearly across the tested N=3–10 range on the Apple M3 Pro
macOS ARM64 host. Target embedded or real-time suitability requires target-hardware benchmarking.

## Reproduction

```bash
cmake --build --preset mac-release
ctest --preset mac-release --output-on-failure
./build/mac-release/apps/swarmkit-dissertation-experiment \
  --runs 50 \
  --steps-per-scenario 100 \
  --seed-base 42 \
  --uav-count 3 \
  --output-dir results/dissertation
```

Persisted replay reconstructs evidence, session, clock, membership, request-frontier state, and
certificates from disk and verifies them against the separately supplied fixed canonical
`StateQualityContract`.
