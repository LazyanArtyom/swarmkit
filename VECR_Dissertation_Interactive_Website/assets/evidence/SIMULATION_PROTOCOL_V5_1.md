# VECR v5.1 — endpoint-correctness replication

Written 2026-10-26 before the corrected campaign. Preserve every v5 and earlier
artifact. This is a complete replication of the frozen v5 matrix after two
endpoint defects were reproduced during the dissertation-reader audit.

## Corrections

1. Traversal credit previously included the entire estimated endpoint interval.
   Bounds of 2 m around estimates at 0 and 10 m allow an actual path 2→8 m.
   The old code could report full traversal. Credit now contracts by endpoint
   errors; overlapping endpoint bounds cannot manufacture motion.
2. The nadir adapter used a segment-clamped camera projection. A camera 5 m
   before A with a 1.763 m along half-footprint and wide cross footprint could
   incorrectly credit the route. Compute signed along-track and perpendicular
   cross-track position; clip the footprint interval, never the camera center.

The new regressions record failures before each fix. Prior v5 independently
reconstructed actual containment remains valid for those recorded trials, but
does not establish soundness for all endpoint configurations. No old evidence
is relabeled as a corrected-code run.

## Fixed design and criteria

Use exactly the v5 21 primary × 100 paired seeds and 18 sensitivity × 30 paired
seeds (5,280 attempts), with unchanged sensors, dynamics, faults, policies,
deadlines, priors, exclusions and inference. Preserve every assigned result.
Do not select conditions based on improvement. Keep all deliberate dishonest
controls and unsuccessful repairs. The goal is faithful replication under the
corrected implementation; equal or positive performance is not required.

Retain a separate 78-attempt engineering check (one paired seed 10004 for every
primary/sensitivity condition) before final execution. Freeze source and binary
before final runs. Record this amendment's SHA-256 with the original protocol
SHA-256. Use a new output directory named `corrected_endpoints_v5_1`.

After final science completes, run the original 80 serial benchmark assignments
into `benchmark_v5_1` using the same executable. Independent Python reconstruction,
all raw/hash/provenance gates, paired 10,000-resample bootstrap and whole-seed
aggregate uncertainty stay unchanged. Record changed and unchanged metrics
relative to v5; do not overwrite or merge datasets.

No ArduPilot SITL, hardware, remote/multi-UAV or real-camera experiment is
authorized by this replication. They do not acquire evidence through website edits.
