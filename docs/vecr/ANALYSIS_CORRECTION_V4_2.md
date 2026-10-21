# VECR v4.2 post-collection analysis correction

The final v4.2 agent-backed run finished with 2,400 raw attempts under trial
source revision `5adb6872f4b593cdd21ab2ce5c63063814a03b7f`. Its binary,
protocol, commands, outcomes and raw rows are unchanged by this correction.
The analyzer is committed separately before it produces the final tables,
figures and prose. The final manifest records both source revisions and
SHA-256 hashes.

The pre-analysis audit found that the cross-scenario `true_complete` paired
difference used private survey coverage alone. Per-scenario truth counts
already required both private full-strip coverage and true terminal arrival.
The corrected macro calculation uses the same two-part definition. Paired
distance, time, reported-completion and false-completion calculations are
unchanged. The validator now recomputes all five paired point estimates from
raw B2/P rows, including the macro result.

The figure generator also assumed that the first saved trace had seed 1. It
now uses the first seed listed in each manifest, allowing the separately
collected 101–103 sensitivity run to be analyzed without relabeling or
mixing it with the 1–100 confirmatory run. The scenario inventory records
executed and realized-fault counts. The failure chart explicitly shows zero
paired cancellation and safety counts; the S12 safety intervention remains a
separate diagnostic. Source-file and dependency-manifest hashes are added to
the final verification manifest.

The trial harness sets `sustained_deviation_samples` to 1,000 for both arms.
This isolates recovery strategy after a missing survey is found at B, and
limits the comparison's scope: it does not measure early mid-flight
deviation-triggered re-entry. The effective constant is stated in the
analysis manifest and limitations report. The product's ordinary deviation
logic retains its separate regression coverage.
