#!/usr/bin/env python3
"""Compare matched raw v5/v5.1 rows without pooling or modifying either study."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

METRICS = (
    "distance_m",
    "time_ms",
    "truth_survey",
    "truth_at_b",
    "truth_complete",
    "reported_complete",
    "false_complete",
    "false_negative",
    "recoveries",
    "accepted",
    "accepted_uncontained",
    "reported_covered_m",
)


def compare(previous, corrected, output):
    def load(root):
        with (root / "per_attempt.csv").open() as f:
            rows = list(csv.DictReader(f))
        result = {r["run_id"]: r for r in rows}
        assert len(result) == len(rows)
        assert not any(r["infrastructure_error"] for r in rows)
        return result

    before, after = load(previous), load(corrected)
    assert before.keys() == after.keys()
    counts = []
    for metric in METRICS:
        differences = [
            float(after[k][metric]) - float(before[k][metric]) for k in before
        ]
        counts.append(
            dict(
                metric=metric,
                paired_attempts=len(before),
                changed_attempts=sum(x != 0 for x in differences),
                max_absolute_difference=max(abs(x) for x in differences),
            )
        )
    output.mkdir(exist_ok=True, parents=True)
    with (output / "replication_comparison.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=counts[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(counts)
    metadata = {}
    for label, root in (("previous", previous), ("corrected", corrected)):
        metadata[label] = dict(
            source_revision=json.loads((root / "manifest.json").read_text())[
                "source_revision"
            ],
            raw_sha256=hashlib.sha256(
                (root / "per_attempt.csv").read_bytes()
            ).hexdigest(),
        )
    metadata["metrics"] = counts
    (output / "replication_comparison.json").write_text(
        json.dumps(metadata, indent=2) + "\n"
    )
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--previous", type=Path, required=True)
    parser.add_argument("--corrected", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    compare(args.previous, args.corrected, args.output)
