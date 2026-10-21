#!/usr/bin/env python3
"""Recheck v4 manifest hashes, assignments and false-completion arithmetic."""

import argparse
import csv
import hashlib
import json
import pathlib


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(output):
    manifest = json.loads((output / "manifest.json").read_text())
    config = json.loads((output / "protocol.json").read_text())
    if manifest["study_status"] != "ANALYZED":
        raise ValueError("campaign was not fully analyzed")
    if digest(output / "protocol.json") != manifest["config_id_sha256"]:
        raise ValueError("protocol hash mismatch")
    actual_paths = {str(path.relative_to(output)) for path in output.rglob("*")
                    if path.is_file() and path.name != "manifest.json"}
    expected_paths = set(manifest["artifact_sha256"])
    if actual_paths != expected_paths:
        raise ValueError(f"artifact set differs: missing={expected_paths-actual_paths} extra={actual_paths-expected_paths}")
    for relative, expected_hash in manifest["artifact_sha256"].items():
        if digest(output / relative) != expected_hash:
            raise ValueError(f"artifact hash mismatch: {relative}")
    with (output / "raw/per_attempt.csv").open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    cases = [case["id"] for case in config["scenarios"]
             if case["class"] == "COMPARATIVE"]
    expected = {(sid, str(seed), policy) for sid in cases
                for seed in manifest["seeds"] for policy in ("B2", "P")}
    actual = [(row["scenario_id"], row["seed"], row["policy"]) for row in rows]
    if len(actual) != len(set(actual)) or set(actual) != expected:
        raise ValueError("paired assignments missing or duplicated")
    for row in rows:
        if row["config_id"] != manifest["config_id_sha256"]:
            raise ValueError("row protocol hash mismatch")
        if row["reported_phase"] == "INFRASTRUCTURE_ERROR":
            raise ValueError("infrastructure-invalid row")
        false_complete = (row["reported_phase"] == "FULFILLED" and
                          row["truth_survey_complete"] != "1")
        if false_complete != (row["false_complete"] == "1"):
            raise ValueError("false completion arithmetic mismatch")
        if row["fault"] != "none" and row["requested_fault_realized"] != "1":
            raise ValueError("fault not realized")
    methods = list(csv.DictReader((output / "tables/methods_summary.csv").open(newline="")))
    for method in methods:
        subset = [row for row in rows if
                  (method["scenario_id"] == "ALL_DESCRIPTIVE" or
                   row["scenario_id"] == method["scenario_id"]) and
                  row["policy"] == method["policy"]]
        if int(method["assigned_n"]) != len(subset) or \
           int(method["false_complete_n"]) != sum(row["false_complete"] == "1"
                                                   for row in subset):
            raise ValueError("summary denominator/count mismatch")
    print(f"VALID: {len(rows)} paired-assignment rows and {len(expected_paths)} SHA-256 artifacts")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=pathlib.Path)
    args = parser.parse_args()
    main(args.output.resolve())
