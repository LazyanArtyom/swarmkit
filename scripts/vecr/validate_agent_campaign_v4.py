#!/usr/bin/env python3
"""Recheck v4 manifest hashes, assignments and false-completion arithmetic."""

import argparse
import csv
import hashlib
import json
import math
import pathlib
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(output):
    manifest = json.loads((output / "manifest.json").read_text())
    config = json.loads((output / "protocol.json").read_text())
    if manifest["study_status"] != "ANALYZED":
        raise ValueError("campaign was not fully analyzed")
    if "analysis_code_sha256" in manifest:
        source = subprocess.check_output(
            ["git", "show", f"{manifest['analysis_revision']}:scripts/vecr/analyze_agent_campaign_v4.py"])
        if hashlib.sha256(source).hexdigest() != manifest["analysis_code_sha256"]:
            raise ValueError("analysis code hash/revision mismatch")
    for path, expected_hash in manifest.get("source_file_sha256", {}).items():
        source = subprocess.check_output(
            ["git", "show", f"{manifest['source_revision']}:{path}"])
        if hashlib.sha256(source).hexdigest() != expected_hash:
            raise ValueError(f"trial source hash/revision mismatch: {path}")
    if digest(output / "protocol.json") != manifest["config_id_sha256"]:
        raise ValueError("protocol hash mismatch")
    actual_paths = {str(path.relative_to(output)) for path in output.rglob("*")
                    if path.is_file() and path.name != "manifest.json"}
    expected_paths = set(manifest["artifact_sha256"])
    if actual_paths != expected_paths:
        raise ValueError(f"artifact set differs: missing={expected_paths-actual_paths} extra={actual_paths-expected_paths}")
    if manifest["mode"] == "full":
        required_figures = {
            "figures/trajectory_good_bad.svg", "figures/trajectory_detour.svg",
            "figures/coverage_timeline.svg", "figures/distance_by_scenario.svg",
            "figures/time_by_scenario.svg",
            "figures/false_completion_and_availability.svg",
            "figures/failure_modes.svg"}
        if not required_figures <= expected_paths:
            raise ValueError("full campaign is missing required figures")
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
        true_edge = lambda row: (row["truth_survey_complete"] == "1" and
                                 row["truth_at_b"] == "1")
        if "true_survey_n" in method:
            if int(method["true_survey_n"]) != sum(row["truth_survey_complete"] == "1"
                                                    for row in subset) or \
               int(method["true_complete_n"]) != sum(true_edge(row) for row in subset) or \
               int(method["true_positive_n"]) != sum(true_edge(row) and
                                                      row["reported_phase"] == "FULFILLED"
                                                      for row in subset) or \
               int(method["false_edge_complete_n"]) != sum(
                   row["reported_phase"] == "FULFILLED" and not true_edge(row)
                   for row in subset):
                raise ValueError("independent truth/edge-completion arithmetic mismatch")
    comparisons = list(csv.DictReader((output / "tables/paired_comparisons.csv").open(newline="")))
    if {row["scenario_id"] for row in comparisons} != set(cases) | {"ALL_MACRO"}:
        raise ValueError("scenario or macro paired comparison missing")
    by_key = {(row["scenario_id"], row["seed"], row["policy"]): row for row in rows}
    parsers = {
        "distance_m": lambda row: float(row["physical_distance_m"]),
        "time_s": lambda row: float(row["elapsed_simulation_ms"]) / 1000,
        "true_complete": lambda row: float(row["truth_survey_complete"] == "1" and
                                          row["truth_at_b"] == "1"),
        "reported_complete": lambda row: float(row["reported_phase"] == "FULFILLED"),
        "false_complete": lambda row: float(row["false_complete"] == "1"),
    }
    for comparison in comparisons:
        sid = comparison["scenario_id"]
        if int(comparison["paired_seeds_n"]) != len(manifest["seeds"]):
            raise ValueError("paired seed denominator mismatch")
        for metric, parse in parsers.items():
            delta_by_seed = []
            for seed in manifest["seeds"]:
                scenario_ids = cases if sid == "ALL_MACRO" else [sid]
                deltas = [parse(by_key[case, str(seed), "P"]) -
                          parse(by_key[case, str(seed), "B2"])
                          for case in scenario_ids]
                delta_by_seed.append(sum(deltas) / len(deltas))
            measured = sum(delta_by_seed) / len(delta_by_seed)
            if not math.isclose(measured, float(comparison[f"delta_{metric}_mean"]),
                                rel_tol=1e-10, abs_tol=1e-10):
                raise ValueError(f"paired {metric} arithmetic mismatch")
    inventory = list(csv.DictReader((output / "tables/scenario_inventory.csv").open(newline="")))
    if len(inventory) != 21 or {row["scenario_id"] for row in inventory} != {
            item["id"] for item in config["scenarios"]}:
        raise ValueError("21-scenario inventory mismatch")
    for item in inventory:
        if item["class"] != "COMPARATIVE":
            continue
        subset = [row for row in rows if row["scenario_id"] == item["scenario_id"]]
        if int(item["attempts_n"]) != len(subset) or int(item["fault_realized_n"]) != sum(
                row["requested_fault_realized"] == "1" for row in subset):
            raise ValueError("scenario inventory fault/count mismatch")
    print(f"VALID: {len(rows)} paired-assignment rows and {len(expected_paths)} SHA-256 artifacts")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=pathlib.Path)
    args = parser.parse_args()
    main(args.output.resolve())
