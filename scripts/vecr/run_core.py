#!/usr/bin/env python3
"""Run the versioned VECR strip model and regenerate its measured summaries."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import subprocess
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BINARY = ROOT / "build/mac-release/apps/swarmkit-vecr-experiment"
SCENARIOS = ["S00", "S01", "S03", "S04", "S05", "S06", "S07", "S11"]
METHODS = ["B0", "B1", "B2", "P"]


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summarize(rows: list[dict[str, str]]) -> tuple[list[dict], list[dict]]:
    by_method: dict[str, list[dict[str, str]]] = defaultdict(list)
    by_scenario: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        by_method[row["method"]].append(row)
        by_scenario[row["scenario"], row["method"]].append(row)

    def one(method: str, subset: list[dict[str, str]], scenario: str = "ALL") -> dict:
        n = len(subset)
        positives = sum(int(r["reported_complete"]) for r in subset)
        false_positives = sum(
            int(r["reported_complete"]) * (1 - int(r["truth_complete"])) for r in subset
        )
        truth_positives = sum(int(r["truth_complete"]) for r in subset)
        false_negatives = sum(
            (1 - int(r["reported_complete"])) * int(r["truth_complete"]) for r in subset
        )
        interventions = sum(int(r["interventions"]) for r in subset)
        unnecessary = sum(
            int(r["interventions"]) * (float(r["initial_truth_missing_fraction"]) == 0)
            for r in subset
        )
        gap_rows = [r for r in subset if float(r["initial_predicted_missing_fraction"]) >= 0]
        detected = sum(float(r["initial_predicted_missing_fraction"]) > 0 for r in gap_rows)
        true_detected = sum(
            float(r["initial_predicted_missing_fraction"]) > 0
            and float(r["initial_truth_missing_fraction"]) > 0
            for r in gap_rows
        )
        actual_gaps = sum(float(r["initial_truth_missing_fraction"]) > 0 for r in gap_rows)
        return {
            "scenario": scenario,
            "method": method,
            "trials": n,
            "reported_complete": positives,
            "false_completions": false_positives,
            "fcr": false_positives / positives if positives else None,
            "completion_report_rate": positives / n,
            "truth_complete": truth_positives,
            "false_negative_count": false_negatives,
            "false_negative_rate_among_truth_complete": (
                false_negatives / truth_positives if truth_positives else None
            ),
            "unknown_rate": sum(r["outcome"] == "unknown" for r in subset) / n,
            "mean_truth_missing_fraction": sum(float(r["truth_missing_fraction"]) for r in subset) / n,
            "mean_total_distance_m": sum(float(r["total_distance_m"]) for r in subset) / n,
            "interventions": interventions,
            "unnecessary_interventions": unnecessary,
            "initial_gap_detection_precision": true_detected / detected if detected else None,
            "initial_gap_detection_recall": true_detected / actual_gaps if actual_gaps else None,
        }

    return (
        [one(m, by_method[m]) for m in METHODS],
        [one(m, by_scenario[s, m], s) for s in SCENARIOS for m in METHODS],
    )


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def percent(value: float | None) -> str:
    return "N/A" if value is None else f"{value * 100:.1f}%"


def report(methods: list[dict], scenarios: list[dict], seeds: int) -> str:
    lines = [
        "# VECR core strip simulation — measured results",
        "",
        f"Protocol: `core-strip-v2`; seeds 1–{seeds}; {len(SCENARIOS)} coded scenarios; "
        f"{len(SCENARIOS) * seeds * len(METHODS)} paired method trials.",
        "",
        "| Method | Complete reports | False completions | FCR among reports | "
        "Truth complete | Mean distance | Unnecessary interventions |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in methods:
        lines.append(
            f"| {row['method']} | {row['reported_complete']}/{row['trials']} | "
            f"{row['false_completions']} | {percent(row['fcr'])} | "
            f"{row['truth_complete']} | {row['mean_total_distance_m']:.1f} m | "
            f"{row['unnecessary_interventions']} |"
        )
    b2 = next(row for row in methods if row["method"] == "B2")
    proposed = next(row for row in methods if row["method"] == "P")
    distance_saved = b2["mean_total_distance_m"] - proposed["mean_total_distance_m"]
    relative_saved = distance_saved / b2["mean_total_distance_m"]
    lines += [
        "",
        f"The paired modeled mean path saving for P against B2 is "
        f"{distance_saved:.1f} m ({relative_saved:.1%}) per trial. "
        "This includes travel to the gap and return to B.",
        "",
        "FCR denominator is reported-complete attempts. An empty denominator is N/A. "
        "The truth evaluator rasterizes actual modeled captures at 0.25 m; VECR sees "
        "only capture metadata and observed position. B2 and P share camera evidence "
        "and have equal modeled false-completion performance in this matrix. "
        "S05 is a metadata loss case: the terrain was truly observed, but B2/P "
        "repeat it because that evidence was unavailable.",
        "",
        "| Scenario | B0 FCR | B1 FCR | B2 FCR | P FCR | B2 distance | P distance |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for scenario in SCENARIOS:
        item = {r["method"]: r for r in scenarios if r["scenario"] == scenario}
        lines.append(
            f"| {scenario} | {percent(item['B0']['fcr'])} | {percent(item['B1']['fcr'])} | "
            f"{percent(item['B2']['fcr'])} | {percent(item['P']['fcr'])} | "
            f"{item['B2']['mean_total_distance_m']:.1f} m | "
            f"{item['P']['mean_total_distance_m']:.1f} m |"
        )
    lines += [
        "",
        "These are deterministic structural demonstrations, not independent field flights "
        "or statistical evidence for a population of UAVs. Eight of the requested 21 "
        "scenario IDs are implemented. Recovery is a modeled route replay in this "
        "experiment, not a measured backend/autopilot navigation outcome. The sensor "
        "quality predicate is supplied by a simulated adapter; no real imagery quality "
        "or SITL independent truth has been validated.",
        "",
    ]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--smoke", action="store_true")
    group.add_argument("--core-100", action="store_true")
    args = parser.parse_args()
    seeds = 5 if args.smoke else 100
    output = ROOT / "results/vecr"
    for directory in ("raw", "tables", "reports"):
        (output / directory).mkdir(parents=True, exist_ok=True)
    raw = output / "raw" / f"core_v2_{seeds}_seeds.csv"
    if raw.exists():
        raise SystemExit(
            f"{raw} is a frozen v2 research artifact; this historical runner will not overwrite it. "
            "Use a new versioned protocol for new experiments."
        )
    subprocess.run([str(BINARY), str(raw), str(seeds)], check=True, cwd=ROOT)
    with raw.open(newline="") as source:
        rows = list(csv.DictReader(source))
    methods, scenarios = summarize(rows)
    methods_path = output / "tables" / f"methods_v2_{seeds}_seeds.csv"
    scenarios_path = output / "tables" / f"scenarios_v2_{seeds}_seeds.csv"
    report_path = output / "reports" / f"CORE_RESULTS_V2_{seeds}_SEEDS.md"
    write_csv(methods_path, methods)
    write_csv(scenarios_path, scenarios)
    report_path.write_text(report(methods, scenarios, seeds))
    manifest = {
        "protocol": "core-strip-v2",
        "status": "modeled geometric simulation; partial scenario matrix",
        "git_commit": git("rev-parse", "HEAD"),
        "git_dirty": bool(git("status", "--porcelain")),
        "platform": platform.platform(),
        "machine": platform.machine(),
        "python": platform.python_version(),
        "binary_sha256": digest(BINARY),
        "source_sha256": digest(ROOT / "apps/vecr_experiment.cpp"),
        "generator_sha256": digest(Path(__file__)),
        "raw_sha256": digest(raw),
        "methods_table_sha256": digest(methods_path),
        "scenarios_table_sha256": digest(scenarios_path),
        "report_sha256": digest(report_path),
        "build_profile": "Release",
        "seed_range": [1, seeds],
        "scenarios": SCENARIOS,
        "methods": METHODS,
        "route_length_m": 100,
        "truth_cell_m": 0.25,
        "camera_half_length_m": 2,
        "required_strip_half_width_m": 4,
        "maximum_missing_m": 0,
        "excluded_trials": 0,
        "sitl_independent_truth_status": "UNAVAILABLE",
        "live_camera_status": "NOT_EXECUTED",
        "arm_device_benchmark_status": "NOT_EXECUTED",
    }
    (output / f"manifest_v2_{seeds}_seeds.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(report(methods, scenarios, seeds))


if __name__ == "__main__":
    main()
