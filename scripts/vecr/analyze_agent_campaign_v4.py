#!/usr/bin/env python3
"""Validate v4 raw agent trials and derive tables, SVGs and bounded prose."""

import argparse
import collections
import csv
import hashlib
import json
import math
import pathlib
import random
import statistics


def rows(path):
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path, data):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(data[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(data)


def mean(values):
    return statistics.fmean(values) if values else math.nan


def quantile(values, probability):
    if not values:
        return math.nan
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    left = int(position)
    return ordered[left] * (1 - position + left) + ordered[min(left + 1, len(ordered) - 1)] * (position - left)


def wilson(k, n):
    if n == 0:
        return (math.nan, math.nan)
    z = 1.959963984540054
    p = k / n
    divisor = 1 + z * z / n
    center = (p + z * z / (2 * n)) / divisor
    radius = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / divisor
    return center - radius, center + radius


def paired_ci(values, repetitions, seed):
    if not values:
        return math.nan, math.nan
    rng = random.Random(seed)
    n = len(values)
    samples = [mean(values[rng.randrange(n)] for _ in range(n))
               for _ in range(repetitions)]
    return quantile(samples, .025), quantile(samples, .975)


def fmt(value):
    return "NA" if value is None or (isinstance(value, float) and not math.isfinite(value)) else f"{value:.3f}"


def xml(value):
    return (str(value).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;"))


def svg_bars(path, title, labels, series, ylabel, ymax=None, error_bars=None):
    colors = ["#2468b4", "#d27b28", "#7c58a4", "#b54444"]
    width, height = 1020, 500
    all_values = [value for _, values in series for value in values if math.isfinite(value)]
    maximum = max(all_values + [1]) if ymax is None else ymax
    chart_left, chart_top, chart_bottom = 92, 68, 414
    step = (width - chart_left - 30) / max(1, len(labels))
    elements = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
                '<rect width="100%" height="100%" fill="white"/>',
                f'<text x="30" y="34" font-size="20" font-family="sans-serif">{xml(title)}</text>',
                f'<text x="22" y="60" font-size="12" font-family="sans-serif">{xml(ylabel)}</text>',
                f'<path d="M{chart_left} {chart_top}V{chart_bottom}H{width-20}" fill="none" stroke="#333"/>']
    for mark in range(5):
        value = maximum * mark / 4
        y = chart_bottom - (chart_bottom - chart_top) * mark / 4
        elements.append(f'<text x="30" y="{y+4:.1f}" font-size="11" font-family="sans-serif">{value:.1f}</text>')
        elements.append(f'<path d="M{chart_left} {y:.1f}H{width-20}" stroke="#ddd"/>')
    for index, label in enumerate(labels):
        center = chart_left + (index + .5) * step
        elements.append(f'<text x="{center:.1f}" y="436" text-anchor="middle" font-size="12" font-family="sans-serif">{xml(label)}</text>')
        for arm, (_, values) in enumerate(series):
            value = values[index]
            if not math.isfinite(value):
                continue
            bar_width = min(25, step / (len(series) + 1))
            x = center + (arm - (len(series) - 1) / 2) * (bar_width + 2) - bar_width / 2
            y = chart_bottom - (chart_bottom - chart_top) * value / maximum
            elements.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_width:.1f}" height="{chart_bottom-y:.1f}" fill="{colors[arm]}"/>')
            if error_bars:
                low, high = error_bars[arm][index]
                y_low = chart_bottom - (chart_bottom - chart_top) * low / maximum
                y_high = chart_bottom - (chart_bottom - chart_top) * high / maximum
                mid = x + bar_width / 2
                elements.append(f'<path d="M{mid:.1f} {y_low:.1f}V{y_high:.1f} M{mid-4:.1f} {y_low:.1f}H{mid+4:.1f} M{mid-4:.1f} {y_high:.1f}H{mid+4:.1f}" stroke="#222" fill="none" stroke-width="1.3"/>')
    for arm, (name, _) in enumerate(series):
        elements.append(f'<rect x="{chart_left+arm*160}" y="467" width="12" height="12" fill="{colors[arm]}"/>')
        elements.append(f'<text x="{chart_left+18+arm*160}" y="478" font-size="12" font-family="sans-serif">{xml(name)}</text>')
    path.write_text("\n".join(elements + ["</svg>"]) + "\n")


def trajectory_figure(path, traces, title, mode, gap=None):
    colors = {"B2": "#2468b4", "P": "#d27b28"}
    elements = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="400" viewBox="0 0 1000 400">',
                '<rect width="100%" height="100%" fill="white"/>',
                f'<text x="40" y="32" font-size="20" font-family="sans-serif">{xml(title)}</text>']
    if mode == "trajectory":
        elements += ['<rect x="70" y="110" width="840" height="160" fill="#e3f0e7" stroke="#7b9d82"/>',
                     '<text x="72" y="290" font-size="12" font-family="sans-serif">Required corridor ±4 m; A=0 m, B=20 m</text>']
        if gap:
            left, right = gap
            elements.append(f'<rect x="{70+42*left:.1f}" y="110" width="{42*(right-left):.1f}" height="160" fill="#f4d5d0" fill-opacity=".65"/>')
            elements.append(f'<text x="{70+42*left:.1f}" y="98" font-size="12" font-family="sans-serif">Initial camera outage</text>')
    else:
        elements.append('<text x="70" y="345" font-size="12" font-family="sans-serif">Time in simulated seconds; solid=private truth cells, dashed=agent-reported coverage converted to 0.25 m cells</text>')
    for policy, data in traces.items():
        points = []
        for row in data:
            if mode == "trajectory":
                x = 70 + 42 * float(row["truth_east_m"])
                y = 190 - 20 * float(row["truth_north_m"])
            else:
                x = 70 + 12 * float(row["simulation_time_ms"]) / 1000
                y = 305 - 2.5 * float(row["truth_covered_cells"])
            points.append(f"{x:.1f},{y:.1f}")
        elements.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{colors[policy]}" stroke-width="2"/>')
        if mode == "coverage":
            observed_points = [f"{70+12*float(row['simulation_time_ms'])/1000:.1f},{305-2.5*float(row['reported_covered_m'])/.25:.1f}"
                               for row in data]
            elements.append(f'<polyline points="{" ".join(observed_points)}" fill="none" stroke="{colors[policy]}" stroke-width="1.5" stroke-dasharray="5 4"/>')
        elements.append(f'<text x="{700 if policy == "B2" else 810}" y="365" fill="{colors[policy]}" font-size="14" font-family="sans-serif">{policy}</text>')
    path.write_text("\n".join(elements + ["</svg>"]) + "\n")


def analyze(output):
    config = json.loads((output / "protocol.json").read_text())
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if manifest["study_status"] != "RAW_COMPLETE":
        raise ValueError("raw campaign incomplete; refusing dissertation analysis")
    raw = rows(output / "raw/per_attempt.csv")
    expected = [case for case in config["scenarios"] if case["class"] == "COMPARATIVE"]
    expected_keys = {(case["id"], str(seed), policy) for case in expected
                     for seed in manifest["seeds"] for policy in ("B2", "P")}
    keys = [(r["scenario_id"], r["seed"], r["policy"]) for r in raw]
    if len(keys) != len(set(keys)) or set(keys) != expected_keys:
        raise ValueError("paired assignment missing, extra or duplicated")
    if len(raw) != manifest["attempts_completed"]:
        raise ValueError("manifest/raw row count mismatch")
    for row in raw:
        if row["config_id"] != manifest["config_id_sha256"]:
            raise ValueError("mixed configuration hashes")
        if row["reported_phase"] == "INFRASTRUCTURE_ERROR":
            raise ValueError("infrastructure-invalid attempt in final results")
        truth = row["truth_survey_complete"] == "1"
        fulfilled = row["reported_phase"] == "FULFILLED"
        if (fulfilled and not truth) != (row["false_complete"] == "1"):
            raise ValueError("false completion was dropped or misclassified")
        if row["fault"] != "none" and row["requested_fault_realized"] != "1":
            raise ValueError("scheduled fault absent")
    by_key = {(r["scenario_id"], r["seed"], r["policy"]): r for r in raw}
    nboot = config["bootstrap_resamples"]
    methods = []
    scenarios = []
    paired = []
    for case in expected:
        sid = case["id"]
        subsets = {arm: [r for r in raw if r["scenario_id"] == sid and r["policy"] == arm]
                   for arm in ("B2", "P")}
        for arm, subset in subsets.items():
            n = len(subset)
            truth_n = sum(r["truth_survey_complete"] == "1" for r in subset)
            report_n = sum(r["reported_phase"] == "FULFILLED" for r in subset)
            false_n = sum(r["false_complete"] == "1" for r in subset)
            tp_n = sum(r["truth_survey_complete"] == "1" and
                       r["reported_phase"] == "FULFILLED" for r in subset)
            distance = [float(r["physical_distance_m"]) for r in subset]
            seconds = [float(r["elapsed_simulation_ms"]) / 1000 for r in subset]
            low, high = wilson(false_n, n)
            completed_low, completed_high = wilson(report_n, n)
            distance_low, distance_high = paired_ci(
                distance, nboot, config["bootstrap_seed"] + sum(map(ord, sid + arm + "distance")))
            time_low, time_high = paired_ci(
                seconds, nboot, config["bootstrap_seed"] + sum(map(ord, sid + arm + "time")))
            method = {"scenario_id": sid, "policy": arm, "assigned_n": n,
                      "true_complete_n": truth_n, "reported_complete_n": report_n,
                      "true_positive_n": tp_n, "false_complete_n": false_n,
                      "false_complete_rate": false_n / n,
                      "false_complete_wilson_low": low,
                      "false_complete_wilson_high": high,
                      "false_fraction_among_reports": false_n / report_n if report_n else "NA",
                      "reported_complete_rate": report_n / n,
                      "reported_complete_wilson_low": completed_low,
                      "reported_complete_wilson_high": completed_high,
                      "unknown_n": sum(r["reported_phase"] == "UNKNOWN" for r in subset),
                      "interrupted_n": sum(r["reported_phase"] == "INTERRUPTED" for r in subset),
                      "censored_n": sum(r["reported_phase"] not in
                                        ("FULFILLED", "INTERRUPTED", "UNKNOWN") for r in subset),
                      "distance_mean_m": mean(distance),
                      "distance_mean_ci_low": distance_low,
                      "distance_mean_ci_high": distance_high,
                      "distance_median_m": quantile(distance, .5),
                      "distance_p95_m": quantile(distance, .95),
                      "distance_iqr_m": quantile(distance, .75) - quantile(distance, .25),
                      "time_mean_s": mean(seconds), "time_median_s": quantile(seconds, .5),
                      "time_mean_ci_low": time_low, "time_mean_ci_high": time_high,
                      "time_p95_s": quantile(seconds, .95),
                      "recoveries_mean": mean([int(r["recovery_attempts"]) for r in subset]),
                      "truth_missing_fraction_mean": mean([float(r["truth_missing_fraction"]) for r in subset]),
                      "captures_acquired_mean": mean([int(r["capture_acquired"]) for r in subset])}
            methods.append(method)
            scenarios.append(dict(method))
        pairs = [(by_key[sid, str(seed), "B2"], by_key[sid, str(seed), "P"])
                 for seed in manifest["seeds"]]
        diffs = {}
        for key, parser in (("distance_m", lambda r: float(r["physical_distance_m"])),
                            ("time_s", lambda r: float(r["elapsed_simulation_ms"]) / 1000),
                            ("true_complete", lambda r: float(r["truth_survey_complete"])),
                            ("reported_complete", lambda r: float(r["reported_phase"] == "FULFILLED")),
                            ("false_complete", lambda r: float(r["false_complete"]))):
            values = [parser(p) - parser(b) for b, p in pairs]
            low, high = paired_ci(values, nboot,
                                  config["bootstrap_seed"] + sum(map(ord, sid + key)))
            diffs[f"delta_{key}_mean"] = mean(values)
            diffs[f"delta_{key}_ci_low"] = low
            diffs[f"delta_{key}_ci_high"] = high
        paired.append({"scenario_id": sid, "paired_seeds_n": len(pairs), **diffs})
    scenario_paired = list(paired)
    macro_diffs = {}
    for key, parser in (("distance_m", lambda r: float(r["physical_distance_m"])),
                        ("time_s", lambda r: float(r["elapsed_simulation_ms"]) / 1000),
                        ("true_complete", lambda r: float(r["truth_survey_complete"])),
                        ("reported_complete", lambda r: float(r["reported_phase"] == "FULFILLED")),
                        ("false_complete", lambda r: float(r["false_complete"]))):
        # The scenario set is fixed. Resample independent seed clusters, not
        # individual scenario attempts, for the average scenario effect.
        values = [mean(parser(by_key[case["id"], str(seed), "P"]) -
                       parser(by_key[case["id"], str(seed), "B2"])
                       for case in expected) for seed in manifest["seeds"]]
        low, high = paired_ci(values, nboot,
                              config["bootstrap_seed"] + sum(map(ord, "ALL_MACRO" + key)))
        macro_diffs[f"delta_{key}_mean"] = mean(values)
        macro_diffs[f"delta_{key}_ci_low"] = low
        macro_diffs[f"delta_{key}_ci_high"] = high
    paired.append({"scenario_id": "ALL_MACRO", "paired_seeds_n": len(manifest["seeds"]),
                   **macro_diffs})
    for arm in ("B2", "P"):
        subset = [r for r in raw if r["policy"] == arm]
        distance = [float(r["physical_distance_m"]) for r in subset]
        seconds = [float(r["elapsed_simulation_ms"]) / 1000 for r in subset]
        fulfilled = sum(r["reported_phase"] == "FULFILLED" for r in subset)
        false = sum(r["false_complete"] == "1" for r in subset)
        methods.append({"scenario_id": "ALL_DESCRIPTIVE", "policy": arm,
                        "assigned_n": len(subset),
                        "true_complete_n": sum(r["truth_survey_complete"] == "1" for r in subset),
                        "reported_complete_n": fulfilled,
                        "true_positive_n": fulfilled - false,
                        "false_complete_n": false,
                        "false_complete_rate": false / len(subset),
                        "false_complete_wilson_low": "NA",
                        "false_complete_wilson_high": "NA",
                        "false_fraction_among_reports": false / fulfilled if fulfilled else "NA",
                        "reported_complete_rate": fulfilled / len(subset),
                        "reported_complete_wilson_low": "NA",
                        "reported_complete_wilson_high": "NA",
                        "unknown_n": sum(r["reported_phase"] == "UNKNOWN" for r in subset),
                        "interrupted_n": sum(r["reported_phase"] == "INTERRUPTED" for r in subset),
                        "censored_n": sum(r["reported_phase"] not in
                                          ("FULFILLED", "INTERRUPTED", "UNKNOWN") for r in subset),
                        "distance_mean_m": mean(distance),
                        "distance_mean_ci_low": "NA", "distance_mean_ci_high": "NA",
                        "distance_median_m": quantile(distance, .5),
                        "distance_p95_m": quantile(distance, .95),
                        "distance_iqr_m": quantile(distance, .75) - quantile(distance, .25),
                        "time_mean_s": mean(seconds), "time_median_s": quantile(seconds, .5),
                        "time_mean_ci_low": "NA", "time_mean_ci_high": "NA",
                        "time_p95_s": quantile(seconds, .95),
                        "recoveries_mean": mean(int(r["recovery_attempts"]) for r in subset),
                        "truth_missing_fraction_mean": mean(float(r["truth_missing_fraction"]) for r in subset),
                        "captures_acquired_mean": mean(int(r["capture_acquired"]) for r in subset)})
    write_csv(output / "tables/methods_summary.csv", methods)
    write_csv(output / "tables/scenarios_summary.csv", scenarios)
    write_csv(output / "tables/paired_comparisons.csv", paired)
    inventory = rows(output / "tables/scenario_inventory.csv")
    diagnostic_rows = (rows(output / "diagnostics/agent_attempts.csv")
                       if (output / "diagnostics/agent_attempts.csv").is_file() else [])
    regression_rows = (rows(output / "diagnostics/regression_results.csv")
                       if (output / "diagnostics/regression_results.csv").is_file() else [])
    for row in inventory:
        if row["class"] == "COMPARATIVE":
            row["validation_status"] = "EXECUTED_AGENT_BACKED"
        elif any(case["scenario_id"] == row["scenario_id"] for case in diagnostic_rows):
            matching = [case for case in diagnostic_rows
                        if case["scenario_id"] == row["scenario_id"]]
            row["validation_status"] = ("EXECUTED_AGENT_DIAGNOSTIC" if all(
                case["fault_realized"] == "1" for case in matching)
                else "INCONCLUSIVE_FAULT_NOT_REALIZED")
        elif any(case["scenario_id"] == row["scenario_id"] and case["exit_code"] == "0"
                 for case in regression_rows):
            row["validation_status"] = "REGRESSION_TEST_ONLY"
    write_csv(output / "tables/scenario_inventory.csv", inventory)
    ids = [case["id"] for case in expected]
    for metric, filename, title, ylabel in (("distance_mean_m", "distance_by_scenario.svg", "Travel distance by scenario", "Mean actual distance, m"),
                                             ("time_mean_s", "time_by_scenario.svg", "Mission-clock time by scenario", "Mean elapsed, s")):
        svg_bars(output / "figures" / filename, title, ids,
                 [(arm, [next(r[metric] for r in methods if r["scenario_id"] == sid and r["policy"] == arm)
                 for sid in ids]) for arm in ("B2", "P")], ylabel,
                 error_bars=[[(next(r[metric.split("_")[0] + "_mean_ci_low"] for r in methods
                                   if r["scenario_id"] == sid and r["policy"] == arm),
                              next(r[metric.split("_")[0] + "_mean_ci_high"] for r in methods
                                   if r["scenario_id"] == sid and r["policy"] == arm))
                             for sid in ids] for arm in ("B2", "P")])
    svg_bars(output / "figures/false_completion_and_availability.svg",
             "False survey completion and reported availability", ids,
             [(label, [next(r[key] for r in methods if r["scenario_id"] == sid and r["policy"] == arm)
                       for sid in ids]) for label, arm, key in
              (("B2 fulfilled", "B2", "reported_complete_rate"),
               ("P fulfilled", "P", "reported_complete_rate"),
               ("B2 false complete", "B2", "false_complete_rate"),
               ("P false complete", "P", "false_complete_rate"))],
             "Fraction of all assigned attempts", 1)
    phases = ["FULFILLED", "INTERRUPTED", "UNKNOWN", "MOVING", "RECOVERING"]
    svg_bars(output / "figures/failure_modes.svg", "Reported trial outcomes",
             phases, [(arm, [sum(r["policy"] == arm and r["reported_phase"] == phase
                                for r in raw) for phase in phases]) for arm in ("B2", "P")],
             "Assigned attempts")
    traces = {arm: rows(output / "raw/traces" / f"v4-{manifest['mode']}-S04-001-{arm}.csv")
              for arm in ("B2", "P")}
    gap_row = by_key["S04", str(manifest["seeds"][0]), "P"]
    gap = float(gap_row["gap_begin_m"]), float(gap_row["gap_end_m"])
    trajectory_figure(output / "figures/trajectory_good_bad.svg", traces,
                      "S04 physical paths under paired camera outage", "trajectory", gap)
    trajectory_figure(output / "figures/coverage_timeline.svg", traces,
                      "S04 independent true camera coverage through repair", "coverage")
    totals = {arm: [r for r in raw if r["policy"] == arm] for arm in ("B2", "P")}
    lines = ["# Agent-backed VECR simulator results", "",
             "## Experimental goal", "",
             "Evaluate truthfulness and recovery cost for a directed A→B surveying obligation.",
             "", "## System under test", "",
             "SDK → TLS AgentService → production edge runtime → SimBackend; synthetic camera and private 0.25 m truth oracle.",
             "", "## Baselines and fairness", "",
             "B2 full-edge restart and P targeted-gap repair share the same verifier, camera, dynamic vehicle, safety bounds, timeout and retry limit. Only repair re-entry policy differs.",
             "", "## Scenarios", "",
             f"{len(expected)} comparative cases × {len(manifest['seeds'])} seeds × 2 policies = {len(raw)} actual agent-backed attempts. S13, S14 and S17 remain unsupported; other noncomparative cases are excluded from these statistics.",
             "", "## Ground-truth design", "",
             "Private simulator poses and actual camera acquisition feed a separate 0.25 m along-edge cell oracle. A cell counts only when the modeled camera footprint spans the full required strip width. This is not a two-dimensional terrain-image oracle. The agent receives normalized telemetry and adapter metadata only.",
             "", "## Metrics", "",
             "Distance integrates true 20 ms physical steps. Time is simulated mission-clock duration. False completion is FULFILLED with at least one required truth cell missing. All assigned trials stay in denominators.",
             "", "## Results (actual values with uncertainty)", "",
             "| Scenario | n pairs | B2 true/reported/false | P true/reported/false | B2 mean m | P mean m | Δ P−B2 m (95% paired CI) | Δ time s (95% paired CI) |",
             "|---|---:|---|---|---:|---:|---|---|"]
    for comparison in scenario_paired:
        sid = comparison["scenario_id"]
        b = next(r for r in methods if r["scenario_id"] == sid and r["policy"] == "B2")
        p = next(r for r in methods if r["scenario_id"] == sid and r["policy"] == "P")
        counts = lambda r: f"{r['true_complete_n']}/{r['reported_complete_n']}/{r['false_complete_n']}"
        lines.append(f"| {sid} | {comparison['paired_seeds_n']} | {counts(b)} | {counts(p)} | {fmt(b['distance_mean_m'])} | {fmt(p['distance_mean_m'])} | {fmt(comparison['delta_distance_m_mean'])} [{fmt(comparison['delta_distance_m_ci_low'])}, {fmt(comparison['delta_distance_m_ci_high'])}] | {fmt(comparison['delta_time_s_mean'])} [{fmt(comparison['delta_time_s_ci_low'])}, {fmt(comparison['delta_time_s_ci_high'])}] |")
    b2_all = next(r for r in methods if r["scenario_id"] == "ALL_DESCRIPTIVE" and r["policy"] == "B2")
    relative_saving = -macro_diffs["delta_distance_m_mean"] / b2_all["distance_mean_m"]
    lines += ["", "Each true/reported/false entry is a count of assigned attempts, not a conditional success-only mean. Wilson intervals and additional distributions are in `tables/methods_summary.csv`; all paired effect intervals are in `tables/paired_comparisons.csv`.",
              f"Across the fixed 12-scenario set, the descriptive macro mean P−B2 distance is {fmt(macro_diffs['delta_distance_m_mean'])} m (95% seed-cluster bootstrap [{fmt(macro_diffs['delta_distance_m_ci_low'])}, {fmt(macro_diffs['delta_distance_m_ci_high'])}]); the corresponding relative travel saving is {fmt(100 * relative_saving)}% of B2 mean assigned-task distance. This includes failed tasks and is not a recovery-efficiency claim where completion differs.",
              "", "### Separate noncomparative diagnostics", "",
              "| Scenario | Variant | Reported | True survey | False complete | Fault realized |",
              "|---|---|---|---|---|---|"]
    for row in diagnostic_rows:
        lines.append(f"| {row['scenario_id']} | {row['variant']} | {row['reported_phase']} | {row['truth_survey_complete']} | {row['false_complete']} | {row['fault_realized']} |")
    lines += ["", "These single diagnostic attempts are not probability estimates and are not pooled with the B2/P comparison.",
              "", "## Tradeoffs/negative results", "",
              "Shorter travel on failed or interrupted tasks is not a recovery benefit. Compare completion counts before interpreting a negative distance difference. Persistent faults can cause both policies to fail. A plausible dishonest camera can create false fulfillment, outside honest-adapter assumptions.",
              "", "## Threats to validity", "",
              "The simulator has simplified horizontal acceleration, instant modeled camera acquisition, a one-dimensional full-strip coverage oracle, perfect base telemetry and no terrain occlusion. Location-tied interventions may have policy-dependent realized duration. Scenario seeds vary preregistered gap position/length and wind strength but do not sample a full field distribution. S15 recreates AgentService in one host process with a surviving simulator; an OS process power loss was not tested. No electrical energy or CPU/RSS benchmark is reported.",
              "", "## Reproducibility", "",
              f"Source revision `{manifest['source_revision']}`; binary SHA-256 `{manifest['binary_sha256']}`; config SHA-256 `{manifest['config_id_sha256']}`. See manifest for host, commands and artifact hashes.",
              "", "## What is not demonstrated", "",
              "Only simulator evidence has been obtained. SITL, real camera calibration, and physical flights are not validated. Remote transport-loss replay, peer interruption, OS process crash/power loss and simultaneous multi-UAV throughput are not validated by this campaign.", ""]
    (output / "reports/SCIENTIFIC_RESULTS.md").write_text("\n".join(lines))
    (output / "reports/EXECUTIVE_RESULTS.md").write_text(
        f"# Executive results\n\n{len(raw)} actual B2/P agent-backed attempts across {len(expected)} scenarios and {len(manifest['seeds'])} paired seeds.\n\nSee `SCIENTIFIC_RESULTS.md` and raw tables for measured values and uncertainty. Unsupported cases are excluded.\n")
    (output / "reports/LIMITATIONS_AND_NEGATIVE_RESULTS.md").write_text(
        "# Limitations and negative results\n\nThe S11 dishonest camera case can fool the verifier when authenticated metadata lies; the independent oracle detects the unobserved ground. S13/S14/S17 are unsupported in this campaign. S07 does not guarantee an intersample excursion, and S10 cannot isolate a brief passage. S15 recreates a service object, not an OS process crash. Failed/interrupted tasks remain in all denominators. Real sensing, SITL and physical flight are unvalidated.\n")
    artifact_hashes = {}
    for path in sorted(output.rglob("*")):
        if path.is_file() and path != manifest_path:
            artifact_hashes[str(path.relative_to(output))] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest["study_status"] = "ANALYZED"
    manifest["artifact_sha256"] = artifact_hashes
    manifest["validation"] = {"expected_attempts": len(expected_keys),
                              "observed_attempts": len(raw), "paired_rows_valid": True,
                              "fault_realization_checked": True,
                              "false_completion_recomputed": True}
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Analyzed {len(raw)} actual agent attempts; {len(artifact_hashes)} artifact hashes")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=pathlib.Path, required=True)
    args = parser.parse_args()
    analyze(args.output.resolve())
