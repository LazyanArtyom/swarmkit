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
import subprocess


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


def truth_edge(row):
    return row["truth_survey_complete"] == "1" and row["truth_at_b"] == "1"


def committed_sha(revision, path):
    content = subprocess.check_output(["git", "show", f"{revision}:{path}"])
    return hashlib.sha256(content).hexdigest()


def xml(value):
    return (str(value).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;"))


def svg_bars(path, title, labels, series, ylabel, ymax=None, error_bars=None):
    colors = ["#2468b4", "#d27b28", "#7c58a4", "#b54444"]
    width, height = 1020, 500
    all_values = [value for _, values in series for value in values if math.isfinite(value)]
    error_highs = ([high for arm in error_bars for _, high in arm]
                   if error_bars else [])
    maximum = max(all_values + error_highs + [1]) * 1.08 if ymax is None else ymax
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
    left, right, top, bottom = 86, 940, 72, 320
    max_time = max(float(row["simulation_time_ms"]) / 1000
                   for data in traces.values() for row in data)
    time_to_x = lambda seconds: left + (right - left) * seconds / max_time
    if mode == "trajectory":
        y_max = 20.0
        y_of = lambda row: float(row["truth_east_m"])
        ylabel = "True along-edge position (m)"
    else:
        y_max = max(float(row["truth_covered_cells"])
                    for data in traces.values() for row in data)
        y_of = lambda row: float(row["truth_covered_cells"])
        ylabel = "Covered 0.25 m cells"
    value_to_y = lambda value: bottom - (bottom - top) * value / y_max
    elements = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="440" viewBox="0 0 1000 440">',
                '<rect width="100%" height="100%" fill="white"/>',
                f'<text x="36" y="32" font-size="20" font-family="sans-serif">{xml(title)}</text>',
                f'<text x="38" y="54" font-size="12" font-family="sans-serif">{xml(ylabel)}</text>']
    if mode == "trajectory" and gap:
        gap_bottom, gap_top = value_to_y(gap[0]), value_to_y(gap[1])
        elements.append(f'<rect x="{left}" y="{gap_top:.1f}" width="{right-left}" height="{gap_bottom-gap_top:.1f}" fill="#f5dad5"/>')
        elements.append(f'<text x="{right-160}" y="{gap_top-7:.1f}" font-size="12" font-family="sans-serif">Initial camera gap</text>')
    for step in range(5):
        value = y_max * step / 4
        y = value_to_y(value)
        elements.append(f'<path d="M{left} {y:.1f}H{right}" stroke="#ddd"/>')
        elements.append(f'<text x="{left-10}" y="{y+4:.1f}" text-anchor="end" font-size="11" font-family="sans-serif">{value:.0f}</text>')
        seconds = max_time * step / 4
        x = time_to_x(seconds)
        elements.append(f'<path d="M{x:.1f} {top}V{bottom}" stroke="#eee"/>')
        elements.append(f'<text x="{x:.1f}" y="{bottom+18}" text-anchor="middle" font-size="11" font-family="sans-serif">{seconds:.0f}</text>')
    elements.append(f'<path d="M{left} {top}V{bottom}H{right}" fill="none" stroke="#333"/>')
    elements.append(f'<text x="{(left+right)/2:.1f}" y="{bottom+39}" text-anchor="middle" font-size="12" font-family="sans-serif">Simulated mission time (s)</text>')
    for policy, data in traces.items():
        points = [f'{time_to_x(float(row["simulation_time_ms"])/1000):.1f},{value_to_y(y_of(row)):.1f}'
                  for row in data]
        elements.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{colors[policy]}" stroke-width="2.5"/>')
        if mode == "coverage":
            observed_points = [f'{time_to_x(float(row["simulation_time_ms"])/1000):.1f},{value_to_y(float(row["reported_covered_m"])/.25):.1f}'
                               for row in data]
            elements.append(f'<polyline points="{" ".join(observed_points)}" fill="none" stroke="{colors[policy]}" stroke-width="2" stroke-dasharray="5 4"/>')
        else:
            repair = [row for row in data if int(row["recovery_attempts"]) > 0]
            if repair:
                reentry = min(repair, key=lambda row: float(row["truth_east_m"]))
                x = time_to_x(float(reentry["simulation_time_ms"]) / 1000)
                y = value_to_y(float(reentry["truth_east_m"]))
                elements.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="{colors[policy]}" stroke="white"/>')
    if mode == "trajectory":
        legend = [("B2 true path", "#2468b4", False), ("P true path", "#d27b28", False),
                  ("Re-entry point", "#555", True)]
    else:
        legend = [("B2 truth", "#2468b4", False), ("B2 accepted", "#2468b4", True),
                  ("P truth", "#d27b28", False), ("P accepted", "#d27b28", True)]
    for index, (label, color, dashed) in enumerate(legend):
        x = left + index * 205
        dash = ' stroke-dasharray="5 4"' if dashed and mode == "coverage" else ""
        if mode == "trajectory" and dashed:
            elements.append(f'<circle cx="{x+12}" cy="405" r="5" fill="{color}"/>')
        else:
            elements.append(f'<path d="M{x} 405H{x+25}" stroke="{color}" stroke-width="2.5"{dash}/>')
        elements.append(f'<text x="{x+32}" y="409" font-size="12" font-family="sans-serif">{label}</text>')
    path.write_text("\n".join(elements + ["</svg>"]) + "\n")


def detour_map_figure(path, traces, wind_zone):
    colors = {"B2": "#2468b4", "P": "#d27b28"}
    x_of = lambda east: 75 + 42 * east
    y_of = lambda north: 215 - 23 * north
    elements = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="440" viewBox="0 0 1000 440">',
                '<rect width="100%" height="100%" fill="white"/>',
                '<text x="35" y="32" font-size="20" font-family="sans-serif">S03 true trajectories under localized cross-track wind</text>',
                f'<rect x="75" y="{y_of(4):.1f}" width="840" height="{y_of(-4)-y_of(4):.1f}" fill="#e4f0e8" stroke="#7c9e86"/>',
                f'<rect x="{x_of(wind_zone[0]):.1f}" y="55" width="{42*(wind_zone[1]-wind_zone[0]):.1f}" height="320" fill="#f5dad5" fill-opacity=".6"/>',
                f'<path d="M75 {y_of(0)}H915" stroke="#8b958e" stroke-dasharray="6 5"/>',
                '<text x="78" y="386" font-size="12" font-family="sans-serif">Green: required ±4 m corridor. Red: scripted crosswind zone. A=0 m, B=20 m.</text>']
    for east in (0, 5, 10, 15, 20):
        elements.append(f'<text x="{x_of(east):.1f}" y="409" text-anchor="middle" font-size="11" font-family="sans-serif">{east}</text>')
    elements.append('<text x="480" y="427" text-anchor="middle" font-size="12" font-family="sans-serif">East along edge (m)</text>')
    for policy, data in traces.items():
        points = [f'{x_of(float(row["truth_east_m"])):.1f},{y_of(float(row["truth_north_m"])):.1f}'
                  for row in data]
        elements.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{colors[policy]}" stroke-width="2.3"/>')
        elements.append(f'<path d="M{580 if policy == "B2" else 705} 360h25" stroke="{colors[policy]}" stroke-width="2.5"/>')
        elements.append(f'<text x="{612 if policy == "B2" else 737}" y="364" font-size="12" font-family="sans-serif">{policy} true path</text>')
    path.write_text("\n".join(elements + ["</svg>"]) + "\n")


def analyze(output):
    analysis_dirty = subprocess.run(
        ["git", "status", "--porcelain", "--untracked-files=no"],
        capture_output=True, text=True, check=True).stdout.strip()
    if analysis_dirty:
        raise ValueError("analysis requires a clean tracked source revision")
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
            survey_n = sum(r["truth_survey_complete"] == "1" for r in subset)
            truth_n = sum(truth_edge(r) for r in subset)
            report_n = sum(r["reported_phase"] == "FULFILLED" for r in subset)
            false_n = sum(r["false_complete"] == "1" for r in subset)
            false_edge_n = sum(r["reported_phase"] == "FULFILLED" and
                               not truth_edge(r) for r in subset)
            tp_n = sum(truth_edge(r) and r["reported_phase"] == "FULFILLED"
                       for r in subset)
            distance = [float(r["physical_distance_m"]) for r in subset]
            seconds = [float(r["elapsed_simulation_ms"]) / 1000 for r in subset]
            low, high = wilson(false_n, n)
            true_low, true_high = wilson(truth_n, n)
            tp_low, tp_high = wilson(tp_n, n)
            completed_low, completed_high = wilson(report_n, n)
            distance_low, distance_high = paired_ci(
                distance, nboot, config["bootstrap_seed"] + sum(map(ord, sid + arm + "distance")))
            time_low, time_high = paired_ci(
                seconds, nboot, config["bootstrap_seed"] + sum(map(ord, sid + arm + "time")))
            method = {"scenario_id": sid, "policy": arm, "assigned_n": n,
                      "true_survey_n": survey_n,
                      "true_complete_n": truth_n, "reported_complete_n": report_n,
                      "true_positive_n": tp_n, "false_complete_n": false_n,
                      "false_edge_complete_n": false_edge_n,
                      "true_complete_rate": truth_n / n,
                      "true_complete_wilson_low": true_low,
                      "true_complete_wilson_high": true_high,
                      "true_positive_rate": tp_n / n,
                      "true_positive_wilson_low": tp_low,
                      "true_positive_wilson_high": tp_high,
                      "false_negative_n": truth_n - tp_n,
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
                      "time_iqr_s": quantile(seconds, .75) - quantile(seconds, .25),
                      "recoveries_mean": mean([int(r["recovery_attempts"]) for r in subset]),
                      "recovery_attempted_n": sum(int(r["recovery_attempts"]) > 0 for r in subset),
                      "recovery_true_positive_n": sum(int(r["recovery_attempts"]) > 0 and
                                                      truth_edge(r) and
                                                      r["reported_phase"] == "FULFILLED" for r in subset),
                      "truth_missing_fraction_mean": mean([float(r["truth_missing_fraction"]) for r in subset]),
                      "captures_acquired_mean": mean([int(r["capture_acquired"]) for r in subset])}
            methods.append(method)
            scenarios.append(dict(method))
        pairs = [(by_key[sid, str(seed), "B2"], by_key[sid, str(seed), "P"])
                 for seed in manifest["seeds"]]
        diffs = {}
        for key, parser in (("distance_m", lambda r: float(r["physical_distance_m"])),
                            ("time_s", lambda r: float(r["elapsed_simulation_ms"]) / 1000),
                            ("true_complete", lambda r: float(truth_edge(r))),
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
                        ("true_complete", lambda r: float(truth_edge(r))),
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
                        "true_survey_n": sum(r["truth_survey_complete"] == "1" for r in subset),
                        "true_complete_n": sum(truth_edge(r) for r in subset),
                        "reported_complete_n": fulfilled,
                        "true_positive_n": sum(truth_edge(r) and r["reported_phase"] == "FULFILLED" for r in subset),
                        "false_complete_n": false,
                        "false_edge_complete_n": sum(r["reported_phase"] == "FULFILLED" and
                                                      not truth_edge(r) for r in subset),
                        "true_complete_rate": sum(truth_edge(r) for r in subset) / len(subset),
                        "true_complete_wilson_low": "NA",
                        "true_complete_wilson_high": "NA",
                        "true_positive_rate": sum(truth_edge(r) and r["reported_phase"] == "FULFILLED" for r in subset) / len(subset),
                        "true_positive_wilson_low": "NA",
                        "true_positive_wilson_high": "NA",
                        "false_negative_n": sum(truth_edge(r) and r["reported_phase"] != "FULFILLED" for r in subset),
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
                        "time_iqr_s": quantile(seconds, .75) - quantile(seconds, .25),
                        "recoveries_mean": mean(int(r["recovery_attempts"]) for r in subset),
                        "recovery_attempted_n": sum(int(r["recovery_attempts"]) > 0 for r in subset),
                        "recovery_true_positive_n": sum(int(r["recovery_attempts"]) > 0 and
                                                        truth_edge(r) and
                                                        r["reported_phase"] == "FULFILLED" for r in subset),
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
            subset = [attempt for attempt in raw if attempt["scenario_id"] == row["scenario_id"]]
            row["attempts_n"] = len(subset)
            row["fault_realized_n"] = sum(attempt["requested_fault_realized"] == "1"
                                           for attempt in subset)
        elif any(case["scenario_id"] == row["scenario_id"] for case in diagnostic_rows):
            matching = [case for case in diagnostic_rows
                        if case["scenario_id"] == row["scenario_id"]]
            row["validation_status"] = ("EXECUTED_AGENT_DIAGNOSTIC" if all(
                case["fault_realized"] == "1" for case in matching)
                else "INCONCLUSIVE_FAULT_NOT_REALIZED")
            row["attempts_n"] = len(matching)
            row["fault_realized_n"] = sum(case["fault_realized"] == "1" for case in matching)
        elif any(case["scenario_id"] == row["scenario_id"] and case["exit_code"] == "0"
                 for case in regression_rows):
            row["validation_status"] = "REGRESSION_TEST_ONLY"
            row["attempts_n"] = 0
            row["fault_realized_n"] = "NA"
        else:
            row["attempts_n"] = 0
            row["fault_realized_n"] = "NA"
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
    def failure_class(row):
        if row["reported_phase"] == "INTERRUPTED" and row["fault"] == "motor_failure_on_repair":
            return "MOTOR"
        if row["reported_phase"] == "INTERRUPTED" and row["fault"] == "recovery_command_rejected":
            return "REJECTED"
        if row["reported_phase"] == "INTERRUPTED":
            return "INCOMPLETE"
        if row["reported_phase"] == "INFRASTRUCTURE_ERROR":
            return "INFRA"
        return row["reported_phase"]
    phases = ["FULFILLED", "INCOMPLETE", "MOTOR", "REJECTED", "UNKNOWN", "CANCELLED", "SAFETY", "INFRA"]
    svg_bars(output / "figures/failure_modes.svg", "Paired outcomes only; S12 safety is a separate diagnostic",
             phases, [(arm, [sum(r["policy"] == arm and failure_class(r) == phase
                                for r in raw) for phase in phases]) for arm in ("B2", "P")],
             "Assigned attempts")
    first_seed = manifest["seeds"][0]
    trace_paths = {
        (sid, arm): output / "raw/traces" /
        f"v4-{manifest['mode']}-{sid}-{first_seed:03d}-{arm}.csv"
        for sid in ("S03", "S04") for arm in ("B2", "P")}
    has_trace_figures = all(path.is_file() for path in trace_paths.values())
    if manifest["mode"] == "full" and not has_trace_figures:
        raise ValueError("full campaign is missing required example traces")
    if has_trace_figures:
        traces = {arm: rows(trace_paths["S04", arm]) for arm in ("B2", "P")}
        gap_row = by_key["S04", str(first_seed), "P"]
        gap = float(gap_row["gap_begin_m"]), float(gap_row["gap_end_m"])
        trajectory_figure(output / "figures/trajectory_good_bad.svg", traces,
                          "S04 physical paths under paired camera outage", "trajectory", gap)
        trajectory_figure(output / "figures/coverage_timeline.svg", traces,
                          "S04 independent true camera coverage through repair", "coverage")
        detour_traces = {arm: rows(trace_paths["S03", arm]) for arm in ("B2", "P")}
        detour_row = by_key["S03", str(first_seed), "P"]
        detour_map_figure(output / "figures/trajectory_detour.svg", detour_traces,
                          (float(detour_row["gap_begin_m"]), float(detour_row["gap_end_m"])))
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
             "Distance integrates true 20 ms physical steps. Time is simulated mission-clock duration. True edge completion requires both independent full-strip survey and true physical arrival at B. False survey completion is FULFILLED with at least one required truth cell missing; false edge completion additionally includes absence of true arrival. All assigned trials stay in denominators.",
             "", "## Results (actual values with uncertainty)", "",
             "| Scenario | n pairs | B2 true edge/reported/false survey | P true edge/reported/false survey | B2 mean m | P mean m | Δ P−B2 m (95% paired CI) | Δ time s (95% paired CI) |",
             "|---|---:|---|---|---:|---:|---|---|"]
    for comparison in scenario_paired:
        sid = comparison["scenario_id"]
        b = next(r for r in methods if r["scenario_id"] == sid and r["policy"] == "B2")
        p = next(r for r in methods if r["scenario_id"] == sid and r["policy"] == "P")
        counts = lambda r: f"{r['true_complete_n']}/{r['reported_complete_n']}/{r['false_complete_n']}"
        lines.append(f"| {sid} | {comparison['paired_seeds_n']} | {counts(b)} | {counts(p)} | {fmt(b['distance_mean_m'])} | {fmt(p['distance_mean_m'])} | {fmt(comparison['delta_distance_m_mean'])} [{fmt(comparison['delta_distance_m_ci_low'])}, {fmt(comparison['delta_distance_m_ci_high'])}] | {fmt(comparison['delta_time_s_mean'])} [{fmt(comparison['delta_time_s_ci_low'])}, {fmt(comparison['delta_time_s_ci_high'])}] |")
    b2_all = next(r for r in methods if r["scenario_id"] == "ALL_DESCRIPTIVE" and r["policy"] == "B2")
    relative_saving = -macro_diffs["delta_distance_m_mean"] / b2_all["distance_mean_m"]
    zero_event_upper = wilson(0, len(manifest["seeds"]))[1]
    lines += ["", "Each true-edge/reported/false-survey entry is a count of assigned attempts, not a conditional success-only mean. Wilson intervals, independent truth-survey counts and false-edge-completion counts are in `tables/methods_summary.csv`; all paired effect intervals are in `tables/paired_comparisons.csv`.",
              f"Both arms use the same evidence acceptance rules. They recorded B2 {b2_all['false_complete_n']}/{b2_all['assigned_n']} and P {sum(r['false_complete'] == '1' for r in raw if r['policy'] == 'P')}/{b2_all['assigned_n']} false survey completions in the comparative campaign. A zero-event scenario with {len(manifest['seeds'])} attempts still has a Wilson 95% upper bound of {fmt(100*zero_event_upper)}%; zero observed failures is not proof of zero risk or a verification-accuracy advantage for P.",
              f"Across the fixed 12-scenario set, the descriptive macro mean P−B2 distance is {fmt(macro_diffs['delta_distance_m_mean'])} m (95% seed-cluster bootstrap [{fmt(macro_diffs['delta_distance_m_ci_low'])}, {fmt(macro_diffs['delta_distance_m_ci_high'])}]); the corresponding relative travel saving is {fmt(100 * relative_saving)}% of B2 mean assigned-task distance. This includes failed tasks and is not a recovery-efficiency claim where completion differs.",
              "", "### Separate noncomparative diagnostics", "",
              "| Scenario | Variant | Reported | True survey | False complete | Fault realized |",
              "|---|---|---|---|---|---|"]
    for row in diagnostic_rows:
        lines.append(f"| {row['scenario_id']} | {row['variant']} | {row['reported_phase']} | {row['truth_survey_complete']} | {row['false_complete']} | {row['fault_realized']} |")
    negative_lines = []
    for sid in ("S02", "S08", "S09", "S16", "S19"):
        b = next(r for r in methods if r["scenario_id"] == sid and r["policy"] == "B2")
        p = next(r for r in methods if r["scenario_id"] == sid and r["policy"] == "P")
        contrast = next(r for r in scenario_paired if r["scenario_id"] == sid)
        negative_lines.append(
            f"- {sid}: true edge completion B2 {b['true_complete_n']}/{b['assigned_n']}, "
            f"P {p['true_complete_n']}/{p['assigned_n']}; reported fulfillment B2 "
            f"{b['reported_complete_n']}/{b['assigned_n']}, P "
            f"{p['reported_complete_n']}/{p['assigned_n']}; P−B2 mean travel "
            f"{fmt(contrast['delta_distance_m_mean'])} m and time "
            f"{fmt(contrast['delta_time_s_mean'])} s. "
            "Travel and time require the completion and safety context.")
    lines += ["", "These single diagnostic attempts are not probability estimates and are not pooled with the B2/P comparison.",
              "", "## Tradeoffs/negative results", "",
              *negative_lines,
              "", "Persistent faults can cause both policies to fail. A plausible dishonest camera can create false fulfillment, outside honest-adapter assumptions. The S08/S09 travel contrasts must be read beside their reported availability and should not be framed as pure efficiency benefits when completion differs.",
              "", "## Threats to validity", "",
              "The simulator has simplified horizontal acceleration, instant modeled camera acquisition, a one-dimensional full-strip coverage oracle and a terminal-sample true-arrival check; it has perfect base telemetry and no terrain occlusion. The paired trial sets sustained deviation to 1,000 samples to isolate post-arrival repair strategy, so it does not quantify early deviation-triggered re-entry. Location-tied interventions may have policy-dependent realized duration. Scenario seeds vary preregistered gap position/length and wind strength but do not sample a full field distribution. S15 recreates AgentService in one host process with a surviving simulator; an OS process power loss was not tested. No electrical energy or CPU/RSS benchmark is reported.",
              "", "## Reproducibility", "",
             f"Trial source revision `{manifest['source_revision']}`; analysis revision `{subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()}`; binary SHA-256 `{manifest['binary_sha256']}`; config SHA-256 `{manifest['config_id_sha256']}`. See manifest for host, commands and artifact hashes.",
              "", "## What is not demonstrated", "",
              "Only simulator evidence has been obtained. SITL, real camera calibration, and physical flights are not validated. Remote transport-loss replay, peer interruption, OS process crash/power loss and simultaneous multi-UAV throughput are not validated by this campaign.", ""]
    (output / "reports/SCIENTIFIC_RESULTS.md").write_text("\n".join(lines))
    p_all = next(r for r in methods if r["scenario_id"] == "ALL_DESCRIPTIVE" and r["policy"] == "P")
    (output / "reports/EXECUTIVE_RESULTS.md").write_text(
        f"# Executive results\n\n{len(raw)} actual B2/P agent-backed attempts across {len(expected)} scenarios and {len(manifest['seeds'])} paired seeds.\n\n"
        f"B2: true edge {b2_all['true_complete_n']}/{b2_all['assigned_n']}, reported fulfilled {b2_all['reported_complete_n']}/{b2_all['assigned_n']}, false survey completion {b2_all['false_complete_n']}/{b2_all['assigned_n']}. "
        f"P: true edge {p_all['true_complete_n']}/{p_all['assigned_n']}, reported fulfilled {p_all['reported_complete_n']}/{p_all['assigned_n']}, false survey completion {p_all['false_complete_n']}/{p_all['assigned_n']}.\n\n"
        f"Across the fixed scenario set, P−B2 mean assigned-task distance was {fmt(macro_diffs['delta_distance_m_mean'])} m (95% seed-cluster bootstrap [{fmt(macro_diffs['delta_distance_m_ci_low'])}, {fmt(macro_diffs['delta_distance_m_ci_high'])}]); relative travel reduction was {fmt(100*relative_saving)}%. This includes failures; read scenario completion counts before treating it as a benefit.\n\n"
        "The independent truth is a simplified along-edge full-strip model. S11's dishonest sensor variant produced a false completion. S13/S14/S17 remain unsupported. See `SCIENTIFIC_RESULTS.md` for measured tradeoffs and limits.\n")
    (output / "reports/LIMITATIONS_AND_NEGATIVE_RESULTS.md").write_text(
        "# Limitations and negative results\n\n" + "\n".join(negative_lines) +
        "\n\nThe S11 dishonest camera case can fool the verifier when authenticated metadata lies; the independent oracle detects the unobserved ground. S13/S14/S17 are unsupported in this campaign. S07 does not guarantee an intersample excursion, and S10 cannot isolate a brief passage. The paired trial uses a 1,000-sample sustained-deviation threshold to isolate post-arrival repair, so active mid-flight re-entry is not measured here. S15 recreates a service object, not an OS process crash. The private oracle is a 0.25 m along-edge raster with a full-strip camera-width condition, not a two-dimensional photo-quality oracle; true arrival is checked at the terminal sample, not through an independent dwell history. Failed/interrupted tasks remain in all denominators. Real sensing, SITL and physical flight are unvalidated.\n")
    artifact_hashes = {}
    for path in sorted(output.rglob("*")):
        if path.is_file() and path != manifest_path:
            artifact_hashes[str(path.relative_to(output))] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest["study_status"] = "ANALYZED"
    manifest["analysis_revision"] = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], text=True).strip()
    manifest["analysis_code_sha256"] = hashlib.sha256(
        pathlib.Path(__file__).read_bytes()).hexdigest()
    trial_source_paths = (
        "src/core/vecr.cpp", "src/agent/edge_execution_runtime.cpp",
        "src/agent/edge_attempt_journal.cpp", "src/agent/backends/sim_backend.cpp",
        "tests/vecr_agent_trial.cpp", "tests/vecr_agent_trial_main.cpp",
        "tests/vecr_truth_oracle.cpp", "scripts/vecr/run_agent_campaign_v4.py",
        "conanfile.py", "CMakePresets.json")
    manifest["source_file_sha256"] = {
        path: committed_sha(manifest["source_revision"], path)
        for path in trial_source_paths}
    manifest["dependency_lockfile"] = "none in repository; conanfile.py hash recorded"
    manifest["simulator_version"] = f"SimBackend at {manifest['source_revision']}"
    manifest["effective_runtime_constants"] = {
        "sustained_deviation_samples": 1000,
        "corridor_half_width_m": 4,
        "arrival_radius_m": 2,
        "edge_timeout_ms": 60000,
        "max_recovery_attempts": 1}
    manifest["analysis_correction"] = (
        "Post-raw correctness correction: macro true completion requires both private "
        "survey and arrival; held-out statistics do not invent absent trajectory traces. "
        "Raw data unchanged.")
    manifest["trace_figures_status"] = (
        "generated from saved traces" if has_trace_figures else
        "not generated: no saved example traces for these seeds")
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
