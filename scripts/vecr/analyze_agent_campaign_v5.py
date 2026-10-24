#!/usr/bin/env python3
"""Deterministic paired v5 analysis. No filtering on direction or significance."""

import argparse, collections, csv, gzip, hashlib, json, math, subprocess
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from validate_agent_campaign_v5 import validate, read

NEGATIVE = {"false_frame", "undeclared_angle"}
LABELS = [
    "Nominal",
    "Short gust",
    "Sustained wind",
    "Early departure",
    "Visible strip departure",
    "Hidden strip departure",
    "Biased + delayed",
    "Missing metadata",
    "GNSS outage",
    "Invalid estimator",
    "Unbounded pose",
    "Stale captures",
    "Wrong calibration",
    "Declared yaw offset",
    "False frame success*",
    "Undeclared yaw offset*",
    "Command rejected",
    "Motor failure",
    "No recovery budget",
    "Low battery",
    "Sparse captures",
]
RNG = np.random.default_rng(20261024)


def write_csv(path, rows):
    with path.open("w") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else ["no_data"])
        w.writeheader()
        w.writerows(rows)


def ci(values):
    a = np.array(values, dtype=float)
    if not len(a):
        return (None, None, None)
    # Chunk bootstrap to keep memory bounded; random generator seed frozen above.
    samples = np.concatenate(
        [a[RNG.integers(0, len(a), (500, len(a)))].mean(axis=1) for _ in range(20)]
    )
    return (
        float(a.mean()),
        float(np.quantile(samples, 0.025)),
        float(np.quantile(samples, 0.975)),
    )


def wilson(n, total):
    if not total:
        return (None, None)
    z = 1.959963984540054
    p = n / total
    d = 1 + z * z / total
    c = (p + z * z / (2 * total)) / d
    h = z * math.sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / d
    return max(0, c - h), min(1, c + h)


def val(r, k):
    return float(r[k])


def stat(rows):
    n = len(rows)
    tp = sum(r["truth_complete"] == "1" and r["reported_complete"] == "1" for r in rows)
    fp = sum(int(r["false_complete"]) for r in rows)
    fn = sum(int(r["false_negative"]) for r in rows)
    report = sum(int(r["reported_complete"]) for r in rows)
    truth = sum(int(r["truth_complete"]) for r in rows)
    lo, hi = wilson(fp, n)
    rec_lo, rec_hi = wilson(tp, tp + fn)
    d = [val(r, "distance_m") for r in rows]
    t = [val(r, "time_ms") / 1000 for r in rows]
    return dict(
        n=n,
        true_survey=sum(int(r["truth_survey"]) for r in rows),
        true_complete=truth,
        reported=report,
        TP=tp,
        FP=fp,
        FN=fn,
        TN=n - tp - fp - fn,
        precision=tp / report if report else None,
        recall=tp / truth if truth else None,
        recall_ci_low=rec_lo,
        recall_ci_high=rec_hi,
        acceptance=report / n,
        false_complete_rate=fp / n,
        false_ci_low=lo,
        false_ci_high=hi,
        mean_distance_m=np.mean(d),
        p50_distance_m=np.median(d),
        p95_distance_m=np.quantile(d, 0.95),
        mean_time_s=np.mean(t),
        p50_time_s=np.median(t),
        p95_time_s=np.quantile(t, 0.95),
        accepted_uncontained=sum(int(r["accepted_uncontained"]) for r in rows),
        midflight_repairs=sum(int(r["midflight_repairs"]) for r in rows),
        interrupted=sum(r["phase"] == "INTERRUPTED" for r in rows),
        unknown_final=sum(r["phase"] == "UNKNOWN" for r in rows),
    )


def formatnum(x):
    return "NA" if x is None else f"{x:.3f}"


def save(fig, folder, name):
    fig.savefig(folder / (name + ".svg"), bbox_inches="tight", metadata={"Date": None})
    fig.savefig(
        folder / (name + ".pdf"),
        bbox_inches="tight",
        metadata={"CreationDate": None, "ModDate": None},
    )
    fig.savefig(folder / (name + ".png"), dpi=170, bbox_inches="tight")
    plt.close(fig)


def hash_all(out):
    paths = sorted(p for p in out.rglob("*") if p.is_file() and p.name != "SHA256SUMS")
    (out / "SHA256SUMS").write_text(
        "".join(
            hashlib.sha256(p.read_bytes()).hexdigest()
            + "  "
            + p.relative_to(out).as_posix()
            + "\n"
            for p in paths
        )
    )


def main(out, benchmark):
    gates = validate(out, False)
    rows = read(out / "per_attempt.csv")
    manifest = json.loads((out / "manifest.json").read_text())
    for folder in ["tables", "figures", "reports"]:
        (out / folder).mkdir(exist_ok=True)
    tables = out / "tables"
    matrix = {}
    for assignment in manifest["assignments"]:
        scenario = assignment["scenario"]
        if scenario not in matrix:
            matrix[scenario] = {
                k: v for k, v in assignment.items() if k not in ("seed", "policy")
            }
            matrix[scenario]["label"] = (
                LABELS[int(scenario[1:])]
                if scenario.startswith("M")
                else "Sensitivity: " + assignment["factor"]
            )
            matrix[scenario]["paired_seeds"] = len(
                {
                    a["seed"]
                    for a in manifest["assignments"]
                    if a["scenario"] == scenario
                }
            )
    matrix_columns = sorted(set().union(*(r.keys() for r in matrix.values())))
    write_csv(
        tables / "scenario_matrix.csv",
        [
            {k: row.get(k, "") for k in matrix_columns}
            for _, row in sorted(matrix.items())
        ],
    )
    figs = out / "figures"
    groups = collections.defaultdict(list)
    for r in rows:
        groups[r["scenario"], r["policy"]].append(r)
    stats = [
        dict(scenario=s, policy=p, **stat(g)) for (s, p), g in sorted(groups.items())
    ]
    write_csv(tables / "per_scenario.csv", stats)
    lookup = {(r["scenario"], r["policy"]): r for r in stats}
    paired = []
    conditional = []
    for scenario in sorted({r["scenario"] for r in rows}):
        left = {r["seed"]: r for r in groups[scenario, "B2"]}
        right = {r["seed"]: r for r in groups[scenario, "P"]}
        assert left.keys() == right.keys()
        for population in ["all_assigned", "both_true_and_reported"]:
            pairs = [
                (left[s], right[s])
                for s in sorted(left, key=int)
                if population == "all_assigned"
                or all(
                    r["reported_complete"] == "1" and r["truth_complete"] == "1"
                    for r in (left[s], right[s])
                )
            ]
            for metric, scale in [
                ("distance_m", 1),
                ("time_ms", 0.001),
                ("reported_complete", 1),
                ("truth_complete", 1),
                ("false_complete", 1),
            ]:
                mean, low, high = ci(
                    [(val(p, metric) - val(b, metric)) * scale for b, p in pairs]
                )
                entry = dict(
                    scenario=scenario,
                    population=population,
                    metric="time_s" if metric == "time_ms" else metric,
                    pairs=len(pairs),
                    B2_mean=(
                        np.mean([val(b, metric) * scale for b, p in pairs])
                        if pairs
                        else None
                    ),
                    P_mean=(
                        np.mean([val(p, metric) * scale for b, p in pairs])
                        if pairs
                        else None
                    ),
                    delta_P_minus_B2=mean,
                    ci_low=low,
                    ci_high=high,
                )
                paired.append(entry)
                if population == "both_true_and_reported" and metric == "distance_m":
                    conditional.append(entry)
    write_csv(tables / "paired_effects.csv", paired)
    # All missed completions and all rejection reasons; no seed/sample deletion.
    failures = []
    rejections = collections.Counter()
    raster = []
    for r in rows:
        if r["reported_complete"] != "1" or r["false_complete"] == "1":
            failures.append(
                {
                    k: r[k]
                    for k in [
                        "run_id",
                        "scenario",
                        "seed",
                        "policy",
                        "fault",
                        "phase",
                        "reason",
                        "truth_survey",
                        "truth_at_b",
                        "truth_complete",
                        "false_complete",
                        "false_negative",
                        "missing",
                        "recoveries",
                        "time_ms",
                    ]
                }
            )
        captures = read(out / "trials" / r["run_id"] / "captures.csv.gz")
        for c in captures:
            if c["accepted"] != "1":
                rejections[r["scenario"], r["policy"], c["submitted"], c["reason"]] += 1
        for cell in [0.1, 0.25, 0.5, 1]:
            raster.append(
                dict(
                    run_id=r["run_id"],
                    scenario=r["scenario"],
                    policy=r["policy"],
                    cell_m=cell,
                    exact_complete=r["truth_survey"],
                    full_cell_fraction=r[f"raster_full_{cell}"],
                    center_fraction=r[f"raster_center_{cell}"],
                    center_false_complete=int(
                        float(r[f"raster_center_{cell}"]) == 1
                        and r["truth_survey"] != "1"
                    ),
                )
            )
    write_csv(tables / "unsuccessful_attempts.csv", failures)
    write_csv(
        tables / "capture_rejections.csv",
        [
            dict(scenario=k[0], policy=k[1], submitted=k[2], reason=k[3], count=v)
            for k, v in sorted(rejections.items())
        ],
    )
    write_csv(tables / "raster_sensitivity.csv", raster)
    primary = [
        r for r in rows if r["scenario"].startswith("M") and r["fault"] not in NEGATIVE
    ]
    aggregates = []
    for population, data in [
        ("honest_primary", primary),
        ("negative_controls", [r for r in rows if r["fault"] in NEGATIVE]),
        ("sensitivity", [r for r in rows if r["scenario"].startswith("X")]),
    ]:
        for p in ["B2", "P"]:
            aggregates.append(
                dict(
                    population=population,
                    policy=p,
                    **stat([r for r in data if r["policy"] == p]),
                )
            )
    write_csv(tables / "aggregate_confusion.csv", aggregates)
    plt.rcParams.update({"font.size": 9, "svg.hashsalt": "vecr-v5", "figure.dpi": 100})
    # Availability on all assigned attempts, with conditional recall separately tabulated.
    fig, ax = plt.subplots(figsize=(11, 7))
    y = np.arange(21)
    for p, offset, color in [("B2", -0.18, "#687987"), ("P", 0.18, "#087e8b")]:
        ax.barh(
            y + offset,
            [lookup[f"M{i:02d}", p]["acceptance"] for i in range(21)],
            height=0.34,
            label=p,
            color=color,
        )
    ax.set(
        yticks=y,
        yticklabels=[f"M{i:02d} {LABELS[i]}" for i in range(21)],
        xlabel="Reported fulfillment / all assigned attempts",
        xlim=(0, 1.05),
    )
    ax.invert_yaxis()
    ax.legend()
    ax.set_title("Agent-backed v5 availability (* deliberately dishonest sensor)")
    save(fig, figs, "01_availability")
    fig, ax = plt.subplots(figsize=(10, 6))
    chosen = [x for x in conditional if x["pairs"] and x["scenario"].startswith("M")]
    if chosen:
        means = [x["delta_P_minus_B2"] for x in chosen]
        ax.errorbar(
            means,
            np.arange(len(chosen)),
            xerr=[
                [m - x["ci_low"] for x, m in zip(chosen, means)],
                [x["ci_high"] - m for x, m in zip(chosen, means)],
            ],
            fmt="o",
            color="#087e8b",
            capsize=3,
        )
        ax.set(
            yticks=np.arange(len(chosen)),
            yticklabels=[f"{x['scenario']} (n={x['pairs']} pairs)" for x in chosen],
        )
    ax.axvline(0, color="gray", lw=1)
    ax.set_xlabel("VECR − B2 distance (m), paired bootstrap 95% CI")
    ax.set_title(
        "Conditional cost: both policies truly complete and report fulfillment"
    )
    save(fig, figs, "02_matched_success_distance")
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for ax, s in zip(axes, ["M03", "M02"]):
        for p, color in [("B2", "#687987"), ("P", "#087e8b")]:
            r = next(
                x
                for x in rows
                if x["scenario"] == s
                and x["policy"] == p
                and int(x["seed"]) == min(int(r["seed"]) for r in rows)
            )
            t = read(out / "trials" / r["run_id"] / "trajectory.csv.gz")
            ax.plot(
                [val(x, "east_m") for x in t],
                [val(x, "north_m") for x in t],
                label=p,
                color=color,
            )
            events = read(out / "trials" / r["run_id"] / "events.csv.gz")
            go = [e for e in events if e["kind"] == "goto_accepted"]
            ax.scatter(
                [val(e, "target_east") for e in go],
                [val(e, "target_north") for e in go],
                marker="x",
                color=color,
            )
        ax.axhspan(-4, 4, alpha=0.1, color="gray")
        ax.set(
            xlabel="East (m)", ylabel="North (m)", title=s + " " + LABELS[int(s[1:])]
        )
        ax.legend()
    save(fig, figs, "03_positive_negative_trajectories")
    fig, ax = plt.subplots(figsize=(10, 4))
    sens = [x for x in conditional if x["scenario"].startswith("X")]
    means = [x["delta_P_minus_B2"] if x["pairs"] else np.nan for x in sens]
    ax.bar([x["scenario"] for x in sens], means, color="#087e8b")
    ax.axhline(0, color="gray")
    ax.set(
        ylabel="Conditional VECR − B2 distance (m)",
        xlabel="Sensitivity condition; blank = no matched successful pairs",
        title="Sensitivity: exact levels in manifest assignments",
    )
    save(fig, figs, "04_sensitivity")
    fig, ax = plt.subplots(figsize=(8, 4))
    cells = [0.1, 0.25, 0.5, 1]
    ax.bar(
        [str(x) for x in cells],
        [
            sum(r["center_false_complete"] for r in raster if r["cell_m"] == x)
            for x in cells
        ],
        color="#b34b32",
    )
    ax.set(
        xlabel="Cell width (m)",
        ylabel="Cell-center complete but exact strip incomplete",
        title="Raster discretization errors across all assigned attempts",
    )
    save(fig, figs, "05_raster_sensitivity")
    # Time and reported coverage traces show costs of persistent failures.
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for s in ["M05", "M08"]:
        for p, style in [("B2", "--"), ("P", "-")]:
            r = next(
                x
                for x in rows
                if x["scenario"] == s
                and x["policy"] == p
                and int(x["seed"]) == min(int(r["seed"]) for r in rows)
            )
            ts = read(out / "trials" / r["run_id"] / "trace.csv.gz")
            x = [val(t, "time_ms") / 1000 for t in ts]
            axes[0].plot(
                x, [val(t, "truth_covered_m") for t in ts], style, label=s + " " + p
            )
            axes[1].plot(
                x, [val(t, "reported_covered_m") for t in ts], style, label=s + " " + p
            )
    axes[0].set_ylabel("Actual full-strip coverage (m)")
    axes[1].set(ylabel="Accepted coverage (m)", xlabel="Simulation time (s)")
    axes[0].legend(ncol=4)
    save(fig, figs, "06_truth_and_verified_coverage")
    perf = []
    if benchmark:
        benchmark_manifest = json.loads((benchmark / "manifest.json").read_text())
        assert (
            benchmark_manifest["binary_sha256"] == manifest["binary_sha256"]
        ), "benchmark must use the same executable"
        assert benchmark_manifest["workers"] == 1, "benchmark must run serially"
        validate(benchmark, False)
        brows = read(benchmark / "per_attempt.csv")
        for r in brows:
            perf.append(
                dict(
                    run_id=r["run_id"],
                    scenario=r["scenario"],
                    policy=r["policy"],
                    cpu_s=r["cpu_s"],
                    peak_rss_MiB=val(r, "peak_rss_bytes") / 1048576,
                    wall_s=r["wall_s"],
                    process_wall_s=r["process_wall_s"],
                    captures_per_wall_s=val(r, "submitted") / val(r, "wall_s"),
                    reports_per_wall_s=val(r, "reports") / val(r, "wall_s"),
                    capture_latency_p50_us=r["capture_latency_p50_us"],
                    capture_latency_p95_us=r["capture_latency_p95_us"],
                )
            )
        write_csv(tables / "isolated_host_performance.csv", perf)
        fig, axes = plt.subplots(1, 3, figsize=(11, 4))
        for ax, key, title in zip(
            axes,
            ["cpu_s", "peak_rss_MiB", "capture_latency_p95_us"],
            ["Process CPU seconds", "Peak process RSS (MiB)", "Capture RPC p95 (µs)"],
        ):
            ax.boxplot(
                [[val(r, key) for r in perf if r["policy"] == p] for p in ["B2", "P"]],
                tick_labels=["B2", "P"],
            )
            ax.set_title(title)
        fig.suptitle(
            "Serial host benchmark: SDK + agent + backend + trace writer, not embedded hardware"
        )
        save(fig, figs, "07_host_performance")
    primary_pairs = [
        x
        for x in paired
        if x["scenario"] in ("M03", "M04", "M05")
        and x["population"] == "both_true_and_reported"
        and x["metric"] == "distance_m"
    ]
    aggregate_by = {(x["population"], x["policy"]): x for x in aggregates}
    lines = [
        "# VECR v5 digital-simulation results",
        "",
        f"Source revision: `{manifest['source_revision']}`. Binary SHA-256: `{manifest['binary_sha256']}`.",
        f"Executed **{len(rows)}** assigned attempts: {len({r['scenario'] for r in rows if r['scenario'].startswith('M')})} primary scenarios and {len({r['scenario'] for r in rows if r['scenario'].startswith('X')})} sensitivity conditions; exact paired seed counts are in the manifest and per-scenario table. Every attempted run remains in the denominator. Evidence: SDK → local mTLS AgentService → production runtime → SimBackend. No SITL or hardware trials.",
        "",
        "## Primary honest-sensor outcomes",
        "",
        "| Policy | N | True survey | True survey + stopped B | Reported | False completion | True complete but unaccepted | Recall |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for p in ["B2", "P"]:
        x = aggregate_by["honest_primary", p]
        lines.append(
            f"| {p} | {x['n']} | {x['true_survey']} | {x['true_complete']} | {x['reported']} | {x['FP']} | {x['FN']} | {x['recall']:.3%} |"
        )
    lines += [
        "",
        "The honest primary aggregate excludes only the two preregistered dishonest-sensor controls (M14/M15). Missing bounds, stale captures, safety failures and timeouts remain included. Zero observed false completions is conditional on sensor assumptions and the modeled strip; it is not proof of zero risk or a verification advantage.",
        "",
        "## Midflight comparison",
        "",
        "| Scenario | Matched successful pairs | B2 mean m | VECR mean m | VECR − B2 m [95% CI] |",
        "|---|---:|---:|---:|---|",
    ]
    for x in primary_pairs:
        lines.append(
            f"| {x['scenario']} | {x['pairs']} | {x['B2_mean']:.3f} | {x['P_mean']:.3f} | {x['delta_P_minus_B2']:.3f} [{x['ci_low']:.3f}, {x['ci_high']:.3f}] |"
        )
    lines += [
        "",
        "These costs are conditional on both methods actually completing and reporting fulfillment. Full per-scenario results, all-assigned costs, p50/p95 and paired intervals are in `tables/`. B2 and VECR share detection, evidence, dynamics, faults, cooldown, safety and timeout; only the re-entry policy differs. B2 restarts at A; VECR re-enters before the earliest unverified gap, capped at current progress during departure. Off-corridor motion can still observe a wide strip; neither algorithm discards those accepted images.",
        "",
        "## Failures and uncertainty",
        "",
    ]
    for s in [
        "M02",
        "M08",
        "M09",
        "M10",
        "M11",
        "M12",
        "M13",
        "M14",
        "M15",
        "M16",
        "M17",
        "M18",
        "M19",
        "M20",
    ]:
        b, p = lookup[s, "B2"], lookup[s, "P"]
        lines.append(
            f"* {s} {LABELS[int(s[1:])]}: B2/P reported {b['reported']}/{p['reported']}; true survey {b['true_survey']}/{p['true_survey']}; true survey + stopped B {b['true_complete']}/{p['true_complete']}; false reports {b['FP']}/{p['FP']}."
        )
    lines += [
        "",
        "Every unsuccessful attempt is listed with final reason, missing intervals and repair count. Every acquisition and metadata rejection is preserved. Repeated GNSS/frame invalidity prevents the estimator from generating bounded camera poses in the fault region; neither extra motion nor a command ACK restores that evidence. A shared 90 s deadline may stop B2 during its second full restart while VECR has already returned to B. These are policy costs under a fixed time budget, not infrastructure errors. Camera data that arrive after termination remain explicitly pending.",
        "",
        "Confidence intervals use 10,000 paired seed bootstrap resamples, fixed RNG 20261024; binomial intervals use Wilson 95%. Conditional comparisons select pairs successful under both policies and can conceal differences in availability; the all-assigned table must accompany them. Seeds vary bounded pose/velocity/altitude errors and calibrated optical width. They do not represent real airframes, terrains or weather distributions. No multiple-comparison significance claims are made.",
        "",
        "## Oracle and sensor limits",
        "",
        "The camera estimator receives noisy telemetry and fixed calibration; it cannot access private simulator poses. Metadata carry acquisition timestamps, nonzero hard bounds and a conservative optical envelope. Independent private truth uses actual acquisition geometry and physical 20 ms trajectories. A second Python implementation reconstructs geometry from rectangle half-space inequalities and verifies every accepted footprint, final truth outcome and completed physical re-entry dwell.",
        "",
        "The oracle is a one-dimensional union of complete strip cross-sections under individual rectangular nadir footprints. It does not mosaic partial-width views, model terrain, real pixels, pitch/roll or image-quality classification. The angle controls use rotated ground rectangles (yaw), not a calibrated real camera. Cell-center raster estimates can conceal subcell holes; exact union is primary. Geometric simulation v1/v2, agent-backed v3/v4.2 and v5 remain separate evidence levels.",
        "",
        "## Performance",
        "",
    ]
    if perf:
        for key in ["cpu_s", "peak_rss_MiB", "capture_latency_p95_us"]:
            a = [val(r, key) for r in perf]
            lines.append(
                f"* {key}: median {np.median(a):.3f}; p95 {np.quantile(a,.95):.3f}; {len(a)} isolated serial processes."
            )
        lines.append(
            "\nCPU/RSS cover the whole in-process SDK + agent + backend + trace writer, including startup for CPU/RSS. Capture latency covers the mTLS capture RPC. Wall-time rates measure accelerated host execution, not radio/network capacity. This is not an agent-only profiler, Raspberry Pi measurement, energy result or hard real-time bound. Benchmark assignments and source/binary hashes are stored separately."
        )
    lines += [
        "",
        "## Validation and remaining scope",
        "",
        f"Independent reconstruction passed for {gates['attempts']} attempts, {gates['delivered_captures']} delivered captures and {gates['physical_reentries']} physical re-entry dwells. Release/UBSan/install gates are recorded separately in `docs/vecr/FINAL_VALIDATION_V5.md`.",
        "",
        "The original v4.2 300 matched localized-repair pairs and approximately 21.5% distance reduction remain frozen at their original revisions. The v5 availability inventory enumerates all 330 v4.2 missed completions; old logs cannot retrospectively identify every missing rejection reason.",
        "",
        "Remaining research: live calibrated camera evidence, obstacles/geofences and realistic aerodynamics, remote replay/peer transport/multi-UAV experiments, process/power-loss recovery and agent-only resource profiling. No ArduPilot SITL, hardware, rotor-router or gossip experiment was run.",
    ]
    (out / "reports" / "SCIENTIFIC_RESULTS.md").write_text("\n".join(lines) + "\n")
    (out / "analysis_manifest.json").write_text(
        json.dumps(
            dict(
                source_revision=subprocess.check_output(
                    ["git", "rev-parse", "HEAD"], text=True
                ).strip(),
                analysis_script_sha256=hashlib.sha256(
                    Path(__file__).read_bytes()
                ).hexdigest(),
                validator_sha256=hashlib.sha256(
                    Path(__file__)
                    .with_name("validate_agent_campaign_v5.py")
                    .read_bytes()
                ).hexdigest(),
                numpy=np.__version__,
                matplotlib=matplotlib.__version__,
                bootstrap_resamples=10000,
                bootstrap_seed=20261024,
                gates=gates,
            ),
            indent=2,
        )
        + "\n"
    )
    hash_all(out)
    validate(out)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--benchmark", type=Path)
    a = ap.parse_args()
    main(a.output, a.benchmark)
