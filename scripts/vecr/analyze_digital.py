#!/usr/bin/env python3
"""Paired v7 analysis; observed data only, including failures and stronger baselines."""

import argparse, csv, json, hashlib
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

POLICIES = ["B2", "VECR"]
COMPARISONS = [("VECR", "B2")]


def read(p):
    with p.open() as f:
        return list(csv.DictReader(f))


def write(p, rows):
    with p.open("w") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def bootstrap(x, rng):
    if not len(x):
        return (None, None, None)
    vals = np.asarray(x, float)
    boot = vals[rng.integers(0, len(vals), (10000, len(vals)))].mean(axis=1)
    return (
        float(vals.mean()),
        float(np.quantile(boot, 0.025)),
        float(np.quantile(boot, 0.975)),
    )


def wilson(k, n):
    z = 1.959963984540054
    p = k / n
    den = 1 + z * z / n
    mid = (p + z * z / (2 * n)) / den
    half = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0, mid - half), min(1, mid + half)


def analyze(root):
    rows = read(root / "per_attempt.csv")
    assert rows and not any(r["infrastructure_error"] for r in rows)
    assert len({(r["scenario"], r["seed"], r["policy"]) for r in rows}) == len(rows)
    out = root / "analysis"
    out.mkdir(exist_ok=False)
    rng = np.random.default_rng(20261027)
    groups = {}
    for r in rows:
        groups.setdefault((r["scenario"], r["policy"]), []).append(r)
    table = []
    for (scenario, p), rr in sorted(groups.items()):
        n = len(rr)
        reported = sum(int(r["reported_complete"]) for r in rr)
        false = sum(int(r["false_complete"]) for r in rr)
        ci = wilson(reported, n)
        fci = wilson(false, n)
        table.append(
            dict(
                scenario=scenario,
                policy=p,
                n=n,
                fault=rr[0]["fault"],
                reported_complete=reported,
                true_complete=sum(int(r["truth_complete"]) for r in rr),
                false_complete=false,
                false_negative=sum(int(r["false_negative"]) for r in rr),
                reported_ci_low=ci[0],
                reported_ci_high=ci[1],
                false_ci_low=fci[0],
                false_ci_high=fci[1],
                mean_distance_m=float(np.mean([float(r["distance_m"]) for r in rr])),
                mean_time_s=float(np.mean([float(r["time_ms"]) / 1000 for r in rr])),
                mean_recoveries=float(np.mean([int(r["recoveries"]) for r in rr])),
            )
        )
    write(out / "condition_summary.csv", table)
    paired = []
    for scenario in sorted({r["scenario"] for r in rows}):
        for treatment, baseline in COMPARISONS:
            aa = {int(r["seed"]): r for r in groups[scenario, treatment]}
            bb = {int(r["seed"]): r for r in groups[scenario, baseline]}
            assert aa.keys() == bb.keys()
            seeds = sorted(aa)
            both = [
                s
                for s in seeds
                if all(
                    int(r["truth_complete"]) and int(r["reported_complete"])
                    for r in (aa[s], bb[s])
                )
            ]
            rec = dict(
                scenario=scenario,
                treatment=treatment,
                baseline=baseline,
                n_pairs=len(seeds),
                joint_success_pairs=len(both),
            )
            for metric, scale, selected, label in [
                ("reported_complete", 1, seeds, "acceptance_delta"),
                ("truth_complete", 1, seeds, "true_completion_delta"),
                ("distance_m", 1, both, "successful_distance_delta_m"),
                ("time_ms", 0.001, both, "successful_time_delta_s"),
            ]:
                mean, lo, hi = bootstrap(
                    [
                        (float(aa[s][metric]) - float(bb[s][metric])) * scale
                        for s in selected
                    ],
                    rng,
                )
                rec.update({label: mean, label + "_ci_low": lo, label + "_ci_high": hi})
            paired.append(rec)
    write(out / "paired_comparisons.csv", paired)
    scenarios = sorted({r["scenario"] for r in rows})
    plt.rcParams.update(
        {"font.family": "DejaVu Sans", "font.size": 10, "svg.hashsalt": "swarmkit-v7"}
    )
    lookup = {(r["scenario"], r["policy"]): r for r in table}
    fig, ax = plt.subplots(figsize=(10, 10))
    matrix = np.array(
        [
            [lookup[s, p]["reported_complete"] / lookup[s, p]["n"] for p in POLICIES]
            for s in scenarios
        ]
    )
    im = ax.imshow(matrix, vmin=0, vmax=1, cmap="YlGnBu", aspect="auto")
    for y, s in enumerate(scenarios):
        for x, p in enumerate(POLICIES):
            r = lookup[s, p]
            ax.text(
                x,
                y,
                f"{r['reported_complete']}/{r['n']}"
                + ("*" if r["false_complete"] else ""),
                ha="center",
                va="center",
                color="white" if matrix[y, x] > 0.65 else "#123",
                fontsize=8,
            )
    ax.set_xticks(range(len(POLICIES)), POLICIES)
    ax.set_yticks(
        range(len(scenarios)),
        [s + " " + lookup[s, "VECR"]["fault"].replace("_", " ") for s in scenarios],
    )
    ax.set_title(
        "Reported fulfillment by condition and policy\n*Includes false fulfillment: dishonest-input control"
    )
    fig.colorbar(im, ax=ax, label="Reported fraction")
    fig.tight_layout()
    for ext in ("svg", "pdf", "png"):
        fig.savefig(out / f"completion_matrix.{ext}", dpi=180)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 6))
    for ax, base in [(ax, "B2")]:
        data = [
            r
            for r in paired
            if r["treatment"] == "VECR"
            and r["baseline"] == base
            and r["joint_success_pairs"]
            and lookup[r["scenario"], "VECR"]["fault"]
            not in ("false_frame", "undeclared_angle")
        ]
        y = np.arange(len(data))
        means = np.array([r["successful_distance_delta_m"] for r in data])
        lo = np.array([r["successful_distance_delta_m_ci_low"] for r in data])
        hi = np.array([r["successful_distance_delta_m_ci_high"] for r in data])
        ax.errorbar(
            means, y, xerr=[means - lo, hi - means], fmt="o", color="#126b66", capsize=3
        )
        ax.axvline(0, color="#777", lw=1)
        ax.set_yticks(
            y, [f"{r['scenario']} (n={r['joint_success_pairs']})" for r in data]
        )
        ax.invert_yaxis()
        ax.set_xlabel("VECR minus baseline distance (m), paired 95% CI")
        ax.set_title(f"VECR versus {base}: jointly successful pairs")
    fig.tight_layout()
    for ext in ("svg", "pdf", "png"):
        fig.savefig(out / f"paired_distance.{ext}", dpi=180)
    plt.close(fig)
    summary = dict(
        protocol="v7",
        attempts=len(rows),
        conditions=len(scenarios),
        policies=POLICIES,
        source_revision=json.loads((root / "manifest.json").read_text())[
            "source_revision"
        ],
        input_sha256=hashlib.sha256(
            (root / "per_attempt.csv").read_bytes()
        ).hexdigest(),
        bootstrap_resamples=10000,
        rng_seed=20261027,
        by_policy={},
    )
    for p in POLICIES:
        rr = [
            r
            for r in rows
            if r["policy"] == p
            and r["fault"] not in ("false_frame", "undeclared_angle")
        ]
        summary["by_policy"][p] = {
            k: sum(int(r[k]) for r in rr)
            for k in [
                "truth_complete",
                "reported_complete",
                "false_complete",
                "false_negative",
            ]
        }
        summary["by_policy"][p]["honest_attempts"] = len(rr)
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    lines = [
        "# VECR v7 measured results",
        "",
        f"{len(rows):,} assigned attempts; {len(scenarios)} conditions; VECR and a full-restart research baseline.",
        "All unsuccessful attempts remain included. Honest-input and dishonest-input controls are separate.",
        "Cost differences below condition on both policies achieving true and reported completion.",
        "",
        "| Condition | Contrast | Successful pairs | Distance delta, m (95% paired CI) |",
        "|---|---|---:|---:|",
    ]
    for r in paired:
        val = r["successful_distance_delta_m"]
        value = (
            "Unavailable: no jointly successful pairs"
            if val is None
            else f"{val:.3f} [{r['successful_distance_delta_m_ci_low']:.3f}, {r['successful_distance_delta_m_ci_high']:.3f}]"
        )
        lines.append(
            f"| {r['scenario']} | {r['treatment']} − {r['baseline']} | {r['joint_success_pairs']} | {value} |"
        )
    (out / "RESULTS.md").write_text("\n".join(lines) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", type=Path, required=True)
    a = ap.parse_args()
    analyze(a.output)
