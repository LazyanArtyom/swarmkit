#!/usr/bin/env python3
"""Descriptive small-sample qualification report; no population claims or invented CI."""

import argparse, csv, json, statistics
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read(p):
    with p.open() as f:
        return list(csv.DictReader(f))


def main(root):
    validation = json.loads((root / "analysis/validation.json").read_text())
    assert (
        validation["passes"] == 12
        and not validation["errors"]
        and not validation["infrastructure_failures"]
    )
    rows = read(root / "analysis/reconstructed.csv")
    summary = []
    for nav, fault in [
        ("raw_mavlink", "none"),
        ("synthetic_bounded", "none"),
        ("synthetic_bounded", "delay"),
        ("synthetic_bounded", "multi_gap"),
        ("synthetic_bounded", "command_rejected"),
        ("synthetic_bounded", "land_override"),
    ]:
        for policy in ["VECR"]:
            rs = [
                r
                for r in rows
                if (r["navigation"], r["fault"], r["policy"]) == (nav, fault, policy)
            ]
            assert len(rs) == 2
            out = dict(
                navigation=nav,
                fault=fault,
                policy=policy,
                n=2,
                reported=sum(int(r["reported_complete"]) for r in rs),
                sampled_true=sum(int(r["sampled_truth_complete"]) for r in rs),
                false_reported=sum(
                    int(r["reported_complete"]) and not int(r["sampled_truth_complete"])
                    for r in rs
                ),
            )
            for metric in ["distance_m", "duration_s", "peak_horizontal_speed_mps"]:
                vs = [float(r[metric]) for r in rs]
                for label, fn in [
                    ("mean", statistics.mean),
                    ("min", min),
                    ("max", max),
                ]:
                    out[metric + "_" + label] = fn(vs)
            summary.append(out)
    with (root / "analysis/condition_summary.csv").open("w") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0]))
        w.writeheader()
        w.writerows(summary)
    lines = [
        "# ArduPilot SITL current: actual qualification results",
        "",
        "12/12 assignments returned records; all passed independent sampled-truth reconstruction.",
        "Two flights per cell; no broad reliability confidence interval is inferred.",
        "Native navigation accuracy remains backend-specific. Positive acceptance uses synthetic navigation and ideal camera observations.",
        "",
        "| Observation tier / fault | Policy | Reported / 2 | Sampled truth / 2 | Mean travel (m) | Mean duration (s) |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for r in summary:
        lines.append(
            f"| {r['navigation']} / {r['fault']} | {r['policy']} | {r['reported']} | {r['sampled_true']} | {r['distance_m_mean']:.3f} | {r['duration_s_mean']:.3f} |"
        )
    lines += [
        "",
        f"Accepted captures checked: {validation['accepted_captures']}; sampled re-entry dwells checked: {validation['sampled_reentry_dwells']}.",
        "",
        "Costs in failed/interrupting conditions describe actual execution; they are not efficiency gains.",
        "The <=50 ms upstream SIM_STATE age assumption is unproven because source acquisition time is absent. Stop dwell is sampled, not continuous. Camera geometry is ideal, flat and route-aligned; no actual imagery is validated.",
    ]
    (root / "analysis/RESULTS.md").write_text("\n".join(lines) + "\n")
    plt.rcParams.update(
        {"font.family": "DejaVu Sans", "font.size": 10, "svg.fonttype": "none"}
    )
    fig, axes = plt.subplots(
        2, 2, figsize=(11, 6), sharex="col", constrained_layout=True
    )
    for j, fault in enumerate(["delay", "multi_gap"]):
        for policy, color in [("VECR", "#217c68")]:
            r = next(
                r
                for r in rows
                if r["navigation"] == "synthetic_bounded"
                and r["fault"] == fault
                and r["policy"] == policy
                and r["seed"] == "73001"
            )
            d = root / r["run_id"]
            start = float(r["start_ms"])
            end = float(r["end_ms"])
            truth = [
                x for x in read(d / "truth.csv") if start <= float(x["time_ms"]) <= end
            ]
            trace = read(d / "agent_trace.csv")
            import math

            x = [(float(t["time_ms"]) - start) / 1000 for t in truth]
            east = [
                (float(t["lon_deg"]) - 44)
                * math.pi
                / 180
                * 6378137
                * math.cos(math.radians(40))
                for t in truth
            ]
            axes[0, j].plot(x, east, color=color, label=policy)
            axes[1, j].step(
                [(float(t["time_ms"]) - start) / 1000 for t in trace],
                [float(t["covered_m"]) for t in trace],
                where="post",
                color=color,
                label=policy,
            )
        axes[0, j].set_title(
            "Delayed metadata" if fault == "delay" else "Temporary multiple camera gaps"
        )
        axes[1, j].set_xlabel("Time from attempt dispatch (s)")
        for ax in axes[:, j]:
            ax.grid(alpha=0.2)
            ax.axhline(20, color="#999", lw=0.7, ls="--")
    axes[0, 0].set_ylabel("Independent simulator east (m)")
    axes[1, 0].set_ylabel("Agent accepted coverage (m)")
    axes[0, 0].legend()
    fig.suptitle(
        "ArduPilot SITL · actual seed 73001 traces\nSynthetic bounded navigation and ideal camera; no real-image claim",
        fontsize=12,
    )
    for ext in ["svg", "pdf", "png"]:
        fig.savefig(root / "analysis" / f"sitl_traces.{ext}", dpi=180)
    plt.close(fig)
    print("\n".join(lines))


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("output", type=Path)
    main(p.parse_args().output)
