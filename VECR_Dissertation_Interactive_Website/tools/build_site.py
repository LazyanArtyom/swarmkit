#!/usr/bin/env python3
"""Build an offline dissertation reader from frozen evidence; never run experiments."""

import csv
import gzip
import hashlib
import html
import json
import re
import shutil
import statistics
import textwrap
import zipfile
from pathlib import Path

import markdown

SITE = Path(__file__).resolve().parents[1]
ROOT = SITE.parent
STUDY = ROOT / "results/vecr/agent_campaign_v5"
FINAL = STUDY / "corrected_endpoints_v5_1"
ASSETS = SITE / "assets"
DOWNLOADS = ASSETS / "evidence"
esc = html.escape


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    with gzip.open(path, "rt") if path.suffix == ".gz" else path.open() as f:
        return list(csv.DictReader(f))


def table(headers, body, caption=""):
    return (
        '<div class="table-scroll" tabindex="0" role="region" aria-label="'
        + esc(caption or "Data table")
        + '"><table>'
        + (f"<caption>{esc(caption)}</caption>" if caption else "")
        + "<thead><tr>"
        + "".join(f'<th scope="col">{esc(x)}</th>' for x in headers)
        + "</tr></thead><tbody>"
        + "".join("<tr>" + "".join(f"<td>{x}</td>" for x in r) + "</tr>" for r in body)
        + "</tbody></table></div>"
    )


def num(x, digits=3):
    return "—" if x in ("", None) else f"{float(x):,.{digits}f}"


def image_figure(name, caption, folder="figures"):
    return f'<figure class="research-figure"><button class="figure-open" data-image="assets/{folder}/{name}.svg" data-caption="{esc(caption)}" aria-label="Enlarge: {esc(caption)}"><img loading="lazy" src="assets/{folder}/{name}.svg" alt="{esc(caption)}"><span class="enlarge">Enlarge ↗</span></button><figcaption>{esc(caption)}</figcaption></figure>'


def svg(name, title, desc, body, height=360):
    p = ASSETS / "diagrams" / f"{name}.svg"
    p.parent.mkdir(exist_ok=True)
    p.write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 880 {height}" role="img" aria-labelledby="title desc"><title id="title">{esc(title)}</title><desc id="desc">{esc(desc)}</desc><defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0 0L8 4L0 8" fill="none" stroke="#57716d" stroke-width="1.4"/></marker><pattern id="grid" width="24" height="24" patternUnits="userSpaceOnUse"><path d="M24 0H0V24" fill="none" stroke="#e8eeea" stroke-width=".6"/></pattern></defs><rect width="880" height="{height}" rx="16" fill="#f8faf7"/><rect width="880" height="{height}" fill="url(#grid)"/><g font-family="Arial,sans-serif" fill="#183c36">{body}</g></svg>"""
    )


def box(x, y, w, h, title, subtitle="", color="#e3eee7"):
    return (
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="{color}" stroke="#afc7bc"/><text x="{x+w/2}" y="{y+31}" text-anchor="middle" font-size="17" font-weight="600">{esc(title)}</text>'
        + (
            f'<text x="{x+w/2}" y="{y+55}" text-anchor="middle" font-size="13" fill="#4c6660">{esc(subtitle)}</text>'
            if subtitle
            else ""
        )
    )


def arrow(x1, y1, x2, y2, dashed=False):
    return (
        f'<path d="M{x1} {y1} L{x2} {y2}" fill="none" stroke="#57716d" stroke-width="1.8" marker-end="url(#arrow)"'
        + (' stroke-dasharray="5 5"' if dashed else "")
        + "/>"
    )


def diagrams():
    b = '<text x="32" y="34" font-size="14" letter-spacing="2">DISTINCT EVIDENCE · DISTINCT MEANINGS</text>'
    for x, title, sub in [
        (32, "Command ACK", "The backend accepted a request"),
        (248, "Stopped arrival", "Bounded pose + speed + dwell"),
        (464, "Survey evidence", "Accepted full-strip intervals"),
        (680, "Fulfillment", "The requested predicates hold"),
    ]:
        b += box(x, 65, 168, 100, title, color="#fff" if x < 680 else "#d7ece1")
        words = sub.split(" + ") if " + " in sub else textwrap.wrap(sub, width=23)
        for i, line in enumerate(words):
            b += f'<text x="{x+84}" y="{123+i*17}" text-anchor="middle" font-size="12">{esc(line)}</text>'
    b += '<text x="440" y="214" text-anchor="middle" font-size="16">ACK does not establish arrival. Arrival does not establish survey coverage.</text>'
    b += '<path d="M74 290H806" stroke="#9aaea4" stroke-width="12"/><path d="M74 290H360 M510 290H806" stroke="#237966" stroke-width="12"/><path d="M360 290H510" stroke="#c47735" stroke-width="12" stroke-dasharray="4 6"/><text x="74" y="323" font-size="14">A</text><text x="805" y="323" font-size="14">B</text><text x="435" y="270" text-anchor="middle" font-size="13">Unverified interval remains</text>'
    svg(
        "contract",
        "Arrival and survey fulfillment",
        "Conceptual distinctions, not experimental measurements.",
        b,
    )
    b = box(
        260,
        25,
        360,
        72,
        "External mission application",
        "Owns graph planning and coordination",
    ) + arrow(440, 97, 440, 137)
    b += '<rect x="220" y="139" width="440" height="172" rx="16" fill="#e3eee7" stroke="#237966" stroke-width="2"/><text x="440" y="169" text-anchor="middle" font-size="19" font-weight="600">SwarmKit agent · local decisions</text>'
    b += box(
        240,
        190,
        190,
        90,
        "Evidence + identity",
        "Coverage union / missing work",
        "#fff",
    ) + box(
        450, 190, 190, 90, "Recovery + safety", "Arbitrated physical commands", "#fff"
    )
    b += box(20, 190, 170, 90, "Sensor adapter", "Measured metadata", "#fff") + arrow(
        190, 235, 220, 235
    )
    b += box(
        690, 190, 170, 90, "Client / server", "Reports and telemetry", "#fff"
    ) + arrow(660, 235, 690, 235)
    b += arrow(440, 311, 440, 350) + box(
        260,
        352,
        360,
        72,
        "SimBackend in this experiment",
        "MAVLink backend exists; SITL unexecuted",
    )
    svg(
        "architecture",
        "Implemented local execution architecture",
        "Deviation detection, verification and recovery run inside the agent.",
        b,
        450,
    )
    b = (
        box(30, 30, 220, 74, "MOVING", "Consume navigation + captures")
        + arrow(250, 67, 330, 67)
        + box(332, 30, 220, 74, "DEVIATING", "Conformance trigger accumulates")
        + arrow(552, 67, 630, 67)
        + box(632, 30, 220, 74, "RECOVERING", "Command ACK ≠ physical return")
    )
    b += '<path d="M742 104V154H140V105" stroke="#57716d" stroke-width="1.8" fill="none" marker-end="url(#arrow)"/><text x="440" y="143" text-anchor="middle" font-size="13">Bounded re-entry dwell + accepted continuation → MOVING</text>'
    b += (
        box(30, 210, 220, 74, "FULFILLED", "Requested predicates satisfied")
        + box(332, 210, 220, 74, "UNKNOWN", "Usable evidence unavailable", "#fff1dd")
        + box(632, 210, 220, 74, "INTERRUPTED", "Safety, refusal, timeout", "#f8e6df")
    )
    b += '<text x="440" y="330" text-anchor="middle" font-size="14">Terminal reports retain outcomes. Invalid evidence or safety can interrupt the active sequence.</text>'
    svg(
        "states",
        "Canonical execution phases",
        "Simplified state diagram; transition guards are described in the manuscript.",
        b,
        355,
    )
    b = '<text x="32" y="34" font-size="14" letter-spacing="2">SHARED VERIFIER, DYNAMICS, SENSOR, SAFETY AND DEADLINE</text>'
    for y, label, target, color in [
        (126, "B2 · full-edge restart", 80, "#b77133"),
        (254, "VECR · targeted gap", 328, "#167d6a"),
    ]:
        b += f'<text x="32" y="{y-60}" font-size="18" font-weight="600">{label}</text><path d="M80 {y}H804" stroke="#cdd9d1" stroke-width="9"/><path d="M80 {y}H360 M448 {y}H804" stroke="#97c3b2" stroke-width="9"/><path d="M360 {y}H448" stroke="#c47735" stroke-width="9" stroke-dasharray="5 5"/><path d="M560 {y-18} Q440 {y-50} {target} {y-18}" fill="none" stroke="{color}" stroke-width="3" marker-end="url(#arrow)"/><circle cx="{target}" cy="{y}" r="6" fill="{color}"/><text x="80" y="{y+30}" text-anchor="middle" font-size="13">A</text><text x="804" y="{y+30}" text-anchor="middle" font-size="13">B</text>'
    b += '<text x="440" y="325" text-anchor="middle" font-size="13">Schematic only · VECR includes overlap and caps midflight re-entry at current progress.</text>'
    svg(
        "comparison",
        "Recovery strategies compared fairly",
        "Both policies retain accepted coverage; only recovery targets differ.",
        b,
        350,
    )
    b = box(
        280,
        24,
        320,
        72,
        "Private simulator state",
        "Actual 20 ms motion and acquisitions",
        "#f5e7d5",
    )
    b += arrow(330, 96, 220, 142) + arrow(550, 96, 660, 142)
    b += box(
        40, 145, 350, 72, "Noisy telemetry adapter", "Seeded bounds, bias and delay"
    ) + box(
        490,
        145,
        350,
        72,
        "Independent physical oracle",
        "Actual acquired rectangular footprints",
        "#f5e7d5",
    )
    b += arrow(215, 217, 215, 258) + arrow(665, 217, 665, 258)
    b += box(
        40,
        260,
        350,
        72,
        "Camera estimator → agent",
        "Measured pose + fixed calibration",
    ) + box(
        490,
        260,
        350,
        72,
        "Python reconstruction",
        "Half-space geometry + trace checks",
        "#f5e7d5",
    )
    b += '<text x="440" y="376" text-anchor="middle" font-size="14">Hidden truth is never an input to the camera estimator or production verifier.</text>'
    svg(
        "evidence",
        "Independent evidence and truth paths",
        "Measurement estimation and oracle scoring are separate paths.",
        b,
        400,
    )


EQUATIONS = {
    "identity": (
        "id = (mission, revision, edge, attempt, drone)",
        "Exact attempt identity",
    ),
    "footprint": (
        "h⁻ = ĥ − εh; a⁻ = h⁻ tan(θa/2) − εp; c⁻ = h⁻ tan(θc/2) − εp<br>I⁻ = [ŝ − a⁻, ŝ + a⁻] ∩ [0,L], only if a⁻ > 0 and d + w ≤ c⁻",
        "Contracted full-strip interval",
    ),
    "coverage": (
        "V = ⋃ I⁻j; M = [0,L] ∖ V; missing = Σ length(Mi)",
        "Union, complement and residual",
    ),
    "arrival": (
        "√((distanceXY + εp)² + (|ĥ − hB| + εh)²) ≤ rB<br>v̂ + εv ≤ vB; sampled dwell ≥ 200 ms<br>FULFILLEDsurvey = qualified arrival ∧ missing ≤ allowance",
        "Separate arrival and survey requirements",
    ),
    "traversal": (
        "Dk = distance(p̂k, route) + εk<br>distance(p(t), route) ≤ (D0 + D1 + Vmax Δt) / 2 ≤ corridor half-width<br>Guaranteed progress: [ŝ0 + ε0, ŝ1 − ε1], if nonempty",
        "Conservative movement and endpoint bounds",
    ),
    "soundness": (
        "∀j: F⁻j ⊆ Fj ⇒ ⋃ F⁻j ⊆ ⋃ Fj<br>Re ⊆ ⋃ F⁻j ⇒ Re ⊆ ⋃ Fj",
        "Conditional survey soundness, ideal set model",
    ),
}

FIGURES = {
    "01_availability": "Figure 1 · All primary conditions: actual completion and reported fulfillment. Negative controls remain included.",
    "02_matched_success_distance": "Figure 2 · Distance on pairs with both true and reported success. The M03–M05 comparisons each contain 100 pairs.",
    "03_positive_negative_trajectories": "Figure 3 · Actual positive and unsuccessful recovery trajectories from the frozen campaign.",
    "04_sensitivity": "Figure 4 · Sensitivity to route, sensor, uncertainty and recovery parameters; fixed one-factor assignments.",
    "05_raster_sensitivity": "Figure 5 · Exact full-strip union versus secondary raster scoring; numerical approximation is separate from sensor validity.",
    "06_truth_and_verified_coverage": "Figure 6 · Independent actual coverage and agent-accepted coverage, including evidence-limited outcomes.",
    "07_host_performance": "Figure 7 · Separate serial benchmark of the complete instrumented process on the recorded macOS host.",
}


def build():
    DOWNLOADS.mkdir(parents=True, exist_ok=True)
    (ASSETS / "figures").mkdir(exist_ok=True)
    manifest = json.loads((FINAL / "manifest.json").read_text())
    analysis = json.loads((FINAL / "analysis_manifest.json").read_text())
    raw = rows(FINAL / "per_attempt.csv")
    assert len(raw) == 5280 and not any(r["infrastructure_error"] for r in raw)
    honest = [r for r in raw if r["scenario"] not in ("M14", "M15")]
    assert len(honest) == 4880 and sum(int(r["false_complete"]) for r in honest) == 0
    assert sum(int(r["false_complete"]) for r in raw) == 400
    scenarios = rows(FINAL / "tables/per_scenario.csv")
    configs = rows(FINAL / "tables/scenario_matrix.csv")
    paired = rows(FINAL / "tables/paired_effects.csv")
    lookup = {(r["scenario"], r["policy"]): r for r in scenarios}
    effects = {(r["scenario"], r["population"], r["metric"]): r for r in paired}
    for r in scenarios:
        sample = [
            x
            for x in raw
            if (x["scenario"], x["policy"]) == (r["scenario"], r["policy"])
        ]
        assert len(sample) == int(r["n"])
        for a, b in [
            ("reported", "reported_complete"),
            ("true_complete", "truth_complete"),
            ("FP", "false_complete"),
            ("FN", "false_negative"),
        ]:
            assert int(r[a]) == sum(int(x[b]) for x in sample)
    copied = {}
    paths = [
        "manifest.json",
        "analysis_manifest.json",
        "per_attempt.csv",
        "SHA256SUMS",
        "environment.txt",
    ]
    paths += ["tables/" + p.name for p in sorted((FINAL / "tables").glob("*.csv"))]
    paths += ["reports/SCIENTIFIC_RESULTS.md"]
    for rel in paths:
        src = FINAL / rel
        dest = DOWNLOADS / src.name
        shutil.copyfile(src, dest)
        copied[str(dest.relative_to(SITE))] = {
            "source": str(src.relative_to(ROOT)),
            "sha256": sha(src),
        }
    for name in FIGURES:
        for ext in ["svg", "pdf", "png"]:
            src = FINAL / "figures" / f"{name}.{ext}"
            dest = ASSETS / "figures" / src.name
            shutil.copyfile(src, dest)
            copied[str(dest.relative_to(SITE))] = {
                "source": str(src.relative_to(ROOT)),
                "sha256": sha(src),
            }
    for f in [
        "FINAL_VALIDATION_V5_1.md",
        "SIMULATION_PROTOCOL_V5_1.md",
        "FINAL_VALIDATION_V5.md",
        "V5_ANALYSIS_NOTES.md",
        "SIMULATION_PROTOCOL_V5.md",
        "V5_PREIMPLEMENTATION_AUDIT.md",
    ]:
        src = ROOT / "docs/vecr" / f
        dest = DOWNLOADS / f
        shutil.copyfile(src, dest)
        copied[str(dest.relative_to(SITE))] = {
            "source": str(src.relative_to(ROOT)),
            "sha256": sha(src),
        }
    for f in ["replication_comparison.csv", "replication_comparison.json"]:
        src = STUDY / "gates_v5_1" / f
        dest = DOWNLOADS / f
        shutil.copyfile(src, dest)
        copied[str(dest.relative_to(SITE))] = {
            "source": str(src.relative_to(ROOT)),
            "sha256": sha(src),
        }
    for src_name, dest_name in [
        ("manifest.json", "benchmark_manifest.json"),
        ("SHA256SUMS", "benchmark_SHA256SUMS"),
    ]:
        src = STUDY / "benchmark_v5_1" / src_name
        dest = DOWNLOADS / dest_name
        shutil.copyfile(src, dest)
        copied[str(dest.relative_to(SITE))] = {
            "source": str(src.relative_to(ROOT)),
            "sha256": sha(src),
        }
    replay = {}
    replay_ids = ["M03", "M04", "M05", "M08", "M14", "M17"]
    with zipfile.ZipFile(
        DOWNLOADS / "representative-traces.zip", "w", zipfile.ZIP_DEFLATED
    ) as archive:
        for case in replay_ids:
            replay[case] = {}
            for policy in ["B2", "P"]:
                run = f"{case}-00001-{policy}"
                path = FINAL / "trials" / run
                trace = rows(path / "trace.csv.gz")
                events = rows(path / "events.csv.gz")
                replay[case][policy] = {"run_id": run, "trace": trace, "events": events}
                for p in sorted(path.iterdir()):
                    info = zipfile.ZipInfo(str(p.relative_to(FINAL)))
                    info.date_time = (2026, 10, 24, 0, 0, 0)
                    info.compress_type = zipfile.ZIP_DEFLATED
                    archive.writestr(info, p.read_bytes())
                    copied[
                        "representative-traces.zip!/" + str(p.relative_to(FINAL))
                    ] = {"source": str(p.relative_to(ROOT)), "sha256": sha(p)}
    data = {
        "scenarios": scenarios,
        "configs": configs,
        "effects": paired,
        "replays": replay,
        "experiment_revision": manifest["source_revision"],
        "analysis_revision": analysis["source_revision"],
        "counts": {"attempts": 5280, "honest": 4880, "controls": 400, "benchmark": 80},
        "scope": "v5.1 corrected-code survey replication; original v5 remains separate and frozen",
    }
    (ASSETS / "experiment-data.js").write_text(
        "window.VECR_DATA = "
        + json.dumps(data, separators=(",", ":"), ensure_ascii=False)
        + ";\n"
    )
    (ASSETS / "research-metadata.json").write_text(
        json.dumps(
            {
                "title": "VECR · From arrival to verified coverage",
                "edition_date": "2026-10-26",
                "experiment": manifest["source_revision"],
                "analysis": analysis["source_revision"],
                "copied_evidence": copied,
            },
            indent=2,
        )
        + "\n"
    )
    (ASSETS / "math-index.json").write_text(
        json.dumps(EQUATIONS, ensure_ascii=False, indent=2) + "\n"
    )
    diagrams()
    components = {"analysis_revision": analysis["source_revision"]}
    comparison = rows(STUDY / "gates_v5_1/replication_comparison.csv")
    components["replication_table"] = table(
        ["Metric", "Paired attempts", "Unequal values", "Maximum absolute difference"],
        [
            [
                esc(r["metric"]),
                r["paired_attempts"],
                r["changed_attempts"],
                f'{float(r["max_absolute_difference"]):.3g}',
            ]
            for r in comparison
        ],
        "Original v5 versus corrected v5.1; exact numerical comparison, no pooling",
    )
    body = []
    for case, label in [
        ("M03", "Departure before B"),
        ("M04", "Strip remains visible"),
        ("M05", "Hidden strip gap"),
    ]:
        r = effects[(case, "both_true_and_reported", "distance_m")]
        body.append(
            [
                f"<strong>{case}</strong> · {label}",
                r["pairs"],
                num(r["B2_mean"]) + " m",
                num(r["P_mean"]) + " m",
                f"{num(r['delta_P_minus_B2'])} [{num(r['ci_low'])}, {num(r['ci_high'])}] m",
                f"{100*(1-float(r['P_mean'])/float(r['B2_mean'])):.1f}%",
            ]
        )
    components["matched_table"] = table(
        [
            "Condition",
            "Pairs",
            "B2 mean",
            "VECR mean",
            "VECR − B2 [95% CI]",
            "Reduction",
        ],
        body,
        "Recorded distance, equally successful pairs",
    )
    body = []
    for c in configs:
        case = c["scenario"]
        b = lookup[(case, "B2")]
        p = lookup[(case, "P")]
        r = effects[(case, "both_true_and_reported", "distance_m")]
        delta = (
            "Undefined · 0 pairs"
            if r["pairs"] == "0"
            else f"{num(r['delta_P_minus_B2'])} [{num(r['ci_low'])}, {num(r['ci_high'])}]"
        )
        cells = [
            f'<button class="case-button" data-case="{case}">{case} ↗</button><span class="case-name">{esc(c["label"])}</span>'
            + (
                '<span class="control-tag">Dishonest control</span>'
                if case in ("M14", "M15")
                else ""
            ),
            b["n"],
            f'{b["true_complete"]} / {p["true_complete"]}',
            f'{b["reported"]} / {p["reported"]}',
            f'{b["FP"]} / {p["FP"]}',
            f'{num(b["mean_distance_m"],2)} / {num(p["mean_distance_m"],2)}',
            delta,
        ]
        body.append(
            f'<tr data-scenario="{case}" data-group="{"primary" if case.startswith("M") else "sensitivity"}">'
            + "".join(f"<td>{x}</td>" for x in cells)
            + "</tr>"
        )
    components["scenario_explorer"] = (
        """<div class="explorer"><div class="explorer-controls"><label>Population<select id="scenarioGroup"><option value="all">All 39 conditions</option><option value="primary">21 primary conditions</option><option value="sensitivity">18 sensitivity conditions</option><option value="controls">Dishonest-sensor controls</option></select></label><label>Find a condition<input id="scenarioSearch" type="search" placeholder="e.g. M17, wind, bound"></label><span id="scenarioCount" role="status">39 conditions</span></div><div class="table-scroll" tabindex="0" role="region" aria-label="Every experimental condition"><table id="scenarioTable"><caption>Within paired columns: B2 / VECR. Distance in metres. CI: paired 95%.</caption><thead><tr><th scope="col">Condition</th><th scope="col">N each</th><th scope="col">True complete</th><th scope="col">Reported</th><th scope="col">False reports</th><th scope="col">All-assigned mean m</th><th scope="col">Matched-success Δ m [CI]</th></tr></thead><tbody>"""
        + "".join(body)
        + """</tbody></table></div><section id="caseDetails" class="case-details" aria-live="polite"><h4>Inspect a condition</h4><p>Select its code for full configuration, precision, recall, acceptance and uncertainty.</p></section></div>"""
    )
    sens = []
    for c in configs:
        if not c["scenario"].startswith("X"):
            continue
        sens.append(
            [
                c["scenario"],
                esc(c["factor"]),
                c["length"],
                c["along"],
                c["cross"],
                c["period"],
                c["bound"],
                c["wind"],
                c["attempts"],
                c["gap-begin"] + "–" + c["gap-end"],
                c["paired_seeds"],
            ]
        )
    components["sensitivity_matrix"] = table(
        [
            "ID",
            "Factor",
            "L m",
            "Along half m",
            "Cross half m",
            "Capture s",
            "Bound m",
            "Wind m/s",
            "Repairs",
            "Gap m",
            "Pairs",
        ],
        sens,
        "Frozen sensitivity settings",
    )
    summary = []
    for policy, label in [("B2", "B2"), ("P", "VECR")]:
        rs = [
            r
            for r in raw
            if r["policy"] == policy
            and r["scenario"].startswith("M")
            and r["scenario"] not in ("M14", "M15")
        ]
        values = [
            sum(int(r[k]) for r in rs)
            for k in [
                "truth_survey",
                "truth_complete",
                "reported_complete",
                "false_complete",
                "false_negative",
            ]
        ]
        summary.append(
            [
                label,
                str(len(rs)),
                *[str(x) for x in values],
                f"{100*values[2]/values[1]:.2f}%",
            ]
        )
    components["confusion_table"] = table(
        [
            "Policy",
            "N",
            "True survey",
            "True + stopped B",
            "Reported",
            "False reports",
            "Unaccepted true",
            "Recall",
        ],
        summary,
        "Honest primary population; different recall denominators",
    )
    perf = rows(FINAL / "tables/isolated_host_performance.csv")

    def percentile(v, q):
        v = sorted(v)
        idx = (len(v) - 1) * q
        i = int(idx)
        return v[i] + (v[min(i + 1, len(v) - 1)] - v[i]) * (idx - i)

    metrics = [
        ("cpu_s", "Whole-process CPU", "s"),
        ("peak_rss_MiB", "Peak process RSS", "MiB"),
        ("capture_latency_p95_us", "Per-trial capture RPC p95", "µs"),
        ("captures_per_wall_s", "Captures / accelerated wall second", "1/s"),
        ("reports_per_wall_s", "Reports / accelerated wall second", "1/s"),
    ]
    components["performance_table"] = table(
        ["Metric", "Median across 80 trials", "p95 across trials", "Unit"],
        [
            [
                label,
                num(statistics.median([float(r[k]) for r in perf])),
                num(percentile([float(r[k]) for r in perf], 0.95)),
                unit,
            ]
            for k, label, unit in metrics
        ],
        "Instrumented macOS host measurements",
    )
    gates = [
        ["Release / UBSan", "Pass", "2/2 CTest targets each after both endpoint fixes"],
        [
            "New endpoint regressions",
            "Pass",
            "3 cases; 20 assertions; defects reproduced before fixes",
        ],
        [
            "SDK installation / external consumer",
            "Pass",
            "Installed target compiled and linked",
        ],
        ["Analysis regressions", "Pass", "2 tests, including seed-cluster dependence"],
        [
            "Independent reconstruction",
            "Pass",
            "5,280 attempts; 453,524 delivered captures; 4,246 re-entry dwells",
        ],
        [
            "Honest-sensor containment",
            "Pass",
            "256,395 accepted captures; zero uncontained footprints",
        ],
        [
            "Serial benchmark reconstruction",
            "Pass",
            "80 attempts; 6,531 delivered captures; 80 re-entry dwells",
        ],
        [
            "Current artifact integrity",
            "Pass",
            "47,559 science + 727 benchmark SHA-256 entries",
        ],
        ["Historical preservation", "Pass", "No previous research results changed"],
        [
            "Historical v5 revalidation",
            "Pass",
            "Full frozen v5 validator rerun; see local audit log",
        ],
        [
            "Corrected-code replication",
            "Pass",
            "78 engineering trials, 5,280 new science attempts and 80 serial benchmarks; no infrastructure errors",
        ],
    ]
    components["gates"] = table(
        ["Gate", "Outcome", "Observed evidence"],
        gates,
        "Current v5.1 engineering and scientific validation gates",
    )
    components[
        "gates"
    ] += '<p class="source-note">Logs: <a href="validation/release_ctest.txt">release</a> · <a href="validation/ubsan_ctest.txt">UBSan</a> · <a href="validation/endpoint_bounds_before.txt">traversal defect reproduced</a> · <a href="validation/camera_endpoints_before.txt">camera defect reproduced</a> · <a href="validation/endpoints_after.txt">both fixes</a> · <a href="validation/corrected_v5_1_validation.txt">v5.1 independent reconstruction</a> · <a href="validation/benchmark_v5_1_validation.txt">benchmark integrity</a> · <a href="validation/frozen_v5_revalidation.txt">historical v5 revalidation</a>.</p>'
    components["provenance"] = table(
        ["Artifact / revision", "Exact identifier"],
        [
            [esc(label), f'<code class="hash">{esc(value)}</code>']
            for label, value in [
                ("Experimental source", manifest["source_revision"]),
                ("Analysis source", analysis["source_revision"]),
                ("Release binary SHA-256", manifest["binary_sha256"]),
                ("Assignments SHA-256", sha(FINAL / "manifest.json")),
                ("Raw per-attempt SHA-256", sha(FINAL / "per_attempt.csv")),
                ("SHA256SUMS file SHA-256", sha(FINAL / "SHA256SUMS")),
            ]
        ],
        "Independent experimental and analysis identities",
    )
    downloads = [
        ("per_attempt.csv", "5,280 raw attempt rows"),
        ("per_scenario.csv", "Per-condition statistics and Wilson intervals"),
        ("paired_effects.csv", "Paired effects and confidence intervals"),
        (
            "paired_primary_cluster_effects.csv",
            "Whole-seed-cluster aggregate intervals",
        ),
        ("scenario_matrix.csv", "All experimental settings"),
        ("unsuccessful_attempts.csv", "Every unsuccessful attempt"),
        ("capture_rejections.csv", "Sensor rejection inventory"),
        ("isolated_host_performance.csv", "All 80 serial process measurements"),
        ("representative-traces.zip", "Lossless logs for the 12 replayed attempts"),
        ("manifest.json", "Frozen assignments and environment identity"),
        ("analysis_manifest.json", "Analysis source, software and bootstrap identity"),
        ("benchmark_manifest.json", "Separate serial benchmark assignments"),
        ("benchmark_SHA256SUMS", "Complete benchmark hash inventory"),
        ("SHA256SUMS", "Complete scientific artifact hash inventory"),
        ("FINAL_VALIDATION_V5_1.md", "Current full validation record"),
        ("SIMULATION_PROTOCOL_V5_1.md", "Endpoint-correction replication protocol"),
        ("SIMULATION_PROTOCOL_V5.md", "Full experimental specification"),
        ("replication_comparison.csv", "Original versus corrected raw-row comparison"),
        ("V5_ANALYSIS_NOTES.md", "Historical v5 analysis corrections"),
    ]
    components["downloads"] = (
        '<div class="download-grid">'
        + "".join(
            f'<a href="assets/evidence/{name}" download><span>↓</span><div><strong>{esc(label)}</strong><small>{esc(name)}</small></div></a>'
            for name, label in downloads
        )
        + '</div><p class="source-note">Full raw trace collection: <a href="../results/vecr/agent_campaign_v5/corrected_endpoints_v5_1/">repository-local v5.1 directory</a>. This link needs the surrounding repository; bundled downloads above travel with the website.</p>'
    )
    questions = [
        (
            "What is the core innovation?",
            "An implemented edge-obligation contract combining qualified arrival, conservative full-strip accounting, retained partial work, exact attempt identity and bounded onboard recovery. The experiment isolates recovery target selection against a coverage-aware baseline.",
        ),
        (
            "Did you invent graph traversal or gossip?",
            "No. The graph planner and communication policy remain external. The project implements neither rotor-router nor gossip; algorithm-neutral APIs do not prove arbitrary planner compatibility.",
        ),
        (
            "Is the baseline fair?",
            "Both policies share verification, sensor generation, dynamics, faults, thresholds and safety. B2 is coverage-aware and retains accepted images. Its explicitly different choice is full-edge restart at A.",
        ),
        (
            "What are the strongest quantitative findings?",
            "M03–M05 each have 100 true and reported successes under both policies. VECR uses 25.3–27.8% less modeled travel, with paired intervals in Chapter 8. This is conditional on those synthetic conditions.",
        ),
        (
            "Why is VECR recall lower?",
            "It actually returned to B in more uncertain-sensor trials, enlarging its true-completion denominator. Persistent missing evidence still prevented certification. Reported totals are 794 VECR versus 700 B2 across honest primary cases, while recall is 72.58% versus 87.50%.",
        ),
        (
            "Does zero false fulfillment prove safety?",
            "No. It is conditional on the declared model and truthful bounded sensor evidence. All 400 deliberately dishonest controls falsely fulfilled. A sensor authentication boundary cannot verify physical truth.",
        ),
        (
            "Where does correction run?",
            "Inside the production SwarmKit agent. The SDK requests an operation; the agent evaluates observations and commands the backend. A cloud service is not required for that local loop.",
        ),
        (
            "Is every negative outcome a bug?",
            "No. Sustained wind, command refusal, motor failure, insufficient evidence and low battery are real negative outcomes in the model. Protective uncertainty predicates were not relaxed to improve acceptance.",
        ),
        (
            "Is this real camera or flight validation?",
            "No. The camera model is a flat, full-strip geometric abstraction. Simulator measurements, SITL and real hardware are distinct evidence levels. The SITL chapter currently has no executed trials.",
        ),
        (
            "What happened to earlier evidence?",
            "All historical research stays frozen. This reader presents the new v5.1 replication after both endpoint fixes. Original v5 data and analysis remain separately identified; the datasets are not pooled.",
        ),
    ]
    components["defense_questions"] = (
        '<div class="questions">'
        + "".join(
            f"<details><summary>{esc(q)}</summary><p>{esc(a)}</p></details>"
            for q, a in questions
        )
        + "</div>"
    )
    options = "".join(
        f'<option value="{case}">{case} · {esc(next(c["label"] for c in configs if c["scenario"]==case))}</option>'
        for case in replay_ids
    )
    components["trace_explorer"] = (
        f"""<div class="trace-explorer"><div class="trace-toolbar"><label>Recorded condition<select id="replayCase">{options}</select></label><button id="playReplay" class="button">Play replay</button><button id="resetReplay" class="button secondary">Reset</button><label>Speed<select id="replaySpeed"><option value="1">1×</option><option value="4" selected>4×</option><option value="10">10×</option></select></label></div><div class="trace-legend"><span class="legend-b2">B2 · full restart</span><span class="legend-p">VECR · targeted</span><span>Faint line: complete recorded path · ×: accepted target</span></div><svg id="replayPlot" viewBox="0 0 880 370" role="img" aria-labelledby="replayTitle replayDesc"><title id="replayTitle">Recorded physical trajectories</title><desc id="replayDesc">Use the time slider to compare B2 and VECR at the same simulation time.</desc><g id="replayDrawing"></g></svg><label class="time-control">Simulation time <output id="replayTime">0.0 s</output><input id="replaySlider" type="range" min="100" max="40000" step="100" value="100" aria-label="Replay simulation time"></label><div id="replayMetrics" class="replay-metrics" aria-live="off"></div><div class="trace-events"><h4>Recorded event timeline</h4><p>Click an event to inspect that time. Only phase, command and fault events are shown here; complete logs are in the download.</p><div id="eventList"></div></div><noscript><p>Replay controls require JavaScript. The original trajectory figure and downloadable traces remain available.</p></noscript></div>"""
    )
    source = (SITE / "source/dissertation.md").read_text()

    def expand(m):
        key = m.group(1)
        if key.startswith("diagram:"):
            name = key.split(":")[1]
            return image_figure(
                name,
                {
                    "contract": "Conceptual contract · four different claims; the gap schematic is not a measured trial.",
                    "architecture": "Implemented architecture · all physical decisions stay in the agent.",
                    "states": "State semantics · ACK and completed physical re-entry remain distinct.",
                    "comparison": "Schematic recovery comparison · identical shared contracts, different re-entry targets.",
                    "evidence": "Observation separation · noisy estimates and private acquisition truth follow different paths.",
                }[name],
                "diagrams",
            )
        if key.startswith("figure:"):
            name = key.split(":")[1]
            return image_figure(name, FIGURES[name])
        if key.startswith("equation:"):
            formula, title = EQUATIONS[key.split(":")[1]]
            return f'<div class="equation" role="group" aria-label="{esc(title)}"><div>{formula}</div><small>{esc(title)}</small></div>'
        return components[key]

    rendered = markdown.markdown(
        re.sub(r"\{\{([^}]+)\}\}", expand, source),
        extensions=["extra", "toc", "sane_lists"],
    )
    # Markdown-authored tables also need a bounded scroll region on small screens.
    rendered = re.sub(
        r'(?<!<div class="table-scroll">)<table>.*?</table>',
        lambda m: (
            '<div class="table-scroll" tabindex="0" role="region" aria-label="Dissertation table">'
            + m[0]
            + "</div>"
            if "\n" in m[0]
            else m[0]
        ),
        rendered,
        flags=re.S,
    )
    headings = re.findall(r'<h1 id="([^"]+)">(.*?)</h1>', rendered)
    assert len(headings) == 16
    rendered = re.sub(
        r"<(/?)h([123])(?=[ >])", lambda m: f"<{m[1]}h{int(m[2])+1}", rendered
    )
    rendered = re.sub(
        r'<h2 id="([^"]+)">',
        r'</section><section class="chapter" aria-labelledby="\1"><h2 id="\1">',
        rendered,
    )
    rendered = rendered.removeprefix("</section>") + "</section>"
    nav = "".join(
        f'<a href="#{id}"><span>{title.split(" / ")[0]}</span>{title.split(" / ",1)[1]}</a>'
        for id, title in headings
    )
    hero_trace = replay["M03"]
    paths = ""
    for policy, color in [("B2", "#b77133"), ("P", "#167d6a")]:
        points = " ".join(
            f'{40+float(r["east_m"])*11:.2f},{195-float(r["north_m"])*13:.2f}'
            for r in hero_trace[policy]["trace"]
        )
        paths += f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="3" stroke-linejoin="round"/>'
    hero = f"""<svg viewBox="0 0 520 290" role="img" aria-labelledby="heroDiagramTitle"><title id="heroDiagramTitle">Actual M03 seed 1 paths: full restart and targeted recovery</title><rect x="40" y="143" width="440" height="104" fill="#dce9df" opacity=".6"/><path d="M40 195H480" stroke="#879d91" stroke-dasharray="5 6"/>{paths}<circle cx="40" cy="195" r="5" fill="#183c36"/><circle cx="480" cy="195" r="5" fill="#183c36"/><g font-family="Arial,sans-serif" font-size="12" fill="#466359"><text x="40" y="274">A · 0 m</text><text x="442" y="274">B · 40 m</text><text x="40" y="30">Recorded M03 · seed 1</text><text x="40" y="49">Cross-track displacement shown vertically</text></g></svg>"""
    template = (SITE / "tools/template.html").read_text()
    for key, value in {"NAV": nav, "CONTENT": rendered, "HERO": hero}.items():
        template = template.replace("{{" + key + "}}", value)
    assert not re.search(r"\{\{[^}]+\}\}", template)
    (SITE / "index.htm").write_text(template)
    print(
        f"Built 16 chapters, {len(scenarios)} policy rows, {len(replay_ids)*2} recorded replays and {len(FIGURES)} original research figures."
    )


if __name__ == "__main__":
    build()
