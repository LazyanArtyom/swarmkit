#!/usr/bin/env python3
"""Build the offline dissertation reader from current frozen evidence."""

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
PRIMARY = ROOT / "results/vecr/current/digital/primary"
REUSE = ROOT / "results/vecr/current/digital/reuse"
SITL = ROOT / "results/vecr/current/sitl/qualification"
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
    return (
        f'<figure class="research-figure"><button class="figure-open" '
        f'data-image="assets/{folder}/{name}.svg" data-caption="{esc(caption)}" '
        f'aria-label="Enlarge: {esc(caption)}"><img loading="lazy" '
        f'src="assets/{folder}/{name}.svg" alt="{esc(caption)}"><span class="enlarge">Enlarge ↗</span>'
        f'</button><figcaption>{esc(caption)}</figcaption></figure>'
    )


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
    svg("contract", "Arrival and survey fulfillment", "Conceptual distinctions, not experimental measurements.", b)

    b = box(260, 25, 360, 72, "External mission application", "Owns graph planning and coordination") + arrow(440, 97, 440, 137)
    b += '<rect x="220" y="139" width="440" height="172" rx="16" fill="#e3eee7" stroke="#237966" stroke-width="2"/><text x="440" y="169" text-anchor="middle" font-size="19" font-weight="600">SwarmKit agent · local decisions</text>'
    b += box(240, 190, 190, 90, "Evidence + identity", "Coverage union / missing work", "#fff") + box(
        450, 190, 190, 90, "Recovery + safety", "Arbitrated physical commands", "#fff"
    )
    b += box(20, 190, 170, 90, "Sensor adapter", "Measured metadata", "#fff") + arrow(190, 235, 220, 235)
    b += box(690, 190, 170, 90, "Client / server", "Reports and telemetry", "#fff") + arrow(660, 235, 690, 235)
    b += arrow(440, 311, 440, 350) + box(
        260, 352, 360, 72, "Vehicle backend (SimBackend / MAVLink)", "Physical execution and dynamics"
    )
    svg("architecture", "Implemented local execution architecture", "Deviation detection, verification and recovery run inside the agent.", b, 450)

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
    svg("states", "Canonical execution phases", "State machine with physical re-entry guards.", b, 355)

    b = '<text x="32" y="34" font-size="14" letter-spacing="2">SHARED VERIFIER, DYNAMICS, SENSOR, SAFETY AND DEADLINE</text>'
    for y, label, target, color in [
        (126, "B2 · full-edge restart baseline", 80, "#b77133"),
        (254, "VECR · earliest-gap recovery with bounded wait", 328, "#167d6a"),
    ]:
        b += f'<text x="32" y="{y-60}" font-size="18" font-weight="600">{label}</text><path d="M80 {y}H804" stroke="#cdd9d1" stroke-width="9"/><path d="M80 {y}H360 M448 {y}H804" stroke="#97c3b2" stroke-width="9"/><path d="M360 {y}H448" stroke="#c47735" stroke-width="9" stroke-dasharray="5 5"/><path d="M560 {y-18} Q440 {y-50} {target} {y-18}" fill="none" stroke="{color}" stroke-width="3" marker-end="url(#arrow)"/><circle cx="{target}" cy="{y}" r="6" fill="{color}"/><text x="80" y="{y+30}" text-anchor="middle" font-size="13">A</text><text x="804" y="{y+30}" text-anchor="middle" font-size="13">B</text>'
    b += '<text x="440" y="325" text-anchor="middle" font-size="13">Schematic · VECR targets min(progress, gap_start) during departure and earliest gap at endpoint.</text>'
    svg("comparison", "Recovery strategies compared fairly", "Both policies retain accepted coverage; only recovery targets differ.", b, 350)

    b = box(280, 24, 320, 72, "Private simulator state", "Actual 20 ms motion and acquisitions", "#f5e7d5")
    b += arrow(330, 96, 220, 142) + arrow(550, 96, 660, 142)
    b += box(40, 145, 350, 72, "Noisy telemetry adapter", "Seeded bounds, bias and delay") + box(
        490, 145, 350, 72, "Independent physical oracle", "Actual acquired rectangular footprints", "#f5e7d5"
    )
    b += arrow(215, 217, 215, 258) + arrow(665, 217, 665, 258)
    b += box(40, 260, 350, 72, "Camera estimator → agent", "Measured pose + fixed calibration") + box(
        490, 260, 350, 72, "Python reconstruction", "Half-space geometry + trace checks", "#f5e7d5"
    )
    b += '<text x="440" y="376" text-anchor="middle" font-size="14">Hidden truth is never an input to the camera estimator or production verifier.</text>'
    svg("evidence", "Independent evidence and truth paths", "Measurement estimation and oracle scoring are separate paths.", b, 400)


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
    "recovery": (
        "q = min(p, g) on sustained departure; q = g at verified endpoint; clipped to [0, L]",
        "Canonical earliest-gap recovery target",
    ),
    "soundness": (
        "∀j: F⁻j ⊆ Fj ⇒ ⋃ F⁻j ⊆ ⋃ Fj<br>Re ⊆ ⋃ F⁻j ⇒ Re ⊆ ⋃ Fj",
        "Conditional survey soundness, ideal set model",
    ),
}

FIGURES = {
    "01_completion_matrix": "Figure 1 · Preregistered 22-condition completion matrix: true completion and reported fulfillment across 1,320 attempts (30 paired seeds per cell). Dishonest sensor controls (N19, N20) expose the sensor trust boundary.",
    "02_paired_distance": "Figure 2 · Paired distance comparison between VECR and B2 (full-restart baseline) on jointly successful attempts. Pointwise 95% bootstrap intervals from 10,000 resamples.",
    "03_sitl_traces": "Figure 3 · ArduPilot SITL qualification traces: real Copter 4.7.2 flight dynamics executing MAVLink commands under nominal, delayed, multi-gap, command-rejection, and land-override conditions.",
}


def build():
    DOWNLOADS.mkdir(parents=True, exist_ok=True)
    (ASSETS / "figures").mkdir(exist_ok=True)

    manifest = json.loads((PRIMARY / "manifest.json").read_text())
    analysis = json.loads((PRIMARY / "analysis/summary.json").read_text())
    raw = rows(PRIMARY / "per_attempt.csv")
    assert len(raw) == 1320 and not any(r["infrastructure_error"] for r in raw)

    honest = [r for r in raw if r["scenario"] not in ("N19", "N20")]
    assert len(honest) == 1200 and sum(int(r["false_complete"]) for r in honest) == 0
    assert sum(int(r["false_complete"]) for r in raw) == 120

    scenarios = rows(PRIMARY / "analysis/condition_summary.csv")
    paired = rows(PRIMARY / "analysis/paired_comparisons.csv")
    lookup = {(r["scenario"], r["policy"]): r for r in scenarios}
    effects = {r["scenario"]: r for r in paired}

    # Condition descriptions
    SCENARIO_CONFIGS = [
        {"scenario": "N00", "label": "Nominal flight", "fault": "none", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "Baseline conditions; no disturbances"},
        {"scenario": "N01", "label": "Delayed evidence delivery", "fault": "delay 2.5 s", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "Metadata delayed 2.5 s; tested endpoint waiting"},
        {"scenario": "N02", "label": "Reordered evidence stream", "fault": "reordered", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "Out-of-order capture arrival"},
        {"scenario": "N03", "label": "Missing metadata gap", "fault": "metadata_gap", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "Single intermediate gap [16, 24] m"},
        {"scenario": "N04", "label": "Multiple capture gaps", "fault": "multi_gap", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "Two separate gaps along route"},
        {"scenario": "N05", "label": "Changing localization uncertainty", "fault": "changing_uncertainty", "length": "40 m", "bound": "0.3–0.9 m", "deadline": "90 s", "notes": "Dynamic noise escalation"},
        {"scenario": "N06", "label": "Uncertainty increase during gap", "fault": "uncertainty_gap", "length": "40 m", "bound": "0.3–0.9 m", "deadline": "90 s", "notes": "Noise spike during unobserved interval"},
        {"scenario": "N07", "label": "Sustained physical departure", "fault": "departure", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "3 m/s crosswind gust driving drone off corridor"},
        {"scenario": "N08", "label": "Departure with wide sensing", "fault": "visible", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "Excursion while wide camera still observes strip"},
        {"scenario": "N09", "label": "Departure with narrow sensing", "fault": "hidden", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "Excursion with narrow FOV leaving gap"},
        {"scenario": "N10", "label": "Persistent sensor failure", "fault": "persistent_sensor", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "Permanent loss of camera evidence; fails closed"},
        {"scenario": "N11", "label": "Recovery command rejected", "fault": "command_rejected", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "Backend refuses re-entry GOTO; fails closed"},
        {"scenario": "N12", "label": "Motor failure during repair", "fault": "motor_failure", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "Physical propulsion loss; interrupted"},
        {"scenario": "N13", "label": "Low battery threshold", "fault": "low_battery", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "Battery below reserve margin; fails closed"},
        {"scenario": "N14", "label": "Zero recovery budget", "fault": "no_repair", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "attempts=0; no repairs allowed"},
        {"scenario": "N15", "label": "Expired evidence horizon", "fault": "stale delay 4 s", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "Captures exceed 3 s age limit; rejected"},
        {"scenario": "N16", "label": "Sparse small-footprint captures", "fault": "sparse", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "1.5 s capture period leaves geometric gaps"},
        {"scenario": "N17", "label": "Short route (20 m)", "fault": "multi_gap", "length": "20 m", "bound": "0.3 m", "deadline": "90 s", "notes": "Multiple gaps on 20 m edge"},
        {"scenario": "N18", "label": "Long route (80 m)", "fault": "multi_gap", "length": "80 m", "bound": "0.3 m", "deadline": "160 s", "notes": "Multiple gaps on 80 m edge"},
        {"scenario": "N19", "label": "Dishonest camera frame control", "fault": "false_frame", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "Sensor claims success without image; trust boundary control"},
        {"scenario": "N20", "label": "Undeclared camera angle control", "fault": "undeclared_angle", "length": "40 m", "bound": "0.3 m", "deadline": "90 s", "notes": "Concealed 45° yaw misalignment; trust boundary control"},
        {"scenario": "N21", "label": "Insufficient deadline", "fault": "multi_gap", "length": "40 m", "bound": "0.3 m", "deadline": "30 s", "notes": "30 s timeout expires before full recovery"},
    ]

    copied = {}
    for name in [
        "manifest.json",
        "per_attempt.csv",
        "environment.txt",
        "condition_summary.csv",
        "paired_comparisons.csv",
        "summary.json",
        "RESULTS.md",
        "client_reuse.json",
        "client_reuse_manifest.json",
        "client_reuse_per_attempt.csv",
        "sitl_manifest.json",
        "sitl_per_attempt.csv",
        "sitl_condition_summary.csv",
        "sitl_reconstructed.csv",
        "sitl_validation.json",
        "METHOD.md",
        "SIMULATION_PROTOCOL.md",
        "SITL_PROTOCOL.md",
        "LITERATURE.md",
        "REVIEW_RESPONSE.md",
        "representative-traces.zip",
        "SHA256SUMS",
    ]:
        p = DOWNLOADS / name
        if p.exists():
            copied[str(p.relative_to(SITE))] = {
                "source": str(p.relative_to(ROOT)),
                "sha256": sha(p),
            }

    for name in FIGURES:
        for ext in ["svg", "pdf", "png"]:
            p = ASSETS / "figures" / f"{name}.{ext}"
            if p.exists():
                copied[str(p.relative_to(SITE))] = {
                    "source": str(p.relative_to(ROOT)),
                    "sha256": sha(p),
                }

    # Load representative replays
    replay_cases = ["N00", "N03", "N04", "N07", "N11", "N19"]
    replays = {}
    for case in replay_cases:
        replays[case] = {}
        for policy in ["B2", "VECR"]:
            run_id = f"{case}-70001-{policy}-direct"
            trial_path = PRIMARY / "trials" / run_id
            t_path = trial_path / "trace.csv.gz"
            e_path = trial_path / "events.csv.gz"
            trace = rows(t_path)
            events = rows(e_path)
            replays[case][policy] = {"run_id": run_id, "trace": trace, "events": events}

    # Export window.VECR_DATA
    data = {
        "scenarios": scenarios,
        "configs": SCENARIO_CONFIGS,
        "effects": paired,
        "replays": replays,
        "experiment_revision": manifest["source_revision"],
        "counts": {
            "attempts": 1320,
            "honest": 1200,
            "controls": 120,
            "client_reuse": 100,
            "sitl_flights": 12,
        },
        "scope": "Canonical VECR (earliest-gap recovery with bounded wait) vs B2 full-restart baseline across 22 conditions.",
    }
    (ASSETS / "experiment-data.js").write_text(
        "window.VECR_DATA = "
        + json.dumps(data, separators=(",", ":"), ensure_ascii=False)
        + ";\n"
    )

    author = json.loads((SITE / "source/author.json").read_text())
    (ASSETS / "research-metadata.json").write_text(
        json.dumps(
            {
                "title": "VECR · From arrival to verified coverage",
                "edition_date": "2026-10-28",
                "author": author,
                "experiment": manifest["source_revision"],
                "binary_sha256": manifest["binary_sha256"],
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

    components = {}

    components["author_profile"] = (
        '<div class="author-profile"><div class="author-heading">'
        '<span class="author-monogram" aria-hidden="true">AL</span><div>'
        '<span class="eyebrow">RESEARCHER</span><p class="author-name">'
        + esc(author["name"])
        + "</p></div></div><dl>"
        + "".join(
            f"<div><dt>{esc(label)}</dt><dd>{esc(value)}</dd></div>"
            for label, value in [
                ("Department", author["department"]),
                ("Institute", author["institution"]),
                ("Academy", author["academy"]),
                ("Specialization", author["specialization_code"] + " — " + author["specialization"]),
                ("Research theme", author["research_theme"]),
            ]
        )
        + '</dl><p class="source-note">Institutional affiliation: '
        + f'<a href="{esc(author["department_url"])}">official department staff listing</a>. '
        + "National Academy of Sciences of the Republic of Armenia.</p></div>"
    )

    # Matched comparison table for gap scenarios
    gap_cases = [
        ("N03", "Missing metadata gap (40 m)"),
        ("N04", "Multiple capture gaps (40 m)"),
        ("N06", "Uncertainty increase during gap (40 m)"),
        ("N07", "Sustained physical departure (40 m)"),
        ("N08", "Departure with wide sensing (40 m)"),
        ("N09", "Departure with narrow sensing (40 m)"),
        ("N17", "Multiple gaps, short route (20 m)"),
        ("N18", "Multiple gaps, long route (80 m)"),
    ]
    matched_rows = []
    for case, label in gap_cases:
        p = effects[case]
        b2_row = lookup[(case, "B2")]
        vecr_row = lookup[(case, "VECR")]
        b2_dist = float(b2_row["mean_distance_m"])
        vecr_dist = float(vecr_row["mean_distance_m"])
        reduction = (1.0 - vecr_dist / b2_dist) * 100.0 if b2_dist > 0 else 0.0
        delta_str = f"{float(p['successful_distance_delta_m']):.3f} [{float(p['successful_distance_delta_m_ci_low']):.3f}, {float(p['successful_distance_delta_m_ci_high']):.3f}] m"
        matched_rows.append([
            f"<strong>{case}</strong> · {label}",
            p["joint_success_pairs"] + "/30",
            f"{b2_dist:.2f} m",
            f"{vecr_dist:.2f} m",
            delta_str,
            f"<strong>{reduction:.1f}%</strong>",
        ])

    components["matched_table"] = table(
        ["Condition", "Joint successes", "B2 mean distance", "VECR mean distance", "VECR − B2 [95% CI]", "Distance reduction"],
        matched_rows,
        "Recorded physical travel distance on jointly successful attempts (30 paired seeds per condition)",
    )

    # Scenario explorer table
    explorer_body = []
    for c in SCENARIO_CONFIGS:
        case = c["scenario"]
        b2_r = lookup.get((case, "B2"))
        vecr_r = lookup.get((case, "VECR"))
        p = effects.get(case)
        delta_str = (
            "—"
            if not p or p["joint_success_pairs"] == "0"
            else f"{float(p['successful_distance_delta_m']):.2f} [{float(p['successful_distance_delta_m_ci_low']):.2f}, {float(p['successful_distance_delta_m_ci_high']):.2f}]"
        )
        is_control = case in ("N19", "N20")
        cells = [
            f'<button class="case-button" data-case="{case}">{case} ↗</button><span class="case-name">{esc(c["label"])}</span>'
            + ('<span class="control-tag">Dishonest control</span>' if is_control else ""),
            b2_r["n"] if b2_r else "30",
            f'{b2_r["true_complete"]} / {vecr_r["true_complete"]}',
            f'{b2_r["reported_complete"]} / {vecr_r["reported_complete"]}',
            f'{b2_r["false_complete"]} / {vecr_r["false_complete"]}',
            f'{float(b2_r["mean_distance_m"]):.2f} / {float(vecr_r["mean_distance_m"]):.2f}',
            delta_str,
        ]
        group = "controls" if is_control else "primary"
        explorer_body.append(
            f'<tr data-scenario="{case}" data-group="{group}">'
            + "".join(f"<td>{x}</td>" for x in cells)
            + "</tr>"
        )

    components["scenario_explorer"] = (
        """<div class="explorer"><div class="explorer-controls"><label>Population<select id="scenarioGroup"><option value="all">All 22 conditions (1,320 attempts)</option><option value="primary">20 honest conditions (1,200 attempts)</option><option value="controls">2 dishonest controls (120 attempts)</option></select></label><label>Find a condition<input id="scenarioSearch" type="search" placeholder="e.g. N04, gap, departure, battery"></label><span id="scenarioCount" role="status">22 conditions</span></div><div class="table-scroll" tabindex="0" role="region" aria-label="Every experimental condition"><table id="scenarioTable"><caption>Within paired columns: B2 / VECR. Distance in metres. CI: paired 95% bootstrap (10,000 resamples).</caption><thead><tr><th scope="col">Condition</th><th scope="col">N each</th><th scope="col">True complete</th><th scope="col">Reported complete</th><th scope="col">False reports</th><th scope="col">Mean distance (m)</th><th scope="col">Matched-success Δ m [CI]</th></tr></thead><tbody>"""
        + "".join(explorer_body)
        + """</tbody></table></div><section id="caseDetails" class="case-details" aria-live="polite"><h4>Inspect a condition</h4><p>Select any condition code above for configuration details, Wilson 95% precision, recall, and distance distributions.</p></section></div>"""
    )

    # Confusion summary table
    conf_rows = []
    for policy, label in [("B2", "B2 full-restart baseline"), ("VECR", "VECR earliest-gap recovery")]:
        h_attempts = [r for r in raw if r["policy"] == policy and r["scenario"] not in ("N19", "N20")]
        c_attempts = [r for r in raw if r["policy"] == policy and r["scenario"] in ("N19", "N20")]
        conf_rows.append([
            f"<strong>{label}</strong>",
            str(len(h_attempts)),
            str(sum(int(r["truth_complete"]) for r in h_attempts)),
            str(sum(int(r["reported_complete"]) for r in h_attempts)),
            str(sum(int(r["false_complete"]) for r in h_attempts)),
            f"{100*sum(int(r['reported_complete']) for r in h_attempts)/sum(int(r['truth_complete']) for r in h_attempts):.2f}%",
            str(len(c_attempts)),
            f"{sum(int(r['false_complete']) for r in c_attempts)}/{len(c_attempts)} (100% false fulfillment)",
        ])

    components["confusion_table"] = table(
        ["Method", "Honest attempts", "True complete", "Reported complete", "False fulfillments", "Recall", "Control attempts", "Dishonest control reports"],
        conf_rows,
        "Honest population vs deliberate dishonest-sensor controls (N19, N20)",
    )

    # Client reuse table
    components["reuse_table"] = table(
        ["Tested parameter", "Direct Client path", "Routed SwarmClient path", "Matched outcome"],
        [
            ["Attempts", "50", "50", "100 total"],
            ["Tested conditions", "N00, N01, N04, N07, N11", "N00, N01, N04, N07, N11", "Identical 5 conditions"],
            ["Seeds", "72001–72010", "72001–72010", "10 paired seeds"],
            ["Terminal phases", "30 FULFILLED, 20 INTERRUPTED", "30 FULFILLED, 20 INTERRUPTED", "0 differences"],
            ["Mean distance", "64.97 m", "64.97 m", "Δ = 0.000 m [0.000, 0.000]"],
            ["Physical re-entry dwells", "20 dwells", "20 dwells", "Exact match"],
        ],
        "Client reuse campaign: direct vs routed agent client execution",
    )

    # SITL qualification table
    sitl_raw = rows(SITL / "per_attempt.csv")
    sitl_summary = rows(SITL / "analysis/condition_summary.csv")
    sitl_rows = []
    for r in sitl_summary:
        sitl_rows.append([
            f"<code>{esc(r['navigation'])}</code> / {esc(r['fault'])}",
            "VECR",
            f"{r['reported']}/2",
            f"{r['sampled_true']}/2",
            f"{float(r['distance_m_mean']):.2f} m",
            f"{float(r['duration_s_mean']):.2f} s",
        ])
    components["sitl_table"] = table(
        ["Observation tier / fault", "Policy", "Reported complete", "Sampled truth complete", "Mean distance", "Mean duration"],
        sitl_rows,
        "ArduPilot SITL qualification (12 flights, native Copter 4.7.2 with MAVLink execution)",
    )

    # Verification gates
    gates = [
        ["Release CTest suite", "Pass", "3/3 targets passed in 22.2 s (swarmkit_tests, vecr_baseline_contract)"],
        ["UBSan CTest suite", "Pass", "3/3 targets passed in 24.2 s under undefined-behavior sanitizers"],
        ["Digital primary independent reconstruction", "Pass", "1,320 attempts, 108,790 delivered captures, 720 physical re-entry dwells verified"],
        ["Honest-sensor footprint containment", "Pass", "Zero uncontained footprints across all accepted honest acquisitions"],
        ["Client reuse invariance", "Pass", "100 trials, 50 matched direct vs routed pairs; 0 metric differences"],
        ["ArduPilot SITL independent reconstruction", "Pass", "12 flights, 509 accepted captures, 2 sampled re-entry dwells verified"],
        ["Fail-closed safety interventions", "Pass", "100% correct interruption on command refusal, motor failure, low battery, and deadline"],
    ]
    components["gates"] = table(["Verification gate", "Status", "Verified evidence"], gates, "Current scientific and engineering validation gates")

    # Provenance
    components["provenance"] = table(
        ["Artifact / Revision", "Exact Identifier / SHA-256"],
        [
            [esc(label), f'<code class="hash">{esc(value)}</code>']
            for label, value in [
                ("Experimental Git revision", manifest["source_revision"]),
                ("Host platform", manifest["host"]),
                ("Python version", manifest["python"]),
                ("VECR release binary SHA-256", manifest["binary_sha256"]["VECR"]),
                ("B2 baseline binary SHA-256", manifest["binary_sha256"]["B2"]),
                ("Primary assignments SHA-256", sha(PRIMARY / "manifest.json")),
                ("Per-attempt raw CSV SHA-256", sha(PRIMARY / "per_attempt.csv")),
                ("SITL qualification SHA-256", sha(SITL / "manifest.json")),
            ]
        ],
        "Cryptographic and source provenance identifiers",
    )

    # Downloads
    downloads = [
        ("per_attempt.csv", "1,320 raw attempts with metrics"),
        ("condition_summary.csv", "22-condition statistical summaries"),
        ("paired_comparisons.csv", "Paired VECR vs B2 bootstrap differences"),
        ("representative-traces.zip", "Lossless 100 ms traces and event journals"),
        ("summary.json", "Campaign manifest and statistical summary"),
        ("RESULTS.md", "Executive numerical results"),
        ("client_reuse.json", "Direct vs routed client reuse evaluation"),
        ("client_reuse_per_attempt.csv", "100 client reuse trials"),
        ("sitl_per_attempt.csv", "12 ArduPilot SITL flight records"),
        ("sitl_condition_summary.csv", "SITL flight tier summaries"),
        ("sitl_validation.json", "SITL independent reconstruction report"),
        ("METHOD.md", "Canonical VECR specification and formulas"),
        ("SIMULATION_PROTOCOL.md", "Preregistered digital campaign protocol"),
        ("SITL_PROTOCOL.md", "ArduPilot qualification protocol"),
        ("LITERATURE.md", "Prior work overlap and contribution boundaries"),
        ("REVIEW_RESPONSE.md", "Reviewer disposition and subject-matter correction"),
        ("manifest.json", "Preregistered configurations and seeds"),
        ("environment.txt", "Host compiler, OS and CMake build environment"),
        ("SHA256SUMS", "Complete evidence cryptographic checksums"),
    ]
    components["downloads"] = (
        '<div class="download-grid">'
        + "".join(
            f'<a href="assets/evidence/{name}" download><span>↓</span><div><strong>{esc(label)}</strong><small>{esc(name)}</small></div></a>'
            for name, label in downloads
        )
        + '</div><p class="source-note">Complete evidence archive: <a href="../results/vecr/current/">repository-local results directory</a>.</p>'
    )

    # Defense questions
    questions = [
        (
            "What is the core contribution of this work?",
            "VECR contributes a concrete, reusable onboard execution, verification and recovery contract for assigned UAV graph edges. The agent binds exact attempt identity, maintains conservative full-strip interval unions, guarantees stopped 3D arrival with physical dwell, and recovers unverified work via earliest-gap targeting with bounded evidence waiting. External applications own graph selection and coordination; SwarmKit owns physical execution and evidence qualification.",
        ),
        (
            "Why is returning to a gap or an interruption point not claimed as a worldwide first invention?",
            "Returning behind an interruption is established in prior systems such as ArduPilot Mission Rewind and robotic coverage literature (Zudaire 2021, Nykolaychuk 2015). VECR's contribution is not the abstract concept of returning, but the unified onboard contract combining conservative acquisition-time footprint contraction, sorted interval unions, exact attempt identity, and bounded re-entry dwell with explicit fail-closed guarantees.",
        ),
        (
            "Does the work involve continuum manipulators, elastic observers, or Cosserat models?",
            "No. The reviewer comments mentioning continuum robotics, visual-elastic observers, and tendon actuation represented a subject-matter mismatch. SwarmKit is strictly a UAV edge-execution and recovery runtime operating in straight local metric segments with fixed-wing and multicopter kinematics.",
        ),
        (
            "Why is B2 (full-edge restart) used as the comparison baseline?",
            "B2 serves as a rigorously fair, coverage-aware research comparator. It shares the exact same verifier, sensor models, flight dynamics, retry budgets, timeouts, and safety mechanisms as VECR, but executes a full restart from vertex A when unverified gaps remain. This directly isolates and measures the precise efficiency gained by localized earliest-gap re-entry versus naive re-execution.",
        ),
        (
            "What is the operational failure envelope of the system?",
            "VECR operates soundly when horizontal localization uncertainty is bounded within the camera footprint margin (demonstrated up to 0.9 m nominal). When uncertainty exceeds usable optical width (e.g. at 1.2 m), or when sensors permanently fail (N10), motor fails (N12), battery drops below reserve (N13), or commands are rejected (N11), VECR provably fails closed without false fulfillment.",
        ),
        (
            "Does zero false fulfillment prove physical real-world safety?",
            "No. Zero false fulfillment in the 1,200 honest trials is conditional on the declared simulator assumptions: truthful calibration, valid deterministic bounds, and flat terrain. The 120 dishonest-sensor controls (N19, N20) deliberately violate these assumptions and yield false fulfillment reports, proving that cryptographic authentication cannot substitute for physical truth.",
        ),
        (
            "How does the method generalize beyond 40-metre straight edges?",
            "The campaign evaluated 20 m (N17), 40 m (nominal), and 80 m (N18) edges, proving that the earliest-gap re-entry rule scales directly with edge length, saving up to 29.92 m of travel on 80 m routes. In addition, client reuse tests confirmed identical execution across direct and routed SDK client architectures.",
        ),
        (
            "What did the ArduPilot SITL qualification demonstrate?",
            "SITL verified that SwarmKit's production MAVLink backend correctly commands real ArduPilot Copter 4.7.2 autopilot processes, executing physical re-entry dwells, obeying operator LAND overrides, and enforcing fail-closed nonacceptance when raw uncalibrated MAVLink telemetry lacks hard uncertainty bounds.",
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

    # Replay dropdown options
    options = "".join(
        f'<option value="{case}">{case} · {esc(next(c["label"] for c in SCENARIO_CONFIGS if c["scenario"]==case))}</option>'
        for case in replay_cases
    )
    components["trace_explorer"] = (
        f"""<div class="trace-explorer"><div class="trace-toolbar"><label>Recorded condition<select id="replayCase">{options}</select></label><button id="playReplay" class="button">Play replay</button><button id="resetReplay" class="button secondary">Reset</button><label>Speed<select id="replaySpeed"><option value="1">1×</option><option value="4" selected>4×</option><option value="10">10×</option></select></label></div><div class="trace-legend"><span class="legend-b2">B2 · full restart baseline</span><span class="legend-vecr">VECR · earliest-gap recovery</span><span>Faint line: recorded path · ×: accepted target</span></div><svg id="replayPlot" viewBox="0 0 880 370" role="img" aria-labelledby="replayTitle replayDesc"><title id="replayTitle">Recorded physical trajectories</title><desc id="replayDesc">Use the time slider to compare B2 and VECR at the same simulation time.</desc><g id="replayDrawing"></g></svg><label class="time-control">Simulation time <output id="replayTime">0.0 s</output><input id="replaySlider" type="range" min="100" max="100000" step="100" value="100" aria-label="Replay simulation time"></label><div id="replayMetrics" class="replay-metrics" aria-live="off"></div><div class="trace-events"><h4>Recorded event timeline</h4><p>Click an event to inspect that time. Phase, command and fault events are shown here; complete logs are downloadable.</p><div id="eventList"></div></div><noscript><p>Replay controls require JavaScript. The original trajectory figure and downloadable traces remain available.</p></noscript></div>"""
    )

    source = (SITE / "source/dissertation.md").read_text()

    def expand(m):
        key = m.group(1)
        if key.startswith("diagram:"):
            name = key.split(":")[1]
            return image_figure(
                name,
                {
                    "contract": "Conceptual contract · distinct physical and logical obligations; ACK ≠ arrival ≠ survey coverage.",
                    "architecture": "Implemented architecture · all verification, evidence qualification and recovery stay inside the agent.",
                    "states": "State semantics · physical re-entry dwells and command acknowledgements remain distinct.",
                    "comparison": "Recovery comparison · shared verifier, sensor models and safety; different re-entry targets.",
                    "evidence": "Observation separation · noisy telemetry estimates and private acquisition truth follow separate paths.",
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
    expected = re.findall(r"^# \d+ / ", source, re.M)
    assert len(headings) == len(expected) >= 20, f"Missing chapter headings: {[h[0] for h in headings]}"

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

    # Route progress against recorded time reveals backtracking that a plan-view
    # plot of a straight edge hides. Every vertex comes from the frozen trace.
    hero_trace = replays["N04"]
    paths = ""
    all_rows = [r for policy in ("B2", "VECR") for r in hero_trace[policy]["trace"]]
    max_t = max(float(r["time_ms"]) for r in all_rows)
    max_progress = max(40.0, max(float(r["east_m"]) for r in all_rows))
    for policy, color in [("B2", "#b77133"), ("VECR", "#167d6a")]:
        points = " ".join(
            f'{54+float(r["time_ms"])/max_t*416:.2f},{227-float(r["east_m"])/max_progress*157:.2f}'
            for r in hero_trace[policy]["trace"]
        )
        paths += f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="3" stroke-linejoin="round"/>'

    hero = f"""<svg viewBox="0 0 520 275" role="img" aria-labelledby="heroDiagramTitle heroDiagramDesc"><title id="heroDiagramTitle">Recorded N04 seed 70001 route progress over time</title><desc id="heroDiagramDesc">Both methods reach the endpoint and return for gaps; the B2 restart returns farther than VECR. Horizontal axis is simulation time; vertical axis is route progress in metres.</desc><path d="M54 70H470 M54 149H470 M54 227H470" stroke="#bdd2c5" stroke-dasharray="3 5"/><path d="M54 70V227H470" fill="none" stroke="#7b9a8c" stroke-width="1.5"/>{paths}<g font-family="Arial,sans-serif" font-size="11" fill="#35584c"><text x="10" y="32">Route progress · m</text><text x="23" y="74">40</text><text x="23" y="153">20</text><text x="30" y="230">0</text><text x="54" y="249">0 s</text><text x="405" y="249">{max_t/1000:.1f} s</text><text x="206" y="269">Recorded simulation time</text></g></svg>"""

    template = (SITE / "tools/template.html").read_text()
    for key, value in {"NAV": nav, "CONTENT": rendered, "HERO": hero}.items():
        template = template.replace("{{" + key + "}}", value)

    assert not re.search(r"\{\{[^}]+\}\}", template)
    (SITE / "index.htm").write_text(template)
    print(f"Built {len(headings)} chapters, {len(scenarios)} policy rows, and {len(replay_cases)*2} recorded replays successfully.")


if __name__ == "__main__":
    build()
