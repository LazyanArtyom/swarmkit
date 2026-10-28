"""Current v6 evidence components; historical v5.1 components stay separately labeled."""

import csv
import hashlib
import html
import json
import shutil
import zipfile


def components(site, root, table, figure):
    study = root / "results/vecr/agent_campaign_v6"
    final = study / "confirmatory_v6_1"
    dest = site / "assets/evidence/v6"
    dest.mkdir(parents=True, exist_ok=True)
    copied = {}

    def copy(source, name):
        target = dest / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        copied[str(target.relative_to(site))] = dict(
            source=str(source.relative_to(root)),
            sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        )

    for name in ["manifest.json", "per_attempt.csv", "SHA256SUMS"]:
        if (final / name).exists():
            copy(final / name, name)
    for name in [
        "summary.json",
        "condition_summary.csv",
        "paired_comparisons.csv",
        "RESULTS.md",
    ]:
        copy(final / "analysis" / name, name)
    for name in [
        "FINAL_VALIDATION_V6.md",
        "RECOVERY_MODEL_V6.md",
        "SIMULATION_PROTOCOL_V6.md",
        "NOVELTY_AND_DEFENSE_ASSESSMENT_V6_2026-10-28.md",
        "SITL_PROTOCOL_V6.md",
        "SITL_VALIDATION_V6.md",
    ]:
        if (root / "docs/vecr" / name).exists():
            copy(root / "docs/vecr" / name, name)
    copy(study / "client_reuse/reuse_comparison.json", "client_reuse.json")
    for name in ["completion_matrix", "paired_distance"]:
        for ext in ["svg", "png", "pdf"]:
            copy(final / "analysis" / f"{name}.{ext}", f"{name}.{ext}")

    def read(path):
        with path.open() as f:
            return list(csv.DictReader(f))

    data = read(final / "analysis/condition_summary.csv")
    raw = read(final / "per_attempt.csv")
    assert len(raw) == 5940 and not any(r["infrastructure_error"] for r in raw)
    assert (
        sum(
            int(r["false_complete"]) for r in raw if r["scenario"] not in ["N19", "N20"]
        )
        == 0
    )
    false_controls = sum(
        int(r["false_complete"]) for r in raw if r["scenario"] in ["N19", "N20"]
    )
    lookup = {(r["scenario"], r["policy"]): r for r in data}
    policies = ["B2", "D", "R2", "R8", "P", "G", "GW", "E0", "E"]
    matrix = []
    for scenario in sorted({r["scenario"] for r in data}):
        r = lookup[scenario, "E"]
        matrix.append(
            [html.escape(scenario + " · " + r["fault"])]
            + [lookup[scenario, p]["reported_complete"] + "/30" for p in policies]
        )
    counts = table(
        ["Condition"] + policies,
        matrix,
        "v6: reported completion in every condition. N19/N20 are dishonest-sensor controls, so their reports are false fulfillment.",
    )
    effects = read(final / "analysis/paired_comparisons.csv")
    contrasts = []
    for scenario, treatment, baseline in [
        ("N01", "GW", "G"),
        ("N01", "E", "GW"),
        ("N04", "E", "GW"),
        ("N06", "E", "GW"),
        ("N07", "E", "GW"),
        ("N07", "P", "B2"),
        ("N09", "E", "GW"),
    ]:
        r = next(
            r
            for r in effects
            if (r["scenario"], r["treatment"], r["baseline"])
            == (scenario, treatment, baseline)
        )
        val = lambda key: f"{float(r[key]):+.5f}"
        contrasts.append(
            [
                scenario,
                treatment + " − " + baseline,
                r["joint_success_pairs"],
                val("successful_distance_delta_m"),
                f"[{val('successful_distance_delta_m_ci_low')}, {val('successful_distance_delta_m_ci_high')}]",
                val("successful_time_delta_s"),
            ]
        )
    contrast = table(
        [
            "Condition",
            "Contrast",
            "Joint successes",
            "Distance Δ, m",
            "95% paired CI, m",
            "Time Δ, s",
        ],
        contrasts,
        "v6: pointwise paired bootstrap intervals, 10,000 resamples. Positive distance means more travel. Cost is conditional on joint success.",
    )
    traces = dest / "representative-traces.zip"
    with zipfile.ZipFile(traces, "w", zipfile.ZIP_DEFLATED) as z:
        for scenario in ["N01", "N04", "N06", "N07", "N11"]:
            for policy in ["B2", "GW", "E"]:
                path = final / "trials" / f"{scenario}-30001-{policy}-direct"
                for p in sorted(path.rglob("*")):
                    if p.is_file():
                        z.write(p, str(p.relative_to(final)))
    copied[str(traces.relative_to(site))] = dict(
        source="selected unmodified v6 primary trial files",
        sha256=hashlib.sha256(traces.read_bytes()).hexdigest(),
    )
    links = [
        ("FINAL_VALIDATION_V6.md", "Full digital validation report"),
        ("per_attempt.csv", "All 5,940 primary results"),
        ("condition_summary.csv", "All 198 condition/policy rows"),
        ("paired_comparisons.csv", "Paired differences and confidence intervals"),
        ("manifest.json", "Frozen assignments and software identity"),
        ("SHA256SUMS", "Primary artifact hash inventory"),
        ("RECOVERY_MODEL_V6.md", "Conditional rule and derivation"),
        ("SIMULATION_PROTOCOL_V6.md", "Preregistered digital protocol"),
        (
            "NOVELTY_AND_DEFENSE_ASSESSMENT_V6_2026-10-28.md",
            "Updated literature and novelty assessment",
        ),
        ("client_reuse.json", "100 matched client pairs"),
        ("representative-traces.zip", "15 unmodified primary trial records"),
        ("completion_matrix.pdf", "Export completion figure (PDF)"),
        ("paired_distance.pdf", "Export comparison figure (PDF)"),
    ]
    downloads = (
        '<div class="download-grid">'
        + "".join(
            f'<a href="assets/evidence/v6/{n}" download><span>↓</span><div><strong>{html.escape(label)}</strong><small>{n}</small></div></a>'
            for n, label in links
            if (dest / n).exists()
        )
        + "</div>"
    )
    downloads += '<p class="source-note">All raw trials and unsuccessful pilots: <a href="../results/vecr/agent_campaign_v6/">repository-local digital collection</a>. Historical research is preserved and never pooled. Full raw-directory links require the surrounding repository.</p>'
    sitl = root / "results/vecr/sitl_v6/confirmatory"
    sitl_html = "<p>SITL qualification is in progress; no final measurement is asserted here.</p>"
    if (sitl / "analysis/validation.json").exists():
        validation = json.loads((sitl / "analysis/validation.json").read_text())
        assert (
            validation["passes"] == 24
            and not validation["errors"]
            and not validation["infrastructure_failures"]
        )
        sr = read(sitl / "analysis/reconstructed.csv")
        copy(sitl / "analysis/reconstructed.csv", "sitl_reconstructed.csv")
        copy(sitl / "analysis/validation.json", "sitl_validation.json")
        copy(sitl / "manifest.json", "sitl_manifest.json")
        copy(sitl / "analysis/condition_summary.csv", "sitl_condition_summary.csv")
        for extension in ["svg", "pdf", "png"]:
            copy(
                sitl / "analysis" / ("sitl_traces." + extension),
                "sitl_traces." + extension,
            )
        body = []
        for nav, fault in [
            ("raw_mavlink", "none"),
            ("synthetic_bounded", "none"),
            ("synthetic_bounded", "delay"),
            ("synthetic_bounded", "multi_gap"),
            ("synthetic_bounded", "command_rejected"),
            ("synthetic_bounded", "land_override"),
        ]:
            for policy in ["GW", "E"]:
                selected = [
                    r
                    for r in sr
                    if (r["navigation"], r["fault"], r["policy"])
                    == (nav, fault, policy)
                ]
                assert len(selected) == 2
                body.append(
                    [
                        html.escape(nav + " / " + fault),
                        policy,
                        f"{sum(int(r['reported_complete']) for r in selected)}/2",
                        f"{sum(int(r['sampled_truth_complete']) for r in selected)}/2",
                        f"{sum(float(r['distance_m']) for r in selected)/2:.2f}",
                        f"{sum(float(r['duration_s']) for r in selected)/2:.2f}",
                        "/".join(r["phase"] for r in selected),
                    ]
                )
        sitl_html = table(
            [
                "Observation tier / condition",
                "Policy",
                "Reported",
                "Sampled truth",
                "Mean travel, m",
                "Mean duration, s",
                "Final phases",
            ],
            body,
            "24 ArduPilot SITL qualifications; two repeats per policy/condition. Means describe these flights, not a powered population comparison.",
        )
        sitl_html += '<p>Measured records: <a href="assets/evidence/v6/sitl_reconstructed.csv" download>all 24 reconstructed trials</a> · <a href="assets/evidence/v6/sitl_manifest.json" download>frozen manifest</a> · <a href="assets/evidence/v6/sitl_validation.json" download>independent checks</a> · <a href="assets/evidence/v6/SITL_VALIDATION_V6.md">qualification report</a> · <a href="../results/vecr/sitl_v6/">complete logs and pilots (repository)</a>.</p>'
        sitl_html += figure(
            "sitl_traces",
            "Actual ArduPilot seed 61001 trajectories and accepted-coverage histories. Navigation bounds and camera observations are synthetic.",
            "evidence/v6",
        )
    result = dict(
        v6_rule=figure(
            "recovery_v6",
            "Schematic: accepted history, conditional target and bounded waiting. Planning never creates sensing evidence.",
            "diagrams",
        ),
        v6_matrix=counts,
        v6_contrasts=contrast,
        v6_downloads=downloads,
        v6_sitl=sitl_html,
        v6_false_controls=str(false_controls),
        v6_figures=figure(
            "completion_matrix",
            "v6: reported completion; dishonest sensor controls are explicitly marked.",
            "evidence/v6",
        )
        + figure(
            "paired_distance",
            "v6: paired distance differences under joint success, including the disadvantage of E against GW.",
            "evidence/v6",
        ),
    )
    return result, copied
