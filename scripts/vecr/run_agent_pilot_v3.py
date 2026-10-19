#!/usr/bin/env python3
"""Record two real SDK→Agent→SimBackend VECR pilot attempts.

This is an integration pilot, not the 21-scenario comparative experiment.
It refuses to replace a previous run or operate on uncommitted tracked source.
"""

from __future__ import annotations

import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "results/vecr/agent_pilot_v3"
TEST_BINARY = ROOT / "build/mac-release/tests/swarmkit_tests"
PHASE = {0: "MOVING", 1: "RECOVERING", 2: "FULFILLED", 3: "INTERRUPTED", 4: "UNKNOWN"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def plot(rows: list[tuple[str, list[dict[str, str]]]]) -> str:
    colors = {"agent_camera_gap": "#136f63", "agent_motor_failure": "#b14b38"}
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="760" height="410" viewBox="0 0 760 410">',
        '<rect width="760" height="410" fill="white"/>',
        '<path d="M70 25 V345 H720" fill="none" stroke="#222" stroke-width="2"/>',
        '<text x="340" y="395" font-family="sans-serif" font-size="15">Physical distance traveled (m)</text>',
        '<text x="18" y="200" transform="rotate(-90 18 200)" font-family="sans-serif" font-size="15">Reported surveyed length (m)</text>',
    ]
    max_distance = max(float(series[-1]["physical_distance_m"]) for _, series in rows)
    x_limit = max(25.0, max_distance * 1.05)
    for distance in range(0, int(x_limit) + 1, 10):
        x = 70 + distance / x_limit * 650
        parts.append(f'<text x="{x:.1f}" y="365" text-anchor="middle" font-family="sans-serif" font-size="12">{distance}</text>')
    for surveyed in (0, 5, 10, 15, 20):
        y = 345 - surveyed / 20 * 320
        parts.append(f'<text x="57" y="{y + 4:.1f}" text-anchor="end" font-family="sans-serif" font-size="12">{surveyed}</text>')
    for name, series in rows:
        points = " ".join(
            f'{70 + float(row["physical_distance_m"]) / x_limit * 650:.2f},'
            f'{345 - float(row["reported_survey_covered_m"]) / 20 * 320:.2f}'
            for row in series
        )
        parts.append(f'<polyline points="{points}" fill="none" stroke="{colors[name]}" stroke-width="2"/>')
    parts.extend([
        '<text x="440" y="45" font-family="sans-serif" font-size="13" fill="#136f63">Camera gap: repair completed</text>',
        '<text x="440" y="65" font-family="sans-serif" font-size="13" fill="#b14b38">Motor failure: interrupted</text>',
        '</svg>',
    ])
    return "\n".join(parts) + "\n"


def main() -> int:
    if OUTPUT.exists():
        raise SystemExit(f"refusing to overwrite frozen pilot directory: {OUTPUT}")
    if git("status", "--porcelain", "--untracked-files=no"):
        raise SystemExit("commit tracked source changes before recording a versioned pilot")
    if not TEST_BINARY.is_file():
        raise SystemExit(f"build the test binary first: {TEST_BINARY}")
    (OUTPUT / "raw").mkdir(parents=True)
    (OUTPUT / "tables").mkdir()
    (OUTPUT / "figures").mkdir()
    command = [str(TEST_BINARY), "[sim-backend]", "--reporter", "compact"]
    env = os.environ.copy()
    env["SWARMKIT_VECR_V3_TRACE_DIR"] = str(OUTPUT / "raw")
    completed = subprocess.run(command, cwd=ROOT, env=env, text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               check=False)
    (OUTPUT / "execution.log").write_text(completed.stdout)
    cases = []
    traces = []
    for name in ("agent_camera_gap", "agent_motor_failure"):
        path = OUTPUT / "raw" / f"{name}.csv"
        if not path.exists():
            raise RuntimeError(f"missing trace {path}; test exit code {completed.returncode}")
        with path.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        if not rows:
            raise RuntimeError(f"empty trace {path}")
        last = rows[-1]
        traces.append((name, rows))
        cases.append({
            "scenario": name,
            "attempts": 1,
            "terminal_phase": PHASE[int(last["reported_phase"])],
            "truth_covered_cells": int(last["truth_covered_cells"]),
            "truth_cells": 80,
            "reported_survey_covered_m": float(last["reported_survey_covered_m"]),
            "physical_distance_m": float(last["physical_distance_m"]),
            "recovery_attempts": int(last["recovery_attempts"]),
            "motor_failure_injected": bool(int(last["motor_failure_injected"])),
            "trace_rows": len(rows),
        })
    table = OUTPUT / "tables" / "observed_attempts.csv"
    with table.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=cases[0].keys())
        writer.writeheader()
        writer.writerows(cases)
    (OUTPUT / "figures" / "survey_vs_physical_distance.svg").write_text(plot(traces))
    (OUTPUT / "README.md").write_text(
        "# VECR v3 agent-backed integration pilot\n\n"
        "Two executed attempts use Client SDK → AgentService → EdgeExecutionRuntime → "
        "SimBackend in manual-clock mode. The private `SimBackendControl::Truth` channel "
        "drives the independent raster check and physical-distance measurement. The "
        "synthetic camera producer uses simulator truth to model calibrated pose; this "
        "does not validate a real camera. `reported_phase` encodes MOVING=0, "
        "RECOVERING=1, FULFILLED=2, INTERRUPTED=3, UNKNOWN=4.\n\n"
        "The camera-gap attempt tests completed onboard re-entry and continuation. "
        "The motor-failure attempt records a failed recovery. These are one run per "
        "scenario, with no fair B2 comparator or uncertainty interval. They are not "
        "the planned 21-scenario campaign, ArduPilot SITL, or hardware evidence. "
        "The results support only the observed software integration behavior.\n"
    )
    files = sorted(p for p in OUTPUT.rglob("*") if p.is_file())
    manifest = {
        "protocol": "agent-pilot-v3",
        "evidence_class": "agent-backed simulation",
        "source_revision": git("rev-parse", "HEAD"),
        "test_binary_sha256": sha256(TEST_BINARY),
        "command": command,
        "test_exit_code": completed.returncode,
        "cases": cases,
        "sha256": {str(p.relative_to(OUTPUT)): sha256(p) for p in files},
    }
    (OUTPUT / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(cases, indent=2))
    return completed.returncode


if __name__ == "__main__":
    sys.exit(main())
