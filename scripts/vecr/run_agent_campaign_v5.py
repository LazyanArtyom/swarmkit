#!/usr/bin/env python3
"""Frozen paired v5 assignment, actual subprocess trials, no trial replacement."""

import argparse, concurrent.futures, csv, datetime, gzip, hashlib, json, os, platform, shutil, subprocess, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PRIMARY = [
    dict(scenario="M00", fault="none"),
    dict(scenario="M01", fault="gust", **{"wind-end": 6.2}),
    dict(scenario="M02", fault="crosswind"),
    dict(scenario="M03", fault="departure"),
    dict(scenario="M04", fault="visible", cross=14),
    dict(scenario="M05", fault="hidden", cross=5),
    dict(scenario="M06", fault="none", bound=0.9, delay=0.4),
    dict(scenario="M07", fault="metadata_gap"),
    dict(scenario="M08", fault="gnss_outage"),
    dict(scenario="M09", fault="estimator_invalid"),
    dict(scenario="M10", fault="unbounded"),
    dict(scenario="M11", fault="stale"),
    dict(scenario="M12", fault="bad_calibration"),
    dict(scenario="M13", fault="declared_angle"),
    dict(scenario="M14", fault="false_frame"),
    dict(scenario="M15", fault="undeclared_angle"),
    dict(scenario="M16", fault="command_rejected"),
    dict(scenario="M17", fault="motor_failure"),
    dict(scenario="M18", fault="no_repair", attempts=0),
    dict(scenario="M19", fault="low_battery"),
    dict(scenario="M20", fault="none", period=2),
]
SENSITIVITY = []
for factor, values in [
    ("length", [20, 80]),
    ("along", [1, 3]),
    ("period", [0.2, 1]),
    ("bound", [0.1, 1]),
    ("wind", [2, 4]),
    ("attempts", [1, 3]),
    ("cross", [5, 14]),
]:
    for value in values:
        row = dict(scenario=f"X{len(SENSITIVITY):02d}", fault="hidden", cross=5)
        row[factor] = value
        if factor == "length":
            row["timeout"] = 180 if value == 80 else 90
        row["factor"] = factor
        SENSITIVITY.append(row)
for begin, end in [(6, 10), (26, 34), (12, 28), (18, 22)]:
    SENSITIVITY.append(
        dict(
            scenario=f"X{len(SENSITIVITY):02d}",
            fault="metadata_gap",
            factor="gap",
            **{"gap-begin": begin, "gap-end": end},
        )
    )
DEFAULTS = dict(
    length=40,
    along=2,
    cross=7,
    bound=0.3,
    wind=3,
    **{"wind-begin": 6, "wind-end": 10, "gap-begin": 16, "gap-end": 24},
    period=0.5,
    delay=0.2,
    attempts=2,
    timeout=90,
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def configurations(mode):
    matrix = PRIMARY + ([] if mode == "benchmark" else SENSITIVITY)
    if mode == "benchmark":
        matrix = [PRIMARY[i] for i in (0, 3, 5, 8)]
    return [{**DEFAULTS, **r} for r in matrix]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--mode", choices=["pilot", "final", "benchmark"], default="pilot")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--seed-start", type=int)
    ap.add_argument("--seeds", type=int)
    ap.add_argument("--only", default="")
    ap.add_argument(
        "--binary",
        type=Path,
        default=ROOT / "build/mac-release/tests/swarmkit_vecr_v5_trial",
    )
    a = ap.parse_args()
    a.output = a.output.resolve()
    a.binary = a.binary.resolve()
    a.output.mkdir(parents=True, exist_ok=False)
    source = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
    ).strip()
    dirty = subprocess.check_output(
        ["git", "status", "--porcelain"], cwd=ROOT, text=True
    )
    tracked_dirty = subprocess.check_output(
        ["git", "diff", "--name-only"], cwd=ROOT, text=True
    )
    if a.mode == "final" and any(
        not x.startswith("results/") for x in tracked_dirty.splitlines()
    ):
        raise RuntimeError("freeze source before final run")
    assignments = []
    for cfg in configurations(a.mode):
        if a.only and cfg["scenario"] not in a.only.split(","):
            continue
        count = a.seeds or (100 if cfg["scenario"].startswith("M") else 30)
        if a.mode == "pilot":
            count = a.seeds or 3
        if a.mode == "benchmark":
            count = a.seeds or 10
        start = a.seed_start or (
            10001 if a.mode == "pilot" else 20001 if a.mode == "benchmark" else 1
        )
        for seed in range(start, start + count):
            # Common random numbers; no policy identifier enters perturbation/fault RNG.
            for policy in (["B2", "P"] if seed % 2 else ["P", "B2"]):
                assignments.append({**cfg, "seed": seed, "policy": policy})
    manifest = dict(
        protocol="v5",
        mode=a.mode,
        status="RUNNING",
        source_revision=source,
        source_status=dirty,
        binary_sha256=sha(a.binary),
        runner_sha256=sha(__file__),
        protocol_sha256=sha(ROOT / "docs/vecr/SIMULATION_PROTOCOL_V5.md"),
        created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        host=platform.platform(),
        python=platform.python_version(),
        workers=a.workers,
        assignments=assignments,
    )
    (a.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (a.output / "source.patch").write_text(
        subprocess.check_output(["git", "diff"], cwd=ROOT, text=True)
    )
    (a.output / "environment.txt").write_text(
        subprocess.check_output(
            [
                "sh",
                "-c",
                "uname -a; sw_vers; sysctl -n machdep.cpu.brand_string; clang --version; cmake --version",
            ],
            text=True,
        )
    )
    for f in ("CMakeCache.txt", "compile_commands.json"):
        shutil.copyfile(ROOT / "build/mac-release" / f, a.output / f)
    (a.output / "trials").mkdir()

    def trial(cfg):
        name = f"{cfg['scenario']}-{cfg['seed']:05d}-{cfg['policy']}"
        out = a.output / "trials" / name
        cmd = [str(a.binary), "--output", str(out)]
        for k, v in cfg.items():
            if k != "factor":
                cmd += ["--" + k, str(v)]
        begin = time.monotonic()
        proc = subprocess.run(
            cmd, cwd=ROOT, capture_output=True, text=True, timeout=180
        )
        (a.output / "trials" / f"{name}.stdout.txt").write_text(
            proc.stdout + proc.stderr
        )
        row = dict(
            cfg,
            run_id=name,
            process_wall_s=time.monotonic() - begin,
            infrastructure_error="",
        )
        if proc.returncode or not (out / "attempt.csv").exists():
            row["infrastructure_error"] = (
                f"exit={proc.returncode}: {proc.stderr[-1000:]}"
            )
        else:
            row.update(next(csv.DictReader((out / "attempt.csv").open())))
        # Retain complete traces losslessly; gzip mtime=0 makes the compression reproducible.
        for path in sorted(out.glob("*.csv")):
            if path.name == "attempt.csv":
                continue
            with path.open("rb") as src, (path.with_suffix(".csv.gz")).open(
                "wb"
            ) as raw:
                with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as dst:
                    shutil.copyfileobj(src, dst)
            path.unlink()
        return row

    rows = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=a.workers) as pool:
        futures = {pool.submit(trial, c): c for c in assignments}
        with (a.output / "completed.jsonl").open("w") as journal:
            for future in concurrent.futures.as_completed(futures):
                try:
                    row = future.result()
                except Exception as exc:
                    row = {**futures[future], "infrastructure_error": repr(exc)}
                rows.append(row)
                journal.write(json.dumps(row) + "\n")
                journal.flush()
                if len(rows) % 20 == 0 or row["infrastructure_error"]:
                    print(
                        f'{len(rows)}/{len(assignments)} {row["scenario"]} {row["seed"]} {row["policy"]} {row.get("phase", "ERROR")}',
                        flush=True,
                    )
    fields = sorted(set().union(*(r.keys() for r in rows)))
    with (a.output / "per_attempt.csv").open("w") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(
            sorted(rows, key=lambda r: (r["scenario"], int(r["seed"]), r["policy"]))
        )
    manifest["status"] = (
        "INFRASTRUCTURE_FAILURE"
        if any(r["infrastructure_error"] for r in rows)
        else "COMPLETE"
    )
    manifest["completed_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    manifest["completed_rows"] = len(rows)
    (a.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(manifest["status"], len(rows), flush=True)


if __name__ == "__main__":
    main()
