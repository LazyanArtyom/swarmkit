#!/usr/bin/env python3
"""Frozen paired v7 assignment, actual subprocess trials, no trial replacement."""

import argparse, concurrent.futures, csv, datetime, gzip, hashlib, json, os, platform, shutil, subprocess, time, tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
POLICIES = ["B2", "VECR"]
PRIMARY = [
    dict(scenario="N00", fault="none"),
    dict(scenario="N01", fault="none", delay=2.5),
    dict(scenario="N02", fault="reordered"),
    dict(scenario="N03", fault="metadata_gap"),
    dict(scenario="N04", fault="multi_gap"),
    dict(scenario="N05", fault="changing_uncertainty"),
    dict(scenario="N06", fault="uncertainty_gap"),
    dict(scenario="N07", fault="departure"),
    dict(scenario="N08", fault="visible", cross=14),
    dict(scenario="N09", fault="hidden", cross=5),
    dict(scenario="N10", fault="persistent_sensor"),
    dict(scenario="N11", fault="command_rejected"),
    dict(scenario="N12", fault="motor_failure"),
    dict(scenario="N13", fault="low_battery"),
    dict(scenario="N14", fault="no_repair", attempts=0),
    dict(scenario="N15", fault="none", delay=4),
    dict(scenario="N16", fault="none", along=1, period=1.5),
    dict(scenario="N17", fault="multi_gap", length=20),
    dict(scenario="N18", fault="multi_gap", length=80, timeout=160),
    dict(scenario="N19", fault="false_frame"),
    dict(scenario="N20", fault="undeclared_angle"),
    dict(scenario="N21", fault="multi_gap", timeout=30),
]
DEFAULTS = dict(
    length=40,
    along=2,
    cross=7,
    bound=0.3,
    wind=3,
    **{
        "wind-begin": 6,
        "wind-end": 10,
        "gap-begin": 16,
        "gap-end": 24,
        "max-age": 3,
        "wait": 3,
        "client-kind": "direct",
    },
    period=0.5,
    delay=0.2,
    attempts=2,
    timeout=90,
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def configurations(mode):
    return [{**DEFAULTS, **r} for r in PRIMARY]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--mode", choices=["pilot", "final", "reuse"], default="pilot")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--seed-start", type=int)
    ap.add_argument("--seeds", type=int)
    ap.add_argument("--only", default="")
    ap.add_argument("--policies", default=",".join(POLICIES))
    ap.add_argument(
        "--binary",
        type=Path,
        default=ROOT / "build/mac-release/tests/swarmkit_vecr_trial",
    )
    ap.add_argument("--baseline-binary", type=Path, default=ROOT / "build/mac-release/tests/swarmkit_vecr_baseline")
    a = ap.parse_args()
    a.output = a.output.resolve()
    a.binary = a.binary.resolve()
    binaries = {"VECR": a.binary, "B2": a.baseline_binary.resolve()}
    for policy, binary in binaries.items():
        description = json.loads(subprocess.check_output([str(binary), "--describe"], text=True))
        if description != {"protocol": "v7", "policy": policy}:
            raise RuntimeError("binary does not implement its assigned method")
    a.output.mkdir(parents=True, exist_ok=False)
    source = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
    ).strip()
    dirty = subprocess.check_output(
        ["git", "status", "--porcelain"], cwd=ROOT, text=True
    )
    source_paths = ["CMakeLists.txt", "cmake", "include", "src", "proto", "apps", "tests", "scripts/vecr", "docs/vecr/SIMULATION_PROTOCOL.md", "docs/vecr/METHOD.md"]
    source_changes = subprocess.check_output(["git", "diff", "HEAD", "--name-only", "--", *source_paths], cwd=ROOT, text=True)
    source_changes += subprocess.check_output(["git", "ls-files", "--others", "--exclude-standard", "--", *source_paths], cwd=ROOT, text=True)
    if a.mode in ("final", "reuse") and source_changes.strip():
        raise RuntimeError("commit experiment sources before frozen run: " + source_changes)
    assignments = []
    for cfg in configurations(a.mode):
        if a.only and cfg["scenario"] not in a.only.split(","):
            continue
        count = a.seeds or 30
        if a.mode == "pilot":
            count = a.seeds or 2
        if a.mode == "reuse":
            count = a.seeds or 10
        start = a.seed_start or (
            71001 if a.mode == "pilot" else 72001 if a.mode == "reuse" else 70001
        )
        for seed in range(start, start + count):
            # Common random numbers; no policy identifier enters perturbation/fault RNG.
            policies = a.policies.split(",")
            if any(p not in POLICIES for p in policies):
                raise ValueError("unknown policy")
            offset = seed % len(policies)
            for policy in policies[offset:] + policies[:offset]:
                for client_kind in (
                    ["direct", "routed"] if a.mode == "reuse" else ["direct"]
                ):
                    assignments.append(
                        {
                            **cfg,
                            "seed": seed,
                            "policy": policy,
                            "client-kind": client_kind,
                        }
                    )
    if a.mode == "final" and (
        len(assignments) != 1320
        or a.only
        or a.policies.split(",") != POLICIES
        or {x["seed"] for x in assignments} != set(range(70001, 70031))
    ):
        raise RuntimeError(
            "confirmatory assignments must match 22 x 30 x 2 preregistration"
        )
    manifest = dict(
        protocol="v7",
        mode=a.mode,
        status="RUNNING",
        source_revision=source,
        source_status=dirty,
        binary_sha256={p: sha(b) for p,b in binaries.items()},
        runner_sha256=sha(__file__),
        protocol_sha256=sha(ROOT / "docs/vecr/SIMULATION_PROTOCOL.md"),
        model_sha256=sha(ROOT / "docs/vecr/METHOD.md"),
        created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        host=platform.platform(),
        python=platform.python_version(),
        workers=a.workers,
        assignments=assignments,
    )
    (a.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (a.output / "source.patch").write_text(subprocess.check_output(["git", "diff", "HEAD", "--", *source_paths], cwd=ROOT, text=True))
    binary_dir = Path(tempfile.mkdtemp(prefix="vecr-v7-frozen-"))
    for policy, source_binary in list(binaries.items()):
        frozen = binary_dir / source_binary.name
        shutil.copy2(source_binary, frozen)
        binaries[policy] = frozen

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
        name = (
            f"{cfg['scenario']}-{cfg['seed']:05d}-{cfg['policy']}-{cfg['client-kind']}"
        )
        out = a.output / "trials" / name
        cmd = [str(binaries[cfg["policy"]]), "--output", str(out)]
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
