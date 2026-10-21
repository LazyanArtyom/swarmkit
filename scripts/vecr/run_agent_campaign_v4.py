#!/usr/bin/env python3
"""Run the preregistered agent-backed v4 paired simulator campaign."""

import argparse
import csv
import hashlib
import json
import pathlib
import platform
import shutil
import subprocess
import sys
import time


ROOT = pathlib.Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stream_unit(scenario, seed, stream):
    data = f"v4|{scenario}|{seed}|{stream}".encode()
    return int.from_bytes(hashlib.sha256(data).digest()[:8], "big") / 2**64


def seed_range(value):
    left, right = value.split(":")
    seeds = list(range(int(left), int(right) + 1))
    if not seeds or min(seeds) < 1:
        raise ValueError("seeds must be a nonempty positive inclusive range")
    return seeds


def params(scenario, seed, config):
    gap_center = 10 + (stream_unit(scenario["id"], seed, "gap_position") - .5) * 1.0
    gap_length = 4 + (stream_unit(scenario["id"], seed, "gap_length") - .5) * .6
    wind = scenario.get("wind_north_mps", 0)
    if wind:
        wind *= .96 + .08 * stream_unit(scenario["id"], seed, "wind_strength")
    return {
        "gap_begin_m": gap_center - gap_length / 2,
        "gap_end_m": gap_center + gap_length / 2,
        "wind_north_mps": wind,
        "speed_mps": scenario.get("speed_mps", config["commanded_speed_mps"]),
        "acceleration_mps2": config["acceleration_mps2"],
        "capture_period_ticks": config["capture_period_ticks"],
        "camera_along_half_m": config["camera_along_half_m"],
        "camera_cross_half_m": config["camera_cross_half_m"],
    }


def command(exe, scenario, seed, policy, settings, output):
    cmd = [str(exe), "--scenario", scenario["id"], "--seed", str(seed),
           "--policy", policy, "--fault", scenario["fault"],
           "--output", str(output)]
    for key, value in settings.items():
        cmd += ["--" + key.replace("_", "-"), str(value)]
    return cmd


def read_rows(path):
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def append_rows(path, rows):
    if not rows:
        return
    with path.open("a", newline="") as handle:
        writer = csv.DictWriter(handle, rows[0].keys(), lineterminator="\n")
        if handle.tell() == 0:
            writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=pathlib.Path, required=True)
    parser.add_argument("--mode", choices=("smoke", "full"), required=True)
    parser.add_argument("--seeds", required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--executable", type=pathlib.Path,
                        default=ROOT / "build/mac-release/tests/swarmkit_vecr_agent_trial")
    args = parser.parse_args()
    config_path = args.config.resolve()
    config = json.loads(config_path.read_text())
    seeds = seed_range(args.seeds)
    if args.mode == "full" and seeds != list(range(config["full_seed_begin"],
                                                    config["full_seed_end"] + 1)):
        raise ValueError("full mode requires the exact preregistered seed list")
    exe = args.executable.resolve()
    if not exe.is_file():
        raise FileNotFoundError(f"build trial executable first: {exe}")
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite campaign output: {output}")
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    dirty = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"], cwd=ROOT,
                           capture_output=True, text=True, check=True).stdout.strip()
    if args.mode == "full" and dirty:
        raise RuntimeError("full campaign requires a clean tracked source commit")
    output.mkdir(parents=True, exist_ok=False)
    for name in ("raw", "raw/traces", "raw/trials", "tables", "figures", "reports"):
        (output / name).mkdir()
    shutil.copy2(config_path, output / "protocol.json")
    config_id = sha(config_path)
    manifest = {"study_status": "RUNNING", "mode": args.mode, "seeds": seeds,
                "config_id_sha256": config_id, "binary_sha256": sha(exe),
                "source_revision": commit, "dirty_tree": bool(dirty),
                "host": platform.platform(), "architecture": platform.machine(),
                "python": platform.python_version(), "run_command": sys.argv,
                "compiler": subprocess.check_output(["clang++", "--version"], text=True).splitlines()[0],
                "cmake": subprocess.check_output(["cmake", "--version"], text=True).splitlines()[0],
                "conan": subprocess.check_output(["conan", "--version"], text=True).strip(),
                "rng_streams": ["gap_position", "gap_length", "wind_strength", "gps_jitter"],
                "started_unix_s": time.time(), "attempts_completed": 0}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    comparative = [item for item in config["scenarios"]
                   if item["class"] == "COMPARATIVE"]
    inventory = []
    for scenario in config["scenarios"]:
        inventory.append({"scenario_id": scenario["id"],
                          "class": scenario["class"], "purpose": scenario["purpose"],
                          "fault": scenario.get("fault", ""),
                          "eligible_policies": "B2|P" if scenario in comparative else "",
                          "validation_status": "SCHEDULED" if scenario in comparative
                          else "NOT_IN_PAIRED_CAMPAIGN"})
    append_rows(output / "tables/scenario_inventory.csv", inventory)
    attempts_path = output / "raw/per_attempt.csv"
    events_path = output / "raw/execution_events.csv"
    faults_path = output / "raw/fault_events.csv"
    total = len(comparative) * len(seeds) * 2
    for scenario in comparative:
        for seed in seeds:
            settings = params(scenario, seed, config)
            for policy in ("B2", "P"):
                run_id = f"v4-{args.mode}-{scenario['id']}-{seed:03d}-{policy}"
                trial_dir = output / "raw/trials" / run_id
                cmd = command(exe, scenario, seed, policy, settings, trial_dir)
                started = time.monotonic()
                proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
                wall_s = time.monotonic() - started
                if proc.returncode != 0 or not (trial_dir / "attempt.csv").is_file():
                    row = {"run_id": run_id, "scenario_id": scenario["id"],
                           "seed": seed, "policy": policy, "config_id": config_id,
                           "reported_phase": "INFRASTRUCTURE_ERROR",
                           "infrastructure_error": proc.stderr[-1000:],
                           "wall_time_s": wall_s}
                    append_rows(attempts_path, [row])
                    manifest["study_status"] = "INCOMPLETE"
                    manifest["infrastructure_error"] = row
                    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
                    raise RuntimeError(f"{run_id}: {proc.stderr[-1000:]}")
                rows = read_rows(trial_dir / "attempt.csv")
                if len(rows) != 1:
                    raise RuntimeError(f"{run_id}: expected one attempt row")
                row = {"run_id": run_id, "config_id": config_id,
                       "edge_id": "A-B", "mission_id": "v4-graph-edge",
                       "mission_revision": 4, "fault": scenario["fault"],
                       "wall_time_s": wall_s, "infrastructure_error": "", **rows[0]}
                if row["scenario_id"] != scenario["id"] or int(row["seed"]) != seed or \
                   row["policy"] != policy:
                    raise RuntimeError(f"{run_id}: identity mismatch")
                if scenario["fault"] != "none" and row["requested_fault_realized"] != "1":
                    raise RuntimeError(f"{run_id}: requested fault never realized")
                if row["reported_phase"] == "FULFILLED" and row["false_complete"] == "1":
                    print(f"ADVERSARIAL OUTCOME {run_id}: false completion", flush=True)
                append_rows(attempts_path, [row])
                for event in read_rows(trial_dir / "events.csv"):
                    enriched = {"run_id": run_id, "config_id": config_id, **event}
                    append_rows(events_path, [enriched])
                    if "fault" in event["kind"] or "failure" in event["kind"] or \
                       "rejected" in event["kind"] or "battery" in event["kind"] or \
                       "wind" in event["kind"]:
                        append_rows(faults_path, [enriched])
                if seed <= 3:
                    shutil.move(trial_dir / "trace.csv", output / "raw/traces" / f"{run_id}.csv")
                shutil.rmtree(trial_dir)
                manifest["attempts_completed"] += 1
                if manifest["attempts_completed"] % 20 == 0 or \
                   manifest["attempts_completed"] == total:
                    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
                    print(f"{manifest['attempts_completed']}/{total} agent attempts", flush=True)
    manifest["study_status"] = "RAW_COMPLETE"
    manifest["finished_unix_s"] = time.time()
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Completed {total} agent attempts. Analyze raw data before interpretation.")


if __name__ == "__main__":
    main()
