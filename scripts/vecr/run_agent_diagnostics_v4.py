#!/usr/bin/env python3
"""Run v4 noncomparative cases and preserve exact test logs separately."""

import argparse
import csv
import json
import pathlib
import subprocess
import sys

from run_agent_campaign_v4 import command, params, read_rows, sha


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign", required=True, type=pathlib.Path)
    parser.add_argument("--executable", type=pathlib.Path,
                        default="build/mac-release/tests/swarmkit_vecr_agent_trial")
    parser.add_argument("--tests-executable", type=pathlib.Path,
                        default="build/mac-release/tests/swarmkit_tests")
    args = parser.parse_args()
    campaign = args.campaign.resolve()
    config = json.loads((campaign / "protocol.json").read_text())
    diagnostic_dir = campaign / "diagnostics"
    diagnostic_dir.mkdir(exist_ok=False)
    exe = args.executable.resolve()
    tests_exe = args.tests_executable.resolve()
    cases = [("S10", "fast_fly_through"),
             ("S11", "detectable_bad_metadata"),
             ("S11", "dishonest_camera"),
             ("S12", "safety_battery")]
    attempted = []
    for scenario_id, fault in cases:
        scenario = next(item for item in config["scenarios"] if item["id"] == scenario_id)
        setting = params({"id": scenario_id, **scenario}, 1, config)
        case = {"id": scenario_id, "fault": fault}
        folder = diagnostic_dir / f"{scenario_id}-{fault}"
        proc = subprocess.run(command(exe, case, 1, "P", setting, folder),
                              capture_output=True, text=True)
        if proc.returncode:
            raise RuntimeError(f"diagnostic {scenario_id}/{fault}: {proc.stderr[-1000:]}")
        row = read_rows(folder / "attempt.csv")[0]
        attempted.append({"scenario_id": scenario_id, "variant": fault,
                          "policy": "P_DIAGNOSTIC", "reported_phase": row["reported_phase"],
                          "truth_survey_complete": row["truth_survey_complete"],
                          "false_complete": row["false_complete"],
                          "fault_realized": row["requested_fault_realized"],
                          "trial_binary_sha256": sha(exe)})
    with (diagnostic_dir / "agent_attempts.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, attempted[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(attempted)
    regression_names = {
        "S15": "VECR restart gates a new attempt until simulator hold is confirmed",
        "S18": "VECR agent restart preserves terminal reports and blocks ambiguous replay",
        "S20": "Verified edge agent RPC separates arrival from survey and retains gaps",
    }
    results = []
    for scenario_id, test_name in regression_names.items():
        proc = subprocess.run([str(tests_exe), test_name], capture_output=True, text=True)
        (diagnostic_dir / f"{scenario_id}_regression.log").write_text(
            proc.stdout + "\n" + proc.stderr)
        results.append({"scenario_id": scenario_id, "test_name": test_name,
                        "exit_code": proc.returncode, "test_binary_sha256": sha(tests_exe)})
        if proc.returncode:
            raise RuntimeError(f"{scenario_id} diagnostic regression failed")
    with (diagnostic_dir / "regression_results.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, results[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(results)
    print(f"Recorded {len(attempted)} agent diagnostics and {len(results)} regression diagnostics")


if __name__ == "__main__":
    main()
