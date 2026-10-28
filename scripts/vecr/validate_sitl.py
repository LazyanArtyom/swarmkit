#!/usr/bin/env python3
"""Reconstruct SITL outcomes from independent SIM_STATE and acquisition records.

This checks sampled geometry; SIM_STATE lacks acquisition timestamps. It cannot
certify the assumed <=50 ms upstream transport age or real image quality.
"""

import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path


def read(p):
    with p.open() as f:
        return list(csv.DictReader(f))


def local(lat, lon):
    return (
        (lon - 44) * math.pi / 180 * 6378137 * math.cos(math.radians(40)),
        (lat - 40) * math.pi / 180 * 6378137,
    )


def union_length(intervals):
    end, total = 0.0, 0.0
    for lo, hi in sorted(intervals):
        total += max(0.0, hi - max(lo, end))
        end = max(end, hi)
    return total


def stopped_samples(samples, target, terminal):
    """Longest sampled stop, never bridge a gap >150 ms."""
    first = previous = None
    longest = 0.0
    for t in samples:
        good = (
            math.hypot(t["x"] - target[0], t["y"] - target[1]) <= 2
            and math.hypot(t["vn"], t["ve"]) <= 0.5
        )
        if not good or (previous is not None and t["time_ms"] - previous > 150):
            first = None
        if good:
            first = t["time_ms"] if first is None else first
            longest = max(longest, t["time_ms"] - first)
        previous = t["time_ms"]
    # Terminal arrival needs a current stop, not an earlier visit.
    if terminal:
        return 0.0 if first is None else samples[-1]["time_ms"] - first
    return longest


def validate(root):
    manifest = json.loads((root / "manifest.json").read_text())
    rows = read(root / "per_attempt.csv")
    assert len(rows) == len(manifest["assignments"])
    keys = lambda r: (
        r["navigation"].split("_")[0],
        r["fault"],
        r["policy"],
        str(r["seed"]),
    )
    assert {keys(r) for r in rows} == {keys(r) for r in manifest["assignments"]}
    if manifest["mode"] == "final":
        assert len(rows) == 12 and {int(r["seed"]) for r in rows} == {73001, 73002}
    checked = []
    errors = []
    for r in rows:
        path = root / r["run_id"]
        out = dict(r)
        if r["infrastructure_error"]:
            out["validation"] = "INFRASTRUCTURE_FAILURE"
            checked.append(out)
            continue
        try:
            truth = [
                {k: float(v) for k, v in t.items()} for t in read(path / "truth.csv")
            ]
            times = [t["time_ms"] for t in truth]
            assert all(b > a for a, b in zip(times, times[1:]))
            for t in truth:
                t["x"], t["y"] = local(t["lat_deg"], t["lon_deg"])

            def exact(ms):
                i = bisect.bisect_left(times, float(ms))
                assert i < len(times) and abs(times[i] - float(ms)) < 0.001
                return truth[i]

            start, end = float(r["start_ms"]), float(r["end_ms"])
            flight = [t for t in truth if start <= t["time_ms"] <= end]
            assert flight and end - flight[-1]["time_ms"] < 150
            assert max(math.hypot(t["vn"], t["ve"]) for t in flight) <= 10
            intervals = []
            acqs = {}
            for a in read(path / "acquisitions.csv"):
                t = exact(a["truth_time_ms"])
                assert (
                    math.hypot(
                        t["x"] - float(a["east_m"]), t["y"] - float(a["north_m"])
                    )
                    < 1e-6
                )
                assert abs(t["alt_m"] - 100 - float(a["up_m"])) < 1e-6
                age = float(a["time_ms"]) - t["time_ms"]
                assert -1 <= age <= 100, age
                h, w = 3 * (t["alt_m"] - 100) / 10, 8 * (t["alt_m"] - 100) / 10
                assert (
                    abs(h - float(a["along_half_m"])) < 1e-6
                    and abs(w - float(a["cross_half_m"])) < 1e-6
                )
                interval = (
                    (max(0.0, t["x"] - h), min(20.0, t["x"] + h))
                    if a["acquired"] == "1" and abs(t["y"]) + 4 <= w
                    else None
                )
                if interval and interval[1] > interval[0]:
                    intervals.append(interval)
                acqs[a["time_ms"]] = interval
            accepted = 0
            for c in read(path / "captures.csv"):
                t = exact(c["truth_time_ms"])
                assert (
                    math.hypot(
                        t["x"] - float(c["actual_east"]),
                        t["y"] - float(c["actual_north"]),
                    )
                    < 1e-6
                )
                if c["accepted"] != "1":
                    continue
                accepted += 1
                interval = acqs[c["acquisition_ms"]]
                assert interval and c["claimed_begin"] and c["claimed_end"]
                assert (
                    float(c["claimed_begin"]) >= interval[0] - 1e-5
                    and float(c["claimed_end"]) <= interval[1] + 1e-5
                ), c
                assert 0 <= float(c["delivery_ms"]) - float(c["acquisition_ms"]) <= 3000
            max_error = 0.0
            for o in read(path / "observations.csv"):
                if o["synthetic"] != "1":
                    continue
                t = exact(o["truth_time_ms"])
                err = math.hypot(
                    float(o["measured_east"]) - t["x"],
                    float(o["measured_north"]) - t["y"],
                )
                assert err <= float(o["hard_bound"])
                max_error = max(max_error, err)
            distance = sum(
                math.hypot(b["x"] - a["x"], b["y"] - a["y"])
                for a, b in zip(flight, flight[1:])
            )
            coverage = union_length(intervals)
            # 50 ms sampled observations cannot prove continuous 200 ms dwell.
            dwell = stopped_samples(flight, (20, 0), True)
            complete = coverage >= 20 - 0.01 and dwell >= 150
            trace = read(path / "agent_trace.csv")
            assert trace[-1]["phase"] == r["phase"]
            assert abs(float(trace[-1]["covered_m"]) - float(r["covered_m"])) < 0.001
            if r["reported_complete"] == "1":
                assert complete, (coverage, dwell)
            all_commands = read(path / "commands.csv")
            if r["fault"] == "command_rejected":
                assert any(c["accepted"] == "0" for c in all_commands)
                assert r["phase"] == "INTERRUPTED" and r["reported_complete"] == "0"
            if r["fault"] == "land_override":
                bridge_events = [
                    json.loads(line)
                    for line in (path / "bridge_events.jsonl").read_text().splitlines()
                ]
                land = next(
                    e["time_ms"]
                    for e in bridge_events
                    if e.get("event") == "injected_LAND_safety_override"
                )
                assert not any(float(c["time_ms"]) > land for c in all_commands)
            commands = [c for c in read(path / "commands.csv") if c["accepted"] == "1"]
            dwells = []
            # Commands alternate B, re-entry, B; only resumed repairs have a dwell obligation.
            for i in range(1, len(commands) - 1, 2):
                c, nxt = commands[i], commands[i + 1]
                segment = [
                    t
                    for t in truth
                    if float(c["time_ms"]) <= t["time_ms"] <= float(nxt["time_ms"])
                ]
                d = stopped_samples(
                    segment, (float(c["target_east"]), float(c["target_north"])), False
                )
                assert d >= 150, (r["run_id"], i, d)
                dwells.append(d)
            if r["fault"] == "land_override":
                assert r["phase"] == "INTERRUPTED" and r["reported_complete"] == "0"
            if r["navigation"] == "raw_mavlink":
                assert r["reported_complete"] == "0" and accepted == 0
            out.update(
                validation="PASS",
                truth_coverage_m=coverage,
                sampled_stopped_at_b_ms=dwell,
                sampled_truth_complete=int(complete),
                accepted_captures=accepted,
                sampled_reentry_dwells=len(dwells),
                distance_m=distance,
                duration_s=(end - start) / 1000,
                peak_horizontal_speed_mps=max(
                    math.hypot(t["vn"], t["ve"]) for t in flight
                ),
                max_synthetic_position_error_m=max_error,
            )
        except (AssertionError, ValueError, KeyError) as e:
            out.update(validation="FAIL", validation_error=repr(e))
            errors.append(dict(run_id=r["run_id"], error=repr(e)))
        checked.append(out)
    folder = root / "analysis"
    folder.mkdir(exist_ok=True)
    with (folder / "reconstructed.csv").open("w") as f:
        w = csv.DictWriter(
            f, fieldnames=sorted(set().union(*(r.keys() for r in checked)))
        )
        w.writeheader()
        w.writerows(checked)
    result = dict(
        assignments=len(rows),
        passes=sum(r["validation"] == "PASS" for r in checked),
        infrastructure_failures=sum(
            r["validation"] == "INFRASTRUCTURE_FAILURE" for r in checked
        ),
        errors=errors,
        accepted_captures=sum(r.get("accepted_captures", 0) for r in checked),
        sampled_reentry_dwells=sum(r.get("sampled_reentry_dwells", 0) for r in checked),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        limitations=[
            "SIM_STATE acquisition time absent: <=50 ms upstream age is assumed, not proven.",
            "Truth and stop dwell are sampled; no continuous-flight guarantee.",
            "Camera and bounded navigation synthetic; no real image-quality claim.",
        ],
    )
    (folder / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return not errors and result["infrastructure_failures"] == 0


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("output", type=Path)
    raise SystemExit(0 if validate(p.parse_args().output) else 1)
