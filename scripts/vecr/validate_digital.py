#!/usr/bin/env python3
"""Independent reconstruction of current truth, evidence containment, assignments and hashes."""

import argparse, csv, gzip, hashlib, json, math
from pathlib import Path


def read(path):
    with gzip.open(path, "rt") if path.suffix == ".gz" else path.open() as f:
        rows = list(csv.DictReader(f))
    return [
        {
            (
                k.rsplit("_", 1)[0] + "_" + format(float(k.rsplit("_", 1)[1]), "g")
                if k and k.startswith(("raster_full_", "raster_center_"))
                else k
            ): v
            for k, v in row.items()
        }
        for row in rows
    ]


def near(a, b, tol=1e-5):
    if abs(float(a) - float(b)) > tol:
        raise ValueError(f"mismatch {a} != {b}")


def actual_interval(a, length):
    if a["acquired"] != "1":
        return None
    x, y = float(a["east_m"]), float(a["north_m"])
    c = math.cos(math.radians(float(a["yaw_deg"])))
    s = math.sin(math.radians(float(a["yaw_deg"])))
    low, high = 0, length
    # Solve the four rectangle half-space inequalities at each strip boundary.
    for yy in (-4, 4):
        for coeff, offset, width in (
            (c, s * (yy - y), float(a["along_half_m"])),
            (-s, c * (yy - y), float(a["cross_half_m"])),
        ):
            if abs(coeff) < 1e-12:
                if abs(offset) > width + 1e-9:
                    return None
            else:
                b = sorted(
                    [x + (-width - offset) / coeff, x + (width - offset) / coeff]
                )
                low = max(low, b[0])
                high = min(high, b[1])
    return (low, high) if high > low else None


def validate(out, hashes=True):
    manifest = json.loads((out / "manifest.json").read_text())
    rows = read(out / "per_attempt.csv")
    keys = lambda r: (
        r["scenario"],
        int(r["seed"]),
        r["policy"],
        r.get("client-kind", r.get("client_kind", "direct")),
    )
    assigned = {keys(x): x for x in manifest["assignments"]}
    seen = {keys(x) for x in rows}
    assert len(rows) == len(assigned) == len(seen) and seen == set(
        assigned
    ), "missing or duplicate assignments"
    assert manifest["status"] == "COMPLETE"
    negative = {"false_frame", "undeclared_angle"}
    checked_captures = 0
    checked_reentries = 0
    for row in rows:
        assert not row["infrastructure_error"], row
        root = out / "trials" / row["run_id"]
        cfg = assigned[keys(row)]
        length = float(cfg["length"])
        one = read(root / "attempt.csv")
        assert len(one) == 1
        for k, v in one[0].items():
            assert row[k] == v, (root, k)
        truth = read(root / "trajectory.csv.gz")
        trace = read(root / "trace.csv.gz")
        acq = read(root / "acquisitions.csv.gz")
        caps = read(root / "captures.csv.gz")
        events = read(root / "events.csv.gz")
        assert trace and truth and trace[-1]["phase"] == row["phase"]
        assert trace[-1]["reason"] == row["reason"]
        assert int(row["time_ms"]) == int(truth[-1]["time_ms"])
        ranges = []
        acquisitions = {}
        for a in acq:
            interval = actual_interval(a, length)
            acquisitions[int(a["time_ms"]) + 1000] = interval
            if interval is None:
                assert not a["begin_m"] and not a["end_m"]
            else:
                near(interval[0], a["begin_m"])
                near(interval[1], a["end_m"])
                ranges.append(interval)
        merged = []
        for a, b in sorted(ranges):
            if not merged or a > merged[-1][1] + 1e-9:
                merged.append([a, b])
            else:
                merged[-1][1] = max(merged[-1][1], b)
        covered = sum(b - a for a, b in merged)
        near(covered, row["truth_covered_m"])
        survey = (
            len(merged) == 1 and merged[0][0] <= 1e-7 and merged[0][1] >= length - 1e-7
        )
        assert survey == (row["truth_survey"] == "1")
        distance = 0
        last = (0, 0)
        arrival = -1
        first_qualified_arrival = None
        by_time = {}
        for t in truth:
            time = int(t["time_ms"])
            assert time not in by_time
            by_time[time] = t
            x, y, z = float(t["east_m"]), float(t["north_m"]), float(t["up_m"])
            distance += math.hypot(x - last[0], y - last[1])
            last = (x, y)
            if (
                math.sqrt((x - length) ** 2 + y * y + (z - 10) ** 2) <= 2
                and float(t["speed_mps"]) <= 0.5
            ):
                if arrival < 0:
                    arrival = time
            else:
                arrival = -1
            if arrival >= 0 and time - arrival >= 200 and first_qualified_arrival is None:
                first_qualified_arrival = time
        at_b = arrival >= 0 and int(truth[-1]["time_ms"]) - arrival >= 200
        near(distance, row["distance_m"])
        assert at_b == (row["truth_at_b"] == "1")
        tc = survey and at_b
        rc = row["phase"] == "ARRIVED" and row["survey_verified"] == "1"
        assert (row["phase"] == "ARRIVED") == (row["reported_arrived"] == "1")
        near(length - float(row["reported_covered_m"]), row["unresolved_m"])
        assert tc == (row["truth_complete"] == "1")
        assert rc == (row["reported_complete"] == "1")
        assert (rc and not tc) == (row["false_complete"] == "1")
        assert (tc and not rc) == (row["false_negative"] == "1")
        uncontained = 0
        for capture in caps:
            if capture["submitted"] != "1":
                continue
            checked_captures += 1
            if capture["accepted"] == "1":
                a, b = float(capture["claimed_begin"]), float(capture["claimed_end"])
                actual = acquisitions[int(capture["acquisition_ms"])]
                contained = (
                    actual is not None
                    and a >= actual[0] - 1e-6
                    and b <= actual[1] + 1e-6
                )
                assert contained == (capture["contained"] == "1")
                uncontained += not contained
        assert uncontained == int(row["accepted_uncontained"])
        if row["fault"] not in negative:
            assert not uncontained and not (rc and not tc), (root, "honest soundness")
        # Verify measurement envelope and absence of direct true-pose camera claims.
        measurements = read(root / "measurements.csv.gz")
        poses = {int(m["time_ms"]) + 1000: m for m in measurements}
        for m in measurements:
            t = by_time.get(int(m["time_ms"]))
            if t:
                err = math.hypot(
                    float(m["east_m"]) - float(t["east_m"]),
                    float(m["north_m"]) - float(t["north_m"]),
                )
                assert err <= float(m["horizontal_bound_m"]) + 1e-6 and err > 1e-5
                assert abs(float(m["altitude_m"]) - float(t["up_m"])) <= 0.05002
        for capture in caps:
            if capture["submitted"] == "1":
                m = poses[int(capture["acquisition_ms"])]
                near(capture["claimed_east"], m["east_m"])
                near(capture["claimed_north"], m["north_m"])
        go = [e for e in events if e["kind"] == "goto_accepted"]
        assert len(go) == int(row["gotos"])
        if first_qualified_arrival is not None:
            assert not any(
                int(e["time_ms"]) > first_qualified_arrival
                and abs(float(e["target_east"]) - length) > 1e-4
                for e in go
            ), (root, "automatic post-arrival recovery is forbidden")
        for i, e in enumerate(go):
            if i == 0 or abs(float(e["target_east"]) - length) > 1e-4:
                continue
            prior = go[i - 1]
            target = float(prior["target_east"])
            time = int(e["time_ms"])
            if abs(target - length) < 1e-4:
                continue
            dwell = [
                by_time[t] for t in range(time - 200, time + 1, 20) if t in by_time
            ]
            assert len(dwell) == 11, (root, "reentry dwell incomplete")
            assert all(
                math.hypot(float(p["east_m"]) - target, float(p["north_m"])) <= 2 + 1e-5
                and float(p["speed_mps"]) <= 0.5 + 1e-5
                for p in dwell
            ), (root, "ACK confused with reentry")
            checked_reentries += 1
        if int(row["midflight_repairs"]):
            assert 0 <= float(row["first_repair_east"]) < length - 2
        if row["scenario"] in ("M03", "M04", "M05"):
            assert int(row["midflight_repairs"]) >= 1, (
                root,
                "departure did not trigger before B",
            )
            phases = [t["phase"] for t in trace]
            assert "DEVIATING" in phases and "RECOVERING" in phases
            recovery_index = phases.index("RECOVERING")
            if rc:
                assert "MOVING" in phases[recovery_index + 1 :]
        if row["scenario"] == "M04":
            assert all(a["begin_m"] for a in acq if a["acquired"] == "1"), (
                root,
                "wide strip not visible",
            )
        if row["scenario"] == "M05":
            assert any(not a["begin_m"] for a in acq if a["acquired"] == "1"), (
                root,
                "hidden strip intervention absent",
            )
        if row["scenario"] == "M02":
            assert int(row["midflight_repairs"]) >= 1 and not rc
        if row["scenario"] == "M16":
            assert int(row["rejected_commands"]) >= 1 and not rc
        if row["scenario"] == "M17":
            assert (
                int(row["midflight_repairs"]) >= 1
                and any(e["kind"] == "motor_failure" for e in events)
                and not rc
            )
        if row["scenario"] == "M19":
            assert "battery" in row["reason"] and not rc
        for cell in (0.1, 0.25, 0.5, 1):
            full = center = n = 0
            a = 0
            while a < length - 1e-9:
                b = min(a + cell, length)
                p = (a + b) / 2
                n += 1
                full += any(l <= a + 1e-7 and r >= b - 1e-7 for l, r in merged)
                center += any(l <= p + 1e-7 and r >= p + 1e-6 - 1e-7 for l, r in merged)
                a += cell
            near(full / n, row[f"raster_full_{cell}"])
            near(center / n, row[f"raster_center_{cell}"])
    if hashes and (out / "SHA256SUMS").exists():
        entries = [
            line.split("  ", 1)
            for line in (out / "SHA256SUMS").read_text().splitlines()
        ]
        for digest, name in entries:
            assert hashlib.sha256((out / name).read_bytes()).hexdigest() == digest, name
        expected = {
            p.relative_to(out).as_posix()
            for p in out.rglob("*")
            if p.is_file() and p.name != "SHA256SUMS"
        }
        assert expected == {name for _, name in entries}, "hash inventory incomplete"
    print(
        f"VALID: {len(rows)} attempts, {checked_captures} delivered captures, {checked_reentries} physical re-entry dwells; independent truth reconstruction and soundness checks passed"
    )
    return dict(
        attempts=len(rows),
        delivered_captures=checked_captures,
        physical_reentries=checked_reentries,
    )


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", type=Path, required=True)
    a = ap.parse_args()
    validate(a.output)
