#!/usr/bin/env python3
"""Owned native ArduPilot processes, localhost MAVLink bridge, honest evidence labels."""

import argparse, csv, hashlib, json, os, socket, subprocess, time, concurrent.futures, sys
import shutil
from pathlib import Path

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[2]


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def clock():
    # libc++ steady_clock is MONOTONIC_RAW on Darwin, unlike Python monotonic.
    return (
        time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
        if sys.platform == "darwin"
        else time.monotonic()
    )


def command(m, cmd, *params):
    m.mav.command_long_send(1, 1, cmd, 0, *(list(params) + [0] * (7 - len(params))))


def trial(assignment, root, aproot, binary, ordinal):
    name = "-".join(
        str(assignment[k]) for k in ["navigation", "fault", "policy", "seed"]
    )
    out = root / name
    out.mkdir()
    settings = (aproot / "Tools/autotest/default_params/copter.parm").read_text()
    settings += "\nSERIAL0_PROTOCOL 2\nSIM_SPEEDUP 1\nFS_GCS_ENABLE 0\nLOG_DISARMED 1\n"
    (out / "defaults.parm").write_text(settings)
    port = 24000 + ordinal * 20
    # Check the whole assigned block; serial ports and auxiliary ports are isolated.
    for kind in [socket.SOCK_STREAM, socket.SOCK_DGRAM]:
        for candidate in range(port, port + 20):
            with socket.socket(socket.AF_INET, kind) as probe:
                probe.bind(("127.0.0.1", candidate))
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("127.0.0.1", 0))
    udp.setblocking(False)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        agent_port = probe.getsockname()[1]
    args = [
        str(aproot / "build/sitl/bin/arducopter"),
        "--model",
        "+",
        "--home",
        "40,44,100,0",
        "--speedup",
        "1",
        "--wipe",
        "--defaults",
        str(out / "defaults.parm"),
        "--serial0",
        f"tcp:{port}",
        "--base-port",
        str(port),
        "--instance",
        str(100 + ordinal),
        "--rc-in-port",
        str(port + 12),
        "--sim-port-in",
        str(port + 13),
        "--sim-port-out",
        str(port + 14),
        "--irlock-port",
        str(port + 15),
    ]
    (out / "assignment.json").write_text(
        json.dumps(dict(assignment, ardupilot_command=args), indent=2) + "\n"
    )
    stdout = (out / "ardupilot.log").open("w")
    ap = subprocess.Popen(args, cwd=out, stdout=stdout, stderr=subprocess.STDOUT)
    child = None
    m = None
    row = dict(assignment, run_id=name, infrastructure_error="")
    truth = (out / "truth.csv").open("w")
    tw = csv.writer(truth)
    tw.writerow(
        [
            "time_ms",
            "lat_deg",
            "lon_deg",
            "alt_m",
            "vn",
            "ve",
            "vd",
            "roll",
            "pitch",
            "yaw",
            "sequence",
        ]
    )
    binary_log = (out / "mavlink.raw").open("wb")
    events = (out / "bridge_events.jsonl").open("w")
    state = {}
    seq = 0
    start = clock()
    guided = False
    arm_at = None
    takeoff = False
    started = None
    land_sent = False
    last_heartbeat = 0
    last_rc = 0
    last_takeoff = 0
    try:
        # Connect only to the newly owned child's loopback port.
        while clock() - start < 15:
            if ap.poll() is not None:
                raise RuntimeError("ArduPilot exited at startup")
            try:
                m = mavutil.mavlink_connection(
                    f"tcp:127.0.0.1:{port}", source_system=255, source_component=190
                )
                break
            except OSError:
                time.sleep(0.1)
        if m is None:
            raise RuntimeError("SITL TCP startup timeout")
        last_request = 0
        last_mode = 0
        armed = False
        while clock() - start < 240:
            now = clock()
            if now - last_rc > 0.1:
                # A live simulated receiver, with neutral attitude and low throttle.
                m.mav.rc_channels_override_send(
                    1, 1, 1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000
                )
                last_rc = now
            if now - last_heartbeat > 1:
                m.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
                last_heartbeat = now
            if now - last_request > 3:
                m.mav.request_data_stream_send(
                    1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1
                )
                command(m, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 108, 50000)
                last_request = now
            for _ in range(100):
                msg = m.recv_match(blocking=False)
                if msg is None:
                    break
                kind = msg.get_type()
                if kind == "BAD_DATA":
                    continue
                raw = msg.get_msgbuf()
                binary_log.write(raw)
                udp.sendto(raw, ("127.0.0.1", agent_port))
                state[kind] = msg
                if kind in ("COMMAND_ACK", "STATUSTEXT"):
                    events.write(
                        json.dumps(dict(time_ms=now * 1000, message=msg.to_dict()))
                        + "\n"
                    )
                    events.flush()
                if (
                    kind == "COMMAND_ACK"
                    and msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                ):
                    takeoff = msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                if kind == "HEARTBEAT":
                    armed = bool(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                if kind == "SIM_STATE":
                    seq += 1
                    lat = getattr(msg, "lat_int", 0) / 1e7 or msg.lat / 1e7
                    lon = getattr(msg, "lon_int", 0) / 1e7 or msg.lon / 1e7
                    values = [
                        clock() * 1000,
                        lat,
                        lon,
                        msg.alt,
                        msg.vn,
                        msg.ve,
                        msg.vd,
                        msg.roll,
                        msg.pitch,
                        msg.yaw,
                        seq,
                    ]
                    tw.writerow(values)
                    truth.flush()
                    temp = out / "truth_snapshot.tmp"
                    temp.write_text(" ".join(map(str, values)) + "\n")
                    temp.replace(out / "truth_snapshot.txt")
            while True:
                try:
                    data, _ = udp.recvfrom(65535)
                    m.write(data)
                except BlockingIOError:
                    break
            gps = state.get("GPS_RAW_INT")
            hb = state.get("HEARTBEAT")
            pos = state.get("GLOBAL_POSITION_INT")
            if (
                not guided
                and gps
                and gps.fix_type >= 3
                and hb
                and now - start > 10
                and now - last_mode > 2
            ):
                m.mav.set_mode_send(
                    1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4
                )
                last_mode = now
                if hb.custom_mode == 4:
                    guided = True
            if guided and not armed and now - last_mode > 2:
                command(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
                last_mode = now
            if armed and not takeoff and now - last_takeoff > 3:
                command(m, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10)
                last_takeoff = now
                arm_at = now
            if (
                takeoff
                and pos
                and pos.relative_alt >= 9500
                and seq > 0
                and child is None
                and now - arm_at > 7
            ):
                log = (out / "agent.log").open("w")
                child = subprocess.Popen(
                    [
                        str(binary),
                        "--output",
                        str(out),
                        "--bind",
                        f"127.0.0.1:{agent_port}",
                        *sum(([f"--{k}", str(v)] for k, v in assignment.items()), []),
                    ],
                    cwd=ROOT,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
                log.close()
                started = now
            if (
                child
                and assignment["fault"] == "land_override"
                and not land_sent
                and now - started > 6
            ):
                m.mav.set_mode_send(
                    1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9
                )
                land_sent = True
                events.write(
                    json.dumps(
                        dict(time_ms=now * 1000, event="injected_LAND_safety_override")
                    )
                    + "\n"
                )
                events.flush()
            if child and child.poll() is not None:
                if child.returncode:
                    raise RuntimeError(
                        f"agent harness exited {child.returncode}: "
                        + (out / "agent.log").read_text()[-1200:]
                    )
                with (out / "attempt.csv").open() as f:
                    row.update(next(csv.DictReader(f)))
                break
            if ap.poll() is not None:
                raise RuntimeError("ArduPilot exited during experiment")
            time.sleep(0.005)
        else:
            raise RuntimeError("SITL trial startup/execution deadline")
    except Exception as exc:
        row["infrastructure_error"] = str(exc)
    finally:
        for proc in [child, ap]:
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
        if m:
            m.close()
        udp.close()
        truth.close()
        binary_log.close()
        events.close()
        stdout.close()
    (out / "result.json").write_text(json.dumps(row, indent=2) + "\n")
    return row


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, required=True)
    p.add_argument(
        "--ardupilot", type=Path, default=Path("/private/tmp/swarmkit-ardupilot-v6")
    )
    p.add_argument("--mode", choices=["pilot", "final"], default="pilot")
    p.add_argument("--workers", type=int, default=2)
    p.add_argument("--only", help="pilot assignment prefix for targeted diagnosis")
    a = p.parse_args()
    root = a.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    aproot = a.ardupilot.resolve()
    binary = ROOT / "build/mac-release/tests/swarmkit_vecr_sitl_trial"
    assignments = []
    for navigation, fault in [
        ("raw", "none"),
        ("synthetic", "none"),
        ("synthetic", "delay"),
        ("synthetic", "multi_gap"),
        ("synthetic", "command_rejected"),
        ("synthetic", "land_override"),
    ]:
        for policy in ["GW", "E"]:
            for seed in ([60001] if a.mode == "pilot" else [61001, 61002]):
                assignments.append(
                    dict(navigation=navigation, fault=fault, policy=policy, seed=seed)
                )
    if a.only:
        if a.mode != "pilot":
            raise ValueError("final assignments cannot be filtered")
        assignments = [
            x
            for x in assignments
            if "-".join(
                str(x[k]) for k in ["navigation", "fault", "policy", "seed"]
            ).startswith(a.only)
        ]
        if not assignments:
            raise ValueError("empty pilot filter")
    before = clock() * 1000
    cpp_clock = int(subprocess.check_output([str(binary), "--clock-ms"], text=True))
    after = clock() * 1000
    if not before - 2 <= cpp_clock <= after + 2:
        raise RuntimeError("Python/C++ steady clock epochs disagree")
    source_paths = [
        "src",
        "include",
        "proto",
        "tests",
        "scripts/vecr/run_sitl_v6.py",
        "scripts/vecr/validate_sitl_v6.py",
        "docs/vecr/SITL_PROTOCOL_V6.md",
    ]
    dirty_source = subprocess.check_output(
        ["git", "status", "--porcelain", "--", *source_paths], cwd=ROOT, text=True
    )
    if a.mode == "final" and (dirty_source or len(assignments) != 24):
        raise RuntimeError(
            "final SITL requires committed source and all 24 assignments"
        )
    frozen_binary = Path("/private/tmp") / ("swarmkit-v6-sitl-" + root.name + "-frozen")
    if frozen_binary.exists():
        raise FileExistsError(frozen_binary)
    shutil.copy2(binary, frozen_binary)
    binary = frozen_binary
    manifest = dict(
        protocol="v6-sitl",
        mode=a.mode,
        status="RUNNING",
        assignments=assignments,
        source_revision=subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        source_patch_sha256=hashlib.sha256(
            subprocess.check_output(["git", "diff"], cwd=ROOT)
        ).hexdigest(),
        agent_binary_sha256=sha(binary),
        ardupilot_revision=subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=aproot, text=True
        ).strip(),
        ardupilot_binary_sha256=sha(aproot / "build/sitl/bin/arducopter"),
        runner_sha256=sha(__file__),
        frozen_binary_path=str(binary),
        dirty_experimental_source=dirty_source,
        workers=a.workers,
        clock_domain=(
            "CLOCK_MONOTONIC_RAW" if sys.platform == "darwin" else "CLOCK_MONOTONIC"
        ),
        clock_probe_ms=[before, cpp_clock, after],
    )
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (root / "source.patch").write_bytes(
        subprocess.check_output(["git", "diff"], cwd=ROOT)
    )
    rows = []
    with concurrent.futures.ThreadPoolExecutor(a.workers) as pool:
        futures = [
            pool.submit(trial, x, root, aproot, binary, i)
            for i, x in enumerate(assignments)
        ]
        for future in concurrent.futures.as_completed(futures):
            try:
                r = future.result()
            except Exception as e:
                r = dict(infrastructure_error=repr(e))
            rows.append(r)
            print(len(rows), "/", len(assignments), r, flush=True)
    fields = sorted(set().union(*(r.keys() for r in rows)))
    with (root / "per_attempt.csv").open("w") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    manifest["status"] = (
        "COMPLETE"
        if all(not r["infrastructure_error"] for r in rows)
        else "INFRASTRUCTURE_FAILURE"
    )
    manifest["completed_rows"] = len(rows)
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
