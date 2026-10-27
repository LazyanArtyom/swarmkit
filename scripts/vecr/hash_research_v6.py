#!/usr/bin/env python3
"""Seal or verify new v6 research, including failed pilots; never touch older studies."""

import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ALLOWED = [ROOT / "results/vecr/agent_campaign_v6", ROOT / "results/vecr/sitl_v6"]


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for b in iter(lambda: f.read(1048576), b""):
            h.update(b)
    return h.hexdigest()


def inventory(root, write):
    files = sorted(
        p
        for p in root.rglob("*")
        if p.is_file()
        and p.name != "SHA256SUMS"
        and "__pycache__" not in p.parts
        and p.name != ".DS_Store"
    )
    actual = {p.relative_to(root).as_posix(): digest(p) for p in files}
    target = root / "SHA256SUMS"
    if write:
        target.write_text("".join(f"{h}  {p}\n" for p, h in actual.items()))
    else:
        recorded = {
            p: h
            for h, p in (
                line.split("  ", 1) for line in target.read_text().splitlines()
            )
        }
        assert recorded == actual, f"missing, changed or unlisted evidence: {root}"
    print(
        f'{"SEALED" if write else "VERIFIED"}: {root.relative_to(ROOT)}: {len(files)} files'
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--write", action="store_true")
    p.add_argument("--collection", choices=["digital", "sitl", "all"], default="all")
    a = p.parse_args()
    for root in (
        ALLOWED if a.collection == "all" else [ALLOWED[a.collection == "sitl"]]
    ):
        for sub in sorted(root.iterdir()):
            if not sub.is_dir() or not (sub / "manifest.json").exists():
                continue
            manifest = json.loads((sub / "manifest.json").read_text())
            assert (
                manifest["status"] != "RUNNING"
            ), f"cannot seal active experiment: {sub}"
            inventory(sub, a.write)
        inventory(root, a.write)


if __name__ == "__main__":
    main()
