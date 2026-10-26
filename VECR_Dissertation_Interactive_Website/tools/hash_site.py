#!/usr/bin/env python3
"""Write or verify the portable reader's SHA-256 inventory after validation."""

import argparse
import hashlib
from pathlib import Path

SITE = Path(__file__).resolve().parents[1]
INVENTORY = SITE / "SHA256SUMS"


def files():
    return sorted(
        p
        for p in SITE.rglob("*")
        if p.is_file()
        and p != INVENTORY
        and "__pycache__" not in p.parts
        and p.name != ".DS_Store"
    )


def main(write):
    current = {
        p.relative_to(SITE).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in files()
    }
    if write:
        INVENTORY.write_text("".join(f"{h}  {p}\n" for p, h in current.items()))
        print(f"Wrote SHA-256 inventory for {len(current)} files")
    else:
        recorded = {
            p: h
            for h, p in (
                line.split("  ", 1) for line in INVENTORY.read_text().splitlines()
            )
        }
        assert set(recorded) == set(current), "missing or unlisted files"
        assert recorded == current, "reader content differs from its SHA-256 inventory"
        print(f"Verified all {len(current)} reader files")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--write",
        action="store_true",
        help="Create a new inventory after an intentional rebuild and validation",
    )
    main(parser.parse_args().write)
