#!/usr/bin/env python3
"""Generate runtime-percent voltage anchors from battery calibration CSV.

Input format (current firmware):
  timestamp_ms,sample_num,raw_adc,adc_mv,battery_volts,battery_pct,charging

This tool intentionally treats percent as runtime-percent for a representative
load profile, not electrochemical SOC.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from statistics import median
from typing import Iterable


@dataclass
class Row:
    t_ms: int
    v: float
    charging: int


@dataclass
class Anchor:
    pct: int
    volts: float


def parse_rows(path: Path) -> list[Row]:
    rows: list[Row] = []
    with path.open(newline="") as f:
        r = csv.DictReader(f)
        required = {"timestamp_ms", "battery_volts", "charging"}
        missing = required - set(r.fieldnames or [])
        if missing:
            raise ValueError(f"missing columns: {sorted(missing)}")

        for row in r:
            try:
                rows.append(
                    Row(
                        t_ms=int(row["timestamp_ms"]),
                        v=float(row["battery_volts"]),
                        charging=int(row["charging"]),
                    )
                )
            except (TypeError, ValueError):
                continue

    if not rows:
        raise ValueError("no valid rows")

    rows.sort(key=lambda x: x.t_ms)
    return rows


def pick_discharge_segment(rows: list[Row], min_points: int) -> list[Row]:
    best: list[Row] = []
    cur: list[Row] = []

    for row in rows:
        if row.charging == 0:
            cur.append(row)
        else:
            if len(cur) > len(best):
                best = cur
            cur = []

    if len(cur) > len(best):
        best = cur

    if len(best) < min_points:
        raise ValueError(
            f"longest discharge segment has {len(best)} points; need at least {min_points}"
        )

    return best


def runtime_percent(segment: list[Row], row: Row) -> float:
    t0 = segment[0].t_ms
    t1 = segment[-1].t_ms
    if t1 <= t0:
        return 0.0
    return max(0.0, min(100.0, 100.0 * (t1 - row.t_ms) / (t1 - t0)))


def generate_anchors(segment: list[Row], targets: Iterable[int], window: float) -> list[Anchor]:
    anchors: list[Anchor] = []

    for pct in sorted(set(targets), reverse=True):
        candidates = [
            row.v
            for row in segment
            if abs(runtime_percent(segment, row) - pct) <= window
        ]

        if not candidates:
            closest = min(segment, key=lambda r: abs(runtime_percent(segment, r) - pct))
            v = closest.v
        else:
            v = float(median(candidates))

        anchors.append(Anchor(pct=pct, volts=v))

    # enforce monotonic voltage decrease as percent decreases
    fixed: list[Anchor] = []
    last_v = float("inf")
    for a in anchors:
        v = min(a.volts, last_v)
        fixed.append(Anchor(a.pct, v))
        last_v = v

    return fixed


def print_report(rows: list[Row], segment: list[Row], anchors: list[Anchor]) -> None:
    all_t0, all_t1 = rows[0].t_ms, rows[-1].t_ms
    all_duration_min = (all_t1 - all_t0) / 60_000.0

    t0, t1 = segment[0].t_ms, segment[-1].t_ms
    duration_min = (t1 - t0) / 60_000.0
    duration_h = duration_min / 60.0

    print(f"all-row start/end ms: {all_t0} -> {all_t1}")
    print(f"all-row duration minutes: {all_duration_min:.2f}")
    print()
    print(f"discharge points: {len(segment)}")
    print(f"segment start/end ms: {t0} -> {t1}")
    print(f"duration minutes: {duration_min:.2f}")
    print(f"duration hours: {duration_h:.2f}")
    print(f"voltage range: {segment[0].v:.3f}V -> {segment[-1].v:.3f}V")
    print()
    print("Suggested runtime-percent anchors:")
    for a in anchors:
        print(f"  {a.pct:>3}% -> {a.volts:.3f}V")

    print("\nC++ table:")
    print("static const BatteryAnchor SOC_TABLE[] = {")
    for a in anchors:
        print(f"  {{{a.volts:.3f}f, {a.pct}}},")
    print("};")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv", type=Path, help="path to battery_cal.csv")
    p.add_argument(
        "--targets",
        default="100,90,80,70,60,50,40,30,20,10,5,0",
        help="comma-separated runtime-percent anchors",
    )
    p.add_argument(
        "--window",
        type=float,
        default=1.5,
        help="runtime-percent window used to collect anchor samples",
    )
    p.add_argument(
        "--min-points",
        type=int,
        default=100,
        help="minimum required rows in discharge segment",
    )
    args = p.parse_args()

    targets = [int(x.strip()) for x in args.targets.split(",") if x.strip()]

    rows = parse_rows(args.csv)
    segment = pick_discharge_segment(rows, min_points=args.min_points)
    anchors = generate_anchors(segment, targets=targets, window=args.window)
    print_report(rows, segment, anchors)


if __name__ == "__main__":
    main()
