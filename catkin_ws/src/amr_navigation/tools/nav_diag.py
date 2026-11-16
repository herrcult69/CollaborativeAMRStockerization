#!/usr/bin/env python3
"""nav_diag.py - Consolidated Navigation Telemetry & Gate Diagnostic Analyzer.

Usage:
    python nav_diag.py <run_dir>

Reports exclusively the high-reliability metrics:
  1. Angular velocity (w) sign flips: total, at full speed (vx >= 0.30 m/s), and per moving-minute.
  2. Safety Gate HOLD count grouped by trigger reason (crossing, blocked, flank, etc.).
  3. Safety Gate GIVEUP count.
  4. Lateral plan bulges > 0.5 m (max lateral deviation from Navfn global plan).
  5. TEB re-inits by reason and re-spins within 3 s of a spin stop.

Drops the unreliable cumulative-turn and chord-ratio loop metrics.
"""

import argparse
import csv
import math
import os
import re
import sys
from pathlib import Path


def parse_timestamp(val):
    """Normalize timestamp from seconds or nanoseconds to floating seconds."""
    try:
        t = float(val)
        if t > 1e8:
            return t * 1e-9
        return t
    except (ValueError, TypeError):
        return None


def point_to_segment_dist(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    l2 = dx * dx + dy * dy
    if l2 < 1e-8:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def point_to_polyline_dist(px, py, polyline):
    if not polyline:
        return float("inf")
    if len(polyline) == 1:
        return math.hypot(px - polyline[0][0], py - polyline[0][1])
    best = float("inf")
    for i in range(len(polyline) - 1):
        d = point_to_segment_dist(px, py, polyline[i][0], polyline[i][1], polyline[i + 1][0], polyline[i + 1][1])
        if d < best:
            best = d
    return best


def find_dataset_dir(target):
    candidates = [
        Path(target),
        Path("CollaborativeAMRStockerization/catkin_ws") / target,
        Path("catkin_ws") / target,
    ]
    for p in candidates:
        if p.is_dir() and ((p / "cmd_vel_nav.csv").exists() or (p / "cmd_vel.csv").exists()):
            return p
    for p in candidates:
        if p.is_dir() and any(p.glob("*.csv")):
            return p
    return Path(target)


def find_rosout_files(run_dir):
    search_dirs = [Path(run_dir), Path(run_dir).parent, Path(".agents/logs")]
    rosout_files = []
    seen = set()
    for d in search_dirs:
        if d.is_dir():
            found = list(d.glob("**/rosout.log")) + list(d.glob("**/rosout*.log"))
            if not found and d == Path(run_dir):
                found = list(d.glob("*.log"))
            for rf in found:
                resolved = rf.resolve()
                if resolved not in seen:
                    seen.add(resolved)
                    rosout_files.append(rf)
    return rosout_files


def line_time(line):
    """Seconds from a rosout line: `rostopic echo -p` row (time in ns first), else a roslog `[1731.500]` stamp, else None."""
    m = re.match(r"\s*(\d{9,})\s*,", line)
    if m:
        return int(m.group(1)) / 1e9
    m = re.search(r"\[(\d+\.\d+)\]", line)
    return float(m.group(1)) if m else None


def load_rosout_reasons(run_dir):
    """Optionally find rosout.log in run_dir or nearby to extract exact gate log reasons."""
    reasons = {}
    rosout_files = find_rosout_files(run_dir)

    gate_pattern = re.compile(r"\[gate\]\s+(?:free|giveup)\s+->\s+hold\s+\[(.*?)\]", re.IGNORECASE)
    for rf in rosout_files:
        try:
            with open(rf, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    m = gate_pattern.search(line)
                    if m:
                        reason = m.group(1).strip()
                        # Extract timestamp if available
                        t_val = line_time(line)
                        if t_val is not None:
                            reasons[round(t_val, 1)] = reason
        except Exception:
            pass
    return reasons


def analyze_rosout(run_dir):
    """Scan rosout logs for TEB re-inits by reason and re-spins (spin start within 3s of previous spin stop)."""
    rosout_files = find_rosout_files(run_dir)
    reinit_by_reason = {}
    total_reinits = 0
    spin_events = []

    reinit_pattern = re.compile(r"\[gate\]\s+teb\s+reinit\s+\((.*?)\)", re.IGNORECASE)
    spin_start_pattern = re.compile(r"\[gate\]\s+spin\s+start", re.IGNORECASE)
    spin_stop_pattern = re.compile(r"\[gate\]\s+spin\s+stop", re.IGNORECASE)

    for rf in rosout_files:
        try:
            with open(rf, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    m = reinit_pattern.search(line)
                    if m:
                        reason = m.group(1).strip()
                        reinit_by_reason[reason] = reinit_by_reason.get(reason, 0) + 1
                        total_reinits += 1

                    is_start = spin_start_pattern.search(line)
                    is_stop = spin_stop_pattern.search(line)
                    if is_start or is_stop:
                        t = line_time(line)
                        if is_start:
                            spin_events.append((t, "start"))
                        elif is_stop:
                            spin_events.append((t, "stop"))
        except Exception:
            pass

    re_spins = 0
    timed_events = [ev for ev in spin_events if ev[0] is not None]
    if timed_events:
        timed_events.sort(key=lambda x: x[0])
        last_stop_t = None
        for t, ev_type in timed_events:
            if ev_type == "stop":
                last_stop_t = t
            elif ev_type == "start":
                if last_stop_t is not None and 0.0 <= (t - last_stop_t) <= 3.0:
                    re_spins += 1
                last_stop_t = None

    return {
        "total_reinits": total_reinits,
        "reinit_by_reason": reinit_by_reason,
        "re_spins": re_spins,
    }


def analyze_flips(cmd_records, deadband=0.05, full_speed_threshold=0.30):
    """Compute w sign flips, flips at full speed, and flips per moving-minute."""
    if not cmd_records:
        return {
            "total_flips": 0,
            "cruising_flips": 0,
            "flips_per_min": 0.0,
            "flips_per_moving_min": 0.0,
            "total_duration": 0.0,
            "moving_duration": 0.0,
            "sample_count": 0,
        }

    cmd_records.sort(key=lambda r: r[0])
    t_start = cmd_records[0][0]
    t_end = cmd_records[-1][0]
    total_duration = max(t_end - t_start, 1e-3)

    moving_duration = 0.0
    for i in range(1, len(cmd_records)):
        dt = cmd_records[i][0] - cmd_records[i - 1][0]
        if dt > 0.0 and dt <= 1.0:
            vx = abs(cmd_records[i][1])
            wz = abs(cmd_records[i][2])
            if vx >= 0.02 or wz >= 0.02:
                moving_duration += dt

    moving_duration = max(moving_duration, 1e-3)

    flips = []
    prev_sign = 0
    prev_w = 0.0

    for t, vx, wz in cmd_records:
        if abs(wz) >= deadband:
            cur_sign = 1 if wz > 0 else -1
            if prev_sign != 0 and cur_sign != prev_sign:
                flips.append({
                    "t": t,
                    "w_prev": prev_w,
                    "w_curr": wz,
                    "vx": vx,
                    "is_full_speed": vx >= full_speed_threshold,
                })
            prev_sign = cur_sign
            prev_w = wz

    total_flips = len(flips)
    cruising_flips = sum(1 for f in flips if f["is_full_speed"])
    flips_per_min = (total_flips / total_duration) * 60.0
    flips_per_moving_min = (total_flips / moving_duration) * 60.0

    return {
        "total_flips": total_flips,
        "cruising_flips": cruising_flips,
        "flips_per_min": flips_per_min,
        "flips_per_moving_min": flips_per_moving_min,
        "total_duration": total_duration,
        "moving_duration": moving_duration,
        "sample_count": len(cmd_records),
        "t_start": t_start,
        "t_end": t_end,
    }


def analyze_gate(data_dir, log_reasons):
    """Analyze gate_state.csv for HOLD transitions by reason and GIVEUP count."""
    csv_path = Path(data_dir) / "gate_state.csv"
    if not csv_path.is_file():
        return {"total_holds": 0, "holds_by_reason": {}, "total_giveups": 0, "events": []}

    events = []
    with open(csv_path, "r", encoding="utf-8", errors="ignore") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        prev_st = None
        for row in reader:
            if not row:
                continue
            t = parse_timestamp(row[0])
            raw_st = row[1].strip().lower()

            # State may contain reason e.g. "hold [crossing]"
            st = "free"
            reason = "unspecified"
            if "giveup" in raw_st:
                st = "giveup"
            elif "hold" in raw_st:
                st = "hold"
                m = re.search(r"\[(.*?)\]", raw_st)
                if m:
                    reason = m.group(1).strip()
            elif "free" in raw_st:
                st = "free"

            if prev_st is not None and st != prev_st:
                events.append({"t": t, "from": prev_st, "to": st, "raw": raw_st, "reason": reason})
            prev_st = st

    holds_by_reason = {}
    total_holds = 0
    total_giveups = 0

    for ev in events:
        if ev["to"] == "hold":
            total_holds += 1
            reason = ev["reason"]
            if reason == "unspecified" and log_reasons and ev["t"] is not None:
                # Check log reason around this timestamp
                for log_t, log_r in log_reasons.items():
                    if abs(log_t - ev["t"]) <= 1.0:
                        reason = log_r
                        break
            holds_by_reason[reason] = holds_by_reason.get(reason, 0) + 1
        elif ev["to"] == "giveup":
            total_giveups += 1

    return {
        "total_holds": total_holds,
        "holds_by_reason": holds_by_reason,
        "total_giveups": total_giveups,
        "events": events,
    }


def analyze_lateral_bulge(data_dir, threshold=0.50):
    """Compute lateral distance from local plans to active global plan, flagging bulges > 0.5m."""
    data_dir = Path(data_dir)
    gplan_file = data_dir / "global_plan.csv"
    lplan_file = data_dir / "local_plan.csv"
    teb_file = data_dir / "teb_feedback.csv"

    if not gplan_file.is_file():
        return {"bulge_count": 0, "max_bulge": 0.0, "intervals": []}

    # 1. Load global plans
    global_plans = []
    with open(gplan_file, "r", encoding="utf-8", errors="ignore") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if not row:
                continue
            t = parse_timestamp(row[0])
            if t is None:
                continue
            pts = []
            for idx in range(7, len(row), 10):
                try:
                    pts.append((float(row[idx]), float(row[idx + 1])))
                except (ValueError, IndexError):
                    break
            if pts:
                global_plans.append({"t": t, "pts": pts, "goal": pts[-1]})
    global_plans.sort(key=lambda p: p["t"])

    if not global_plans:
        return {"bulge_count": 0, "max_bulge": 0.0, "intervals": []}

    # 2. Load local plans (prefer local_plan.csv, fallback to teb_feedback.csv)
    local_plans = []
    if lplan_file.is_file():
        with open(lplan_file, "r", encoding="utf-8", errors="ignore") as f:
            reader = csv.reader(f)
            next(reader, None)
            for row in reader:
                if not row:
                    continue
                t = parse_timestamp(row[0])
                if t is None:
                    continue
                pts = []
                for idx in range(7, len(row), 10):
                    try:
                        pts.append((float(row[idx]), float(row[idx + 1])))
                    except (ValueError, IndexError):
                        break
                if pts:
                    local_plans.append({"t": t, "pts": pts})
    elif teb_file.is_file():
        with open(teb_file, "r", encoding="utf-8", errors="ignore") as f:
            reader = csv.reader(f)
            next(reader, None)
            for row in reader:
                if not row:
                    continue
                t = parse_timestamp(row[0])
                if t is None:
                    continue
                odom_idx = [i for i, v in enumerate(row) if v == "odom"]
                if len(odom_idx) < 3:
                    continue
                obs_hdr = odom_idx[2]
                traj_raw = row[7 : obs_hdr - 3]
                pts = []
                for p in range(len(traj_raw) // 20):
                    try:
                        pts.append((float(traj_raw[p * 20]), float(traj_raw[p * 20 + 1])))
                    except (ValueError, IndexError):
                        pass
                if pts:
                    local_plans.append({"t": t, "pts": pts})

    local_plans.sort(key=lambda p: p["t"])
    if not local_plans:
        return {"bulge_count": 0, "max_bulge": 0.0, "intervals": []}

    def get_global_plan(t_query):
        best = global_plans[0]
        for gp in global_plans:
            if gp["t"] <= t_query:
                best = gp
            else:
                break
        return best

    flagged_cycles = []
    for lp in local_plans:
        t = lp["t"]
        gp = get_global_plan(t)
        gpts = gp["pts"]

        max_lat = 0.0
        for px, py in lp["pts"]:
            d = point_to_polyline_dist(px, py, gpts)
            if d > max_lat:
                max_lat = d

        if max_lat > threshold:
            flagged_cycles.append({"t": t, "max_lat": max_lat, "goal": gp["goal"]})

    # Cluster contiguous cycles into intervals
    intervals = []
    cur_int = []
    for c in flagged_cycles:
        if not cur_int:
            cur_int.append(c)
        else:
            if c["t"] - cur_int[-1]["t"] <= 0.60:
                cur_int.append(c)
            else:
                intervals.append(cur_int)
                cur_int = [c]
    if cur_int:
        intervals.append(cur_int)

    summary_intervals = []
    max_all_bulge = 0.0
    for int_cycles in intervals:
        t0 = int_cycles[0]["t"]
        t1 = int_cycles[-1]["t"]
        peak = max(c["max_lat"] for c in int_cycles)
        max_all_bulge = max(max_all_bulge, peak)
        summary_intervals.append({
            "t_start": t0,
            "t_end": t1,
            "duration": max(t1 - t0, 0.05),
            "cycles": len(int_cycles),
            "max_bulge": peak,
            "goal": int_cycles[0]["goal"],
        })

    return {
        "bulge_count": len(summary_intervals),
        "max_bulge": max_all_bulge,
        "intervals": summary_intervals,
    }


def main():
    parser = argparse.ArgumentParser(description="Consolidated Navigation Diagnostic Analyzer")
    parser.add_argument("run_dir", nargs="?", default="CollaborativeAMRStockerization/catkin_ws/diag_run4",
                        help="Path to diagnostic directory containing CSV dumps")
    parser.add_argument("--deadband", type=float, default=0.05, help="w sign flip deadband (rad/s)")
    parser.add_argument("--cruise-speed", type=float, default=0.30, help="Full speed threshold (m/s)")
    parser.add_argument("--bulge-thr", type=float, default=0.50, help="Lateral bulge threshold (m)")
    args = parser.parse_args()

    data_dir = find_dataset_dir(args.run_dir)
    if not data_dir.is_dir():
        print(f"Error: dataset directory not found: {args.run_dir}", file=sys.stderr)
        sys.exit(1)

    # 1. Load cmd_vel / cmd_vel_nav
    cmd_records = []
    cmd_file = data_dir / "cmd_vel_nav.csv"
    if not cmd_file.is_file():
        cmd_file = data_dir / "cmd_vel.csv"

    if cmd_file.is_file():
        with open(cmd_file, "r", encoding="utf-8", errors="ignore") as f:
            reader = csv.DictReader(f)
            for row in reader:
                t = parse_timestamp(row.get("%time") or row.get("time"))
                if t is None:
                    continue
                try:
                    vx = float(row.get("field.linear.x", 0.0))
                    wz = float(row.get("field.angular.z", 0.0))
                    cmd_records.append((t, vx, wz))
                except ValueError:
                    pass

    # 2. Analyze w flips
    flip_res = analyze_flips(cmd_records, deadband=args.deadband, full_speed_threshold=args.cruise_speed)

    # 3. Analyze Gate transitions
    log_reasons = load_rosout_reasons(data_dir)
    gate_res = analyze_gate(data_dir, log_reasons)

    # 4. Analyze Lateral Bulges
    bulge_res = analyze_lateral_bulge(data_dir, threshold=args.bulge_thr)

    # 5. Analyze TEB re-inits and re-spins from rosout
    rosout_res = analyze_rosout(data_dir)

    # Print Clean Formatted Report
    print("=" * 65)
    print(f"  NAV DIAGNOSTIC REPORT: {data_dir.name}")
    print(f"  Directory: {data_dir}")
    if flip_res["sample_count"] > 0:
        print(f"  Window   : t = {flip_res['t_start']:.2f}s -> {flip_res['t_end']:.2f}s "
              f"(Duration: {flip_res['total_duration']:.2f}s, Moving: {flip_res['moving_duration']:.2f}s)")
    print("=" * 65)

    print("\n1. ANGULAR VELOCITY (w) SIGN FLIPS (deadband >= %.2f rad/s):" % args.deadband)
    print(f"   - Total w sign flips        : {flip_res['total_flips']}")
    print(f"   - Flips at full speed       : {flip_res['cruising_flips']} (vx >= {args.cruise_speed:.2f} m/s)")
    print(f"   - Flips per minute (total)  : {flip_res['flips_per_min']:.2f} flips/min")
    print(f"   - Flips per moving-minute   : {flip_res['flips_per_moving_min']:.2f} flips/moving-min")

    print("\n2. SAFETY GATE TRANSITIONS & REASONS:")
    print(f"   - Total HOLD transitions    : {gate_res['total_holds']}")
    if gate_res["holds_by_reason"]:
        for reason, count in sorted(gate_res["holds_by_reason"].items()):
            print(f"     * {reason:<22}: {count}")
    else:
        print("     * (No HOLD transitions recorded)")
    print(f"   - Total GIVEUP transitions  : {gate_res['total_giveups']}")

    print("\n3. LATERAL PLAN BULGES (> %.2f m from Navfn global plan):" % args.bulge_thr)
    print(f"   - Significant bulge intervals: {bulge_res['bulge_count']}")
    print(f"   - Peak lateral bulge distance: {bulge_res['max_bulge']:.3f} m")
    if bulge_res["intervals"]:
        for idx, iv in enumerate(bulge_res["intervals"], 1):
            goal_str = f"({iv['goal'][0]:.2f}, {iv['goal'][1]:.2f})" if iv["goal"] else "N/A"
            print(f"     [{idx}] t = {iv['t_start']:.2f}s -> {iv['t_end']:.2f}s (dur: {iv['duration']:.2f}s, {iv['cycles']} cycles) | "
                  f"max_bulge: {iv['max_bulge']:.2f}m | goal: {goal_str}")
    else:
        print("     * (No lateral bulges > 0.50 m detected)")

    print("\n4. TEB RE-INITS & RE-SPINS (from rosout log):")
    print(f"   - Total TEB re-inits        : {rosout_res['total_reinits']}")
    if rosout_res["reinit_by_reason"]:
        for reason, count in sorted(rosout_res["reinit_by_reason"].items()):
            print(f"     * {reason:<22}: {count}")
    else:
        print("     * (No TEB re-inits recorded)")
    print(f"   - Re-spins within 3 s       : {rosout_res['re_spins']}")

    print("=" * 65)


if __name__ == "__main__":
    main()
