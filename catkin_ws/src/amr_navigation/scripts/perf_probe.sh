#!/usr/bin/env bash
# perf_probe.sh: Probes navigation rates, missed controller loops, and CPU usage for 60s.
# Evaluates PASS/FAIL against Section B acceptance criteria:
# - /cmd_vel_nav >= 9.0 Hz
# - /clock >= 95.0 Hz
# - move_base missed rate warnings <= 1 per minute
# - move_base CPU < 60% of one core (mean over the run; the first top sample is skipped, it averages since process start)
# Run inside the ROS container while the sim route is running. Rates are in ROS time (sim time when use_sim_time is set).
# Usage: perf_probe.sh [seconds]   (default 60)
set -euo pipefail
export PYTHONUNBUFFERED=1   # rostopic prints through a file; without this the buffer is lost when we kill it

DURATION="${1:-60}"
TMP_DIR=$(mktemp -d /tmp/perf_probe_XXXXXX)
trap 'rm -rf "$TMP_DIR"' EXIT

echo "=== AMR Navigation Performance Probe (${DURATION}s) ==="

# Check active ROS master
if ! rostopic list > /dev/null 2>&1; then
    echo "ERROR: roscore or ros_tcp_endpoint not reachable."
    exit 1
fi

echo "[1/4] Sampling topic frequencies for ${DURATION}s..."
rostopic hz /clock > "$TMP_DIR/hz_clock.txt" 2>&1 &
PID_CLOCK=$!
rostopic hz /odom > "$TMP_DIR/hz_odom.txt" 2>&1 &
PID_ODOM=$!
rostopic hz /scan > "$TMP_DIR/hz_scan.txt" 2>&1 &
PID_SCAN=$!
rostopic hz /cmd_vel_nav > "$TMP_DIR/hz_cmd_vel_nav.txt" 2>&1 &
PID_CMD_NAV=$!
rostopic hz /cmd_vel > "$TMP_DIR/hz_cmd_vel.txt" 2>&1 &
PID_CMD_VEL=$!

echo "[2/4] Capturing move_base rosout log lines..."
rostopic echo -p /rosout > "$TMP_DIR/rosout.csv" 2>&1 &
PID_ROSOUT=$!

echo "[3/4] Sampling CPU load (top)..."
top -b -d 2 -n $(( DURATION / 2 )) > "$TMP_DIR/top.txt" 2>&1 &
PID_TOP=$!

sleep "$DURATION"

# Wait or kill sampling processes
kill "$PID_CLOCK" "$PID_ODOM" "$PID_SCAN" "$PID_CMD_NAV" "$PID_CMD_VEL" "$PID_ROSOUT" "$PID_TOP" 2>/dev/null || true
wait "$PID_CLOCK" "$PID_ODOM" "$PID_SCAN" "$PID_CMD_NAV" "$PID_CMD_VEL" "$PID_ROSOUT" "$PID_TOP" 2>/dev/null || true

echo "[4/4] Analyzing performance probe telemetry..."

extract_rate() {
    local file="$1"
    if [ ! -f "$file" ]; then
        echo "0.0"
        return
    fi
    local r
    r=$(grep -oP 'average rate:\s*\K[0-9.]+' "$file" | tail -n 1 || true)
    echo "${r:-0.0}"
}

RATE_CLOCK=$(extract_rate "$TMP_DIR/hz_clock.txt")
RATE_ODOM=$(extract_rate "$TMP_DIR/hz_odom.txt")
RATE_SCAN=$(extract_rate "$TMP_DIR/hz_scan.txt")
RATE_CMD_NAV=$(extract_rate "$TMP_DIR/hz_cmd_vel_nav.txt")
RATE_CMD_VEL=$(extract_rate "$TMP_DIR/hz_cmd_vel.txt")

MISSED_LOOPS=0
if [ -f "$TMP_DIR/rosout.csv" ]; then
    MISSED_LOOPS=$(grep -c "Control loop missed its desired rate" "$TMP_DIR/rosout.csv" || true)
fi

# top column 9 is %CPU; drop the first sample (since-start average), then take mean and peak
MOVE_BASE_CPU=$(grep -E " move_base$" "$TMP_DIR/top.txt" | awk 'NR > 1 {s += $9; n++} END {printf "%.1f", n ? s / n : 0}')
MOVE_BASE_CPU_PEAK=$(grep -E " move_base$" "$TMP_DIR/top.txt" | awk 'NR > 1 && $9 > m {m = $9} END {printf "%.1f", m + 0}')

echo "--------------------------------------------------------"
echo "Topic Rates:"
echo "  /clock:       ${RATE_CLOCK} Hz (target: >= 95.0 Hz)"
echo "  /odom:        ${RATE_ODOM} Hz"
echo "  /scan:        ${RATE_SCAN} Hz"
echo "  /cmd_vel_nav: ${RATE_CMD_NAV} Hz (target: >= 9.0 Hz)"
echo "  /cmd_vel:     ${RATE_CMD_VEL} Hz"
echo "Stability & CPU:"
echo "  Missed control loops in move_base: ${MISSED_LOOPS} (target: <= 1 per minute)"
echo "  move_base CPU, mean (peak ${MOVE_BASE_CPU_PEAK}%):  ${MOVE_BASE_CPU}% (target: < 60.0%)"
echo "--------------------------------------------------------"

PASS=true

# Evaluate acceptance criteria
if awk "BEGIN {exit !($RATE_CLOCK < 95.0)}"; then
    echo "[FAIL] /clock rate (${RATE_CLOCK} Hz) < 95.0 Hz"
    PASS=false
fi

if awk "BEGIN {exit !($RATE_CMD_NAV < 9.0)}"; then
    echo "[FAIL] /cmd_vel_nav rate (${RATE_CMD_NAV} Hz) < 9.0 Hz"
    PASS=false
fi

if [ "$MISSED_LOOPS" -gt 1 ]; then
    echo "[FAIL] Missed control loops (${MISSED_LOOPS}) > 1"
    PASS=false
fi

if awk "BEGIN {exit !($MOVE_BASE_CPU >= 60.0)}"; then
    echo "[FAIL] Mean move_base CPU (${MOVE_BASE_CPU}%) >= 60.0%"
    PASS=false
fi

if [ "$PASS" = true ]; then
    echo "OVERALL VERDICT: PASS"
    exit 0
else
    echo "OVERALL VERDICT: FAIL"
    exit 1
fi
