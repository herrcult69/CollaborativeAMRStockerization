#!/usr/bin/env bash
# record_diag.sh - Step 1 Diagnostic Recorder for TEB Local Planner & Velocity Gate
# Records high-rate diagnostic topics to CSV format via rostopic echo -p.
#
# Usage inside the ROS container (ros1_amr_core):
#   bash record_diag.sh [output_dir]
#
# Running from host via docker:
#   docker exec -i ros1_amr_core bash -s /catkin_ws/diag_latest < .agents/scratch/record_diag.sh
#   OR
#   docker cp .agents/scratch/record_diag.sh ros1_amr_core:/catkin_ws/record_diag.sh
#   docker exec -it ros1_amr_core bash /catkin_ws/record_diag.sh /catkin_ws/diag_latest

set -e

# Always source ROS (ROS_DISTRO can be set while rostopic is still off PATH)
source /opt/ros/noetic/setup.bash
[ -f /catkin_ws/devel/setup.bash ] && source /catkin_ws/devel/setup.bash || true

OUTDIR="${1:-/catkin_ws/diag_latest}"
mkdir -p "$OUTDIR"

echo "=========================================================="
echo "[record_diag] Step 1 Diagnostic Recorder Active"
echo "[record_diag] Output directory: $OUTDIR"
echo "[record_diag] Press Ctrl-C to finish recording and flush CSVs."
echo "=========================================================="

# List of background process IDs
PIDS=()

cleanup() {
    echo ""
    echo "[record_diag] Signal received. Terminating rostopic loggers..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGINT "$pid" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
    echo "[record_diag] Flush complete. Saved CSV files in $OUTDIR:"
    ls -lh "$OUTDIR"/*.csv 2>/dev/null || echo "No CSV files generated."
    echo "=========================================================="
    exit 0
}

# Trap SIGINT (Ctrl-C), SIGTERM, and EXIT
trap cleanup SIGINT SIGTERM EXIT

# 1. TEB Trajectory Feedback (trajectories, selected index, obstacles)
rostopic echo -p /move_base/TebLocalPlannerROS/teb_feedback > "$OUTDIR/teb_feedback.csv" 2>/dev/null &
PIDS+=($!)
echo "  [+] Recording /move_base/TebLocalPlannerROS/teb_feedback -> teb_feedback.csv (PID: ${PIDS[-1]})"

# 2. Raw Planner Velocity Commands (before velocity_gate)
rostopic echo -p /cmd_vel_nav > "$OUTDIR/cmd_vel_nav.csv" 2>/dev/null &
PIDS+=($!)
echo "  [+] Recording /cmd_vel_nav -> cmd_vel_nav.csv (PID: ${PIDS[-1]})"

# 3. Filtered Velocity Commands (output of velocity_gate to robot)
rostopic echo -p /cmd_vel > "$OUTDIR/cmd_vel.csv" 2>/dev/null &
PIDS+=($!)
echo "  [+] Recording /cmd_vel -> cmd_vel.csv (PID: ${PIDS[-1]})"

# 4. TEB Local Plan Path
rostopic echo -p /move_base/TebLocalPlannerROS/local_plan > "$OUTDIR/local_plan.csv" 2>/dev/null &
PIDS+=($!)
echo "  [+] Recording /move_base/TebLocalPlannerROS/local_plan -> local_plan.csv (PID: ${PIDS[-1]})"

# 5. Navfn Global Plan Path
rostopic echo -p /move_base/NavfnROS/plan > "$OUTDIR/global_plan.csv" 2>/dev/null &
PIDS+=($!)
echo "  [+] Recording /move_base/NavfnROS/plan -> global_plan.csv (PID: ${PIDS[-1]})"

# 6. Velocity Gate State (/velocity_gate/state: free, hold, giveup)
rostopic echo -p /velocity_gate/state > "$OUTDIR/gate_state.csv" 2>/dev/null &
PIDS+=($!)
echo "  [+] Recording /velocity_gate/state -> gate_state.csv (PID: ${PIDS[-1]})"

echo "=========================================================="
echo "[record_diag] All loggers running. Now execute the test mission in Unity/ROS."
echo "[record_diag] Waiting for Ctrl-C to finish..."
echo "=========================================================="

# Block until user sends Ctrl-C
wait
