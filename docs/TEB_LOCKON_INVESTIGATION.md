# TEB local planner looping / fishtailing: investigation log

Date: 2026-11-13. Status: open. One parameter change applied (2B). The long detour loop is not fixed.

## Problem
The TEB local plan (blue, `/move_base/TebLocalPlannerROS/local_plan`) sometimes loops or fishtails around racks while the navfn global plan (green) stays clean. `velocity_gate` then HOLDs, and after 10 s of HOLD it GIVEs UP, which publishes a 60 s blocker and forces a replan.

## Method and why
The first idea was to change TEB parameters directly. We instead measured first, so each change would be justified by data and tested one at a time.

1. Added `publish_feedback: true` to `config/teb_warehouse.yaml` (diagnostic only) so TEB publishes `teb_feedback`.
2. Recorded six topics to CSV with a script (`record_diag.sh`): `teb_feedback`, `cmd_vel_nav`, `cmd_vel`, `local_plan`, Navfn plan, `velocity_gate/state`. CSV with `rostopic echo -p` can be analysed offline and compared between runs.
3. Had the helper agent (agy) write analysers and briefs. Its conclusions were spot-checked against the code and data, and several were rejected (see "Reliability of the analysis").
4. Ran two recordings before the change (runs 1 and 2) and two after (runs 3 and 4).

## Fix tried: 2B (viapoint anchor)
In `teb_warehouse.yaml`: `global_plan_viapoint_sep` -0.1 -> 0.4 and `weight_viapoint` 0.0 -> 1.0.

Why: the data showed TEB hunting left and right in open aisle with a tiny (0.06-0.07 m), one-sided deviation. With `weight_viapoint: 0` nothing ties the band to the global path.

### What was effective
| Metric (about 138 s each) | Before (run 2) | After (run 3) |
|---|---|---|
| `w` sign flips | 58 (25/min) | 41 (18/min) |
| Flips at full speed | 55 | 28 |
| Gate HOLDs | 10 | 3 |
| False "crossing obstacle" HOLDs | 7 | 2 |

Caveat: run 3 contains a 42 s stationary dwell, so per-minute rates are flattered. Single run each.

### What was not effective
Run 4 (manual RViz goals, post-2B) still had a ~35 s detour loop: the local plan bulged up to 2.5 m from the straight global path although the goal was directly ahead. It began right after a false crossing-obstacle HOLD (t=101.8 s, 2.6 s freeze). The link is plausible, not proven. The anchor only constrains TEB near the sampled global points and does not pull the band back after a stop.

## Hypotheses rejected, and why
- 2A (raise obstacle clearance): TEB clearance was already 0.43-0.58 m at the HOLDs, and 0.277 m in run 1 without any HOLD.
- Side-flip around obstacles: deviations during fishtailing were one-sided and small, with bursts in open aisle.
- "Goal behind the robot" as the main loop cause: it explains only one smaller event (t=61 s in run 4).
- Costmap not clearing inflation behind objects the lidar sees: noted by the user, deliberately not pursued.

## Mechanism for the false HOLDs (plausible, not directly tested)
`gate_logic.py:_update_crossing` compensates the previous obstacle centroids for ego motion using the commanded v and w. While the robot wobbles, a static rack leg appears to move sideways, so the gate reports `yield: crossing obstacle` even though the corridor ahead is clear (0.74-0.96 m).

## Reliability of the analysis
- Early agy briefs mixed inference with fact; those were rejected and re-run with stricter instructions.
- agy's cumulative-turn "loop" criterion produces false positives. Only the lateral deviation of the local plan from the current global plan (more than 0.5 m) is reliable.
- agy's reported thresholds for the gate (`d_inner`, `d_hold_static`) disagreed with earlier reads of the code; re-verify before relying on them.
- The robot-position track agy gave for the run 4 loop was not independently verified. The commanded v/w around the loop was.

## Tooling pitfalls to remember
- Scripts written on Windows may have CRLF endings and fail in the container (exit 127). Convert with `sed -i 's/\r$//'`.
- `rostopic echo ... 2>/dev/null` hid a missing ROS source. Always source `/opt/ros/noetic/setup.bash`.
- `herdr pane wait-output` can match old scrollback.
- A `&` inside a background Bash call returns "completed" immediately; wait on the agent itself.

## Open next steps (none approved)
1. Decide whether false HOLDs cause the loops (log TEB state across a HOLD, or run with the freeze disabled).
2. Gate side: stop static rack legs looking like crossing obstacles during turns.
3. TEB side: bound detours (`max_global_plan_lookahead_dist`, viapoint weight) or re-initialise after a HOLD.
4. Check the 5 Hz `controller_frequency` if wobble persists.
5. Compare against older branches that did not loop.
6. Remove `publish_feedback: true` when diagnosis ends.

Detailed run-by-run notes live in `.agents/findings.md` (not versioned).
