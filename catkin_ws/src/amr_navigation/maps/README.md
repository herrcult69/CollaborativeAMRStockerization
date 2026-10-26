# Saved training maps

`milestone_1_01` was captured from live Unity `/scan` with gmapping on 2026-10-04
for the XStack `Milestone_1` scene. Capture route: a complete turn at the starting
pose, an 8 m forward aisle traverse, then another complete turn. Resolution is
0.05 m. Coverage is limited to observed space; unknown/occluded aisles are not
certified free. AMCL initialization and three DWA goals in the mapped area were
verified. See `docs/XSTACK_VERIFICATION.md` for results.

`warehouse_training_01` is the separate legacy `WarehouseTraining` map.

Save map_saver output here so it survives container replacement via the workspace bind mount.
Use a new filename for each subsequent run, e.g.
`warehouse_training_01.pgm` and `warehouse_training_01.yaml`.

The PGM stores occupancy pixels; the YAML stores image path, resolution, origin, and thresholds.
Keep each pair together. These are outputs of a completed mapping run, not the Unity warehouse scene.

`new_map_noobs_01` was captured with gmapping on 2026-10-26 for the XStack Variant in `NewMapnoObs`
(no static clutter, 8 `Rack-DetectReady2` racks, lidar 361 rays at ~0.04 m). At that height racks show
as rows of floor-runner posts. Matches `config/locations_milestone_3.yaml` exported from the same scene.

`new_map_noobs_02` (2026-10-26) replaces `_01` as the xstack default after the `NewMapnoObs` layout changed
(racks removed from the top-right and bottom rows, small floor obstacles added). The station room is fully
explored this time. Pair it with a `locations_milestone_3.yaml` exported from the same saved layout.
