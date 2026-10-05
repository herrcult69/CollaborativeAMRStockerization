"""
amr_navigation Python Package.
Provides autonomous navigation, pallet docking, and mast lift libraries.
"""
from amr_navigation.move_to_point import MoveToPointController, normalize_angle, deg_to_rad
from amr_navigation.docking import PalletDockingController, compute_dock_target_pose, compute_dock_base_pose
from amr_navigation.lift import LiftController

# Backward compatibility alias
ThreePhaseDockingController = MoveToPointController

__all__ = [
    "MoveToPointController",
    "PalletDockingController",
    "compute_dock_target_pose",
    "compute_dock_base_pose",
    "LiftController",
    "ThreePhaseDockingController",
    "normalize_angle",
    "deg_to_rad",
]
