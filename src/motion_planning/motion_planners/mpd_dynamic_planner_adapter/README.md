# MPD Phase 4 dynamic planner adapter

This package is a separate dynamic-world entrypoint.  It depends on and reuses
the pure Phase-3 trajectory/coordinator/handoff utilities, but it does not alter
`mpd_planner_adapter/replan.launch.py` or its static worker contract.

The first implementation gate contains the constant-velocity Kalman filter,
versioned world snapshots, fixed-timing collision-sphere validation, earliest
safe low-speed handoff selection, and bounded braking trajectory generation.
The ROS node and launch entry are added in the next gate.
