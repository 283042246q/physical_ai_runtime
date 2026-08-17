# runtime_resources

ROS 2 application packages for
[`physical_ai_runtime`](https://github.com/Gabriel-Ning/physical_ai_runtime).
The repository is intentionally flat so it can be imported once with
`vcstool` and discovered recursively by `colcon`.

## Packages

| Package | Purpose |
|---|---|
| `hik_camera_bringup` | Hikrobot multi-camera site bringup (8× GigE + optional Foxglove) |
| `franka_controller_bringup` | Marker-free Franka TSJI (Diff-IK + joint impedance → effort) plus optional RViz debug compose |
| `franka_motion_planning_bringup` | Franka impedance+Ruckig planning: RT `planning_bringup` + operator UI split |
| `franka_trajectory_jtc_test` | Distributed EM → effort JTC full-trajectory validation (MoveIt-style, no MoveIt) |
| `marvin_motion_planning_bringup` | Config-driven PyRoki setpoint, trajectory, and MPC demos |
| `marvin_controller_bringup` | Marker-free Marvin controller bringup plus legacy all-in-one RViz debugging |
| `rviz_marker_teleop` | Config-driven local-PC RViz interactive-marker UI for distributed teleop |
| `marvin_trajectory_jtc_test` | EM-to-JTC trajectory-chain validation |
| `episode_recorder_app` | Hardware-agnostic, config-driven episode-recording app (bundled stream-contract profiles + rigorous lifecycle activation) |

Reusable workspace-owned tools such as `rviz_interactive_marker_pose_source` do not
live here. They are maintained directly under
`physical_ai_runtime/src/toolbox`.

## Import into physical_ai_runtime

`physical_ai_runtime/repos/example.repos` maps this repository to
`src/apps`:

```bash
vcs import src < repos/necessary.repos
vcs import src < repos/example.repos
vcs import src < repos/embodiment.repos
pixi run build
```

`necessary.repos` provides reusable execution and planning modules,
`example.repos` provides these application packages, and `embodiment.repos`
provides the Marvin robot description and hardware integration.

Do not copy individual packages or use the removed Example 1 fetch script.
Each directory at this repository root is an independent ROS package.

## License

See each package for its license declaration (Apache-2.0 unless noted).
