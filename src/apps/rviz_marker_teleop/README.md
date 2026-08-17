# rviz_marker_teleop

Config-driven local-PC RViz interactive-marker pose source for Physical AI
Runtime controller bringup apps.

This app does not implement interactive-marker behavior. It composes one or
more nodes from the lower-level `rviz_interactive_marker_pose_source` toolbox
package with embodiment-specific frames, topics, marker identities, and RViz
configuration. Robot profiles belong here; reusable marker behavior belongs
in the toolbox package.

Both bundled profiles use the same pipeline:

```text
this machine                         robot/CPU host
────────────                         ──────────────
RViz interactive marker  --DDS-->   EM -> TSKPC -> ros2_control hardware
```

This package starts only marker source nodes and optional RViz. It never
starts an execution manager, controller manager, or hardware.

## Marvin

```bash
# Robot/CPU host
ros2 launch marvin_controller_bringup controller_bringup.launch.py

# Local PC (`profile:=marvin` is the default)
ros2 launch rviz_marker_teleop rviz_marker_teleop.launch.py \
  profile:=marvin
```

`config/marvin.yaml` creates two markers:

- `Base_L` -> `flange_L` -> `/action_sources/marker_left/pose_target`
- `Base_R` -> `flange_R` -> `/action_sources/marker_right/pose_target`

## Franka FR3

```bash
# Robot/CPU host
ros2 launch franka_controller_bringup controller_bringup.launch.py

# Local PC
ros2 launch rviz_marker_teleop rviz_marker_teleop.launch.py \
  profile:=franka
```

`config/franka.yaml` creates one marker:

- `fr3_link0` -> `fr3_link8` -> `/action_sources/marker/pose_target`

These topics exactly match the execution-manager routes in their corresponding
controller bringup packages.

The description packages' stock RViz files do not preconfigure
`InteractiveMarkers` displays. Add one display per configured
`server_namespace` in RViz:

- Marvin: `target_marker_left`, `target_marker_right`
- Franka: `target_marker_franka`

## Custom profile

Pass an absolute YAML path:

```bash
ros2 launch rviz_marker_teleop rviz_marker_teleop.launch.py \
  config_file:=/absolute/path/to/robot_marker.yaml
```

The schema is:

```yaml
rviz:
  package: robot_description_package
  config: rviz/robot.rviz

markers:
  - name: arm
    base_frame: base_link
    target_frame: flange_link
    pose_topic: /action_sources/marker/pose_target
    publish_frequency: 50.0
    server_namespace: target_marker_arm
    marker_name: robot_target
    marker_description: Robot Target Pose
```

Every marker entry creates one independent source node. `name` must be unique.

## Distributed operation

Both hosts must share `ROS_DOMAIN_ID`. For a dedicated LAN, configure
CycloneDDS using `.config/cyclonedds_template.xml`. Marker nodes wait for the
configured base-to-target TF before creating their initial marker poses.

## Launch arguments

| Argument | Default | Description |
|---|---|---|
| `profile` | `marvin` | Bundled `marvin` or `franka` profile |
| `config_file` | empty | Absolute custom YAML; overrides `profile` |
| `use_rviz` | `true` | Start RViz with the profile's config |
