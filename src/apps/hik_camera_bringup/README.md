# hik_camera_bringup

Site application bringup for **Hikrobot** multi-camera acquisition.

Driver package: `hik_camera` (from the embodiment / sensor stack).

## Layout

| Path | Role |
| ---- | ---- |
| `config/hik_8cam.yaml` | Eight-camera serial identity + PTP Scheduled Action parameters |
| `launch/hik_8cam.launch.py` | Headless 8× camera |
| `launch/hik_8cam_foxglove.launch.py` | 8× camera + Foxglove Bridge |

Camera ROS parameters belong in this package's `config/`. Cross-machine
CycloneDDS should be set via the workspace `CYCLONEDDS_URI` (for example
`.config/cyclonedds_default.xml` in `physical_ai_runtime`).

## Build

```bash
pixi run build --packages-up-to hik_camera_bringup
source install/setup.bash
```

## Launch

```bash
# Pixi activation already sets CYCLONEDDS_URI / ROS_DOMAIN_ID.
ros2 launch hik_camera_bringup hik_8cam.launch.py

# + Foxglove (ws://127.0.0.1:8765)
ros2 launch hik_camera_bringup hik_8cam_foxglove.launch.py

# Validate YAML without opening MVS hardware
ros2 launch hik_camera_bringup hik_8cam.launch.py validate_only:=true
```

Override the parameter file when needed:

```bash
ros2 launch hik_camera_bringup hik_8cam.launch.py \
  config_file:=/absolute/path/to/site_camera.yaml
```
