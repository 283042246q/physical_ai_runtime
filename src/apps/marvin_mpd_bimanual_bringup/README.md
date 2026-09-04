# marvin_mpd_bimanual_bringup

Independent hardware-description composition for MPD bimanual work. It adds a
new entrypoint without changing or replacing the existing arm-only Marvin
description and bringup packages.

## Current scope (phase 4, steps 9-10)

- `urdf/marvin_pika_bimanual.urdf.xacro` composes the Marvin stand, both
  seven-joint arms, two Marvin-to-Pika adaptors, and two prefixed Pika grippers.
- The URDF contains three independent `ros2_control` systems: one 14-joint
  Marvin system and one system per Pika gripper.
- All three systems use `mock_components/GenericSystem` by default. Real Marvin
  and Pika plugins require explicit per-device opt-in.
- `config/pika_mounts.yaml` is the site-owned location for calibrated mounting
  transforms. The reusable external description packages remain untouched.

Controller YAML, execution-manager routing, and launch entrypoints are later
bringup steps and intentionally are not added here yet.

## Build and validate

```bash
cd ~/Projects/physical_ai_runtime
pixi run colcon build --symlink-install \
  --packages-up-to marvin_mpd_bimanual_bringup
pixi run colcon test --packages-select marvin_mpd_bimanual_bringup \
  --event-handlers console_direct+
pixi run colcon test-result \
  --test-result-base build/marvin_mpd_bimanual_bringup --verbose
```

After sourcing the overlay, expand the default fake-hardware description:

```bash
source install/setup.bash
xacro install/marvin_mpd_bimanual_bringup/share/\
marvin_mpd_bimanual_bringup/urdf/marvin_pika_bimanual.urdf.xacro \
  > /tmp/marvin_pika_bimanual.urdf
check_urdf /tmp/marvin_pika_bimanual.urdf
```

The important joint names exposed to later controller/MPD integration are:

```text
Joint1_L .. Joint7_L
Joint1_R .. Joint7_R
left_gripper_left_joint   (left_gripper_right_joint is a mimic joint)
right_gripper_left_joint  (right_gripper_right_joint is a mimic joint)
```

## Real-hardware expansion

Expanding the real-hardware path does not connect to hardware; loading it into
`controller_manager` does. Use stable udev aliases for the two independent Pika
serial devices:

```bash
xacro install/marvin_mpd_bimanual_bringup/share/\
marvin_mpd_bimanual_bringup/urdf/marvin_pika_bimanual.urdf.xacro \
  use_fake_arm_hardware:=false \
  use_fake_left_gripper_hardware:=false \
  use_fake_right_gripper_hardware:=false \
  robot_ip:=10.19.0.191 \
  left_gripper_serial_port:=/dev/pika_left \
  right_gripper_serial_port:=/dev/pika_right
```

Do not start a real `controller_manager` until the robot is powered, safed, and
an operator is at the emergency stop.
