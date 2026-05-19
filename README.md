# rm_bringup

Bringup package for dual RM-65 arms. Provides launch files that wire together the driver, controller, planning adapter, and MoveIt stack.

---

## Launch files

| File | Description |
|---|---|
| `dual_rm_65_hw_bringup.launch.py` | Starts the two RM-65 driver nodes, the dual-arm controller, and optionally the planning adapter. |
| `dual_rm_65_moveit.launch.py` | Starts `move_group`, `robot_state_publisher`, and optionally RViz for the dual-arm setup. |

Internally, `dual_rm_65_hw_bringup.launch.py` includes:
- `rm_driver/dual_rm_65_driver.launch.py` — spawns `left_rm_driver` and `right_rm_driver`
- `rm_control/dual_rm_65_control.launch.py` — spawns the dual-arm joint trajectory controller

---

## Dual-arm simulation

### 1. Start the hardware bringup in simulation mode

```bash
ros2 launch rm_bringup dual_rm_65_hw_bringup.launch.py simulation:=true
```

This replaces the real `rm_driver` executables with `fake_rm_driver` nodes that mirror
`movej_canfd` commands directly into `/joint_states` without any hardware connection.

To also start the standalone planning adapter:

```bash
ros2 launch rm_bringup dual_rm_65_hw_bringup.launch.py simulation:=true use_planning_adapter:=true
```

### 2. Start the MoveIt stack

In a separate terminal:

```bash
ros2 launch rm_bringup dual_rm_65_moveit.launch.py
```

To open RViz at the same time:

```bash
ros2 launch rm_bringup dual_rm_65_moveit.launch.py launch_rviz:=true
```

---

## Gripper launch

The `rm_gripper` package provides a launch file that starts the Modbus gripper node and lets you choose which gripper implementation to run, make sure rm_driver is running before launching the rm_gripper.

```bash
ros2 launch rm_gripper rm_gripper.launch.py gripper_type:=robotiq
```

```bash
ros2 launch rm_gripper rm_gripper.launch.py gripper_type:=hitbot
```

`gripper_type` accepts:
- `robotiq` (default)
- `hitbot`

---

## Launch arguments

### `dual_rm_65_hw_bringup.launch.py`

| Argument | Default | Description |
|---|---|---|
| `simulation` | `false` | Set to `true` to use `fake_rm_driver` instead of the real driver (no TCP/UDP hardware required). |
| `use_planning_adapter` | `true` | Set to `true` to launch the `standalone_dual_arm_adapter` planning adapter node. |

### `dual_rm_65_moveit.launch.py`

| Argument | Default | Description |
|---|---|---|
| `launch_rviz` | `false` | Set to `true` to start RViz with the MoveIt config. |
| `joint_states_topic` | `/joint_states` | Topic that `move_group` subscribes to for joint states. |
| `follow_joint_trajectory_action` | `/dual_arm_controller/follow_joint_trajectory` | Action server used by `move_group` for trajectory execution. |
