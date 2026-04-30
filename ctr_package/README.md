# CTR Package (Concentric Tube Robot)

This package provides a simple simulated concentric tube robot (CTR) with keyboard teleoperation in ROS 2.

## What this includes

- Robot model (Xacro/URDF)
- RViz visualization
- Keyboard teleop split into two nodes:
  - `ctr_key_listener.py`: reads keys and publishes commands on `/ctr_command`
  - `ctr_keyboard_teleop.py`: listens to `/ctr_command` and publishes `/joint_states`

---

## Prerequisites

- Ubuntu + ROS 2 Humble installed
- `colcon` available
- `xterm` installed (used to capture keyboard in a dedicated terminal)

Install xterm if needed:

```bash
sudo apt update
sudo apt install -y xterm
```

---

## Build

From the `flexible_catheter_simulation` workspace:

```bash
cd /home/dm55947-admin/Documents/danislicer/src/flexible_catheter_simulation
colcon build --packages-select ctr_package
source install/setup.bash
```

---

## Run CTR teleop

```bash
ros2 launch ctr_package ctr_teleop.launch.py
```

This launches:

1. `robot_state_publisher`
2. `ctr_key_listener.py` (in an `xterm` window)
3. `ctr_keyboard_teleop.py`
4. `rviz2`

> Important: press keys in the **xterm keyboard listener window**, not in RViz.

---

## 3D Slicer integration (heart/aorta + CTR)

This package can be visualized in 3D Slicer using the SlicerROS2 module, similar to the ISMR catheter workflow.

### Quick workflow (2 terminals)

Terminal A (CTR):

```bash
cd /home/dm55947-admin/Documents/danislicer
./run_ctr_teleop.sh
```

Terminal B (Slicer):

```bash
cd /home/dm55947-admin/Documents/danislicer
./run_slicer_ros2.sh
```

In Slicer:

1. Load anatomy scene:
  - `File -> Load Scene...`
  - `/home/dm55947-admin/Documents/danislicer/src/flexible_catheter_simulation/slicer_scene/2026-04-14-Scene.mrml`
2. Open module `IGT -> ROS2`
3. `+ Add new robot` and set:
  - Robot name: `ctr`
  - Parameter node name: `/robot_state_publisher`
  - Parameter name: `robot_description`
  - Fixed frame: `world`
  - Tf2 prefix: *(empty)*
4. Click `Load robot`

For full instructions, see:

- `/home/dm55947-admin/Documents/danislicer/CTR_SLICER_INTEGRATION.md`

---

## Keyboard commands

- `J` / `L` : move base in X (- / +)
- `K` / `I` : move base in Y (- / +)
- `U` / `O` : move base in Z (- / +)
- `W` : insert inner tube
- `S` : retract inner tube
- `A` : rotate counter-clockwise
- `D` : rotate clockwise
- `R` : reset insertion + rotation
- `Q` : quit keyboard listener

---

## How control works

- Each key press is published as a `std_msgs/Char` message on `/ctr_command`.
- The teleop node receives commands and updates CTR joint values.
- Updated `JointState` messages are published on `/joint_states`.

---

## Useful debug commands

Check command topic:

```bash
source /home/dm55947-admin/Documents/danislicer/src/flexible_catheter_simulation/install/setup.bash
ros2 topic echo /ctr_command
```

Check joint states:

```bash
ros2 topic echo /joint_states
```

List running CTR nodes:

```bash
ros2 node list | grep ctr
```

---

## Troubleshooting

### No response to key presses

- Ensure you are typing in the `xterm` window.
- Verify `ctr_key_listener.py` is running:

```bash
ros2 node list | grep ctr_key_listener
```

### Launch fails because `xterm` is missing

Install it:

```bash
sudo apt install -y xterm
```

### Commands published but robot does not move

Check teleop subscriber:

```bash
ros2 node list | grep ctr_teleop
```

and verify `/joint_states` updates:

```bash
ros2 topic echo /joint_states
```

---

## Stop

Press `Ctrl+C` in the launch terminal to stop all nodes.
