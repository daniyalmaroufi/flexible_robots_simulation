# Flexible Catheter Simulation with ROS2, Gazebo Harmonic, and 3D Slicer

A ROS2-based simulation framework for flexible catheter navigation in anatomical environments. The catheter is modeled as a serial-link chain with spring-damper universal joints, integrated with **Gazebo Harmonic** for physics simulation, **RViz2** for robot visualization, and **3D Slicer (SlicerROS2)** for intraoperative AR navigation.

Two generator scripts are provided:
- **`catheter_urdf_generator.py`** — passive deformation simulation (observe catheter deflection under external forces)
- **`cath_urdf_generator_with_controller.py`** — active control simulation with a 4-DOF motorized base and keyboard teleoperation

---

## Table of Contents

- [Repository Structure](#repository-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [1. SlicerROS2 Docker Setup](#1-slicerros2-docker-setup)
  - [2. Install Gazebo Harmonic and ROS2 Jazzy Bridge](#2-install-gazebo-harmonic-and-ros2-jazzy-bridge)
- [Usage](#usage)
  - [Passive Catheter Simulation](#passive-catheter-simulation)
  - [Controlled Catheter Simulation with Keyboard Teleop](#controlled-catheter-simulation-with-keyboard-teleop)
- [Loading Anatomy in Gazebo](#loading-anatomy-in-gazebo)
- [Catheter Model Parameters](#catheter-model-parameters)
- [References](#references)

---

## Repository Structure

```
.
├── README.md
├── docs/
│   └── images/                                 # Screenshots for README
├── catheter_urdf_generator.py                  # Passive simulation generator
├── cath_urdf_generator_with_controller.py      # Active control simulation generator
└── test_anatomy/                               # Example anatomical model (e.g. aortic arch)
    ├── meshes/                                 # Mesh files (.stl / .dae)
    ├── model.config                            # Gazebo model metadata
    └── model.sdf                               # Gazebo SDF model definition

```

Each generator script produces a self-contained **ROS2 package** in the specified output directory:

```
<output_package>/
├── package.xml
├── CMakeLists.txt
├── urdf/               # Xacro robot description
├── sdf/                # Gazebo SDF model
├── meshes/             # Auto-generated STL cylinder meshes per link
├── worlds/             # Gazebo world file
├── launch/             # ROS2 launch file
└── scripts/            # catheter_keyboard_teleop.py  (controller variant only)
```

---

## Prerequisites

- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- [ros_gz bridge (Jazzy)](https://github.com/gazebosim/ros_gz/tree/jazzy)
- Python 3 with `numpy`
- Docker (for SlicerROS2 integration)

---

## Installation

### 1. SlicerROS2 Docker Setup

Pull and run the pre-built SlicerROS2 Docker image:

```bash
docker pull rosmed/docker-ubuntu-vnc-desktop-slicerros2-lw:ismr2025
docker run -it --rm -p 6080:80 rosmed/docker-ubuntu-vnc-desktop-slicerros2-lw:ismr2025
```

Open the desktop in your browser at `http://localhost:6080`.

> Full setup instructions: https://rosmed.github.io/ismr2025/prerequisites

### 2. Install Gazebo Harmonic and ROS2 Jazzy Bridge

Follow the official guides:
- Gazebo Harmonic: https://gazebosim.org/docs/harmonic/install_ubuntu/
- ros_gz bridge: https://github.com/gazebosim/ros_gz/tree/jazzy

---

## Usage

### Passive Catheter Simulation

Simulates catheter deformation under externally applied forces. The deflection is visualized simultaneously in Gazebo, RViz2, and 3D Slicer.

**Step 1: Generate the ROS2 package**

```bash
cd ~/ros2_ws/src
python3 catheter_urdf_generator.py \
    --N 12 --D 0.003 --L1 0.20 --L2 0.5 --L3 0.05 \
    --K 0.2 --M 0.5 --output my_catheter
```

**Step 2: Build and launch**

```bash
cd ~/ros2_ws
colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch my_catheter my_catheter_launch.py
```

Apply a force to any catheter link in Gazebo to observe spring-driven deformation propagated across all three visualization tools.

---

### Controlled Catheter Simulation with Keyboard Teleop

Extends the passive simulation with a **4-DOF actuated base** (X/Y translation, Z insertion, Z-axis rotation) controlled via ROS2 position controllers and a keyboard teleoperation node.

**Step 1: Generate the ROS2 package**

```bash
cd ~/ros2_ws/src
python3 cath_urdf_generator_with_controller.py \
    --N 50 --D 0.003 --L1 0.04 --L2 0.5 --L3 0.01 \
    --K 10.0 --Kd 0.1 --Kf 0.01 --M 0.5 \
    --output control_catheter_test
```

**Step 2: Build and launch**

```bash
cd ~/ros2_ws
colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch control_catheter_test control_catheter_test_launch.py
```

**Step 3: Launch keyboard teleoperation (new terminal)**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run control_catheter_test catheter_keyboard_teleop.py
```

**Keyboard controls:**

| Key | Action |
|-----|--------|
| `↑` / `↓` | Move base in Y axis |
| `←` / `→` | Move base in X axis |
| `w` / `s` | Insert / retract (Z axis) |
| `a` / `d` | Rotate base (Z-axis twist) |
| `q` | Quit |

The teleop node publishes to the following ROS2 topics:
- `/base_x_joint/cmd_pos`
- `/base_y_joint/cmd_pos`
- `/base_z_joint/cmd_pos`
- `/base_rotation_joint/cmd_pos`

> **Tip:** Before teleoperating, position the anatomy model to an appropriate pose in Gazebo first. For the included `test_anatomy`, a good starting pose is `x=0, y=-0.02, z=0.56` (adjust Z first to align the catheter with the vessel entry).

---

## Loading Anatomy in Gazebo

The `test_anatomy/` folder contains an example anatomical model (e.g., aortic arch). Follow these steps to load it into the simulation.

**Step 1: Set up the anatomy resource directory**

```bash
mkdir -p ~/anatomical_model
cp -r test_anatomy ~/anatomical_model/
```

> **Important:** Folder names under `~/anatomical_model/` must not contain spaces — Gazebo's `model://` URI does not support them.

**Step 2: Register the path in `.bashrc`**

```bash
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/anatomical_model' >> ~/.bashrc
source ~/.bashrc
```

> If you already have terminals running `ros2 launch`, re-source `.bashrc` in those terminals as well.

**Step 3: Launch the simulation and import the anatomy**

In Gazebo:
1. Open the plugin menu and search for **Resource Spawner** 
![Gazebo Resource Spawner](docs/images/ImportAnatomy_1.png)
![Gazebo Resource Spawner](docs/images/ImportAnatomy_2.png)
2. Find `test_anatomy` in the resource list
3. Drag it into the viewport
4. Open **Component Inspector** to set the pose numerically
![Gazebo Component Inspector](docs/images/AdjustPose_1.png)
![Gazebo Component Inspector](docs/images/AdjustPose_2.png)

---

## Catheter Model Parameters

Both generator scripts share the following parameters:

| Parameter | Description | Unit | Default |
|-----------|-------------|------|---------|
| `--N` | Total number of links (min: 3) | — | `5` |
| `--D` | Catheter outer diameter | m | `0.002` |
| `--L1` | Base link length | m | `0.05` |
| `--L2` | Bending section total length | m | `0.1` |
| `--L3` | Tip link length | m | `0.01` |
| `--K` | Joint spring stiffness | Nm/rad | `0.1` |
| `--M` | Total catheter mass | kg | `0.01` |
| `--output` | Output ROS2 package name | — | `catheter_package` |

The controller variant adds two additional parameters:

| Parameter | Description | Unit | Default |
|-----------|-------------|------|---------|
| `--Kd` | Joint damping coefficient | — | `0.1` |
| `--Kf` | Joint friction coefficient | — | `0.01` |

The catheter structure consists of a base link (length `L1`), a bending section of `N-2` uniformly distributed links (total length `L2`), and a tip link (length `L3`). Mass is distributed proportionally to each section's length. Spring stiffness decreases distally by a factor of 0.85 per joint to approximate the softer tip behavior of real catheters.

---

## References

- [SlicerROS2 ISMR2025 Prerequisites](https://rosmed.github.io/ismr2025/prerequisites)
- [Gazebo Harmonic Installation](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- [ros_gz bridge (Jazzy)](https://github.com/gazebosim/ros_gz/tree/jazzy)
- [NAMIC PW43: Robotic Catheter Placement for Cardiac Ablation](https://projectweek.na-mic.org/PW43_2025_Montreal/Projects/ApplicationOfSlicerros2InRoboticCatheterPlacementForCardiacAblation/)
- [NIH 3D Print Exchange](https://3dprint.nih.gov/)
