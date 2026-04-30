#!/usr/bin/env python3
"""Generate a simple concentric tube robot ROS 2 package.

The generated robot has 2 DOF:
  1. insertion (prismatic) of the inner tube inside the outer tube
  2. axial rotation (revolute) of the inner tube about the tube axis

Geometry
--------
- Outer tube: straight, 30 mm long, ID 4.0 mm, OD 5.0 mm
- Inner tube: curved, 30 mm arc length, radius of curvature 50 mm,
              ID 3.1 mm, OD 3.6 mm

The script generates a self-contained ROS 2 package with:
- STL meshes for the two tubes
- a xacro/URDF description
- a display launch file
- a minimal RViz config

This is intentionally lightweight and meant as a starting point for Slicer,
RViz, or Gazebo experiments.
"""

from __future__ import annotations

import argparse
import math
import os
import stat
import struct
import textwrap
from dataclasses import dataclass
from xml.dom import minidom
from typing import Iterable

import numpy as np


MM_TO_M = 0.001


@dataclass
class TubeSpec:
    name: str
    length_mm: float
    inner_diameter_mm: float
    outer_diameter_mm: float
    radius_of_curvature_mm: float | None = None
    color_rgba: tuple[float, float, float, float] = (0.8, 0.8, 0.8, 1.0)

    @property
    def inner_radius_m(self) -> float:
        return 0.5 * self.inner_diameter_mm * MM_TO_M

    @property
    def outer_radius_m(self) -> float:
        return 0.5 * self.outer_diameter_mm * MM_TO_M

    @property
    def length_m(self) -> float:
        return self.length_mm * MM_TO_M

    @property
    def radius_of_curvature_m(self) -> float | None:
        if self.radius_of_curvature_mm is None:
            return None
        return self.radius_of_curvature_mm * MM_TO_M


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def pretty_xml(xml_text: str) -> str:
    return "\n".join(
        line for line in minidom.parseString(xml_text).toprettyxml(indent="  ").splitlines()
        if line.strip()
    )


def normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return v
    return v / n


def straight_centerline(length_m: float, n_points: int = 40) -> np.ndarray:
    z = np.linspace(0.0, length_m, n_points)
    return np.column_stack([np.zeros_like(z), np.zeros_like(z), z])


def curved_centerline(length_m: float, radius_m: float, n_points: int = 120) -> np.ndarray:
    if radius_m <= 0:
        raise ValueError("radius_m must be positive")
    theta = length_m / radius_m
    phi = np.linspace(0.0, theta, n_points)
    x = radius_m * (1.0 - np.cos(phi))
    y = np.zeros_like(phi)
    z = radius_m * np.sin(phi)
    return np.column_stack([x, y, z])


def compute_frames(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    tangents = np.zeros_like(points)
    tangents[1:-1] = points[2:] - points[:-2]
    tangents[0] = points[1] - points[0]
    tangents[-1] = points[-1] - points[-2]
    tangents = np.array([normalize(t) for t in tangents])

    n1 = np.zeros_like(points)
    n2 = np.zeros_like(points)
    ref = np.array([0.0, 1.0, 0.0])

    for i, t in enumerate(tangents):
        local_ref = ref if abs(float(np.dot(t, ref))) <= 0.95 else np.array([1.0, 0.0, 0.0])
        n1[i] = normalize(np.cross(local_ref, t))
        n2[i] = normalize(np.cross(t, n1[i]))

    return tangents, n1, n2


def build_hollow_tube_mesh(
    points: np.ndarray,
    inner_radius_m: float,
    outer_radius_m: float,
    sections: int = 72,
    cap_start: bool = True,
    cap_end: bool = True,
) -> tuple[np.ndarray, np.ndarray]:
    """Create a watertight hollow tube around a centerline."""
    if inner_radius_m <= 0 or outer_radius_m <= inner_radius_m:
        raise ValueError("Invalid tube radii")

    _, n1, n2 = compute_frames(points)
    angles = np.linspace(0.0, 2.0 * np.pi, sections, endpoint=False)

    outer_vertices = []
    inner_vertices = []
    for p, e1, e2 in zip(points, n1, n2):
        ring_dirs = np.outer(np.cos(angles), e1) + np.outer(np.sin(angles), e2)
        outer_vertices.append(p + outer_radius_m * ring_dirs)
        inner_vertices.append(p + inner_radius_m * ring_dirs)

    outer_vertices = np.vstack(outer_vertices)
    inner_vertices = np.vstack(inner_vertices)
    vertices = np.vstack([outer_vertices, inner_vertices])

    n_rings = len(points)
    inner_offset = n_rings * sections
    faces: list[list[int]] = []

    for i in range(n_rings - 1):
        base0 = i * sections
        base1 = (i + 1) * sections
        for j in range(sections):
            jn = (j + 1) % sections

            a = base0 + j
            b = base0 + jn
            c = base1 + j
            d = base1 + jn

            # Outer wall
            faces.append([a, c, b])
            faces.append([b, c, d])

            # Inner wall (reverse orientation)
            ai = inner_offset + a
            bi = inner_offset + b
            ci = inner_offset + c
            di = inner_offset + d
            faces.append([ai, bi, ci])
            faces.append([bi, di, ci])

    if cap_start:
        start = 0
        for j in range(sections):
            jn = (j + 1) % sections
            a = start + j
            b = start + jn
            c = inner_offset + start + j
            d = inner_offset + start + jn
            faces.append([a, c, b])
            faces.append([b, c, d])

    if cap_end:
        end = (n_rings - 1) * sections
        for j in range(sections):
            jn = (j + 1) % sections
            a = end + j
            b = end + jn
            c = inner_offset + end + j
            d = inner_offset + end + jn
            faces.append([a, b, c])
            faces.append([b, d, c])

    return vertices.astype(np.float32), np.asarray(faces, dtype=np.int32)


def write_binary_stl(path: str, vertices: np.ndarray, faces: np.ndarray, solid_name: str) -> None:
    with open(path, "wb") as f:
        header = solid_name.encode("ascii", errors="ignore")[:80].ljust(80, b" ")
        f.write(header)
        f.write(struct.pack("<I", len(faces)))
        for tri in faces:
            v0, v1, v2 = vertices[tri]
            normal = np.cross(v1 - v0, v2 - v0)
            norm = float(np.linalg.norm(normal))
            if norm > 1e-12:
                normal = normal / norm
            else:
                normal = np.array([0.0, 0.0, 1.0], dtype=np.float32)
            f.write(struct.pack("<3f", *normal.astype(np.float32)))
            f.write(struct.pack("<3f", *v0.astype(np.float32)))
            f.write(struct.pack("<3f", *v1.astype(np.float32)))
            f.write(struct.pack("<3f", *v2.astype(np.float32)))
            f.write(struct.pack("<H", 0))


def build_tube_stl(spec: TubeSpec, out_path: str, sections: int = 72) -> str:
    if spec.radius_of_curvature_m is None:
        centerline = straight_centerline(spec.length_m)
    else:
        centerline = curved_centerline(spec.length_m, spec.radius_of_curvature_m)
    vertices, faces = build_hollow_tube_mesh(
        centerline,
        inner_radius_m=spec.inner_radius_m,
        outer_radius_m=spec.outer_radius_m,
        sections=sections,
    )
    write_binary_stl(out_path, vertices, faces, spec.name)
    return out_path


def write_text(path: str, content: str, executable: bool = False) -> str:
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    if executable:
        st = os.stat(path)
        os.chmod(path, st.st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)
    return path


def generate_package_xml(package_name: str) -> str:
    return pretty_xml(
        f"""<?xml version=\"1.0\"?>
<package format=\"3\">
  <name>{package_name}</name>
  <version>0.0.0</version>
  <description>Concentric tube robot generated by ctr_generator.py</description>
  <maintainer email=\"user@example.com\">User</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
    <exec_depend>sensor_msgs</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
    )


def generate_cmakelists_txt() -> str:
    return textwrap.dedent(
        """
        cmake_minimum_required(VERSION 3.8)
        project(ctr_package)

        find_package(ament_cmake REQUIRED)

        install(DIRECTORY urdf/
          DESTINATION share/${PROJECT_NAME}/urdf/
        )
        install(DIRECTORY meshes/
          DESTINATION share/${PROJECT_NAME}/meshes/
        )
        install(DIRECTORY launch/
          DESTINATION share/${PROJECT_NAME}/launch/
          FILES_MATCHING PATTERN "*.py"
        )
        install(DIRECTORY config/
          DESTINATION share/${PROJECT_NAME}/config/
        )
        install(PROGRAMS
          scripts/ctr_key_listener.py
          scripts/ctr_keyboard_teleop.py
          DESTINATION lib/${PROJECT_NAME}
        )

        if(BUILD_TESTING)
          find_package(ament_lint_auto REQUIRED)
          set(ament_cmake_copyright_FOUND TRUE)
          set(ament_cmake_cpplint_FOUND TRUE)
          ament_lint_auto_find_test_dependencies()
        endif()

        ament_package()
        """
    ).lstrip()


def generate_xacro(
    package_name: str,
    outer_mesh: str,
    inner_segment_mesh: str,
    outer_length_m: float,
    inner_length_m: float,
    outer_inner_diam_m: float,
    outer_outer_diam_m: float,
    inner_inner_diam_m: float,
    inner_outer_diam_m: float,
    curvature_radius_m: float,
    inner_segments: int,
    bend_angle_per_joint_rad: float,
    segment_length_m: float,
) -> str:
    segment_links = []
    segment_joints = []

    for i in range(1, inner_segments + 1):
        parent_link = "inner_base_link" if i == 1 else f"inner_segment_{i - 1}"
        segment_links.append(
            f"""
          <link name=\"inner_segment_{i}\">
            <visual>
              <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
              <geometry>
                <mesh filename=\"package://{package_name}/meshes/{inner_segment_mesh}\" scale=\"1 1 1\"/>
              </geometry>
              <material name=\"inner_mat\">
                <color rgba=\"0.95 0.45 0.25 1.0\"/>
              </material>
            </visual>
            <collision>
              <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
              <geometry>
                <mesh filename=\"package://{package_name}/meshes/{inner_segment_mesh}\" scale=\"1 1 1\"/>
              </geometry>
            </collision>
            <inertial>
              <mass value=\"0.0007\"/>
              <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
              <inertia ixx=\"1e-6\" iyy=\"1e-6\" izz=\"1e-6\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/>
            </inertial>
          </link>
          """
        )
        joint_origin_z = 0.0 if i == 1 else segment_length_m
        segment_joints.append(
            f"""
          <joint name=\"inner_bend_joint_{i}\" type=\"revolute\">
            <parent link=\"{parent_link}\"/>
            <child link=\"inner_segment_{i}\"/>
            <origin xyz=\"0 0 {joint_origin_z:.6f}\" rpy=\"0 0 0\"/>
            <axis xyz=\"0 1 0\"/>
            <limit lower=\"-{bend_angle_per_joint_rad:.8f}\" upper=\"{bend_angle_per_joint_rad:.8f}\" effort=\"2.0\" velocity=\"1.0\"/>
          </joint>
          """
        )

    return textwrap.dedent(
        f"""<?xml version=\"1.0\"?>
        <robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"ctr\">
          <xacro:property name=\"outer_length\" value=\"{outer_length_m:.6f}\"/>
          <xacro:property name=\"inner_length\" value=\"{inner_length_m:.6f}\"/>
          <xacro:property name=\"outer_inner_diameter\" value=\"{outer_inner_diam_m:.6f}\"/>
          <xacro:property name=\"outer_outer_diameter\" value=\"{outer_outer_diam_m:.6f}\"/>
          <xacro:property name=\"inner_inner_diameter\" value=\"{inner_inner_diam_m:.6f}\"/>
          <xacro:property name=\"inner_outer_diameter\" value=\"{inner_outer_diam_m:.6f}\"/>
          <xacro:property name=\"curvature_radius\" value=\"{curvature_radius_m:.6f}\"/>
          <xacro:property name=\"inner_segments\" value=\"{inner_segments}\"/>
          <xacro:property name=\"bend_angle_per_joint\" value=\"{bend_angle_per_joint_rad:.8f}\"/>

          <link name=\"world\"/>

          <link name=\"outer_tube_link\">
            <visual>
              <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
              <geometry>
                <mesh filename=\"package://{package_name}/meshes/{outer_mesh}\" scale=\"1 1 1\"/>
              </geometry>
              <material name=\"outer_mat\">
                <color rgba=\"0.35 0.55 0.95 0.35\"/>
              </material>
            </visual>
            <collision>
              <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
              <geometry>
                <mesh filename=\"package://{package_name}/meshes/{outer_mesh}\" scale=\"1 1 1\"/>
              </geometry>
            </collision>
            <inertial>
              <mass value=\"0.01\"/>
              <origin xyz=\"0 0 {outer_length_m / 2.0:.6f}\" rpy=\"0 0 0\"/>
              <inertia ixx=\"1e-4\" iyy=\"1e-4\" izz=\"1e-4\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/>
            </inertial>
          </link>

          <link name="base_x_link"/>
          <link name="base_y_link"/>
          <link name="base_z_link"/>
          <link name="inner_carriage_link"/>
          <link name="inner_base_link"/>

          <joint name="base_x_joint" type="prismatic">
            <parent link="world"/>
            <child link="base_x_link"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <axis xyz="1 0 0"/>
            <limit lower="-0.05" upper="0.05" effort="10.0" velocity="0.05"/>
          </joint>

          <joint name="base_y_joint" type="prismatic">
            <parent link="base_x_link"/>
            <child link="base_y_link"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
            <limit lower="-0.05" upper="0.05" effort="10.0" velocity="0.05"/>
          </joint>

          <joint name="base_z_joint" type="prismatic">
            <parent link="base_y_link"/>
            <child link="base_z_link"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
            <limit lower="-0.05" upper="0.05" effort="10.0" velocity="0.05"/>
          </joint>

          <joint name="base_to_outer" type="fixed">
            <parent link="base_z_link"/>
            <child link="outer_tube_link"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
          </joint>

          <joint name=\"inner_insertion_joint\" type=\"prismatic\">
            <parent link=\"outer_tube_link\"/>
            <child link=\"inner_carriage_link\"/>
            <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
            <axis xyz=\"0 0 1\"/>
            <limit lower=\"0.0\" upper=\"0.03\" effort=\"10.0\" velocity=\"0.05\"/>
          </joint>

          <joint name=\"inner_rotation_joint\" type=\"revolute\">
            <parent link=\"inner_carriage_link\"/>
            <child link=\"inner_base_link\"/>
            <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
            <axis xyz=\"0 0 1\"/>
            <limit lower=\"-3.14159\" upper=\"3.14159\" effort=\"1.0\" velocity=\"2.0\"/>
          </joint>

{''.join(segment_links)}
{''.join(segment_joints)}
        </robot>
        """
    ).lstrip()


def generate_launch(package_name: str) -> str:
  return textwrap.dedent(
    f"""\
    import os
    import shutil

    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import Command, FindExecutable
    from launch_ros.actions import Node


    def generate_launch_description():
      pkg_share = get_package_share_directory('{package_name}')
      xacro_file = os.path.join(pkg_share, 'urdf', 'ctr.urdf.xacro')
      rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

      robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_file])

      teleop_prefix = None
      if shutil.which('xterm'):
        teleop_prefix = 'xterm -e'
      elif shutil.which('gnome-terminal'):
        teleop_prefix = 'gnome-terminal --'
      elif shutil.which('x-terminal-emulator'):
        teleop_prefix = 'x-terminal-emulator -e'

      teleop_node_kwargs = dict(
        package='{package_name}',
        executable='ctr_keyboard_teleop.py',
        output='screen',
      )
      if teleop_prefix:
        teleop_node_kwargs['prefix'] = teleop_prefix

      return LaunchDescription([
        DeclareLaunchArgument(
          'use_gui', default_value='true',
          description='Show joint_state_publisher_gui'
        ),
        Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          output='screen',
          parameters=[{{'robot_description': robot_description}}],
        ),
        Node(
          package='joint_state_publisher_gui',
          executable='joint_state_publisher_gui',
          output='screen',
        ),
        Node(
          package='rviz2',
          executable='rviz2',
          output='screen',
          arguments=['-d', rviz_config],
        ),
      ])
    """
  ).lstrip()


def generate_teleop_launch(package_name: str) -> str:
  return textwrap.dedent(
    f"""\
    import os
    import shutil

    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.substitutions import Command, FindExecutable
    from launch_ros.actions import Node


    def generate_launch_description():
      pkg_share = get_package_share_directory('{package_name}')
      xacro_file = os.path.join(pkg_share, 'urdf', 'ctr.urdf.xacro')
      rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

      robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_file])

      key_listener_kwargs = {{
        'package': '{package_name}',
        'executable': 'ctr_key_listener.py',
        'output': 'screen',
      }}
      if shutil.which('xterm'):
        key_listener_kwargs['prefix'] = 'xterm -e'

      return LaunchDescription([
        Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          output='screen',
          parameters=[{{'robot_description': robot_description}}],
        ),
        Node(**key_listener_kwargs),
        Node(
          package='{package_name}',
          executable='ctr_keyboard_teleop.py',
          output='screen',
        ),
        Node(
          package='rviz2',
          executable='rviz2',
          output='screen',
          arguments=['-d', rviz_config],
        ),
      ])
    """
  ).lstrip()


def generate_key_listener_script() -> str:
  return '''#!/usr/bin/env python3
"""Simple keyboard listener that publishes to /ctr_command topic."""

import os
import select
import sys
import termios
import tty

import rclpy
from std_msgs.msg import Char


def get_key(fd: int, timeout=0.05):
  """Read a single key from the provided tty fd with timeout."""
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
    ready, _, _ = select.select([fd], [], [], timeout)
        if not ready:
            return ''
        key = os.read(fd, 1).decode('utf-8', errors='ignore')
        if key == '\\x1b':
            for _ in range(2):
        r, _, _ = select.select([fd], [], [], 0.02)
                if r:
                    key += os.read(fd, 1).decode('utf-8', errors='ignore')
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('ctr_key_listener')
    pub = node.create_publisher(Char, '/ctr_command', 10)

  tty_file = None
  try:
    if sys.stdin.isatty():
      input_fd = sys.stdin.fileno()
    else:
      tty_file = open('/dev/tty', 'rb', buffering=0)
      input_fd = tty_file.fileno()
  except OSError:
    print('No TTY available for keyboard input. Run in a terminal.')
    node.destroy_node()
    rclpy.shutdown()
    return

    print('\\nCTR Keyboard Listener')
  print('I/K base +Y/-Y | J/L base -X/+X | U/O base -Z/+Z')
  print('W/S insert/retract | A/D rotate | R reset | Q quit\\n')

    try:
        while rclpy.ok():
      key = get_key(input_fd, timeout=0.05)
            if key:
                msg = Char()
                msg.data = ord(key[0])
                pub.publish(msg)
                print(f'Published: {key[0]} ({msg.data})')
                
                if key.lower() == 'q':
                    break
    except KeyboardInterrupt:
        pass
    finally:
        print('\\nShutting down keyboard listener.')
      if tty_file is not None:
        tty_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''


def generate_teleop_script(inner_segments: int, inner_length_m: float, curvature_radius_m: float) -> str:
  segment_length = inner_length_m / inner_segments
  full_theta = inner_length_m / curvature_radius_m
  bend_per_joint = full_theta / inner_segments
  bend_joint_names = [f"inner_bend_joint_{i}" for i in range(1, inner_segments + 1)]
  return f'''#!/usr/bin/env python3
"""CTR command subscriber that publishes JointState."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Char


INSERT_STEP = 0.001
BASE_STEP = 0.001
ROT_STEP = math.radians(2)

INSERT_MIN = 0.0
INSERT_MAX = {inner_length_m:.6f}
BASE_X_MIN = -0.050
BASE_X_MAX = 0.050
BASE_Y_MIN = -0.050
BASE_Y_MAX = 0.050
BASE_Z_MIN = -0.050
BASE_Z_MAX = 0.050
ROT_MIN = -math.pi
ROT_MAX = math.pi
SEGMENT_LENGTH = {segment_length:.8f}
BEND_PER_JOINT = {bend_per_joint:.8f}
NUM_BEND_JOINTS = {inner_segments}
BEND_JOINT_NAMES = {bend_joint_names}

KEY_INSERT = 'w'
KEY_RETRACT = 's'
KEY_CCW = 'a'
KEY_CW = 'd'
KEY_RESET = 'r'
KEY_QUIT = 'q'
KEY_BASE_X_NEG = 'j'
KEY_BASE_X_POS = 'l'
KEY_BASE_Y_NEG = 'k'
KEY_BASE_Y_POS = 'i'
KEY_BASE_Z_NEG = 'u'
KEY_BASE_Z_POS = 'o'


class CtrTeleop(Node):
    def __init__(self):
        super().__init__('ctr_teleop')
    self.base_x = 0.0
    self.base_y = 0.0
    self.base_z = 0.0
        self.insertion = 0.0
        self.rotation = 0.0
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(Char, '/ctr_command', self.on_command, 10)
        self.create_timer(0.05, self.publish)
        self.get_logger().info('CTR teleop ready, listening on /ctr_command')

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        bend_positions = self.compute_bend_positions(self.insertion)
        msg.name = [
            'base_x_joint',
            'base_y_joint',
            'base_z_joint',
            'inner_insertion_joint',
            'inner_rotation_joint',
        ] + BEND_JOINT_NAMES
        msg.position = [
            self.base_x,
            self.base_y,
            self.base_z,
            self.insertion,
            self.rotation,
        ] + bend_positions
        self.pub.publish(msg)

    def compute_bend_positions(self, insertion: float):
        emerged = max(0.0, min(insertion, INSERT_MAX))
        positions = []
        for idx in range(1, NUM_BEND_JOINTS + 1):
            dist_from_tip = (NUM_BEND_JOINTS - idx) * SEGMENT_LENGTH
            activation = (emerged - dist_from_tip) / SEGMENT_LENGTH
            activation = max(0.0, min(1.0, activation))
            positions.append(BEND_PER_JOINT * activation)
        return positions

    def on_command(self, msg: Char):
        """Handle command from /ctr_command topic."""
        key = chr(msg.data).lower()
        
        if key == KEY_QUIT:
            self.get_logger().info('Quit command received')
            return
        
        if key == KEY_RESET:
            self.base_x = 0.0
            self.base_y = 0.0
            self.base_z = 0.0
            self.insertion = 0.0
            self.rotation = 0.0
            self.get_logger().info('Reset')
            return

        if key == KEY_BASE_X_NEG:
            self.base_x = max(self.base_x - BASE_STEP, BASE_X_MIN)
        elif key == KEY_BASE_X_POS:
            self.base_x = min(self.base_x + BASE_STEP, BASE_X_MAX)
        elif key == KEY_BASE_Y_NEG:
            self.base_y = max(self.base_y - BASE_STEP, BASE_Y_MIN)
        elif key == KEY_BASE_Y_POS:
            self.base_y = min(self.base_y + BASE_STEP, BASE_Y_MAX)
        elif key == KEY_BASE_Z_NEG:
            self.base_z = max(self.base_z - BASE_STEP, BASE_Z_MIN)
        elif key == KEY_BASE_Z_POS:
            self.base_z = min(self.base_z + BASE_STEP, BASE_Z_MAX)
        elif key == KEY_INSERT:
            self.insertion = min(self.insertion + INSERT_STEP, INSERT_MAX)
        elif key == KEY_RETRACT:
            self.insertion = max(self.insertion - INSERT_STEP, INSERT_MIN)
        elif key == KEY_CCW:
            self.rotation = min(self.rotation + ROT_STEP, ROT_MAX)
        elif key == KEY_CW:
            self.rotation = max(self.rotation - ROT_STEP, ROT_MIN)

        self.publish()
        self.get_logger().info(
            f'Base xyz [mm]: ({{self.base_x * 1000:+6.1f}}, {{self.base_y * 1000:+6.1f}}, {{self.base_z * 1000:+6.1f}}) | '
            f'Insertion: {{self.insertion * 1000:+6.1f}} mm | '
            f'Rotation: {{math.degrees(self.rotation):+6.1f}} deg'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CtrTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''

def generate_rviz_config() -> str:
    return textwrap.dedent(
        """
        Panels:
          - Class: rviz_common/Displays
            Name: Displays
        Visualization Manager:
          Class: ""
          Displays:
            - Class: rviz_default_plugins/Grid
              Enabled: true
              Name: Grid
            - Class: rviz_default_plugins/RobotModel
              Description Topic:
                Depth: 5
                Durability Policy: Transient Local
                History Policy: Keep Last
                Reliability Policy: Reliable
                Value: /robot_description
              Enabled: true
              Name: RobotModel
              Value: true
          Global Options:
            Background Color: 48; 48; 48
            Fixed Frame: world
            Frame Rate: 30
          Name: root
          Tools:
            - Class: rviz_default_plugins/Interact
            - Class: rviz_default_plugins/MoveCamera
          Views:
            Current:
              Class: rviz_default_plugins/Orbit
              Name: Current View
              Distance: 0.2
              Pitch: 0.4
              Yaw: 0.8
          Value: true
        Window Geometry:
          Hide Left Dock: false
          Hide Right Dock: true
        """
    ).lstrip()


class CtrGenerator:
  def __init__(self, output: str):
    self.output = os.path.abspath(output)
    self.meshes_dir = os.path.join(self.output, 'meshes')
    self.urdf_dir = os.path.join(self.output, 'urdf')
    self.launch_dir = os.path.join(self.output, 'launch')
    self.config_dir = os.path.join(self.output, 'config')
    self.scripts_dir = os.path.join(self.output, 'scripts')
    for d in (self.output, self.meshes_dir, self.urdf_dir, self.launch_dir, self.config_dir, self.scripts_dir):
      ensure_dir(d)

    self.outer = TubeSpec(
      name='outer_tube',
      length_mm=30.0,
      inner_diameter_mm=4.0,
      outer_diameter_mm=5.0,
      color_rgba=(0.35, 0.55, 0.95, 1.0),
    )
    self.inner = TubeSpec(
      name='inner_tube',
      length_mm=30.0,
      inner_diameter_mm=3.1,
      outer_diameter_mm=3.6,
      radius_of_curvature_mm=50.0,
      color_rgba=(0.95, 0.45, 0.25, 1.0),
    )
    self.inner_segments = 50
    self.segment_length_m = self.inner.length_m / self.inner_segments
    self.total_curve_angle_rad = self.inner.length_m / (self.inner.radius_of_curvature_m or 0.05)
    self.bend_angle_per_joint_rad = self.total_curve_angle_rad / self.inner_segments

  def generate(self) -> None:
    outer_mesh = build_tube_stl(self.outer, os.path.join(self.meshes_dir, 'outer_tube.stl'))
    inner_segment = TubeSpec(
      name='inner_segment',
      length_mm=self.segment_length_m / MM_TO_M,
      inner_diameter_mm=self.inner.inner_diameter_mm,
      outer_diameter_mm=self.inner.outer_diameter_mm,
      radius_of_curvature_mm=None,
      color_rgba=self.inner.color_rgba,
    )
    inner_segment_mesh = build_tube_stl(inner_segment, os.path.join(self.meshes_dir, 'inner_segment.stl'))

    package_name = os.path.basename(self.output)
    write_text(os.path.join(self.output, 'package.xml'), generate_package_xml(package_name))
    write_text(os.path.join(self.output, 'CMakeLists.txt'), generate_cmakelists_txt())
    write_text(
      os.path.join(self.urdf_dir, 'ctr.urdf.xacro'),
      generate_xacro(
        package_name=package_name,
        outer_mesh=os.path.basename(outer_mesh),
        inner_segment_mesh=os.path.basename(inner_segment_mesh),
        outer_length_m=self.outer.length_m,
        inner_length_m=self.inner.length_m,
        outer_inner_diam_m=self.outer.inner_radius_m * 2.0,
        outer_outer_diam_m=self.outer.outer_radius_m * 2.0,
        inner_inner_diam_m=self.inner.inner_radius_m * 2.0,
        inner_outer_diam_m=self.inner.outer_radius_m * 2.0,
        curvature_radius_m=self.inner.radius_of_curvature_m or 0.05,
        inner_segments=self.inner_segments,
        bend_angle_per_joint_rad=self.bend_angle_per_joint_rad,
        segment_length_m=self.segment_length_m,
      ),
    )
    write_text(os.path.join(self.launch_dir, 'ctr_display.launch.py'), generate_launch(package_name))
    write_text(os.path.join(self.launch_dir, 'ctr_teleop.launch.py'), generate_teleop_launch(package_name))
    write_text(os.path.join(self.config_dir, 'rviz_config.rviz'), generate_rviz_config())
    write_text(
      os.path.join(self.scripts_dir, 'ctr_key_listener.py'),
      generate_key_listener_script(),
      executable=True,
    )
    write_text(
      os.path.join(self.scripts_dir, 'ctr_keyboard_teleop.py'),
      generate_teleop_script(
        inner_segments=self.inner_segments,
        inner_length_m=self.inner.length_m,
        curvature_radius_m=self.inner.radius_of_curvature_m or 0.05,
      ),
      executable=True,
    )

    print(f'Generated CTR package at: {self.output}')
    print('Meshes:')
    print(f'  - {os.path.join(self.meshes_dir, "outer_tube.stl")}')
    print(f'  - {os.path.join(self.meshes_dir, "inner_segment.stl")}')
    print('URDF:')
    print(f'  - {os.path.join(self.urdf_dir, "ctr.urdf.xacro")}')
    print('Launch:')
    print(f'  - {os.path.join(self.launch_dir, "ctr_display.launch.py")}')
    print(f'  - {os.path.join(self.launch_dir, "ctr_teleop.launch.py")}')
    print('Teleop script:')
    print(f'  - {os.path.join(self.scripts_dir, "ctr_keyboard_teleop.py")}')


def main() -> int:
    parser = argparse.ArgumentParser(description='Generate a concentric tube robot ROS 2 package')
    parser.add_argument('--output', default='ctr_package', help='Output ROS 2 package directory')
    args = parser.parse_args()

    gen = CtrGenerator(args.output)
    gen.generate()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
