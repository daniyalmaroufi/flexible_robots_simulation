#!/usr/bin/env python3
"""
Xacro Generator for Flexible Catheter

This script generates Xacro data for a flexible catheter modeled as a serial link
with N links and N-1 universal joints. The catheter consists of:
- Tip link (length L1)
- Bending section (N-2 links, total length L2)
- Base link (length L3)

Parameters:
- N: Number of links
- D: Outer diameter of catheter
- L1: Length of tip link
- L2: Total length of bending section
- L3: Length of base link
- K: Spring constant for rotary springs
- M: Total mass of catheter
"""

import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
import math
import argparse
import numpy as np
import os


class CatheterXacroGenerator:
    def __init__(self, N, D, L1, L2, L3, K, Kd, Kf, M, package_dir="catheter_package"):
        self.N = N
        self.D = D
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.K = K
        self.damping = Kd
        self.friction = Kf
        self.M = M
        self.package_dir = package_dir
        self.meshes_dir = os.path.join(package_dir, "meshes")
        self.urdf_dir = os.path.join(package_dir, "urdf")
        self.sdf_dir = os.path.join(package_dir, "sdf")
        self.launch_dir = os.path.join(package_dir, "launch")
        self.worlds_dir = os.path.join(package_dir, "worlds")
        
        self.validate_parameters()
        self.calculate_derived_values()
        self.create_package_structure()
    
    def validate_parameters(self):
        """Validate input parameters"""
        if self.N < 3:
            raise ValueError("N must be at least 3 (tip, at least one bending link, base)")
        if any(val <= 0 for val in [self.D, self.L1, self.L2, self.L3, self.K, self.M]):
            raise ValueError("All parameters must be positive")
    
    def calculate_derived_values(self):
        """Calculate derived values from input parameters"""
        self.bending_links = self.N - 2
        self.bending_link_length = self.L2 / self.bending_links if self.bending_links > 0 else 0
        self.radius = self.D / 2
        
        # Mass distribution
        self.tip_mass = self.M * (self.L3 / (self.L1 + self.L2 + self.L3))
        self.base_mass = self.M * (self.L1 / (self.L1 + self.L2 + self.L3))
        self.bending_mass_per_link = self.M * (self.L2 / (self.L1 + self.L2 + self.L3)) / self.bending_links if self.bending_links > 0 else 0
    
    def create_package_structure(self):
        """Create package directory structure"""
        if not os.path.exists(self.package_dir):
            os.makedirs(self.package_dir)
        if not os.path.exists(self.meshes_dir):
            os.makedirs(self.meshes_dir)
        if not os.path.exists(self.urdf_dir):
            os.makedirs(self.urdf_dir)
        if not os.path.exists(self.sdf_dir):
            os.makedirs(self.sdf_dir)
        if not os.path.exists(self.launch_dir):
            os.makedirs(self.launch_dir)
        if not os.path.exists(self.worlds_dir):
            os.makedirs(self.worlds_dir)
    
    def generate_cylinder_stl(self, radius, length, filename, resolution=20):
        """Generate STL file for a cylinder"""
        theta = np.linspace(0, 2*np.pi, resolution)
        
        top_center = [0, 0, length/2]
        bottom_center = [0, 0, -length/2]
        
        top_circle = [[radius * np.cos(t), radius * np.sin(t), length/2] for t in theta]
        bottom_circle = [[radius * np.cos(t), radius * np.sin(t), -length/2] for t in theta]
        
        vertices = [top_center, bottom_center] + top_circle + bottom_circle
        
        faces = []
        n = len(theta)
        
        for i in range(n):
            faces.append([0, 2 + i, 2 + (i + 1) % n])
        
        for i in range(n):
            faces.append([1, 2 + n + (i + 1) % n, 2 + n + i])
        
        for i in range(n):
            next_i = (i + 1) % n
            faces.append([2 + i, 2 + n + i, 2 + next_i])
            faces.append([2 + next_i, 2 + n + i, 2 + n + next_i])
        
        stl_path = os.path.join(self.meshes_dir, filename)
        with open(stl_path, 'w') as f:
            f.write("solid cylinder\n")
            
            for face in faces:
                v1 = np.array(vertices[face[1]]) - np.array(vertices[face[0]])
                v2 = np.array(vertices[face[2]]) - np.array(vertices[face[0]])
                normal = np.cross(v1, v2)
                normal = normal / np.linalg.norm(normal) if np.linalg.norm(normal) > 0 else [0, 0, 1]
                
                f.write(f"  facet normal {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}\n")
                f.write("    outer loop\n")
                for vertex_idx in face:
                    vertex = vertices[vertex_idx]
                    f.write(f"      vertex {vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n")
                f.write("    endloop\n")
                f.write("  endfacet\n")
            
            f.write("endsolid cylinder\n")
        
        return stl_path
    
    def generate_package_xml(self):
        """Generate package.xml for ROS2 package"""
        package_name = os.path.basename(self.package_dir)
        
        package_xml_content = f'''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypefile="package_format3.xsd"?>
<package format="3">
  <name>{package_name}</name>
  <version>0.0.0</version>
  <description>Flexible catheter URDF package generated by catheter_urdf_generator</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
    <gazebo_ros gazebo_model_path="${{prefix}}/../"/>
    <gazebo_ros gazebo_media_path="${{prefix}}/media"/>
    <gazebo_ros plugin_path="${{prefix}}/plugins"/>
  </export>
</package>'''
        
        package_xml_path = os.path.join(self.package_dir, "package.xml")
        with open(package_xml_path, 'w') as f:
            f.write(package_xml_content)
        
        return package_xml_path
    
    def generate_cmakelists_txt(self):
        """Generate CMakeLists.txt for ROS2 package"""
        package_name = os.path.basename(self.package_dir)
        
        cmake_content = f'''cmake_minimum_required(VERSION 3.8)
project({package_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/
  DESTINATION share/${{PROJECT_NAME}}/urdf/
)

install(DIRECTORY meshes/
  DESTINATION share/${{PROJECT_NAME}}/meshes/
)

install(DIRECTORY sdf/
  DESTINATION share/${{PROJECT_NAME}}/sdf/
)

install(DIRECTORY worlds/
  DESTINATION share/${{PROJECT_NAME}}/worlds/
)

install(DIRECTORY launch/
  DESTINATION share/${{PROJECT_NAME}}/launch/
  FILES_MATCHING PATTERN "*.py"
  PATTERN "*.launch.py"
)

install(PROGRAMS scripts/catheter_keyboard_teleop.py
  DESTINATION lib/${{PROJECT_NAME}}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()'''
        
        cmake_path = os.path.join(self.package_dir, "CMakeLists.txt")
        with open(cmake_path, 'w') as f:
            f.write(cmake_content)
        
        return cmake_path

    # ─────────────────────────────────────────────────────────────────────────
    # SDF helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _make_virtual_link(self, parent_el, name):
        """Create a massless (virtual) intermediate link for the base chain."""
        link = ET.SubElement(parent_el, 'link', name=name)
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = '0.01'
        inertia = ET.SubElement(inertial, 'inertia')
        for tag in ['ixx', 'iyy', 'izz']:
            ET.SubElement(inertia, tag).text = '1e-3'
        for tag in ['ixy', 'ixz', 'iyz']:
            ET.SubElement(inertia, tag).text = '0'
        return link

    def _make_prismatic_joint(self, parent_el, name, parent_link, child_link,
                               axis_xyz, lower, upper,
                               effort='100', velocity='1.0'):
        """Create a prismatic joint with damping/friction dynamics."""
        joint = ET.SubElement(parent_el, 'joint', name=name, type='prismatic')
        ET.SubElement(joint, 'parent').text = parent_link
        ET.SubElement(joint, 'child').text = child_link
        ET.SubElement(joint, 'pose').text = '0 0 0 0 0 0'
        axis = ET.SubElement(joint, 'axis')
        ET.SubElement(axis, 'xyz').text = axis_xyz
        limit = ET.SubElement(axis, 'limit')
        ET.SubElement(limit, 'lower').text = str(lower)
        ET.SubElement(limit, 'upper').text = str(upper)
        ET.SubElement(limit, 'effort').text = effort
        ET.SubElement(limit, 'velocity').text = velocity
        dyn = ET.SubElement(axis, 'dynamics')
        ET.SubElement(dyn, 'damping').text = f'{self.damping}'
        ET.SubElement(dyn, 'friction').text = f'{self.friction}'
        return joint

    def _make_position_controller_plugin(self, parent_el, joint_name, topic,
                                          p='5000.0', i='0.0', d='100.0',
                                          cmd_max='1000.0', cmd_min='-1000.0'):
        """Attach a JointPositionController plugin to an SDF model element."""
        plg = ET.SubElement(parent_el, 'plugin',
                            filename='gz-sim-joint-position-controller-system',
                            name='gz::sim::systems::JointPositionController')
        ET.SubElement(plg, 'joint_name').text = joint_name
        ET.SubElement(plg, 'topic').text = topic
        ET.SubElement(plg, 'use_velocity_commands').text = 'false'
        ET.SubElement(plg, 'p_gain').text = p
        ET.SubElement(plg, 'i_gain').text = i
        ET.SubElement(plg, 'd_gain').text = d
        ET.SubElement(plg, 'i_max').text = '1.0'
        ET.SubElement(plg, 'i_min').text = '-1.0'
        ET.SubElement(plg, 'cmd_max').text = cmd_max
        ET.SubElement(plg, 'cmd_min').text = cmd_min
        return plg

    # ─────────────────────────────────────────────────────────────────────────
    # SDF generation
    # ─────────────────────────────────────────────────────────────────────────

    def generate_sdf(self, xacro_filename):
        """Generate SDF file using ElementTree."""
        package_name = os.path.basename(self.package_dir)
        sdf_filename = xacro_filename.replace('.xacro', '.sdf')
        
        sdf = ET.Element('sdf', version='1.7')
        model = ET.SubElement(sdf, 'model', name=package_name)
        
        # ── Catheter links ──────────────────────────────────────────────────
        self._add_sdf_link(model, 'base_link',
                           parent_name=None,
                           pose=f'0 0 {self.L1/2} 0 0 0',
                           mass=self.base_mass,
                           length=self.L1,
                           mesh_file=f'file://{os.path.abspath(os.path.join(self.meshes_dir, "base_link.stl"))}')
        
        parent_name = 'base_to_bending_1_y'
        for i in range(self.bending_links):
            link_name = f'bending_link_{i+1}'
            self._add_sdf_link(model, link_name,
                               parent_name=parent_name,
                               pose=f'0 0 {self.bending_link_length/2} 0 0 0',
                               mass=self.bending_mass_per_link,
                               length=self.bending_link_length,
                               mesh_file=f'model://{package_name}/meshes/bending_link.stl')
            parent_name = f'bending_{i+1}_to_{i+2}_y'
        
        parent_name = f'bending_{self.bending_links}_to_tip_x'
        self._add_sdf_link(model, 'tip_link',
                           parent_name=parent_name,
                           pose=f'0 0 {self.L3/2} 0 0 0',
                           mass=self.tip_mass,
                           length=self.L3,
                           mesh_file=f'model://{package_name}/meshes/tip_link.stl')

        # ── Base mobility chain ─────────────────────────────────────────────
        # world → base_x_link (prismatic X)
        #       → base_y_link (prismatic Y)
        #       → base_z_link (prismatic Z)
        #       → base_link   (revolute  Z)

        total_length = self.L1 + self.L2 + self.L3 + 1.0  ## Adding 1.0 to extend the motion range

        # X translation
        self._make_virtual_link(model, 'base_x_link')
        self._make_prismatic_joint(model, 'base_x_joint',
                                   parent_link='world', child_link='base_x_link',
                                   axis_xyz='1 0 0',
                                   lower=-total_length, upper=total_length)

        # Y translation
        self._make_virtual_link(model, 'base_y_link')
        self._make_prismatic_joint(model, 'base_y_joint',
                                   parent_link='base_x_link', child_link='base_y_link',
                                   axis_xyz='0 1 0',
                                   lower=-total_length, upper=total_length)

        # Z translation (insertion)
        self._make_virtual_link(model, 'base_z_link')
        self._make_prismatic_joint(model, 'base_z_joint',
                                   parent_link='base_y_link', child_link='base_z_link',
                                   axis_xyz='0 0 1',
                                   lower=-total_length, upper=total_length)

        # Z rotation (twist): base_z_link → base_link
        j_rot = ET.SubElement(model, 'joint', name='base_rotation_joint', type='revolute')
        ET.SubElement(j_rot, 'parent').text = 'base_z_link'
        ET.SubElement(j_rot, 'child').text = 'base_link'
        ET.SubElement(j_rot, 'pose').text = '0 0 0 0 0 0'
        axis_r = ET.SubElement(j_rot, 'axis')
        ET.SubElement(axis_r, 'xyz').text = '0 0 1'
        limit_r = ET.SubElement(axis_r, 'limit')
        ET.SubElement(limit_r, 'lower').text = str(-math.pi)
        ET.SubElement(limit_r, 'upper').text = str(math.pi)
        ET.SubElement(limit_r, 'effort').text = '100'
        ET.SubElement(limit_r, 'velocity').text = '10'
        dyn_r = ET.SubElement(axis_r, 'dynamics')
        ET.SubElement(dyn_r, 'damping').text = f'{self.damping}'
        ET.SubElement(dyn_r, 'friction').text = f'{self.friction}'

        # ── Flexible joints ─────────────────────────────────────────────────
        joint_names = self._add_universal_joints(model)
        
        # ── Plugins ─────────────────────────────────────────────────────────
        self._add_joint_state_publisher(model, joint_names)

        # X / Y translation controllers
        self._make_position_controller_plugin(model, 'base_x_joint', '/base_x_joint/cmd_pos')
        self._make_position_controller_plugin(model, 'base_y_joint', '/base_y_joint/cmd_pos')

        # Z translation controller
        self._make_position_controller_plugin(model, 'base_z_joint', '/base_z_joint/cmd_pos')

        # Z rotation controller (lower gains for revolute)
        self._make_position_controller_plugin(model, 'base_rotation_joint',
                                              '/base_rotation_joint/cmd_pos',
                                              p='100.0', d='10.0',
                                              cmd_max='100.0', cmd_min='-100.0')

        self._save_sdf_file(sdf, sdf_filename)
        return os.path.join(self.sdf_dir, sdf_filename)
    
    def _add_sdf_link(self, parent, name, parent_name, pose, mass, length, mesh_file):
        """Add a catheter link to SDF."""
        link = ET.SubElement(parent, 'link', name=name)
        
        if parent_name is None:
            ET.SubElement(link, 'pose').text = pose
        else:
            ET.SubElement(link, 'pose', relative_to=parent_name).text = pose
        
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = str(mass)
        inertia = ET.SubElement(inertial, 'inertia')
        
        ixx_iyy = (1/12) * mass * (3 * self.radius**2 + length**2)
        izz = 0.5 * mass * self.radius**2
        
        ET.SubElement(inertia, 'ixx').text = str(ixx_iyy)
        ET.SubElement(inertia, 'iyy').text = str(ixx_iyy)
        ET.SubElement(inertia, 'izz').text = str(izz)
        ET.SubElement(inertia, 'ixy').text = '0'
        ET.SubElement(inertia, 'ixz').text = '0'
        ET.SubElement(inertia, 'iyz').text = '0'
        
        visual = ET.SubElement(link, 'visual', name=f'{name}_visual')
        geometry = ET.SubElement(visual, 'geometry')
        cyl = ET.SubElement(geometry, 'cylinder')
        ET.SubElement(cyl, 'radius').text = f'{self.radius}'
        ET.SubElement(cyl, 'length').text = f'{length}'
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'ambient').text = '0.8 0.8 0.8 1.0'
        ET.SubElement(material, 'diffuse').text = '0.8 0.8 0.8 1.0'
        
        collision = ET.SubElement(link, 'collision', name=f'{name}_collision')
        geometry = ET.SubElement(collision, 'geometry')
        cyl = ET.SubElement(geometry, 'cylinder')
        ET.SubElement(cyl, 'radius').text = f'{self.radius}'
        ET.SubElement(cyl, 'length').text = f'{length}'

    def _add_intermediate_link(self, parent, name, parent_name, pose):
        """Add intermediate link for universal joints."""
        link = ET.SubElement(parent, 'link', name=name)
        if parent_name is None:
            ET.SubElement(link, 'pose').text = pose
        else:
            ET.SubElement(link, 'pose', relative_to=parent_name).text = pose

        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = '0.01' ## 0.01 0.001
        inertia = ET.SubElement(inertial, 'inertia')
        ET.SubElement(inertia, 'ixx').text = '1e-3'  ## 1e-3 1e-6
        ET.SubElement(inertia, 'iyy').text = '1e-3'  ## 1e-3 1e-6
        ET.SubElement(inertia, 'izz').text = '1e-3'  ## 1e-3 1e-6
        ET.SubElement(inertia, 'ixy').text = '0'
        ET.SubElement(inertia, 'ixz').text = '0'
        ET.SubElement(inertia, 'iyz').text = '0'

    def _add_revolute_joint(self, parent, name, parent_link, child_link,
                             axis_xyz, pose='0 0 0 0 0 0',
                             stiffness=None, limit_angle=None):
        joint = ET.SubElement(parent, 'joint', name=name, type='revolute')
        ET.SubElement(joint, 'parent').text = parent_link
        ET.SubElement(joint, 'child').text = child_link
        ET.SubElement(joint, 'pose', relative_to=parent_link).text = pose
        
        axis = ET.SubElement(joint, 'axis')
        ET.SubElement(axis, 'xyz').text = axis_xyz
        
        angle = limit_angle if limit_angle is not None else math.pi * 2/3
        limit = ET.SubElement(axis, 'limit')
        ET.SubElement(limit, 'lower').text = str(-angle)
        ET.SubElement(limit, 'upper').text = str(angle)
        ET.SubElement(limit, 'effort').text = '100'
        ET.SubElement(limit, 'velocity').text = '10'
        
        dynamics = ET.SubElement(axis, 'dynamics')
        ET.SubElement(dynamics, 'damping').text = f'{self.damping}'
        ET.SubElement(dynamics, 'friction').text = f'{self.friction}'
        
        k = stiffness if stiffness is not None else self.K
        ET.SubElement(dynamics, 'spring_stiffness').text = str(k)

    def _add_fixed_joint(self, parent, name, parent_link, child_link):
        """Add fixed joint."""
        joint = ET.SubElement(parent, 'joint', name=name, type='fixed')
        ET.SubElement(joint, 'parent').text = parent_link
        ET.SubElement(joint, 'child').text = child_link
        ET.SubElement(joint, 'pose').text = '0 0 0 0 0 0'

    def _add_universal_joints(self, model):
        joint_names = []
        
        if self.bending_links > 0:
            k_values = []
            k = self.K
            for i in range(self.bending_links + 1):
                k_values.append(k)
                k *= 0.85  # uniform stiffness; change multiplier here for gradient


            # Base → first bending link
            intermediate_name = 'base_to_bending_1_x_rotation'
            joint_x = 'base_to_bending_1_x'
            joint_y = 'base_to_bending_1_y'
            self._add_revolute_joint(model, joint_x, 'base_link', intermediate_name,
                                     '1 0 0', f'0 0 {self.L1/2} 0 0 0',
                                     stiffness=k_values[0])
            self._add_intermediate_link(model, name=intermediate_name,
                                        parent_name=joint_x, pose='0 0 0 0 0 0')
            self._add_revolute_joint(model, joint_y, intermediate_name, 'bending_link_1',
                                     '0 1 0', stiffness=k_values[0])
            joint_names.extend([joint_x, joint_y])
            
            # Intermediate bending links
            for i in range(1, self.bending_links):
                intermediate_name = f'bending_{i}_to_{i+1}_x_rotation'
                joint_x = f'bending_{i}_to_{i+1}_x'
                joint_y = f'bending_{i}_to_{i+1}_y'
                self._add_revolute_joint(model, joint_x, f'bending_link_{i}',
                                         intermediate_name, '1 0 0',
                                         f'0 0 {self.bending_link_length/2} 0 0 0',
                                         stiffness=k_values[i])
                self._add_intermediate_link(model, name=intermediate_name,
                                            parent_name=joint_x, pose='0 0 0 0 0 0')
                self._add_revolute_joint(model, joint_y, intermediate_name,
                                         f'bending_link_{i+1}', '0 1 0',
                                         stiffness=k_values[i])
                joint_names.extend([joint_x, joint_y])

            # Last bending link → tip
            intermediate_name = f'bending_{self.bending_links}_to_tip_x_rotation'
            joint_x = f'bending_{self.bending_links}_to_tip_x'
            joint_y = f'bending_{self.bending_links}_to_tip_y'
            self._add_revolute_joint(model, joint_x, f'bending_link_{self.bending_links}',
                                     intermediate_name, '1 0 0',
                                     f'0 0 {self.bending_link_length/2} 0 0 0',
                                     stiffness=k_values[-1])
            self._add_intermediate_link(model, name=intermediate_name,
                                        parent_name=joint_x, pose='0 0 0 0 0 0')
            self._add_revolute_joint(model, joint_y, intermediate_name, 'tip_link',
                                     '0 1 0', stiffness=k_values[-1])
            joint_names.extend([joint_x, joint_y])
        
        return joint_names

    def _add_joint_state_publisher(self, model, joint_names):
        """Add joint state publisher plugin."""
        plugin = ET.SubElement(model, 'plugin',
                               filename='gz-sim-joint-state-publisher-system',
                               name='gz::sim::systems::JointStatePublisher')
        # Include all base DOF joints + flexible joints
        all_joints = ['base_x_joint', 'base_y_joint',
                      'base_z_joint', 'base_rotation_joint'] + joint_names
        for joint_name in all_joints:
            ET.SubElement(plugin, 'joint_name').text = joint_name
    
    def _save_sdf_file(self, sdf_root, filename):
        """Save SDF file with pretty formatting."""
        rough_string = ET.tostring(sdf_root, 'unicode')
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent='  ')
        lines = [line for line in pretty_xml.split('\n') if line.strip()]
        final_xml = '\n'.join(lines)
        
        sdf_path = os.path.join(self.sdf_dir, filename)
        with open(sdf_path, 'w') as f:
            f.write(final_xml)
        return sdf_path

    # ─────────────────────────────────────────────────────────────────────────
    # Launch file
    # ─────────────────────────────────────────────────────────────────────────

    def generate_launch_file(self, xacro_filename):
        """Generate ROS2 launch file for the catheter."""
        package_name = os.path.basename(self.package_dir)
        launch_filename = f"{package_name}_launch.py"
        sdf_filename = xacro_filename.replace('.xacro', '.sdf')
        
        launch_content = f'''import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg_share = get_package_share_directory('{package_name}')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    robot_description_file = os.path.join(pkg_share, 'urdf', '{xacro_filename}')
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {{'robot_description': robot_description_config.toxml()}}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    custom_world_file = os.path.join(pkg_share, 'worlds', 'custom_world.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={{'gz_args': f'-r {{custom_world_file}}'}}.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{{'name': '{package_name}_model',
                    'file': os.path.join(pkg_share, 'sdf', '{sdf_filename}')}}],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/{package_name}_model/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            '/world/catheter_world/model/{package_name}_model/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # ROS2 → Gazebo position commands
            '/base_x_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/base_y_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/base_z_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/base_rotation_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            ('/world/catheter_world/model/{package_name}_model/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn,
        bridge,
        robot_state_publisher,
        rviz,
    ])
'''
        
        launch_path = os.path.join(self.launch_dir, launch_filename)
        with open(launch_path, 'w') as f:
            f.write(launch_content)
        return launch_path
    
    def generate_custom_world(self):
        """Generate custom world SDF file for Gazebo."""
        world_content = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="catheter_world">
    <physics name="1ms" type="ode">
      <max_step_size>0.0005</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>200</iters>
        </solver>
        <constraints>
          <contact_max_correcting_vel>0.1</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
    <gravity>0 0 0</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>'''
        
        world_path = os.path.join(self.worlds_dir, "custom_world.sdf")
        with open(world_path, 'w') as f:
            f.write(world_content)
        return world_path

    # ─────────────────────────────────────────────────────────────────────────
    # URDF / Xacro generation
    # ─────────────────────────────────────────────────────────────────────────

    def create_link_inertia(self, mass, length, radius):
        ixx = iyy = (1/12) * mass * (3 * radius**2 + length**2)
        izz = 0.5 * mass * radius**2
        return {'ixx': ixx, 'iyy': iyy, 'izz': izz, 'ixy': 0, 'ixz': 0, 'iyz': 0}
    
    def generate_xacro(self):
        """Generate the complete Xacro."""
        robot = ET.Element('robot', name="flexible_catheter")
        robot.set('xmlns:xacro', 'http://www.ros.org/wiki/xacro')
        
        self.generate_cylinder_stl(self.radius, self.L3, "tip_link.stl")
        self.generate_cylinder_stl(self.radius, self.L1, "base_link.stl")
        self.generate_cylinder_stl(self.radius, self.bending_link_length, "bending_link.stl")
        
        self.add_xacro_properties(robot)
        self.add_catheter_link_macro(robot)
        self.add_universal_joint_macro(robot)
        
        package_name = os.path.basename(self.package_dir)

        # Catheter links
        ET.SubElement(robot, 'xacro:catheter_link',
                     name='base_link', mass='${base_mass}', length='${base_length}',
                     radius='${catheter_radius}', xyz_origin=f'0 0 {self.L1/2}',
                     mesh_file=f'package://{package_name}/meshes/base_link.stl')
        
        for i in range(self.bending_links):
            ET.SubElement(robot, 'xacro:catheter_link',
                         name=f'bending_link_{i+1}', mass='${bending_mass_per_link}',
                         length='${bending_link_length}', radius='${catheter_radius}',
                         xyz_origin=f'0 0 {self.bending_link_length/2}',
                         mesh_file=f'package://{package_name}/meshes/bending_link.stl')
        
        ET.SubElement(robot, 'xacro:catheter_link',
                     name='tip_link', mass='${tip_mass}', length='${tip_length}',
                     radius='${catheter_radius}', xyz_origin=f'0 0 {self.L3/2}',
                     mesh_file=f'package://{package_name}/meshes/tip_link.stl')

        # ── Base mobility chain ─────────────────────────────────────────────
        # world (virtual) → base_x_link → base_y_link → base_z_link → base_link
        ET.SubElement(robot, 'link', name='world')

        total_length = self.L1 + self.L2 + self.L3

        def _urdf_virtual_link(name):
            lk = ET.SubElement(robot, 'link', name=name)
            inertial = ET.SubElement(lk, 'inertial')
            ET.SubElement(inertial, 'mass', value='0.01')
            ET.SubElement(inertial, 'inertia',
                          ixx='1e-3', iyy='1e-3', izz='1e-3',
                          ixy='0', ixz='0', iyz='0')

        def _urdf_prismatic(jname, parent, child, axis):
            j = ET.SubElement(robot, 'joint', name=jname, type='prismatic')
            ET.SubElement(j, 'parent', link=parent)
            ET.SubElement(j, 'child', link=child)
            ET.SubElement(j, 'origin', xyz='0 0 0', rpy='0 0 0')
            ET.SubElement(j, 'axis', xyz=axis)
            ET.SubElement(j, 'limit',
                          lower=str(-total_length), upper=str(total_length),
                          effort='100', velocity='1.0')
            ET.SubElement(j, 'dynamics',
                          damping=f'{self.damping}', friction=f'{self.friction}')

        # X translation
        _urdf_virtual_link('base_x_link')
        _urdf_prismatic('base_x_joint', 'world', 'base_x_link', '1 0 0')

        # Y translation
        _urdf_virtual_link('base_y_link')
        _urdf_prismatic('base_y_joint', 'base_x_link', 'base_y_link', '0 1 0')

        # Z translation (insertion)
        _urdf_virtual_link('base_z_link')
        _urdf_prismatic('base_z_joint', 'base_y_link', 'base_z_link', '0 0 1')

        # Z rotation (twist): base_z_link → base_link
        j_rot = ET.SubElement(robot, 'joint', name='base_rotation_joint', type='revolute')
        ET.SubElement(j_rot, 'parent', link='base_z_link')
        ET.SubElement(j_rot, 'child', link='base_link')
        ET.SubElement(j_rot, 'origin', xyz='0 0 0', rpy='0 0 0')
        ET.SubElement(j_rot, 'axis', xyz='0 0 1')
        ET.SubElement(j_rot, 'limit',
                      lower=str(-math.pi), upper=str(math.pi),
                      effort='100', velocity='10')
        ET.SubElement(j_rot, 'dynamics',
                      damping=f'{self.damping}', friction=f'{self.friction}')

        # ── Flexible joints ─────────────────────────────────────────────────
        if self.bending_links > 0:
            ET.SubElement(robot, 'xacro:universal_joint',
                         name='base_to_bending_1', parent='base_link', child='bending_link_1',
                         xyz_origin=f'0 0 {self.L1}', spring_k='${spring_constant}')
            
            for i in range(1, self.bending_links):
                ET.SubElement(robot, 'xacro:universal_joint',
                             name=f'bending_{i}_to_{i+1}', parent=f'bending_link_{i}',
                             child=f'bending_link_{i+1}',
                             xyz_origin=f'0 0 {self.bending_link_length}',
                             spring_k='${spring_constant}')
            
            ET.SubElement(robot, 'xacro:universal_joint',
                         name=f'bending_{self.bending_links}_to_tip',
                         parent=f'bending_link_{self.bending_links}', child='tip_link',
                         xyz_origin=f'0 0 {self.bending_link_length}',
                         spring_k='${spring_constant}')
        else:
            ET.SubElement(robot, 'xacro:universal_joint',
                         name='base_to_tip', parent='base_link', child='tip_link',
                         xyz_origin=f'0 0 {self.L1}', spring_k='${spring_constant}')

        return robot
    
    def add_xacro_properties(self, root):
        ET.SubElement(root, 'xacro:property', name='catheter_links', value=str(self.N))
        ET.SubElement(root, 'xacro:property', name='catheter_diameter', value=str(self.D))
        ET.SubElement(root, 'xacro:property', name='catheter_radius', value=str(self.radius))
        ET.SubElement(root, 'xacro:property', name='tip_length', value=str(self.L3))
        ET.SubElement(root, 'xacro:property', name='bending_length', value=str(self.L2))
        ET.SubElement(root, 'xacro:property', name='base_length', value=str(self.L1))
        ET.SubElement(root, 'xacro:property', name='spring_constant', value=str(self.K))
        ET.SubElement(root, 'xacro:property', name='total_mass', value=str(self.M))
        ET.SubElement(root, 'xacro:property', name='bending_links', value=str(self.bending_links))
        ET.SubElement(root, 'xacro:property', name='bending_link_length', value=str(self.bending_link_length))
        ET.SubElement(root, 'xacro:property', name='tip_mass', value=str(self.tip_mass))
        ET.SubElement(root, 'xacro:property', name='base_mass', value=str(self.base_mass))
        ET.SubElement(root, 'xacro:property', name='bending_mass_per_link', value=str(self.bending_mass_per_link))
        material = ET.SubElement(root, 'material', name='catheter_material')
        ET.SubElement(material, 'color', rgba='0.8 0.8 0.8 1.0')
    
    def add_catheter_link_macro(self, root):
        macro = ET.SubElement(root, 'xacro:macro', name='catheter_link')
        macro.set('params', 'name mass length radius xyz_origin mesh_file')
        
        link = ET.SubElement(macro, 'link', name='${name}')
        
        visual = ET.SubElement(link, 'visual')
        ET.SubElement(visual, 'origin', xyz='${xyz_origin}', rpy='0 0 0')
        visual_geometry = ET.SubElement(visual, 'geometry')
        ET.SubElement(visual_geometry, 'mesh', filename='${mesh_file}')
        ET.SubElement(visual, 'material', name='catheter_material')
        
        collision = ET.SubElement(link, 'collision')
        ET.SubElement(collision, 'origin', xyz='${xyz_origin}', rpy='0 0 0')
        collision_geometry = ET.SubElement(collision, 'geometry')
        ET.SubElement(collision_geometry, 'mesh', filename='${mesh_file}')
        
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'origin', xyz='${xyz_origin}', rpy='0 0 0')
        ET.SubElement(inertial, 'mass', value='${mass}')
        ixx_iyy = '${(1/12) * mass * (3 * radius * radius + length * length)}'
        izz = '${0.5 * mass * radius * radius}'
        ET.SubElement(inertial, 'inertia',
                     ixx=ixx_iyy, iyy=ixx_iyy, izz=izz,
                     ixy='0', ixz='0', iyz='0')
    
    def add_universal_joint_macro(self, root):
        macro = ET.SubElement(root, 'xacro:macro', name='universal_joint')
        macro.set('params', 'name parent child xyz_origin spring_k')
        
        intermediate_link = ET.SubElement(macro, 'link', name='${name}_x_rotation')
        inertial = ET.SubElement(intermediate_link, 'inertial')
        ET.SubElement(inertial, 'mass', value='0.01')
        ET.SubElement(inertial, 'inertia', ixx='1e-3', iyy='1e-3', izz='1e-3',
                     ixy='0', ixz='0', iyz='0')
        
        joint_x = ET.SubElement(macro, 'joint', name='${name}_x', type='revolute')
        ET.SubElement(joint_x, 'parent', link='${parent}')
        ET.SubElement(joint_x, 'child', link='${name}_x_rotation')
        ET.SubElement(joint_x, 'origin', xyz='${xyz_origin}', rpy='0 0 0')
        ET.SubElement(joint_x, 'axis', xyz='1 0 0')
        ET.SubElement(joint_x, 'limit', lower=str(-math.pi/2), upper=str(math.pi/2),
                     effort='100', velocity='10')
        ET.SubElement(joint_x, 'dynamics', damping=f'{self.damping}', friction=f'{self.friction}')
        
        joint_y = ET.SubElement(macro, 'joint', name='${name}_y', type='revolute')
        ET.SubElement(joint_y, 'parent', link='${name}_x_rotation')
        ET.SubElement(joint_y, 'child', link='${child}')
        ET.SubElement(joint_y, 'origin', xyz='0 0 0', rpy='0 0 0')
        ET.SubElement(joint_y, 'axis', xyz='0 1 0')
        ET.SubElement(joint_y, 'limit', lower=str(-math.pi/2), upper=str(math.pi/2),
                     effort='100', velocity='10')
        ET.SubElement(joint_y, 'dynamics', damping=f'{self.damping}', friction=f'{self.friction}')
        
        gazebo_x = ET.SubElement(macro, 'gazebo')
        plugin_x = ET.SubElement(gazebo_x, 'plugin', name='${name}_x_spring',
                                filename='libgazebo_ros_joint_spring.so')
        ET.SubElement(plugin_x, 'joint_name').text = '${name}_x'
        ET.SubElement(plugin_x, 'spring_constant').text = '${spring_k}'
        ET.SubElement(plugin_x, 'reference_position').text = '0.0'
        
        gazebo_y = ET.SubElement(macro, 'gazebo')
        plugin_y = ET.SubElement(gazebo_y, 'plugin', name='${name}_y_spring',
                                filename='libgazebo_ros_joint_spring.so')
        ET.SubElement(plugin_y, 'joint_name').text = '${name}_y'
        ET.SubElement(plugin_y, 'spring_constant').text = '${spring_k}'
        ET.SubElement(plugin_y, 'reference_position').text = '0.0'

    # ─────────────────────────────────────────────────────────────────────────
    # Teleop node
    # ─────────────────────────────────────────────────────────────────────────

    def generate_teleop_node(self):
        """Generate catheter_keyboard_teleop.py with X/Y/Z translation and Z rotation."""
        scripts_dir = os.path.join(self.package_dir, "scripts")
        if not os.path.exists(scripts_dir):
            os.makedirs(scripts_dir)

        total_length = self.L1 + self.L2 + self.L3 + 1.0 ## Adding 1.0 to extend the motion range.

        teleop_content = f'''#!/usr/bin/env python3
"""
Catheter Keyboard Teleop Node  (auto-generated by catheter_urdf_generator.py)
==============================================================================
Drives the catheter base by publishing Float64 position commands directly to
Gazebo JointPositionController topics.

  /base_x_joint/cmd_pos        → base_x_joint      (prismatic, X translation)
  /base_y_joint/cmd_pos        → base_y_joint      (prismatic, Y translation)
  /base_z_joint/cmd_pos        → base_z_joint      (prismatic, Z translation)
  /base_rotation_joint/cmd_pos → base_rotation_joint (revolute, Z rotation)

Controls:
  W / S        →  Insert  / Retract  (Z translation)
  A / D        →  CCW     / CW       (Z rotation)
  Arrow Up/Down   →  Y+ / Y-         (Y translation)
  Arrow Left/Right → X- / X+        (X translation)
  Q            →  Quit
"""

import sys
import math
import threading
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# ── Tuneable parameters ───────────────────────────────────────────────
Z_STEP          = 0.002                    # metres per key-press  (2 mm)
XY_STEP         = 0.002                    # metres per key-press  (2 mm)
ROT_STEP        = math.radians(2)           # radians per key-press (2°)

XY_MIN          = -{total_length:.6f}
XY_MAX          =  {total_length:.6f}
Z_MIN           = -{total_length:.6f}
Z_MAX           =  {total_length:.6f}
ROT_MIN         = -math.pi
ROT_MAX         =  math.pi

PUBLISH_RATE_HZ = 50

KEY_INSERT  = \'w\'
KEY_RETRACT = \'s\'
KEY_CCW     = \'a\'
KEY_CW      = \'d\'
KEY_QUIT    = \'q\'

# Arrow key escape sequences
KEY_UP    = \'\\x1b[A\'
KEY_DOWN  = \'\\x1b[B\'
KEY_RIGHT = \'\\x1b[C\'
KEY_LEFT  = \'\\x1b[D\'
# ─────────────────────────────────────────────────────────────────────

BANNER = """
╔══════════════════════════════════════════════════╗
║         Catheter Keyboard Teleop                 ║
╠══════════════════════════════════════════════════╣
║  W / S        →  Insert  / Retract  (Z trans)   ║
║  A / D        →  CCW     / CW       (Z rot)     ║
║  ↑ / ↓        →  Y+      / Y-       (Y trans)   ║
║  → / ←        →  X+      / X-       (X trans)   ║
║  Q            →  Quit                           ║
╠══════════════════════════════════════════════════╣
║  Publishes Float64 to:                          ║
║    /base_x_joint/cmd_pos                        ║
║    /base_y_joint/cmd_pos                        ║
║    /base_z_joint/cmd_pos                        ║
║    /base_rotation_joint/cmd_pos                 ║
╚══════════════════════════════════════════════════╝
"""

def get_key(timeout=0.05):
    """Read one keypress; correctly handles arrow-key escape sequences."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return ''
        import os as _os
        key = _os.read(fd, 1).decode('utf-8', errors='ignore')
        if key == '\x1b':
            r2, _, _ = select.select([sys.stdin], [], [], 0.02)
            if r2:
                key += _os.read(fd, 1).decode('utf-8', errors='ignore')
                r3, _, _ = select.select([sys.stdin], [], [], 0.01)
                if r3:
                    key += _os.read(fd, 1).decode('utf-8', errors='ignore')
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


class CatheterTeleopNode(Node):
    def __init__(self):
        super().__init__(\'catheter_keyboard_teleop\')
        self.x_pos  = 0.0
        self.y_pos  = 0.0
        self.z_pos  = 0.0
        self.z_rot  = 0.0
        self._lock  = threading.Lock()

        self.pub_x   = self.create_publisher(Float64, \'/base_x_joint/cmd_pos\', 10)
        self.pub_y   = self.create_publisher(Float64, \'/base_y_joint/cmd_pos\', 10)
        self.pub_z   = self.create_publisher(Float64, \'/base_z_joint/cmd_pos\', 10)
        self.pub_rot = self.create_publisher(Float64, \'/base_rotation_joint/cmd_pos\', 10)

        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish)
        self.get_logger().info(\'Catheter teleop node started.\')

    def _publish(self):
        with self._lock:
            x, y, z, rot = self.x_pos, self.y_pos, self.z_pos, self.z_rot
        for pub, val in [(self.pub_x, x), (self.pub_y, y),
                         (self.pub_z, z), (self.pub_rot, rot)]:
            msg = Float64()
            msg.data = val
            pub.publish(msg)

    def apply_key(self, key: str) -> bool:
        """Apply keypress; return False to quit."""
        if key.lower() == KEY_QUIT:
            return False
        with self._lock:
            k = key.lower() if len(key) == 1 else key  # preserve escape seqs
            if k == KEY_INSERT:
                self.z_pos = min(self.z_pos + Z_STEP, Z_MAX)
            elif k == KEY_RETRACT:
                self.z_pos = max(self.z_pos - Z_STEP, Z_MIN)
            elif k == KEY_CCW:
                self.z_rot = min(self.z_rot + ROT_STEP, ROT_MAX)
            elif k == KEY_CW:
                self.z_rot = max(self.z_rot - ROT_STEP, ROT_MIN)
            elif k == KEY_UP:
                self.y_pos = min(self.y_pos + XY_STEP, XY_MAX)
            elif k == KEY_DOWN:
                self.y_pos = max(self.y_pos - XY_STEP, XY_MIN)
            elif k == KEY_RIGHT:
                self.x_pos = min(self.x_pos + XY_STEP, XY_MAX)
            elif k == KEY_LEFT:
                self.x_pos = max(self.x_pos - XY_STEP, XY_MIN)
        return True

    def status_line(self) -> str:
        with self._lock:
            return (
                f"  X: {{self.x_pos*1000:+7.2f}} mm  "
                f"Y: {{self.y_pos*1000:+7.2f}} mm  "
                f"Z: {{self.z_pos*1000:+7.2f}} mm  |  "
                f"Rot: {{math.degrees(self.z_rot):+7.2f}} °   \\r"
            )


def main(args=None):
    rclpy.init(args=args)
    node = CatheterTeleopNode()
    print(BANNER)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        while rclpy.ok():
            key = get_key(timeout=0.05)
            if key:
                if not node.apply_key(key):
                    break
            sys.stdout.write(node.status_line())
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        print(\'\\nShutting down catheter teleop.\')
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == \'__main__\':
    main()
'''

        teleop_path = os.path.join(scripts_dir, "catheter_keyboard_teleop.py")
        with open(teleop_path, 'w') as f:
            f.write(teleop_content)
        import stat
        st = os.stat(teleop_path)
        os.chmod(teleop_path, st.st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)
        return teleop_path

    # ─────────────────────────────────────────────────────────────────────────
    # Entry point
    # ─────────────────────────────────────────────────────────────────────────

    def save_xacro(self, filename=None):
        """Save Xacro + all supporting files."""
        robot = self.generate_xacro()
        
        if filename is None:
            package_name = os.path.basename(self.package_dir)
            filename = f"{package_name}.xacro"
        
        rough_string = ET.tostring(robot, 'unicode')
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent="  ")
        lines = [line for line in pretty_xml.split('\n') if line.strip()]
        final_xml = '\n'.join(lines)
        
        xacro_path = os.path.join(self.urdf_dir, filename)
        with open(xacro_path, 'w') as f:
            f.write(final_xml)
        
        package_xml_path = self.generate_package_xml()
        cmake_path       = self.generate_cmakelists_txt()
        sdf_path         = self.generate_sdf(filename)
        launch_path      = self.generate_launch_file(filename)
        world_path       = self.generate_custom_world()
        teleop_path      = self.generate_teleop_node()
        
        print(f"ROS2 package created: {self.package_dir}/")
        print(f"  Package files:")
        print(f"    - {os.path.relpath(package_xml_path, self.package_dir)}")
        print(f"    - {os.path.relpath(cmake_path, self.package_dir)}")
        print(f"  Xacro:   {os.path.relpath(xacro_path, self.package_dir)}")
        print(f"  SDF:     {os.path.relpath(sdf_path, self.package_dir)}")
        print(f"  World:   {os.path.relpath(world_path, self.package_dir)}")
        print(f"  Launch:  {os.path.relpath(launch_path, self.package_dir)}")
        print(f"  Teleop:  {os.path.relpath(teleop_path, self.package_dir)}")
        print(f"  Meshes:  {os.path.relpath(self.meshes_dir, self.package_dir)}/")
        print(f"  Base DOF: X-trans, Y-trans, Z-trans (insert), Z-rot (twist)")


def main():
    parser = argparse.ArgumentParser(description="Generate Xacro for flexible catheter")
    parser.add_argument("--N",   type=int,   default=5,     help="Number of links")
    parser.add_argument("--D",   type=float, default=0.002, help="Outer diameter (m)")
    parser.add_argument("--L1",  type=float, default=0.05,  help="Base link length (m)")
    parser.add_argument("--L2",  type=float, default=0.1,   help="Bending section length (m)")
    parser.add_argument("--L3",  type=float, default=0.01,  help="Tip link length (m)")
    parser.add_argument("--K",   type=float, default=0.1,   help="Spring constant (Nm/rad)")
    parser.add_argument("--Kd",  type=float, default=0.1,   help="Damping constant")
    parser.add_argument("--Kf",  type=float, default=0.01,  help="Friction")
    parser.add_argument("--M",   type=float, default=0.01,  help="Total mass (kg)")
    parser.add_argument("--output",     type=str, default="catheter_package",
                        help="Package directory name")
    parser.add_argument("--xacro-name", type=str, default=None,
                        help="Xacro filename (default: <package_name>.xacro)")
    
    args = parser.parse_args()
    
    try:
        generator = CatheterXacroGenerator(
            args.N, args.D, args.L1, args.L2, args.L3,
            args.K, args.Kd, args.Kf, args.M, args.output)
        generator.save_xacro(args.xacro_name)
        
        print(f"\nGenerated catheter with parameters:")
        print(f"  Links: {args.N}")
        print(f"  Diameter: {args.D} m")
        print(f"  Base length: {args.L1} m")
        print(f"  Bending section length: {args.L2} m")
        print(f"  Tip length: {args.L3} m")
        print(f"  Spring constant: {args.K} Nm/rad")
        print(f"  Damping constant: {args.Kd}")
        print(f"  Friction constant: {args.Kf}")
        print(f"  Total mass: {args.M} kg")
        
    except ValueError as e:
        print(f"Error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())


