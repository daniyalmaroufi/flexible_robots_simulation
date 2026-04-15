#!/usr/bin/env python3
"""
Catheter Generator for Flexible Catheter Simulation

Generates a self-contained ROS2 package for simulating a flexible catheter
in Gazebo Harmonic and visualizing it in RViz2.

The catheter is modeled as a serial-link chain with spring-damper universal
joints, consisting of:
  - Base link       (length L1, rigid)
  - Bending section (N-2 links, total length L2, spring-driven)
  - Tip link        (length L3, rigid)

Usage
-----
Passive mode (observe deformation under external forces):
  python3 catheter_generator.py --output my_catheter

Active mode (4-DOF motorized base + keyboard teleoperation):
  python3 catheter_generator.py --output my_catheter --controller

With a single anatomy STL visible in both RViz and Gazebo:
  python3 catheter_generator.py --output my_catheter \\
      --anatomy-stl path/to/anatomy.stl \\
      --anatomy-x 0.0 --anatomy-y -0.02 --anatomy-z 0.56

With multiple anatomy models loaded from a 3D Slicer MRML scene file:
  python3 catheter_generator.py --output my_catheter \\
      --slicer-scene path/to/Scene.mrml

  All vtkMRMLModelNode entries in the scene are imported.  If a model is
  parented to a vtkMRMLLinearTransformNode the transform (including nested
  chains) is applied automatically.

  Coordinate handling: SlicerROS2 maps ROS ↔ Slicer RAS directly (no flip).
  LPS-stored meshes (DICOM default) have X and Y negated automatically.
  Mesh files must be co-located with the .mrml file.  STL is preferred;
  the parser also accepts VTK polydata files with the .vtk extension if an
  STL counterpart is absent.
"""

import re
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
import math
import argparse
import numpy as np
import os
import struct


# ─────────────────────────────────────────────────────────────────────────────
# 3D Slicer MRML scene parser
# ─────────────────────────────────────────────────────────────────────────────

def parse_slicer_scene(mrml_path, scale=0.001):
    """Parse a 3D Slicer MRML scene file and return anatomy model descriptors.

    Handles vtkMRMLModelNode, vtkMRMLModelStorageNode, vtkMRMLModelDisplayNode,
    and vtkMRMLLinearTransformNode elements.  Nested (chained) linear transforms
    are composed automatically.

    Coordinate conventions
    ----------------------
    SlicerROS2 maps the ROS world frame directly onto Slicer's RAS frame
    (R=+X, A=+Y, S=+Z), with no axis flip.  Transforms stored in the MRML
    file are already in RAS, so they are applied to the ROS world as-is
    (translation units are scaled from mm to metres).

    Mesh files whose storage node declares ``coordinateSystem="LPS"`` are
    stored with axes L=+X, P=+Y, S=+Z — rotated 180° about Z relative to
    RAS.  The ``flip_xy`` flag is set for such models so that
    ``_copy_stl_scaled`` negates X and Y of every vertex (and the stored face
    normal) when writing the output mesh, bringing it into the RAS/ROS frame.

    Parameters
    ----------
    mrml_path : str
        Path to the .mrml XML scene file.  Mesh files are resolved relative
        to the directory that contains the .mrml file.
    scale : float
        Scale factor applied to mesh vertices when copying (default 0.001
        converts millimetres → metres).

    Returns
    -------
    list[dict]  One entry per vtkMRMLModelNode whose mesh file was found:
        name             – sanitised model name (alphanumeric + underscores)
        src_stl          – absolute path to the mesh file (STL preferred over VTK)
        scale            – vertex scale factor
        flip_xy          – True when the mesh is in LPS and needs X/Y negation
        xyz              – (x, y, z) position in ROS frame (metres)
        rpy              – (roll, pitch, yaw) in ROS frame (radians, ZYX convention)
        color            – (r, g, b, a) display colour, floats 0–1
    """
    mrml_path = os.path.abspath(mrml_path)
    scene_dir = os.path.dirname(mrml_path)

    tree = ET.parse(mrml_path)
    root = tree.getroot()

    # ── collect all relevant nodes by id ─────────────────────────────────────
    transforms        = {}   # id -> 4×4 np.ndarray (Slicer RAS, original units)
    transform_parents = {}   # id -> parent transform id or None
    model_infos       = {}   # id -> dict
    storage_files     = {}   # id -> fileName string
    storage_coord_sys = {}   # id -> coordinateSystem string ('LPS' | 'RAS')
    display_colors    = {}   # id -> (r, g, b)

    for node in root:
        nid = node.get('id', '')
        tag = node.tag

        if tag == 'LinearTransform':
            mat_str = node.get('matrixTransformToParent', '').strip()
            if mat_str:
                vals = [float(v) for v in mat_str.split()]
                mat = np.array(vals, dtype=float).reshape(4, 4)
            else:
                mat = np.eye(4)
            transforms[nid] = mat
            transform_parents[nid] = node.get('parentTransformNodeRef') or None

        elif tag == 'Model':
            refs = node.get('references', '')
            storage_id = display_id = None
            for ref in refs.split(';'):
                ref = ref.strip()
                if ref.startswith('storage:'):
                    storage_id = ref[len('storage:'):]
                elif ref.startswith('display:'):
                    display_id = ref[len('display:'):]
            model_infos[nid] = {
                'name':        node.get('name', nid),
                'storage_id':  storage_id,
                'display_id':  display_id,
                'parent_tf_id': node.get('parentTransformNodeRef') or None,
            }

        elif tag == 'ModelStorage':
            storage_files[nid] = node.get('fileName', '')
            storage_coord_sys[nid] = node.get('coordinateSystem', 'LPS').upper()

        elif tag == 'ModelDisplay':
            try:
                c = node.get('color', '0.8 0.2 0.2').split()
                display_colors[nid] = tuple(float(v) for v in c[:3])
            except ValueError:
                display_colors[nid] = (0.8, 0.2, 0.2)

    # ── coordinate conversion ─────────────────────────────────────────────────
    # SlicerROS2 maps the ROS world frame directly onto Slicer RAS (no axis
    # flip).  MRML transforms are therefore used as-is in ROS; only the
    # translation needs to be scaled from mm to metres.

    def world_transform_ras(tf_id, _seen=None):
        """Recursively compose the full transform chain to the scene root."""
        if tf_id is None or tf_id not in transforms:
            return np.eye(4)
        _seen = _seen or set()
        if tf_id in _seen:          # cycle guard
            return np.eye(4)
        _seen.add(tf_id)
        parent_id = transform_parents.get(tf_id)
        return world_transform_ras(parent_id, _seen) @ transforms[tf_id]

    def ras_to_ros(T_ras):
        """Scale the translation of a Slicer RAS transform from mm to metres.

        SlicerROS2 maps ROS ↔ Slicer RAS with a direct axis correspondence
        (no flip), so the rotation part is unchanged.
        """
        T = T_ras.copy()
        T[:3, 3] *= scale           # translate scene units (mm) → metres
        return T

    def mat_to_xyz_rpy(T):
        """Decompose a 4×4 rigid transform into xyz and ZYX Euler angles."""
        x, y, z = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
        R = T[:3, :3]
        sin_p = float(np.clip(-R[2, 0], -1.0, 1.0))
        pitch = math.asin(sin_p)
        cos_p = math.cos(pitch)
        if abs(cos_p) > 1e-10:
            roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
            yaw  = math.atan2(float(R[1, 0]), float(R[0, 0]))
        else:
            roll = 0.0
            yaw  = math.atan2(float(-R[0, 1]), float(R[1, 1]))
        return (x, y, z), (roll, pitch, yaw)

    def find_mesh(filename):
        """Locate the mesh file in the scene directory.

        The MRML storage node may reference a .vtk file while only an STL
        counterpart exists (or vice versa).  Extensions are tried in order of
        preference; the exact filename is always attempted last.
        """
        base = os.path.splitext(os.path.join(scene_dir, filename))[0]
        for ext in ('.stl', '.STL', '.obj', '.OBJ', '.vtk', '.VTK', ''):
            p = base + ext
            if os.path.isfile(p):
                return p
        exact = os.path.join(scene_dir, filename)
        return exact if os.path.isfile(exact) else None

    def sanitise(name):
        """Return a ROS/SDF-safe identifier (alphanumeric + underscores)."""
        return re.sub(r'[^A-Za-z0-9_]', '_', name).strip('_') or 'model'

    # ── build result list ────────────────────────────────────────────────────
    models = []
    for nid, info in model_infos.items():
        stl_path = None
        if info['storage_id'] and info['storage_id'] in storage_files:
            stl_path = find_mesh(storage_files[info['storage_id']])
        if stl_path is None:
            print(f"  [slicer_scene] Warning: mesh not found for model "
                  f"'{info['name']}' — skipping.")
            continue

        T_ras = world_transform_ras(info['parent_tf_id'])
        T_ros = ras_to_ros(T_ras)
        xyz, rpy = mat_to_xyz_rpy(T_ros)

        if info['display_id'] and info['display_id'] in display_colors:
            r, g, b = display_colors[info['display_id']]
        else:
            r, g, b = 0.8, 0.2, 0.2

        # Mesh files stored in LPS need X and Y negated to reach RAS = ROS.
        sid = info['storage_id']
        coord_sys = storage_coord_sys.get(sid, 'LPS') if sid else 'LPS'
        flip_xy = (coord_sys == 'LPS')

        models.append({
            'name':    sanitise(info['name']),
            'src_stl': stl_path,
            'scale':   scale,
            'flip_xy': flip_xy,
            'xyz':     xyz,
            'rpy':     rpy,
            'color':   (r, g, b, 1.0),
        })

    return models


class CatheterGenerator:
    def __init__(self, N, D, L1, L2, L3, K, Kd, Kf, M,
                 package_dir="catheter_package",
                 with_controller=False,
                 anatomy_stl=None, anatomy_xyz=(0, 0, 0), anatomy_rpy=(0, 0, 0),
                 anatomy_scale=0.001,
                 slicer_scene=None, slicer_scale=0.001):
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
        self.with_controller = with_controller
        # Legacy single-anatomy attributes (kept for API compatibility)
        self.anatomy_stl = anatomy_stl
        self.anatomy_xyz = anatomy_xyz
        self.anatomy_rpy = anatomy_rpy
        self.anatomy_scale = anatomy_scale

        # Unified list of anatomy models (dicts) used by all generation methods.
        # Entries from --anatomy-stl come first; Slicer scene entries follow.
        self.anatomy_models = []
        if anatomy_stl:
            self.anatomy_models.append({
                'name':    'anatomy',
                'src_stl': os.path.abspath(anatomy_stl),
                'scale':   anatomy_scale,
                'xyz':     anatomy_xyz,
                'rpy':     anatomy_rpy,
                'color':   (0.8, 0.2, 0.2, 0.5),
            })
        if slicer_scene:
            scene_models = parse_slicer_scene(slicer_scene, slicer_scale)
            existing_names = {m['name'] for m in self.anatomy_models}
            for sm in scene_models:
                # Resolve name collisions by appending a numeric suffix
                base = sm['name']
                name, n = base, 1
                while name in existing_names:
                    n += 1
                    name = f'{base}_{n}'
                sm['name'] = name
                existing_names.add(name)
            self.anatomy_models.extend(scene_models)

        self.meshes_dir = os.path.join(package_dir, "meshes")
        self.urdf_dir   = os.path.join(package_dir, "urdf")
        self.sdf_dir    = os.path.join(package_dir, "sdf")
        self.launch_dir = os.path.join(package_dir, "launch")
        self.worlds_dir = os.path.join(package_dir, "worlds")
        self.scripts_dir = os.path.join(package_dir, "scripts")
        self.config_dir  = os.path.join(package_dir, "config")

        self.validate_parameters()
        self.calculate_derived_values()
        self.create_package_structure()

    # ─────────────────────────────────────────────────────────────────────────
    # Initialisation helpers
    # ─────────────────────────────────────────────────────────────────────────

    def validate_parameters(self):
        if self.N < 3:
            raise ValueError("N must be at least 3 (base, at least one bending link, tip)")
        if any(val <= 0 for val in [self.D, self.L1, self.L2, self.L3, self.K, self.M]):
            raise ValueError("All physical parameters must be positive")

    def calculate_derived_values(self):
        self.bending_links = self.N - 2
        self.bending_link_length = self.L2 / self.bending_links if self.bending_links > 0 else 0
        self.radius = self.D / 2
        total = self.L1 + self.L2 + self.L3
        self.tip_mass          = self.M * (self.L3 / total)
        self.base_mass         = self.M * (self.L1 / total)
        self.bending_mass_per_link = (
            self.M * (self.L2 / total) / self.bending_links
            if self.bending_links > 0 else 0
        )

    def create_package_structure(self):
        for d in [self.package_dir, self.meshes_dir, self.urdf_dir,
                  self.sdf_dir, self.launch_dir, self.worlds_dir,
                  self.config_dir, self.scripts_dir]:
            os.makedirs(d, exist_ok=True)

    # ─────────────────────────────────────────────────────────────────────────
    # Mesh generation
    # ─────────────────────────────────────────────────────────────────────────

    def generate_cylinder_stl(self, radius, length, filename, resolution=20):
        """Write an ASCII STL file for a cylinder."""
        theta = np.linspace(0, 2 * np.pi, resolution)
        top_center    = [0, 0,  length / 2]
        bottom_center = [0, 0, -length / 2]
        top_circle    = [[radius * np.cos(t), radius * np.sin(t),  length / 2] for t in theta]
        bottom_circle = [[radius * np.cos(t), radius * np.sin(t), -length / 2] for t in theta]
        vertices = [top_center, bottom_center] + top_circle + bottom_circle

        n = len(theta)
        faces = []
        for i in range(n):                                          # top cap
            faces.append([0, 2 + i, 2 + (i + 1) % n])
        for i in range(n):                                          # bottom cap
            faces.append([1, 2 + n + (i + 1) % n, 2 + n + i])
        for i in range(n):                                          # side
            ni = (i + 1) % n
            faces.append([2 + i, 2 + n + i, 2 + ni])
            faces.append([2 + ni, 2 + n + i, 2 + n + ni])

        stl_path = os.path.join(self.meshes_dir, filename)
        with open(stl_path, 'w') as f:
            f.write("solid cylinder\n")
            for face in faces:
                v1 = np.array(vertices[face[1]]) - np.array(vertices[face[0]])
                v2 = np.array(vertices[face[2]]) - np.array(vertices[face[0]])
                normal = np.cross(v1, v2)
                norm = np.linalg.norm(normal)
                normal = normal / norm if norm > 0 else np.array([0, 0, 1])
                f.write(f"  facet normal {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}\n")
                f.write("    outer loop\n")
                for idx in face:
                    v = vertices[idx]
                    f.write(f"      vertex {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
                f.write("    endloop\n")
                f.write("  endfacet\n")
            f.write("endsolid cylinder\n")
        return stl_path

    def _copy_stl_scaled(self, src_path, dst_path, scale, flip_xy=False):
        """Copy an STL file with all vertex coordinates multiplied by *scale*.

        If *flip_xy* is True the X and Y coordinates of every vertex — and
        the X and Y components of the stored face normal — are additionally
        negated.  This is a 180° rotation about Z and is used to convert
        meshes from LPS storage convention to the RAS/ROS frame, consistent
        with how SlicerROS2 maps axes between ROS and Slicer.

        Handles both ASCII and binary STL formats.  The output is always
        written as binary STL (more compact and unambiguous).
        """
        sx = -scale if flip_xy else scale
        sy = -scale if flip_xy else scale
        sn = -1.0   if flip_xy else 1.0   # normal sign for X and Y

        with open(src_path, 'rb') as f:
            raw = f.read()

        triangles = []  # list of (normal_3f, v0_3f, v1_3f, v2_3f)

        # ---- Try ASCII ----
        text = raw.decode('ascii', errors='replace')
        if text.lstrip().lower().startswith('solid'):
            try:
                lines = iter(text.splitlines())
                for line in lines:
                    if line.strip().lower().startswith('facet normal'):
                        parts = line.strip().split()
                        normal = (float(parts[2]) * sn,
                                  float(parts[3]) * sn,
                                  float(parts[4]))
                        next(lines)  # outer loop
                        verts = []
                        for _ in range(3):
                            vline = next(lines).strip().split()
                            verts.append((float(vline[1]) * sx,
                                          float(vline[2]) * sy,
                                          float(vline[3]) * scale))
                        triangles.append((normal, verts[0], verts[1], verts[2]))
            except (StopIteration, ValueError, IndexError):
                triangles = []  # fall through to binary

        # ---- Try binary if ASCII parse yielded nothing ----
        if not triangles:
            # skip 80-byte header + 4-byte triangle count
            offset = 84
            n_tri = struct.unpack_from('<I', raw, 80)[0]
            for _ in range(n_tri):
                vals = struct.unpack_from('<12f', raw, offset)
                normal = (vals[0] * sn, vals[1] * sn, vals[2])
                v0 = (vals[3] * sx,  vals[4] * sy,  vals[5]  * scale)
                v1 = (vals[6] * sx,  vals[7] * sy,  vals[8]  * scale)
                v2 = (vals[9] * sx,  vals[10] * sy, vals[11] * scale)
                triangles.append((normal, v0, v1, v2))
                offset += 50  # 12 floats × 4 bytes + 2-byte attribute

        # ---- Write binary STL ----
        with open(dst_path, 'wb') as f:
            f.write(b'\0' * 80)                          # header
            f.write(struct.pack('<I', len(triangles)))   # triangle count
            for normal, v0, v1, v2 in triangles:
                f.write(struct.pack('<3f', *normal))
                f.write(struct.pack('<3f', *v0))
                f.write(struct.pack('<3f', *v1))
                f.write(struct.pack('<3f', *v2))
                f.write(b'\x00\x00')                     # attribute byte count

    # ─────────────────────────────────────────────────────────────────────────
    # ROS2 package files
    # ─────────────────────────────────────────────────────────────────────────

    def generate_package_xml(self):
        package_name = os.path.basename(self.package_dir)
        content = f'''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypefile="package_format3.xsd"?>
<package format="3">
  <name>{package_name}</name>
  <version>0.0.0</version>
  <description>Flexible catheter simulation package generated by catheter_generator.py</description>
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
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>rclpy</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
    <gazebo_ros gazebo_model_path="${{prefix}}/../"/>
    <gazebo_ros gazebo_media_path="${{prefix}}/media"/>
    <gazebo_ros plugin_path="${{prefix}}/plugins"/>
  </export>
</package>'''
        path = os.path.join(self.package_dir, "package.xml")
        with open(path, 'w') as f:
            f.write(content)
        return path

    def generate_cmakelists_txt(self):
        package_name = os.path.basename(self.package_dir)
        if self.with_controller:
            scripts_block = (
                f'\ninstall(PROGRAMS\n'
                f'  scripts/catheter_pose_broadcaster.py\n'
                f'  scripts/catheter_keyboard_teleop.py\n'
                f'  DESTINATION lib/${{PROJECT_NAME}}\n)\n'
            )
        else:
            scripts_block = (
                f'\ninstall(PROGRAMS\n'
                f'  scripts/catheter_pose_broadcaster.py\n'
                f'  DESTINATION lib/${{PROJECT_NAME}}\n)\n'
            )
        content = f'''cmake_minimum_required(VERSION 3.8)
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

install(DIRECTORY config/
  DESTINATION share/${{PROJECT_NAME}}/config/
){scripts_block}
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()'''
        path = os.path.join(self.package_dir, "CMakeLists.txt")
        with open(path, 'w') as f:
            f.write(content)
        return path

    # ─────────────────────────────────────────────────────────────────────────
    # SDF helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _add_sdf_link(self, parent, name, parent_name, pose, mass, length, mesh_file):
        """Add a catheter link (cylinder visual + collision) to the SDF model."""
        link = ET.SubElement(parent, 'link', name=name)
        if parent_name is None:
            ET.SubElement(link, 'pose').text = pose
        else:
            ET.SubElement(link, 'pose', relative_to=parent_name).text = pose

        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = str(mass)
        inertia = ET.SubElement(inertial, 'inertia')
        ixx_iyy = (1 / 12) * mass * (3 * self.radius ** 2 + length ** 2)
        izz     = 0.5 * mass * self.radius ** 2
        for tag, val in [('ixx', ixx_iyy), ('iyy', ixx_iyy), ('izz', izz),
                         ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
            ET.SubElement(inertia, tag).text = str(val)

        for kind in ('visual', 'collision'):
            el = ET.SubElement(link, kind, name=f'{name}_{kind}')
            geom = ET.SubElement(el, 'geometry')
            cyl = ET.SubElement(geom, 'cylinder')
            ET.SubElement(cyl, 'radius').text = str(self.radius)
            ET.SubElement(cyl, 'length').text = str(length)
            if kind == 'visual':
                mat = ET.SubElement(el, 'material')
                ET.SubElement(mat, 'ambient').text = '0.8 0.8 0.8 1.0'
                ET.SubElement(mat, 'diffuse').text = '0.8 0.8 0.8 1.0'

    def _add_intermediate_link(self, parent, name, parent_name, pose):
        """Add a massless intermediate link used inside each universal joint."""
        link = ET.SubElement(parent, 'link', name=name)
        if parent_name is None:
            ET.SubElement(link, 'pose').text = pose
        else:
            ET.SubElement(link, 'pose', relative_to=parent_name).text = pose
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = '0.01'
        inertia = ET.SubElement(inertial, 'inertia')
        for tag in ['ixx', 'iyy', 'izz']:
            ET.SubElement(inertia, tag).text = '1e-3'
        for tag in ['ixy', 'ixz', 'iyz']:
            ET.SubElement(inertia, tag).text = '0'

    def _add_revolute_joint(self, parent, name, parent_link, child_link,
                            axis_xyz, pose='0 0 0 0 0 0',
                            stiffness=None, limit_angle=None):
        """Add a revolute joint with spring stiffness in its dynamics."""
        joint = ET.SubElement(parent, 'joint', name=name, type='revolute')
        ET.SubElement(joint, 'parent').text = parent_link
        ET.SubElement(joint, 'child').text  = child_link
        ET.SubElement(joint, 'pose', relative_to=parent_link).text = pose

        axis = ET.SubElement(joint, 'axis')
        ET.SubElement(axis, 'xyz').text = axis_xyz

        angle = limit_angle if limit_angle is not None else math.pi * 2 / 3
        limit = ET.SubElement(axis, 'limit')
        ET.SubElement(limit, 'lower').text    = str(-angle)
        ET.SubElement(limit, 'upper').text    = str(angle)
        ET.SubElement(limit, 'effort').text   = '100'
        ET.SubElement(limit, 'velocity').text = '10'

        dyn = ET.SubElement(axis, 'dynamics')
        ET.SubElement(dyn, 'damping').text          = str(self.damping)
        ET.SubElement(dyn, 'friction').text         = str(self.friction)
        ET.SubElement(dyn, 'spring_stiffness').text = str(stiffness if stiffness is not None else self.K)

    def _add_fixed_joint(self, parent, name, parent_link, child_link):
        joint = ET.SubElement(parent, 'joint', name=name, type='fixed')
        ET.SubElement(joint, 'parent').text = parent_link
        ET.SubElement(joint, 'child').text  = child_link
        ET.SubElement(joint, 'pose').text   = '0 0 0 0 0 0'

    def _add_universal_joints(self, model):
        """Add spring-driven universal joints between all catheter links.

        Spring stiffness decreases distally by 0.85× per joint to approximate
        the softer tip of a real catheter.
        """
        joint_names = []
        if self.bending_links == 0:
            intermediate = 'base_to_tip_x_rotation'
            jx, jy = 'base_to_tip_x', 'base_to_tip_y'
            self._add_revolute_joint(model, jx, 'base_link', intermediate,
                                     '1 0 0', f'0 0 {self.L1 / 2} 0 0 0',
                                     stiffness=self.K)
            self._add_intermediate_link(model, intermediate, jx, '0 0 0 0 0 0')
            self._add_revolute_joint(model, jy, intermediate, 'tip_link',
                                     '0 1 0', stiffness=self.K)
            return [jx, jy]

        # Build a per-joint stiffness gradient
        k_values = []
        k = self.K
        for _ in range(self.bending_links + 1):
            k_values.append(k)
            k *= 0.85

        # Base → first bending link
        intermediate = 'base_to_bending_1_x_rotation'
        jx, jy = 'base_to_bending_1_x', 'base_to_bending_1_y'
        self._add_revolute_joint(model, jx, 'base_link', intermediate,
                                 '1 0 0', f'0 0 {self.L1 / 2} 0 0 0',
                                 stiffness=k_values[0])
        self._add_intermediate_link(model, intermediate, jx, '0 0 0 0 0 0')
        self._add_revolute_joint(model, jy, intermediate, 'bending_link_1',
                                 '0 1 0', stiffness=k_values[0])
        joint_names.extend([jx, jy])

        # Intermediate bending links
        for i in range(1, self.bending_links):
            intermediate = f'bending_{i}_to_{i + 1}_x_rotation'
            jx = f'bending_{i}_to_{i + 1}_x'
            jy = f'bending_{i}_to_{i + 1}_y'
            self._add_revolute_joint(model, jx, f'bending_link_{i}', intermediate,
                                     '1 0 0', f'0 0 {self.bending_link_length / 2} 0 0 0',
                                     stiffness=k_values[i])
            self._add_intermediate_link(model, intermediate, jx, '0 0 0 0 0 0')
            self._add_revolute_joint(model, jy, intermediate, f'bending_link_{i + 1}',
                                     '0 1 0', stiffness=k_values[i])
            joint_names.extend([jx, jy])

        # Last bending link → tip
        intermediate = f'bending_{self.bending_links}_to_tip_x_rotation'
        jx = f'bending_{self.bending_links}_to_tip_x'
        jy = f'bending_{self.bending_links}_to_tip_y'
        self._add_revolute_joint(model, jx, f'bending_link_{self.bending_links}',
                                 intermediate, '1 0 0',
                                 f'0 0 {self.bending_link_length / 2} 0 0 0',
                                 stiffness=k_values[-1])
        self._add_intermediate_link(model, intermediate, jx, '0 0 0 0 0 0')
        self._add_revolute_joint(model, jy, intermediate, 'tip_link',
                                 '0 1 0', stiffness=k_values[-1])
        joint_names.extend([jx, jy])
        return joint_names

    def _add_joint_state_publisher(self, model, flexible_joint_names):
        """Attach the Gazebo joint-state publisher plugin."""
        plugin = ET.SubElement(model, 'plugin',
                               filename='gz-sim-joint-state-publisher-system',
                               name='gz::sim::systems::JointStatePublisher')
        if self.with_controller:
            # List all joints explicitly so base DOFs are included
            for jname in (['base_x_joint', 'base_y_joint',
                           'base_z_joint', 'base_rotation_joint']
                          + flexible_joint_names):
                ET.SubElement(plugin, 'joint_name').text = jname

    # Controller-only SDF helpers ────────────────────────────────────────────

    def _make_virtual_link(self, parent_el, name):
        """Create a massless intermediate link for the base mobility chain."""
        link = ET.SubElement(parent_el, 'link', name=name)
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = '0.01'
        inertia = ET.SubElement(inertial, 'inertia')
        for tag in ['ixx', 'iyy', 'izz']:
            ET.SubElement(inertia, tag).text = '1e-3'
        for tag in ['ixy', 'ixz', 'iyz']:
            ET.SubElement(inertia, tag).text = '0'

    def _make_prismatic_joint(self, parent_el, name, parent_link, child_link,
                              axis_xyz, lower, upper,
                              effort='100', velocity='1.0'):
        joint = ET.SubElement(parent_el, 'joint', name=name, type='prismatic')
        ET.SubElement(joint, 'parent').text = parent_link
        ET.SubElement(joint, 'child').text  = child_link
        ET.SubElement(joint, 'pose').text   = '0 0 0 0 0 0'
        axis = ET.SubElement(joint, 'axis')
        ET.SubElement(axis, 'xyz').text = axis_xyz
        limit = ET.SubElement(axis, 'limit')
        ET.SubElement(limit, 'lower').text    = str(lower)
        ET.SubElement(limit, 'upper').text    = str(upper)
        ET.SubElement(limit, 'effort').text   = effort
        ET.SubElement(limit, 'velocity').text = velocity
        dyn = ET.SubElement(axis, 'dynamics')
        ET.SubElement(dyn, 'damping').text  = str(self.damping)
        ET.SubElement(dyn, 'friction').text = str(self.friction)

    def _make_position_controller_plugin(self, parent_el, joint_name, topic,
                                         p='5000.0', i='0.0', d='100.0',
                                         cmd_max='1000.0', cmd_min='-1000.0'):
        plg = ET.SubElement(parent_el, 'plugin',
                            filename='gz-sim-joint-position-controller-system',
                            name='gz::sim::systems::JointPositionController')
        ET.SubElement(plg, 'joint_name').text          = joint_name
        ET.SubElement(plg, 'topic').text               = topic
        ET.SubElement(plg, 'use_velocity_commands').text = 'false'
        ET.SubElement(plg, 'p_gain').text              = p
        ET.SubElement(plg, 'i_gain').text              = i
        ET.SubElement(plg, 'd_gain').text              = d
        ET.SubElement(plg, 'i_max').text               = '1.0'
        ET.SubElement(plg, 'i_min').text               = '-1.0'
        ET.SubElement(plg, 'cmd_max').text             = cmd_max
        ET.SubElement(plg, 'cmd_min').text             = cmd_min

    def _save_sdf_file(self, sdf_root, filename):
        rough = ET.tostring(sdf_root, 'unicode')
        pretty = minidom.parseString(rough).toprettyxml(indent='  ')
        lines = [l for l in pretty.split('\n') if l.strip()]
        path = os.path.join(self.sdf_dir, filename)
        with open(path, 'w') as f:
            f.write('\n'.join(lines))
        return path

    # ─────────────────────────────────────────────────────────────────────────
    # SDF generation
    # ─────────────────────────────────────────────────────────────────────────

    def generate_sdf(self, xacro_filename):
        """Generate the Gazebo SDF model file."""
        package_name = os.path.basename(self.package_dir)
        sdf_filename = xacro_filename.replace('.xacro', '.sdf')

        sdf   = ET.Element('sdf', version='1.7')
        model = ET.SubElement(sdf, 'model', name=package_name)

        # Catheter links
        self._add_sdf_link(model, 'base_link',
                           parent_name=None,
                           pose=f'0 0 {self.L1 / 2} 0 0 0',
                           mass=self.base_mass, length=self.L1,
                           mesh_file=f'file://{os.path.abspath(os.path.join(self.meshes_dir, "base_link.stl"))}')

        parent_ref = 'base_to_bending_1_y'
        for i in range(self.bending_links):
            self._add_sdf_link(model, f'bending_link_{i + 1}',
                               parent_name=parent_ref,
                               pose=f'0 0 {self.bending_link_length / 2} 0 0 0',
                               mass=self.bending_mass_per_link,
                               length=self.bending_link_length,
                               mesh_file=f'model://{package_name}/meshes/bending_link.stl')
            parent_ref = f'bending_{i + 1}_to_{i + 2}_y'

        parent_ref = f'bending_{self.bending_links}_to_tip_x'
        self._add_sdf_link(model, 'tip_link',
                           parent_name=parent_ref,
                           pose=f'0 0 {self.L3 / 2} 0 0 0',
                           mass=self.tip_mass, length=self.L3,
                           mesh_file=f'model://{package_name}/meshes/tip_link.stl')

        if self.with_controller:
            # 4-DOF base mobility chain:
            #   world → base_x_link (X) → base_y_link (Y)
            #         → base_z_link (Z) → base_link (Z rotation)
            total_length = self.L1 + self.L2 + self.L3 + 1.0

            self._make_virtual_link(model, 'base_x_link')
            self._make_prismatic_joint(model, 'base_x_joint',
                                       'world', 'base_x_link', '1 0 0',
                                       lower=-total_length, upper=total_length)

            self._make_virtual_link(model, 'base_y_link')
            self._make_prismatic_joint(model, 'base_y_joint',
                                       'base_x_link', 'base_y_link', '0 1 0',
                                       lower=-total_length, upper=total_length)

            self._make_virtual_link(model, 'base_z_link')
            self._make_prismatic_joint(model, 'base_z_joint',
                                       'base_y_link', 'base_z_link', '0 0 1',
                                       lower=-total_length, upper=total_length)

            j_rot = ET.SubElement(model, 'joint', name='base_rotation_joint', type='revolute')
            ET.SubElement(j_rot, 'parent').text = 'base_z_link'
            ET.SubElement(j_rot, 'child').text  = 'base_link'
            ET.SubElement(j_rot, 'pose').text   = '0 0 0 0 0 0'
            axis_r = ET.SubElement(j_rot, 'axis')
            ET.SubElement(axis_r, 'xyz').text = '0 0 1'
            lim = ET.SubElement(axis_r, 'limit')
            ET.SubElement(lim, 'lower').text    = str(-math.pi)
            ET.SubElement(lim, 'upper').text    = str(math.pi)
            ET.SubElement(lim, 'effort').text   = '100'
            ET.SubElement(lim, 'velocity').text = '10'
            dyn = ET.SubElement(axis_r, 'dynamics')
            ET.SubElement(dyn, 'damping').text  = str(self.damping)
            ET.SubElement(dyn, 'friction').text = str(self.friction)
        else:
            # Passive: base_link is fixed to the world frame
            self._add_fixed_joint(model, 'world_to_base', 'world', 'base_link')

        # Flexible catheter joints
        joint_names = self._add_universal_joints(model)

        # Plugins
        self._add_joint_state_publisher(model, joint_names)
        if self.with_controller:
            self._make_position_controller_plugin(model, 'base_x_joint', '/base_x_joint/cmd_pos')
            self._make_position_controller_plugin(model, 'base_y_joint', '/base_y_joint/cmd_pos')
            self._make_position_controller_plugin(model, 'base_z_joint', '/base_z_joint/cmd_pos')
            self._make_position_controller_plugin(model, 'base_rotation_joint',
                                                  '/base_rotation_joint/cmd_pos',
                                                  p='100.0', d='10.0',
                                                  cmd_max='100.0', cmd_min='-100.0')

        self._save_sdf_file(sdf, sdf_filename)
        return os.path.join(self.sdf_dir, sdf_filename)

    # ─────────────────────────────────────────────────────────────────────────
    # World SDF
    # ─────────────────────────────────────────────────────────────────────────

    def generate_custom_world(self):
        """Generate the Gazebo world SDF."""
        if self.with_controller:
            physics_block = '''\
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
    <gravity>0 0 0</gravity>'''
        else:
            physics_block = '''\
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>'''

        # One Gazebo model block per anatomy entry; mesh paths use placeholders
        # resolved at launch time (see generate_launch_file).
        anatomy_blocks = ''
        for i, model in enumerate(self.anatomy_models):
            ax, ay, az = model['xyz']
            ar, ap, ayaw = model['rpy']
            mr, mg, mb, ma = model['color']
            mname = model['name']
            placeholder = f'__ANATOMY_MESH_PATH_{i}__'
            anatomy_blocks += f'''
    <!-- Anatomy model '{mname}': mesh path ({placeholder}) resolved at launch time -->
    <!-- Vertices are pre-scaled to metres by catheter_generator.py -->
    <model name="{mname}">
      <static>true</static>
      <pose>{ax} {ay} {az} {ar} {ap} {ayaw}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>file://{placeholder}</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <material>
            <ambient>{mr} {mg} {mb} 1</ambient>
            <diffuse>{mr} {mg} {mb} {ma}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>file://{placeholder}</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <contact>
              <collide_bitmask>0x01</collide_bitmask>
              <ode>
                <min_depth>0.001</min_depth>
                <max_vel>0.1</max_vel>
              </ode>
            </contact>
            <friction>
              <ode>
                <mu>0.1</mu>
                <mu2>0.1</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
    </model>'''

        world_content = f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="catheter_world">
{physics_block}
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
{anatomy_blocks}
  </world>
</sdf>'''

        path = os.path.join(self.worlds_dir, "custom_world.sdf")
        with open(path, 'w') as f:
            f.write(world_content)
        return path

    # ─────────────────────────────────────────────────────────────────────────
    # Launch file
    # ─────────────────────────────────────────────────────────────────────────

    def generate_launch_file(self, xacro_filename):
        """Generate the ROS2 Python launch file."""
        package_name = os.path.basename(self.package_dir)
        sdf_filename  = xacro_filename.replace('.xacro', '.sdf')

        # Bridge topics differ between modes
        if self.with_controller:
            bridge_args = f'''\
            '/model/{package_name}_model/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            '/world/catheter_world/model/{package_name}_model/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/base_x_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/base_y_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/base_z_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/base_rotation_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double','''
        else:
            bridge_args = f'''\
            '/model/{package_name}_model/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            '/world/catheter_world/model/{package_name}_model/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model','''

        # Gazebo launch: always use OpaqueFunction so that anatomy mesh paths
        # and the world SDF can be resolved at launch time.
        if self.anatomy_models:
            replace_lines = '\n'.join(
                f"        content = content.replace("
                f"'__ANATOMY_MESH_PATH_{i}__', "
                f"os.path.join(pkg, 'meshes', '{os.path.basename(m['src_stl'])}'))"
                for i, m in enumerate(self.anatomy_models)
            )
            gazebo_block = f'''
    def _launch_gazebo(context):
        """Resolve installed anatomy mesh paths and write a temporary world SDF."""
        pkg = get_package_share_directory('{package_name}')
        world_tmpl = os.path.join(pkg, 'worlds', 'custom_world.sdf')
        with open(world_tmpl, 'r') as f:
            content = f.read()
{replace_lines}
        import tempfile
        tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False)
        tmp.write(content)
        tmp.close()
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'),
                             'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={{'gz_args': f'-r {{tmp.name}}'}}.items(),
        )]

    gazebo_action = OpaqueFunction(function=_launch_gazebo)'''
        else:
            gazebo_block = f'''
    custom_world_file = os.path.join(pkg_share, 'worlds', 'custom_world.sdf')
    gazebo_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={{'gz_args': f'-r {{custom_world_file}}'}}.items(),
    )'''

        # Passive mode: a custom broadcaster script publishes world → base_link
        # on /tf (not /tf_static) at 10 Hz with current timestamps.
        # Dynamic /tf entries have current timestamps which always take
        # precedence over RSP's /tf_static entry (timestamp = 0).
        if not self.with_controller:
            initial_pose_tf_block = f'''
    # ── Initial pose: world → base_link dynamic TF ──────────────────────────
    # The URDF fixes world_to_base at origin; this broadcaster continuously
    # publishes the real initial offset on /tf with current timestamps, which
    # overrides the static entry and keeps RViz aligned with Gazebo.
    pose_broadcaster = Node(
        package='{package_name}',
        executable='catheter_pose_broadcaster.py',
        name='catheter_pose_broadcaster',
        parameters=[{{
            'x':     LaunchConfiguration('initial_x'),
            'y':     LaunchConfiguration('initial_y'),
            'z':     LaunchConfiguration('initial_z'),
            'roll':  LaunchConfiguration('initial_roll'),
            'pitch': LaunchConfiguration('initial_pitch'),
            'yaw':   LaunchConfiguration('initial_yaw'),
        }}],
        output='screen',
    )
'''
            ld_initial_pose_entry = '        pose_broadcaster,'
        else:
            initial_pose_tf_block = ''
            ld_initial_pose_entry = ''

        event_handler_imports = (
            'from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,\n'
            '                             OpaqueFunction)'
        )

        launch_content = f'''import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
{event_handler_imports}
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg_share      = get_package_share_directory('{package_name}')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # ── Initial catheter pose ────────────────────────────────────────────────
    # Override at launch time:
    #   ros2 launch {package_name} {package_name}_launch.py initial_z:=0.3
    pose_args = [
        DeclareLaunchArgument('initial_x',     default_value='0.0',
                              description='Catheter initial X position (m)'),
        DeclareLaunchArgument('initial_y',     default_value='0.0',
                              description='Catheter initial Y position (m)'),
        DeclareLaunchArgument('initial_z',     default_value='0.0',
                              description='Catheter initial Z position (m)'),
        DeclareLaunchArgument('initial_roll',  default_value='0.0',
                              description='Catheter initial roll  (rad)'),
        DeclareLaunchArgument('initial_pitch', default_value='0.0',
                              description='Catheter initial pitch (rad)'),
        DeclareLaunchArgument('initial_yaw',   default_value='0.0',
                              description='Catheter initial yaw   (rad)'),
    ]

    # ── robot_state_publisher ────────────────────────────────────────────────
    # Process the xacro once at launch-description time.  The initial pose is
    # NOT baked into the URDF here; it is applied by catheter_pose_broadcaster
    # (passive mode) or by the Gazebo spawn pose (controller mode).
    config = xacro.process_file(
        os.path.join(pkg_share, 'urdf', '{xacro_filename}')
    )
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{{'robot_description': config.toxml()}}],
    )
{initial_pose_tf_block}
    rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{{
            'name':  '{package_name}_model',
            'file':  os.path.join(pkg_share, 'sdf', '{sdf_filename}'),
            'x':     LaunchConfiguration('initial_x'),
            'y':     LaunchConfiguration('initial_y'),
            'z':     LaunchConfiguration('initial_z'),
            'roll':  LaunchConfiguration('initial_roll'),
            'pitch': LaunchConfiguration('initial_pitch'),
            'yaw':   LaunchConfiguration('initial_yaw'),
        }}],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
{bridge_args}
        ],
        remappings=[
            ('/world/catheter_world/model/{package_name}_model/joint_state', '/joint_states'),
        ],
        output='screen',
    )
{gazebo_block}

    return LaunchDescription([
        *pose_args,
        gazebo_action,
        spawn,
        bridge,
        rsp_node,
{ld_initial_pose_entry}
        rviz,
    ])
'''
        path = os.path.join(self.launch_dir, f"{package_name}_launch.py")
        with open(path, 'w') as f:
            f.write(launch_content)
        return path

    # ─────────────────────────────────────────────────────────────────────────
    # RViz config
    # ─────────────────────────────────────────────────────────────────────────

    def generate_rviz_config(self):
        """Write a minimal RViz2 config that sets the fixed frame to 'world'."""
        content = '''\
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
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
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 1.5
      Name: Current View
      Pitch: 0.4
      Yaw: 0.8
Window Geometry:
  Hide Left Dock: false
  Hide Right Dock: true
'''
        path = os.path.join(self.config_dir, 'rviz_config.rviz')
        with open(path, 'w') as f:
            f.write(content)
        return path

    # ─────────────────────────────────────────────────────────────────────────
    # Pose broadcaster (passive mode only)
    # ─────────────────────────────────────────────────────────────────────────

    def generate_pose_broadcaster(self):
        """Generate a Python ROS2 node that broadcasts world→base_link on /tf.

        Dynamic /tf entries (with current timestamps) always take precedence
        over robot_state_publisher's /tf_static entry for the world_to_base
        fixed joint (which has epoch timestamp 0).  This guarantees that the
        catheter appears at the launch-time initial position in RViz without
        any startup race condition.
        """
        content = '''\
#!/usr/bin/env python3
"""Broadcasts world → base_link on /tf at 10 Hz.

Reads the initial catheter pose from ROS2 parameters (x, y, z, roll, pitch,
yaw) and continuously republishes it as a dynamic TF.  Because dynamic /tf
entries carry current timestamps they always override robot_state_publisher's
/tf_static entry for the world_to_base joint (epoch timestamp 0), so RViz
always shows the catheter at the correct initial position.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class CatheterPoseBroadcaster(Node):
    def __init__(self):
        super().__init__('catheter_pose_broadcaster')

        self.declare_parameter('x',     0.0)
        self.declare_parameter('y',     0.0)
        self.declare_parameter('z',     0.0)
        self.declare_parameter('roll',  0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw',   0.0)

        x     = self.get_parameter('x').value
        y     = self.get_parameter('y').value
        z     = self.get_parameter('z').value
        roll  = self.get_parameter('roll').value
        pitch = self.get_parameter('pitch').value
        yaw   = self.get_parameter('yaw').value

        self._tf = TransformStamped()
        self._tf.header.frame_id = 'world'
        self._tf.child_frame_id  = 'base_link'
        self._tf.transform.translation.x = x
        self._tf.transform.translation.y = y
        self._tf.transform.translation.z = z

        # Euler ZYX → quaternion
        cr, sr = math.cos(roll  / 2), math.sin(roll  / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw   / 2), math.sin(yaw   / 2)
        self._tf.transform.rotation.w = cr * cp * cy + sr * sp * sy
        self._tf.transform.rotation.x = sr * cp * cy - cr * sp * sy
        self._tf.transform.rotation.y = cr * sp * cy + sr * cp * sy
        self._tf.transform.rotation.z = cr * cp * sy - sr * sp * cy

        self._br = TransformBroadcaster(self)
        self.create_timer(0.1, self._publish)

    def _publish(self):
        self._tf.header.stamp = self.get_clock().now().to_msg()
        self._br.sendTransform(self._tf)


def main(args=None):
    rclpy.init(args=args)
    node = CatheterPoseBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
        path = os.path.join(self.scripts_dir, 'catheter_pose_broadcaster.py')
        with open(path, 'w') as f:
            f.write(content)
        os.chmod(path, 0o755)
        return path

    # ─────────────────────────────────────────────────────────────────────────
    # URDF / Xacro
    # ─────────────────────────────────────────────────────────────────────────

    def add_xacro_properties(self, root):
        props = {
            'catheter_links':       str(self.N),
            'catheter_diameter':    str(self.D),
            'catheter_radius':      str(self.radius),
            'tip_length':           str(self.L3),
            'bending_length':       str(self.L2),
            'base_length':          str(self.L1),
            'spring_constant':      str(self.K),
            'total_mass':           str(self.M),
            'bending_links':        str(self.bending_links),
            'bending_link_length':  str(self.bending_link_length),
            'tip_mass':             str(self.tip_mass),
            'base_mass':            str(self.base_mass),
            'bending_mass_per_link': str(self.bending_mass_per_link),
        }
        for name, value in props.items():
            ET.SubElement(root, 'xacro:property', name=name, value=value)
        mat = ET.SubElement(root, 'material', name='catheter_material')
        ET.SubElement(mat, 'color', rgba='0.8 0.8 0.8 1.0')

    def add_catheter_link_macro(self, root):
        macro = ET.SubElement(root, 'xacro:macro', name='catheter_link')
        macro.set('params', 'name mass length radius xyz_origin mesh_file')
        link = ET.SubElement(macro, 'link', name='${name}')

        visual = ET.SubElement(link, 'visual')
        ET.SubElement(visual, 'origin', xyz='${xyz_origin}', rpy='0 0 0')
        ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh', filename='${mesh_file}')
        ET.SubElement(visual, 'material', name='catheter_material')

        collision = ET.SubElement(link, 'collision')
        ET.SubElement(collision, 'origin', xyz='${xyz_origin}', rpy='0 0 0')
        ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh', filename='${mesh_file}')

        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'origin', xyz='${xyz_origin}', rpy='0 0 0')
        ET.SubElement(inertial, 'mass', value='${mass}')
        ET.SubElement(inertial, 'inertia',
                      ixx='${(1/12) * mass * (3 * radius * radius + length * length)}',
                      iyy='${(1/12) * mass * (3 * radius * radius + length * length)}',
                      izz='${0.5 * mass * radius * radius}',
                      ixy='0', ixz='0', iyz='0')

    def add_universal_joint_macro(self, root):
        macro = ET.SubElement(root, 'xacro:macro', name='universal_joint')
        macro.set('params', 'name parent child xyz_origin spring_k')

        il = ET.SubElement(macro, 'link', name='${name}_x_rotation')
        inertial = ET.SubElement(il, 'inertial')
        ET.SubElement(inertial, 'mass', value='0.01')
        ET.SubElement(inertial, 'inertia',
                      ixx='1e-3', iyy='1e-3', izz='1e-3',
                      ixy='0', ixz='0', iyz='0')

        for jname, parent_link, child_link, axis, origin in [
            ('${name}_x', '${parent}',           '${name}_x_rotation', '1 0 0', '${xyz_origin}'),
            ('${name}_y', '${name}_x_rotation',  '${child}',           '0 1 0', '0 0 0'),
        ]:
            j = ET.SubElement(macro, 'joint', name=jname, type='revolute')
            ET.SubElement(j, 'parent', link=parent_link)
            ET.SubElement(j, 'child',  link=child_link)
            ET.SubElement(j, 'origin', xyz=origin, rpy='0 0 0')
            ET.SubElement(j, 'axis',   xyz=axis)
            ET.SubElement(j, 'limit',
                          lower=str(-math.pi / 2), upper=str(math.pi / 2),
                          effort='100', velocity='10')
            ET.SubElement(j, 'dynamics', damping=str(self.damping), friction=str(self.friction))

        for suffix in ('x', 'y'):
            gz = ET.SubElement(macro, 'gazebo')
            plg = ET.SubElement(gz, 'plugin',
                                name=f'${{name}}_{suffix}_spring',
                                filename='libgazebo_ros_joint_spring.so')
            ET.SubElement(plg, 'joint_name').text        = f'${{name}}_{suffix}'
            ET.SubElement(plg, 'spring_constant').text   = '${spring_k}'
            ET.SubElement(plg, 'reference_position').text = '0.0'

    def generate_xacro(self):
        """Generate the complete Xacro / URDF tree."""
        robot = ET.Element('robot', name='flexible_catheter')
        robot.set('xmlns:xacro', 'http://www.ros.org/wiki/xacro')

        # Note: initial catheter pose is handled at launch time via a
        # static_transform_publisher (passive mode) or Gazebo spawn position
        # (controller mode), not via xacro args.

        self.generate_cylinder_stl(self.radius, self.L3,                   'tip_link.stl')
        self.generate_cylinder_stl(self.radius, self.L1,                   'base_link.stl')
        self.generate_cylinder_stl(self.radius, self.bending_link_length,  'bending_link.stl')

        self.add_xacro_properties(robot)
        self.add_catheter_link_macro(robot)
        self.add_universal_joint_macro(robot)

        package_name = os.path.basename(self.package_dir)

        # Catheter links
        ET.SubElement(robot, 'xacro:catheter_link',
                      name='base_link', mass='${base_mass}', length='${base_length}',
                      radius='${catheter_radius}', xyz_origin=f'0 0 {self.L1 / 2}',
                      mesh_file=f'package://{package_name}/meshes/base_link.stl')
        for i in range(self.bending_links):
            ET.SubElement(robot, 'xacro:catheter_link',
                          name=f'bending_link_{i + 1}', mass='${bending_mass_per_link}',
                          length='${bending_link_length}', radius='${catheter_radius}',
                          xyz_origin=f'0 0 {self.bending_link_length / 2}',
                          mesh_file=f'package://{package_name}/meshes/bending_link.stl')
        ET.SubElement(robot, 'xacro:catheter_link',
                      name='tip_link', mass='${tip_mass}', length='${tip_length}',
                      radius='${catheter_radius}', xyz_origin=f'0 0 {self.L3 / 2}',
                      mesh_file=f'package://{package_name}/meshes/tip_link.stl')

        # World link + base attachment (differs between modes)
        ET.SubElement(robot, 'link', name='world')

        if self.with_controller:
            total_length = self.L1 + self.L2 + self.L3

            def _vlink(name):
                lk = ET.SubElement(robot, 'link', name=name)
                inertial = ET.SubElement(lk, 'inertial')
                ET.SubElement(inertial, 'mass', value='0.01')
                ET.SubElement(inertial, 'inertia',
                              ixx='1e-3', iyy='1e-3', izz='1e-3',
                              ixy='0', ixz='0', iyz='0')

            def _prismatic(jname, parent, child, axis):
                j = ET.SubElement(robot, 'joint', name=jname, type='prismatic')
                ET.SubElement(j, 'parent', link=parent)
                ET.SubElement(j, 'child',  link=child)
                ET.SubElement(j, 'origin', xyz='0 0 0', rpy='0 0 0')
                ET.SubElement(j, 'axis',   xyz=axis)
                ET.SubElement(j, 'limit',
                              lower=str(-total_length), upper=str(total_length),
                              effort='100', velocity='1.0')
                ET.SubElement(j, 'dynamics',
                              damping=str(self.damping), friction=str(self.friction))

            _vlink('base_x_link')
            _prismatic('base_x_joint', 'world',       'base_x_link', '1 0 0')
            _vlink('base_y_link')
            _prismatic('base_y_joint', 'base_x_link', 'base_y_link', '0 1 0')
            _vlink('base_z_link')
            _prismatic('base_z_joint', 'base_y_link', 'base_z_link', '0 0 1')

            j_rot = ET.SubElement(robot, 'joint', name='base_rotation_joint', type='revolute')
            ET.SubElement(j_rot, 'parent', link='base_z_link')
            ET.SubElement(j_rot, 'child',  link='base_link')
            ET.SubElement(j_rot, 'origin', xyz='0 0 0', rpy='0 0 0')
            ET.SubElement(j_rot, 'axis',   xyz='0 0 1')
            ET.SubElement(j_rot, 'limit',
                          lower=str(-math.pi), upper=str(math.pi),
                          effort='100', velocity='10')
            ET.SubElement(j_rot, 'dynamics',
                          damping=str(self.damping), friction=str(self.friction))
        else:
            # Passive: base_link is anchored to the world frame at the origin.
            # The actual initial pose offset is applied by a static_transform_publisher
            # launched alongside robot_state_publisher (see generate_launch_file).
            j_world = ET.SubElement(robot, 'joint', name='world_to_base', type='fixed')
            ET.SubElement(j_world, 'parent', link='world')
            ET.SubElement(j_world, 'child',  link='base_link')
            ET.SubElement(j_world, 'origin', xyz='0 0 0', rpy='0 0 0')

        # Flexible catheter joints
        if self.bending_links > 0:
            ET.SubElement(robot, 'xacro:universal_joint',
                          name='base_to_bending_1', parent='base_link', child='bending_link_1',
                          xyz_origin=f'0 0 {self.L1}', spring_k='${spring_constant}')
            for i in range(1, self.bending_links):
                ET.SubElement(robot, 'xacro:universal_joint',
                              name=f'bending_{i}_to_{i + 1}',
                              parent=f'bending_link_{i}', child=f'bending_link_{i + 1}',
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

        # Anatomy links — each fixed to the world frame, visible in RViz.
        for model in self.anatomy_models:
            mname = model['name']
            stl_basename = os.path.basename(model['src_stl'])
            mr, mg, mb, ma = model['color']
            ax, ay, az = model['xyz']
            ar, ap, ayaw = model['rpy']

            al = ET.SubElement(robot, 'link', name=f'anatomy_link_{mname}')
            vis = ET.SubElement(al, 'visual')
            ET.SubElement(vis, 'origin', xyz='0 0 0', rpy='0 0 0')
            mesh_el = ET.SubElement(ET.SubElement(vis, 'geometry'), 'mesh')
            mesh_el.set('filename', f'package://{package_name}/meshes/{stl_basename}')
            mesh_el.set('scale', '1 1 1')
            amat = ET.SubElement(vis, 'material', name=f'anatomy_material_{mname}')
            ET.SubElement(amat, 'color', rgba=f'{mr} {mg} {mb} {ma}')

            ja = ET.SubElement(robot, 'joint', name=f'world_to_anatomy_{mname}', type='fixed')
            ET.SubElement(ja, 'parent', link='world')
            ET.SubElement(ja, 'child',  link=f'anatomy_link_{mname}')
            ET.SubElement(ja, 'origin',
                          xyz=f'{ax} {ay} {az}',
                          rpy=f'{ar} {ap} {ayaw}')

        return robot

    # ─────────────────────────────────────────────────────────────────────────
    # Teleop node (controller mode only)
    # ─────────────────────────────────────────────────────────────────────────

    def generate_teleop_node(self):
        """Write the keyboard teleoperation script."""
        total_length = self.L1 + self.L2 + self.L3 + 1.0

        teleop_content = f'''#!/usr/bin/env python3
"""
Catheter Keyboard Teleop Node  (auto-generated by catheter_generator.py)
========================================================================
Drives the catheter base by publishing Float64 position commands to
Gazebo JointPositionController topics.

  /base_x_joint/cmd_pos        → X translation
  /base_y_joint/cmd_pos        → Y translation
  /base_z_joint/cmd_pos        → Z insertion/retraction
  /base_rotation_joint/cmd_pos → Z-axis rotation

Controls:
  W / S          →  Insert  / Retract  (Z)
  A / D          →  CCW     / CW       (Z rotation)
  Arrow Up/Down  →  Y+      / Y-
  Arrow Left/Right → X-    / X+
  Q              →  Quit
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

Z_STEP          = 0.002
XY_STEP         = 0.002
ROT_STEP        = math.radians(2)

XY_MIN, XY_MAX  = -{total_length:.6f}, {total_length:.6f}
Z_MIN,  Z_MAX   = -{total_length:.6f}, {total_length:.6f}
ROT_MIN, ROT_MAX = -math.pi, math.pi

PUBLISH_RATE_HZ = 50

KEY_INSERT  = \'w\'
KEY_RETRACT = \'s\'
KEY_CCW     = \'a\'
KEY_CW      = \'d\'
KEY_QUIT    = \'q\'
KEY_UP      = \'\\x1b[A\'
KEY_DOWN    = \'\\x1b[B\'
KEY_RIGHT   = \'\\x1b[C\'
KEY_LEFT    = \'\\x1b[D\'

BANNER = """
╔══════════════════════════════════════════════════╗
║         Catheter Keyboard Teleop                 ║
╠══════════════════════════════════════════════════╣
║  W / S        →  Insert  / Retract  (Z trans)   ║
║  A / D        →  CCW     / CW       (Z rot)     ║
║  ↑ / ↓        →  Y+      / Y-       (Y trans)   ║
║  → / ←        →  X+      / X-       (X trans)   ║
║  Q            →  Quit                           ║
╚══════════════════════════════════════════════════╝
"""


def get_key(timeout=0.05):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return \'\'
        import os as _os
        key = _os.read(fd, 1).decode(\'utf-8\', errors=\'ignore\')
        if key == \'\\x1b\':
            for _ in range(2):
                r, _, _ = select.select([sys.stdin], [], [], 0.02)
                if r:
                    key += _os.read(fd, 1).decode(\'utf-8\', errors=\'ignore\')
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return key


class CatheterTeleopNode(Node):
    def __init__(self):
        super().__init__(\'catheter_keyboard_teleop\')
        self.x_pos = self.y_pos = self.z_pos = self.z_rot = 0.0
        self._lock = threading.Lock()
        self.pub_x   = self.create_publisher(Float64, \'/base_x_joint/cmd_pos\', 10)
        self.pub_y   = self.create_publisher(Float64, \'/base_y_joint/cmd_pos\', 10)
        self.pub_z   = self.create_publisher(Float64, \'/base_z_joint/cmd_pos\', 10)
        self.pub_rot = self.create_publisher(Float64, \'/base_rotation_joint/cmd_pos\', 10)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish)

    def _publish(self):
        with self._lock:
            vals = self.x_pos, self.y_pos, self.z_pos, self.z_rot
        for pub, val in zip([self.pub_x, self.pub_y, self.pub_z, self.pub_rot], vals):
            msg = Float64(); msg.data = val; pub.publish(msg)

    def apply_key(self, key):
        if key.lower() == KEY_QUIT:
            return False
        with self._lock:
            k = key.lower() if len(key) == 1 else key
            if   k == KEY_INSERT:  self.z_pos = min(self.z_pos + Z_STEP,   Z_MAX)
            elif k == KEY_RETRACT: self.z_pos = max(self.z_pos - Z_STEP,   Z_MIN)
            elif k == KEY_CCW:     self.z_rot = min(self.z_rot + ROT_STEP, ROT_MAX)
            elif k == KEY_CW:      self.z_rot = max(self.z_rot - ROT_STEP, ROT_MIN)
            elif k == KEY_UP:      self.y_pos = min(self.y_pos + XY_STEP,  XY_MAX)
            elif k == KEY_DOWN:    self.y_pos = max(self.y_pos - XY_STEP,  XY_MIN)
            elif k == KEY_RIGHT:   self.x_pos = min(self.x_pos + XY_STEP,  XY_MAX)
            elif k == KEY_LEFT:    self.x_pos = max(self.x_pos - XY_STEP,  XY_MIN)
        return True

    def status_line(self):
        with self._lock:
            return (
                f"  X: {{self.x_pos*1000:+7.2f}} mm  "
                f"Y: {{self.y_pos*1000:+7.2f}} mm  "
                f"Z: {{self.z_pos*1000:+7.2f}} mm  |  "
                f"Rot: {{math.degrees(self.z_rot):+7.2f}}°   \\r"
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
            if key and not node.apply_key(key):
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
        path = os.path.join(self.scripts_dir, "catheter_keyboard_teleop.py")
        with open(path, 'w') as f:
            f.write(teleop_content)
        import stat
        st = os.stat(path)
        os.chmod(path, st.st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)
        return path

    # ─────────────────────────────────────────────────────────────────────────
    # Entry point
    # ─────────────────────────────────────────────────────────────────────────

    def save(self, filename=None):
        """Generate and write all package files."""
        robot = self.generate_xacro()
        package_name = os.path.basename(self.package_dir)
        if filename is None:
            filename = f"{package_name}.xacro"

        rough  = ET.tostring(robot, 'unicode')
        pretty = minidom.parseString(rough).toprettyxml(indent='  ')
        lines  = [l for l in pretty.split('\n') if l.strip()]
        xacro_path = os.path.join(self.urdf_dir, filename)
        with open(xacro_path, 'w') as f:
            f.write('\n'.join(lines))

        pkg_xml_path = self.generate_package_xml()
        cmake_path   = self.generate_cmakelists_txt()

        for model in self.anatomy_models:
            self._copy_stl_scaled(
                model['src_stl'],
                os.path.join(self.meshes_dir, os.path.basename(model['src_stl'])),
                model['scale'],
                flip_xy=model.get('flip_xy', False),
            )

        sdf_path       = self.generate_sdf(filename)
        launch_path    = self.generate_launch_file(filename)
        world_path     = self.generate_custom_world()
        rviz_path      = self.generate_rviz_config()
        broadcaster_path = self.generate_pose_broadcaster()

        teleop_path = None
        if self.with_controller:
            teleop_path = self.generate_teleop_node()

        # Summary
        rel = lambda p: os.path.relpath(p, self.package_dir)
        mode = "controller (4-DOF base + teleop)" if self.with_controller else "passive"
        print(f"ROS2 package created: {self.package_dir}/  [{mode}]")
        print(f"  {rel(pkg_xml_path)},  {rel(cmake_path)}")
        print(f"  {rel(xacro_path)}")
        print(f"  {rel(sdf_path)}")
        print(f"  {rel(world_path)}")
        print(f"  {rel(launch_path)}")
        print(f"  {rel(rviz_path)}")
        if teleop_path:
            print(f"  {rel(teleop_path)}")
        anatomy_mesh_names = [os.path.basename(m['src_stl']) for m in self.anatomy_models]
        extras = (', ' + ', '.join(anatomy_mesh_names)) if anatomy_mesh_names else ''
        print(f"  meshes/  (tip_link, bending_link, base_link{extras})")


def main():
    parser = argparse.ArgumentParser(
        description="Generate a ROS2 catheter simulation package",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # Catheter geometry
    parser.add_argument("--N",   type=int,   default=5,     help="Total number of links (min 3)")
    parser.add_argument("--D",   type=float, default=0.002, help="Outer diameter (m)")
    parser.add_argument("--L1",  type=float, default=0.05,  help="Base link length (m)")
    parser.add_argument("--L2",  type=float, default=0.1,   help="Bending section total length (m)")
    parser.add_argument("--L3",  type=float, default=0.01,  help="Tip link length (m)")
    parser.add_argument("--K",   type=float, default=0.1,   help="Joint spring stiffness (Nm/rad)")
    parser.add_argument("--Kd",  type=float, default=0.1,   help="Joint damping coefficient")
    parser.add_argument("--Kf",  type=float, default=0.01,  help="Joint friction coefficient")
    parser.add_argument("--M",   type=float, default=0.01,  help="Total catheter mass (kg)")
    # Package
    parser.add_argument("--output",     type=str, default="catheter_package",
                        help="Output ROS2 package directory")
    parser.add_argument("--xacro-name", type=str, default=None,
                        help="Xacro filename (default: <package_name>.xacro)")
    # Mode
    parser.add_argument("--controller", action="store_true",
                        help="Generate 4-DOF motorized base with keyboard teleoperation")
    # Single anatomy STL (legacy / simple use-case)
    parser.add_argument("--anatomy-stl",   type=str,   default=None,
                        help="Path to a single anatomy STL file (visible in both RViz and Gazebo)")
    parser.add_argument("--anatomy-x",     type=float, default=0.0,   help="Anatomy X position (m)")
    parser.add_argument("--anatomy-y",     type=float, default=0.0,   help="Anatomy Y position (m)")
    parser.add_argument("--anatomy-z",     type=float, default=0.0,   help="Anatomy Z position (m)")
    parser.add_argument("--anatomy-roll",  type=float, default=0.0,   help="Anatomy roll  (rad)")
    parser.add_argument("--anatomy-pitch", type=float, default=0.0,   help="Anatomy pitch (rad)")
    parser.add_argument("--anatomy-yaw",   type=float, default=0.0,   help="Anatomy yaw   (rad)")
    parser.add_argument("--anatomy-scale", type=float, default=0.001,
                        help="Mesh scale factor for --anatomy-stl (default 0.001 converts mm→m)")
    # 3D Slicer MRML scene (multiple anatomy models with transforms)
    parser.add_argument("--slicer-scene", type=str, default=None,
                        help="Path to a 3D Slicer MRML scene file (.mrml). "
                             "All vtkMRMLModelNode entries are imported with their "
                             "linear transforms applied. "
                             "LPS-stored meshes (DICOM default) have X and Y negated "
                             "automatically to match the RAS/ROS frame used by SlicerROS2.")
    parser.add_argument("--slicer-scale", type=float, default=0.001,
                        help="Mesh scale factor for --slicer-scene models "
                             "(default 0.001 converts mm→m)")

    args = parser.parse_args()

    try:
        gen = CatheterGenerator(
            args.N, args.D, args.L1, args.L2, args.L3,
            args.K, args.Kd, args.Kf, args.M,
            package_dir=args.output,
            with_controller=args.controller,
            anatomy_stl=args.anatomy_stl,
            anatomy_xyz=(args.anatomy_x, args.anatomy_y, args.anatomy_z),
            anatomy_rpy=(args.anatomy_roll, args.anatomy_pitch, args.anatomy_yaw),
            anatomy_scale=args.anatomy_scale,
            slicer_scene=args.slicer_scene,
            slicer_scale=args.slicer_scale,
        )
        gen.save(args.xacro_name)
    except ValueError as e:
        print(f"Error: {e}")
        return 1
    return 0


if __name__ == "__main__":
    exit(main())
