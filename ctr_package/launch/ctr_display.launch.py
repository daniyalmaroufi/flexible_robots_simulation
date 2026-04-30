import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
  pkg_share = get_package_share_directory('ctr_package')
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
    package='ctr_package',
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
      parameters=[{'robot_description': robot_description}],
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
