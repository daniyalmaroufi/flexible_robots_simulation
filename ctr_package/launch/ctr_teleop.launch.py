import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  pkg_share = get_package_share_directory('ctr_package')
  xacro_file = os.path.join(pkg_share, 'urdf', 'ctr.urdf.xacro')
  rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

  robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_file])
  initial_base_x = LaunchConfiguration('initial_base_x')
  initial_base_y = LaunchConfiguration('initial_base_y')
  initial_base_z = LaunchConfiguration('initial_base_z')

  key_listener_kwargs = {
    'package': 'ctr_package',
    'executable': 'ctr_key_listener.py',
    'output': 'screen',
  }
  if shutil.which('xterm'):
    key_listener_kwargs['prefix'] = 'xterm -e'

  return LaunchDescription([
    DeclareLaunchArgument('initial_base_x', default_value='0.0'),
    DeclareLaunchArgument('initial_base_y', default_value='0.0'),
    DeclareLaunchArgument('initial_base_z', default_value='0.0'),
    Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      output='screen',
      parameters=[{'robot_description': robot_description}],
    ),
    Node(**key_listener_kwargs),
    Node(
      package='ctr_package',
      executable='ctr_keyboard_teleop.py',
      output='screen',
      parameters=[{
        'initial_base_x': initial_base_x,
        'initial_base_y': initial_base_y,
        'initial_base_z': initial_base_z,
      }],
    ),
    Node(
      package='rviz2',
      executable='rviz2',
      output='screen',
      arguments=['-d', rviz_config],
    ),
  ])
