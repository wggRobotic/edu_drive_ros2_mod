import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
   
  package_path = FindPackageShare('edu_drive_ros2_mod')
  parameter_file = PathJoinSubstitution([
    package_path,
    'parameter',
    'n10_edu_drive.yaml'
  ])

  n10_edu_drive = Node(
    package='edu_drive_ros2_mod',
    executable='edu_drive_ros2_node', 
    name='edu_drive_ros2_node',
    remappings=[
      ('enable', 'wheels/enable'),
      ('enabled', 'wheels/enabled'),
      ('rpm', 'wheels/rpm/feedback'),
      ('vel/rpm', 'wheels/rpm/cmd'),

      ('joy', '_unused_/joy'),
      ('vel/teleop', '_unused_/vel/teleop')
    ],
    parameters=[parameter_file],
    namespace=os.environ.get('EDU_ROBOT_NAMESPACE', "n10"),
    output='screen'
  )  
    
  return LaunchDescription([
    n10_edu_drive
  ])
