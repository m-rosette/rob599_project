#  launch file for the linear actuator, dynamixels,and tactile sensors 
#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rob599_project',
            executable='record',
            name='record',
    ),
        Node(
            package='rob599_project',
            executable='arduino_control',
            name='arduino_control',
    ),







])
