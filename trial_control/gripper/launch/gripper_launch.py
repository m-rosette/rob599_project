#  launch file for the linear actuator, dynamixels, and tactile sensors 
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Start up the Papillarray Tactile Sensors
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([
				os.path.join(
					get_package_share_directory('papillarray_ros2_v2'),
					'launch/papillarray.launch.py'
				)
			]),
		),

        # Start the dual dynamixel motor interface
        Node(
            package='dynamixel_control',
            executable='dual_motor_interface',
            name='dual_motor_interface',
        ),

        # Start the recording node
        Node(
            package='gripper',
            executable='record',
            name='record',
        ),

        # # Start the linear actuator control
        # Node(
        #     package='gripper',
        #     executable='arduino_control',
        #     name='arduino_control',
        # ),

        # # Start the dual dynamixel motor client
        # Node(
        #     package='dynamixel_control',
        #     executable='dual_motor_client',
        #     name='dual_motor_client',
        # ),

])
