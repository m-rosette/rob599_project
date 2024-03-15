#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import sys
import os
import time

from gripper_msgs.action import RecordData
from dynamixel_control_msgs.srv import DualSetOperatingMode


class GripperControl(Node):
    def __init__(self):
        # Initialize the superclass
        super().__init__('gripper_control')

        self.get_logger().info("Gripper Control node started")

        # Create record action client
        self.record_client = ActionClient(self, RecordData, 'record_server')

        # Wait until the server is ready to accept an action request
        self.record_client.wait_for_server()

        # # Create service clients
        # self.dynamixel_client = self.create_client(DualSetOperatingMode, 'set_operating_mode')
        # # self.linear_actuator_client = self.create_client(PUT STUFF HERE)


def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # get user input filename
    try:
        filename = str(sys.argv[1])
    except:
        filename = "None"

    # Make a node class
    gripper_control = GripperControl()    

    # Log the status of the user input
    gripper_control.get_logger().info(f"Saving data to filename: {filename}")

    # Send the filename to the record action (starts recording)
    goal = RecordData.Goal()
    goal.filename = filename
    gripper_control.record_client.send_goal_async(goal)

    # # Move linear actuator forward
    # desired_position = 1000
    # gripper_control.linear_actuator_client.send_goal_async(desired_position)

    # Close gripper (current mode)
    dynamixel_command = "ros2 run dynamixel_control dual_motor_client 0 65"
    os.system(dynamixel_command)
    # request = DualSetOperatingMode().Request()
    # request.id1 = 1
    # request.id2 = 2
    # request.operating_mode = 0
    # request.operation_target1 = 65
    # gripper_control.dynamixel_client.call_async(request)

    # # Move linear actuator forward
    # desired_position = 200
    # gripper_control.linear_actuator_client.send_goal_async(desired_position)    

    # ========================= Temporary ==========================
    time.sleep(2.5)

    # Close gripper (position mode)
    dynamixel_command = "ros2 run dynamixel_control dual_motor_client 3 50"
    os.system(dynamixel_command)


    # Shutdown everything cleanly
    rclpy.shutdown()

    if __name__ == '__main__':
        main()