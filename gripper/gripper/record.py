#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64
import os
import numpy as np
from sensor_interfaces.msg import SensorState
from time import sleep
import csv
from collections import OrderedDict
from gripper_msgs.action import Record
from rclpy.action import ActionServer
from copy import deepcopy as copy
import threading
import time 


class Record(Node):
    def __init__(self):
        super().__init__('record')
        self.storage_directory = '/home/marcus/classes/rob599/project_ws/src/rob599_project/gripper/resource/'

        self.mutex = threading.Lock()
        self.r = self.create_rate(20)
        #  time component 
        self.start_time = time.time()


        # Initialize dictionaries to store data from subscribers
        self.initialize_tactile_dict()

        # Subscribe to tactile sensor feedback
        self.tactile_0_sub = self.create_subscription(SensorState, 'hub_0/sensor_0', self.tactile_0_callback, 10)
        self.tactile_1_sub = self.create_subscription(SensorState, 'hub_0/sensor_1', self.tactile_1_callback, 10)

        # Create action server
        self.record_server = ActionServer(self, Record, "record_server", self.action_callback)
        # self.record_server.start()
        self.get_logger().info("Everything up!")

        # self.storage_directory = '/home/marcus/classes/rob599/project_ws/src/rob599_project/gripper/resource/'

    def action_callback(self, goal_handle):
        self.file_name = goal_handle.file_name
        self.get_logger().info("Recording starting. Saving to %s.csv", self.file_name)

        # Combine all of the data into one dictionary
        self.mutex.acquire()
        combined_dict = OrderedDict(copy(self.tactile_0).items() + copy(self.tactile_1).items())
        # combined_dict = OrderedDict(copy(self.position).items() + copy(self.tactile_0).items() + copy(self.tactile_1).items())
        self.mutex.release()

        with open(self.storage_directory + str(self.file_name) + '.csv', 'w') as csvfile:
            # Write the header
            w = csv.DictWriter(csvfile, combined_dict)
            w.writeheader()

            # while rclpy.ok():
            # kp changes made 
            current_time = time.time()
            elapsed_time = current_time-self.start_time
            if elapsed_time > 0:
                # Combine all of the data into one dictionary
                self.mutex.acquire()
                combined_dict = OrderedDict(copy(self.tactile_0).items() + copy(self.tactile_1).items())
                # combined_dict = OrderedDict(copy(self.position).items() + copy(self.tactile_0).items() + copy(self.tactile_1).items())
                self.mutex.release()
                w.writerow(combined_dict)
                self.r.sleep()
            if elapsed_time == 10:
                print("its been 10 seconds")

            self.record_server.set_preempted()
            self.get_logger().info("Recording stopped.")

    def tactile_0_callback(self, tac_msg):
        # Saves the subscribed tactile 0 data to variable
        self.mutex.acquire()
        self.tactile_0 = OrderedDict()
        for i in range(8):
            self.tactile_0['0_dX_'+str(i)] = tac_msg.pillars[i].dX
            self.tactile_0['0_dY_'+str(i)] = tac_msg.pillars[i].dY
            self.tactile_0['0_dZ_'+str(i)] = tac_msg.pillars[i].dZ
            self.tactile_0['0_fX_'+str(i)] = tac_msg.pillars[i].fX
            self.tactile_0['0_fY_'+str(i)] = tac_msg.pillars[i].fY
            self.tactile_0['0_fZ_'+str(i)] = tac_msg.pillars[i].fZ
            self.tactile_0['0_incontact_'+str(i)] = tac_msg.pillars[i].in_contact
            self.tactile_0['0_slipstate_'+str(i)] = tac_msg.pillars[i].slip_state

        self.tactile_0['0_friction_est'] = tac_msg.friction_est
        self.tactile_0['0_target_grip_force'] = tac_msg.target_grip_force
        self.tactile_0['0_is_sd_active'] = tac_msg.is_sd_active
        self.tactile_0['0_is_ref_loaded'] = tac_msg.is_ref_loaded
        self.tactile_0['0_is_contact'] = tac_msg.is_contact
        self.mutex.release()

    def tactile_1_callback(self, tac_msg):
        # Saves the subscribed tactile 1 data to variable
        self.mutex.acquire()
        self.tactile_1 = OrderedDict()
        for i in range(8):
            self.tactile_1['1_dX_'+str(i)] = tac_msg.pillars[i].dX
            self.tactile_1['1_dY_'+str(i)] = tac_msg.pillars[i].dY
            self.tactile_1['1_dZ_'+str(i)] = tac_msg.pillars[i].dZ
            self.tactile_1['1_fX_'+str(i)] = tac_msg.pillars[i].fX
            self.tactile_1['1_fY_'+str(i)] = tac_msg.pillars[i].fY
            self.tactile_1['1_fZ_'+str(i)] = tac_msg.pillars[i].fZ
            self.tactile_1['1_incontact_'+str(i)] = tac_msg.pillars[i].in_contact
            self.tactile_1['1_slipstate_'+str(i)] = tac_msg.pillars[i].slip_state

        self.tactile_1['1_friction_est'] = tac_msg.friction_est
        self.tactile_1['1_target_grip_force'] = tac_msg.target_grip_force
        self.tactile_1['1_is_sd_active'] = tac_msg.is_sd_active
        self.tactile_1['1_is_ref_loaded'] = tac_msg.is_ref_loaded
        self.tactile_1['1_is_contact'] = tac_msg.is_contact
        self.mutex.release()

    def initialize_tactile_dict(self):
        """
        Initializes all of the keys in each ordered dictionary. This ensures the header and order is correct even if recording starts before data is published.
        """
        # Position
        # self.position = OrderedDict({'gripper_pos': None})
        # Tactile sensor
        self.tactile_0 = OrderedDict()
        self.tactile_1 = OrderedDict()
        #  edit made by kp for time
        
        for i in range(8):
            self.tactile_0['0_dX_'+str(i)] = None
            self.tactile_0['0_dY_'+str(i)] = None
            self.tactile_0['0_dZ_'+str(i)] = None
            self.tactile_0['0_fX_'+str(i)] = None
            self.tactile_0['0_fY_'+str(i)] = None
            self.tactile_0['0_fZ_'+str(i)] = None
            self.tactile_0['0_incontact_'+str(i)] = None
            self.tactile_0['0_slipstate_'+str(i)] = None

            self.tactile_1['1_dX_'+str(i)] = None
            self.tactile_1['1_dY_'+str(i)] = None
            self.tactile_1['1_dZ_'+str(i)] = None
            self.tactile_1['1_fX_'+str(i)] = None
            self.tactile_1['1_fY_'+str(i)] = None
            self.tactile_1['1_fZ_'+str(i)] = None
            self.tactile_1['1_incontact_'+str(i)] = None
            self.tactile_1['1_slipstate_'+str(i)] = None
        


        self.tactile_0['0_friction_est'] = None
        self.tactile_0['0_target_grip_force'] = None
        self.tactile_0['0_is_sd_active'] = None
        self.tactile_0['0_is_ref_loaded'] = None
        self.tactile_0['0_is_contact'] = None

        self.tactile_1['1_friction_est'] = None
        self.tactile_1['1_target_grip_force'] = None
        self.tactile_1['1_is_sd_active'] = None
        self.tactile_1['1_is_ref_loaded'] = None
        self.tactile_1['1_is_contact'] = None

    def get_start_file_index(self):
        # Returns the starting file number (old number)
        current_files = os.listdir(self.storage_directory)
        try:
            numbers = np.array([i.split('.csv', 1)[0] for i in current_files], dtype=int)
            return np.max(numbers)
        except:
            return 0
        

def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    record = Record()

    # Give control over to ROS2
    rclpy.spin(record)

    # Shutdown cleanly
    rclpy.shutdown()


if __name__ == '__main__':
    main()
