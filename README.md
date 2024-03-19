# ROB 599 Gripper Control

Gripper Project for OSU ROB599


## Project Motivaton
This project aims to integrate a software interface into a custom end-effector intended for use in an HRI study. The study focuses on exploring how individuals distinguish between successful and unsuccessful grasps. The custom end-effector, affixed to a linear actuator, comprises two fingers independently actuated and outfitted with 3D-force-sensing tactile sensors.
The goal of this project is to develop a ROS2 interface that interacts with off-the-shelf hardware components, including tactile sensors, Dynamixel motors, and a linear actuator. This interface will facilitate the recording of tactile data while simultaneously executing linear actuator motions and opening/closing operations of the end effector. These actions will be performed across varying Dynamixel operating modes, namely position and torque. Utilizing ROS2 ensures that commands are transmitted and data is recorded synchronously across all nodes during grasp operations.


## Hardware
- 2X [Dynamixel motors](https://www.robotis.us/dynamixel-xw540-t260-r/?_ga=2.191430958.504289236.1710805799-762848481.1710805799)
- 1X [Contactile tactile sensor array](https://contactile.com/products/)
- 1X [OpenBuilds Linear Actuator](https://openbuildspartstore.com/c-beam-linear-actuator-bundle/)
- 1X [Arduino Uno](https://store.arduino.cc/products/arduino-uno-rev3)

![setup](https://github.com/m-rosette/rob599_project/assets/92352927/36eaef62-c3d8-42ac-a75a-b328b78e153b)


## Functionality 
- End-effector opening/closing
- Change dynamixel operating modes: position and current (torque)
- Operate linear actuator
- Tactile sensor data collection 


## Package Installation
1. Ensure ROS2 is installed [(installation)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

2. Clone this repo into your ros2_ws by running
```bash
cd ~/ros2_ws/src/
git clone $this-repo$

```
3. Install all Dynamixel libraries
```bash
apt install ros-<ros_version>-dynamixel*
```
4. Build and source your workspace
```bash
cd ~/ros2_ws/
colcon build
source install/setup.bash
```
5. Upload the example linear actuator control script to the arduino. This script is located in *rob599_project/example_linear_actuator_control/linear_actuator_control.ino*


## Running a Grasp Sequence 
The following commands will start up all the features of the gripper and operate a predefined grasp sequence. If individual component control is desired, follow the instructions in the subsequent sections.

1. Initialize hardware control by launching: 
```bash
ros2 launch gripper gripper_launch.py
```
2. Start the main gripper sequence and collect tactile data by running the following in another terminal:
```bash
ros2 run gripper data_collection <filename>
```
Tactile data is saved as a .csv in the *gripper/resources/* directory. This can be changed in *gripper/gripper/record.py*.


## Individual Component Control
### Linear Actuator 
1. Start the linear actuator service server:
```bash
ros2 run gripper actuator_linear 
```
2. Send position commands (stepper motor steps) from another terminal:
```bash
ros2 service call /input_number gripper_msgs/srv/LinearActuator "{location_goal: -1000}"
```
In this project, a negative stepper motor position refers to moving the linear actuator "forward", while a negative moves it "backward".
    

### Dynamixels 
This section demonstrates individual motor control or paired motor control.
- ID refers to the individual motor ID. Starting with 1 if there is a single motor in the system.
- Operating mode can be set to **Position** (operating_mode = 3) or **Current** (operating_mode = 0).
- Operation target can be set by either setting a target position (range: 50-500) or target current (range: 25-100) value.

#### Single Control
This example shows current control mode with a target value of 40.
1. In one terminal start communication with the motor:
```bash
ros2 run dynamixel_control motor_interface
```
2. In another terminal operate the motor:
```bash
ros2 service call /set_operating_mode dynamixel_control/srv/SetOperatingMode "{id: 1, operating_mode: 0, operation_target: 40}"
```

#### Paired Control    
1. In one terminal start communication with the motors:
```bash
ros2 run dynamixel_control dual_motor_interface
``` 
2. In another terminal run:
```bash
ros2 run dynamixel_control dual_motor_client <operating_mode> <operation_target>
```


### Tactile Sensor Data Logging
Tactile data is saved as a .csv in the *gripper/resources/* directory. This can be changed in *gripper/gripper/record.py*.
1. To record tactile data, launch the following in one terminal:
In one terminal run:
```bash
ros2 launch papillarray_ros2_v2 papillarray
```
2. In a second terminal start the recording action interface:
```bash
ros2 run gripper record
```
3. In a third terminal set the desired .csv filename:
```bash
ros2 run gripper record_client <filename>
```

## Demonstration
Successful grasp utilizeing dual dynamixel control operating in current mode with a value of 100.

![2finger_gif](https://github.com/m-rosette/rob599_project/assets/92352927/c845ac8a-5005-477b-9b5d-3c359d829ca4)

<!-- Recall each end effector has its dynamixel, so when you run the dynamixels in current mode to each with the value of 100, you get a 2-finger successful grasp. -->


The following image is taken to show resultant tactile forces felt during a grasp operation. This image is captured from a custom PyQt interface that displays live tactile data from each pillar of the tactile array. This particular image is an example snip-it of what a user would see when taking the user study to determine grasp success. When the tactile sensing arrays are compressed, the white grids will darken, ultimately informing the user where in the grasp the object is located. 

![image](https://github.com/m-rosette/rob599_project/assets/92352927/4395f384-e189-41d6-b24d-fa7f6371a590)


<!-- ## Tactile Sensor Hardware Information

Tactile Sensor is from Contactile. The bottom image shows how the sensor functions. 
![tactile_Sense](https://github.com/m-rosette/rob599_project/assets/92352927/49a77d5d-47df-494a-9dc3-b3763a74083f) -->


## Final Video 
https://github.com/m-rosette/rob599_project/assets/92352927/b42fc1a6-66b1-4dcb-a3f1-4789c937f315






