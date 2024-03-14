# ROB 599 Gripper Control

Gripper Project for ROB599
Clone this repo into your ros2_ws/src folder
## Project Motivaton
This project aims to add a software interface to a custom end effector that will be used in an  HRI study that focuses on how people can differentiate between successful and unsuccessful grasps. The custom end effector is mounted to a linear actuator and consists of two independently actuated fingers equipped with tactile sensors. Each finger utilizes a dynamixel servo motor and a Contactile tactile sensor. 
Our main objective for this project is to create a ROS2 interface that interacts with off-the-shelf hardware, namely the tactile sensors, dynamixel motors, and the linear actuator. The interface should enable end effector opening/closing features with varying dynamixel operating conditions; position mode, torque mode, and sensory input from tactile sensors. The utility of ROS2 ensures data is being sent and recorded synchronously across all nodes while the end effector is grasping objects. 

## Image of the Set-Up
![setup](https://github.com/m-rosette/rob599_project/assets/92352927/36eaef62-c3d8-42ac-a75a-b328b78e153b)


## Functionality 
- Enable end effector opening/closing
- Change operating modes
- Sensory input from sensors 


## Installation 
Instructions go here.

## Move Linear Actuator 
    ros2 run Arduino arduino_control

## Move Dynamixels 

Let's first test your dynamixels by running the following commands. Note this is for a single dynamixel (XW540-T260-R)
In one terminal run:

    ros2 run dynamixel_sdk_examples motor_interface
          
In another terminal run:

    ros2 service call /set_operating_mode dynamixel_sdk_custom_interfaces/srv/SetOperatingMode "{operating_mode: 0, id: 1, 
    operation_target: 40}"
To move two dynamixels at a time you will run the following: 
In one terminal run:
     
     ros2 run dynamixel_control dual_motor_interface
This will activate dual motor interface. 
In another terminal run:

    ros2 run dynamixel_control dual_motor_client <operating_mode> <operation_target>
    
You have a few options for the operating mode and target. If you want to use position control mode, then type 3. For current control mode type 0. The operating target for current control is numbers between 25-100 and for the position they are 50-500. 

## Video Test 
The .gif is of the dynamixels in current mode with a value of 100. This shows the dual motor interface working for the dynamixels. 

![2finger_gif](https://github.com/m-rosette/rob599_project/assets/92352927/c845ac8a-5005-477b-9b5d-3c359d829ca4)

Recall each end effector has its dynamixel, so when you run the dynamixels in current mode to each with the value of 100, you get a 2-finger successful grasp.


The following image is taken to show the force the papillary nodes in the tactile sensor feel when grasping the object. Image is a snip it from what a user would see when taking the user study. The interface is made in PyQt. 

![image](https://github.com/m-rosette/rob599_project/assets/92352927/4395f384-e189-41d6-b24d-fa7f6371a590)




https://github.com/m-rosette/rob599_project/assets/92352927/ff6847ef-660b-4223-83da-df95547ecabc




