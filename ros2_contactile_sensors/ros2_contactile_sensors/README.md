# Contactile Sensors ROS2 Driver

ROS2 drivers for papillarray sensor

## Packages in the Repository:

  - `papillarray_ros2_v2` - cpp node for the papillarray sensor.
  - `sensor_interfaces` - custom msgs and srvs used by sensor packages.

## Getting Started
For getting started, you'll basically need five steps:

1. [Install ROS2](https://docs.ros.org/en/foxy/Installation.html). Currently only ROS2 Foxy is supported offically

   Once installed, please make sure to [source ROS2](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) before proceeding.

3. Make sure that `colcon`, its extensions and `vcs` are installed:
   ```
   sudo apt install python3-colcon-common-extensions python3-vcstool
   ```

4. Create a new ROS2 workspace:
   ```
   mkdir -p ~/ros2_contactile/src
   ```

5. Copy relevant packages, compile, and source the workspace by using:
   ```
   cd ~/ros2_contactile/src
   Copy packages here
   cd ~/ros2_contactile
   colcon build
   source ./install/setup.bash
   ```
## Getting Started
To use the ROS2 driver:

1. Launch ROS2 node:
   ```
   ros2 launch papillarray_ros2_v2 papillarray.launch.py
   ```

2. If u need to change defult vlaues u can change it at launch e.g.
   ```
   ros2 launch papillarray_ros2_v2 papillarray.launch.py com_port:=/dev/ttyACM1
   ```