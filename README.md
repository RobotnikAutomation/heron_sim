# heron_sim

Packages for the simulation of the HERON project

<!--p align="center">
  <img src="doc/rbvogui_xl_base.png" height="275" />
</p-->

## Packages

This packages contains: 

### heron_gazebo

Launch files and world files to start the models in gazebo

### heron_sim_bringup

Launch files that execute the complete simulation of the robot

## Requirements

- Ubuntu 20.04
- ROS Noetic
- Python 2.7 or higher

## Simulating RB-Vogui

### 1) Install the following dependencies:

This simulation has been tested using Gazebo 9 version. To facilitate the installation you can use the ```vcstool```:

```bash
sudo apt-get install -y python3-vcstool
```

Install ```catkin_tools``` in order to compile the workspace

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-catkin-tools
```

Install ```rqt_joint_trajectory_controller``` to move the arm joint by joint and ```moveit_commander``` to move it via script

```bash
sudo apt-get install ros-noetic-rqt-joint-trajectory-controller 
sudo apt-get install ros-noetic-moveit-commander
```

### 2) Create a workspace and clone the repository:

Create a new workspace

```bash
mkdir catkin_ws
cd catkin_ws
```

Install one of these versions. Keep in mind that on the stable version the latest features may be not available.

**Install stable version:**

```bash
vcs import --input https://raw.githubusercontent.com/RobotnikAutomation/heron_sim/noetic-main/repos/install.repos
rosdep install --from-paths src --ignore-src -y -r
``` 

### 3) Install the controllers, robotnik_msgs and the rcomponent:


```bash
cd ~/catkin_ws
sudo dpkg -i src/heron_common/libraries/*
```

### 4) Compile:

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

### 5) Run HERON simulation:

These are the different configurations available depending on the use case:

- CONES
- ROADMARKS (TODO)
- RUPS (TODO)
- CRACKS (TODO)
- POTHOLES (TODO)

### 5.1 CONES

Set your robot kinematics to omni/ackermann (In case of ackermann, you will need twist2ackermann node enabled)
  
```bash
roslaunch heron_sim_bringup heron_complete.launch
```
<!--p align="center">
  <img src="doc/rbvogui_cones.png" height="275" />
</p-->

### 5.2 ROADMARKS


### 5.3 RUPS


### 5.4 POTHOLES


### 5.5 CRACKS


## 6) Teleoperation

The robot can be controlled through three manual methods:

- Rviz pad plugin
- Keyboard
- Joystick

### 6.1 Rviz pad plugin

When rviz is launched with the robot, this plugin is loaded automatically. It can be found on the lower left corner of rviz.
It allows the robot to rotate and move forward/backward, but it can not perform omnidirectional movements

<p align="center">
  <img src="doc/rviz_pad_teleop_plugin.png" height="250" />
</p>

It is highly recommended to use this option with simulation because is the fastest.

### 6.2 Keyboard

Install the keyboard node

```bash
sudo apt-get update
sudo apt-get install ros-noetic-teleop-twist-keyboard
```

Open a new terminal and launch the node to move the robot

```bash
ROS_NAMESPACE=robot rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```

### 6.3 Joystick

The robot can also be controller by a PS4 pad controller. This option is usually used with the actual robot but, it works with the simulation too.

Follow the [installation guide of the robotnik_pad](https://github.com/RobotnikAutomation/robotnik_pad)

Once the required software is installed, launch the simulation with ```launch_pad:=true```

Param | Type | Description | Requirements
------------ | -------------  | ------------- | -------------
launch_pad | boolean  | It launches the robotnik_pad package | ds4drv installed, ps4 joystick, bluetooth connection

For example:

```bash
roslaunch heron_sim_bringup heron_complete.launch launch_pad:=true
```

### 7 Troubleshooting

### 7.1  Laser visualization

If the laser does not display via RVIZ is probably because the computer does not use the GPU. You can disable the GPU for the heron simulation. Just add this parameter to the robot:

```bash
roslaunch heron_sim_bringup heron_complete.launch use_gpu:=false
```

## 8) Scripts

The robot can be commanded from a script via the standard ROS interface like move_base or moveit_commander.

### 1. Move robot script

Launch the robot, localization and navigation

```bash
roslaunch heron_sim_bringup heron_complete.launch
```

Then, run the script. The robot will move to (1,1) position

```bash
ROS_NAMESPACE=robot rosrun heron_gazebo move_robot.py
```

You can set your own position by editing the script:

```bash
point.target_pose.pose.position.x = 1.0
point.target_pose.pose.position.y = 1.0
point.target_pose.pose.position.z = 0.0
```

### 2. Move arm script

Launch the robot with the arm:
```bash
  roslaunch heron_sim_bringup heron_complete.launch
```

Launch moveit:
```bash
ROS_NAMESPACE=robot roslaunch heron_moveit_ur10 demo.launch
```

Then run one of these scripts:

#### Joint by joint 

It moves the arm joint by joint

```bash
ROS_NAMESPACE=robot rosrun heron_gazebo move_arm_joint_by_joint.py
```

You can set your own joints positions by editing the script:

```bash
joint_goal[0] = 0
joint_goal[1] = -pi/4
joint_goal[2] = 0
joint_goal[3] = -pi/2
joint_goal[4] = 0
joint_goal[5] = pi/3
```

#### To point

It moves the arm to a point

```bash
ROS_NAMESPACE=robot rosrun heron_gazebo move_arm_to_point.py
```

You can set your own point by editing the script:
```bash
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.7
pose_goal.position.y = 0.4
pose_goal.position.z = 1.5
```

<!--
https://answers.gazebosim.org//question/12723/gpu_ray-sensors-bad-behaviour-when-increasing-samples/
-->

