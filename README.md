## Overview 
![swerve_drive_ros](https://user-images.githubusercontent.com/23667624/38178150-674cf18e-3647-11e8-900b-5a8d76f3ddaa.png)
  
A swerve drive is a special type of drivetrain used in FRC (FIRST Robotics Competition). It allows each individual wheel to be powered and turned independent from the other wheels, giving it unparalleled maneuverability on the field. It has following features:  
  
- Independently steered drive modules
- Simple conceptually
- Simple wheels
- Good traction
- Complex to build
- Complex to program and control
- Maximum pushing force
- Either steered gearboxes or concentric drive
  
## Video  
[![Swerve drive ros](http://img.youtube.com/vi/aw8DVg23Epk/0.jpg)](https://youtu.be/aw8DVg23Epk)  
  
## BOM (Build of Materials)
The off-the-shelf components for building and testing swerve-drive system are listed below. You can also download bracket (.stl) file for 3d printing from 'models' folder.  

| Parts                         | Model             | Qty    |
|-------------------------------|-------------------|--------|
| Dynamixel Servo Motor         | XL430-W250-T      | 2      | 
| Communication Interface       | U2D2              | 1      |
| Wheel+Tire Set                | TB3_WHEEL-ISW-01  | 1      | 
| SMPS                          | SMPS2Dynamixel    | 1      |
| Converting Cable(Molex-TTL)   | ROBOT CABLE-X3P   | 1      |
| Communication Calbe(TTL)      | ROBOT CABLE-X3P   | 2      |
  
  
## How to use  
This work was tested under ROS Kinetic(desktop-full installation) + Ubuntu Xenial(16.04)  
  
### Wiring & motor configure
Motor's ID should be set as described picture below. You can set motor ID via [dynamixel_workbench](https://github.com/ROBOTIS-GIT/dynamixel-workbench/tree/master/dynamixel_workbench_single_manager)(Linux) or [R+ Manager 2.0](http://emanual.robotis.com/docs/en/software/rplus2/manager/)(Windows)
![wiring](https://user-images.githubusercontent.com/23667624/38525760-75d6b8dc-3c8e-11e8-80f0-17de58b6ce12.png)   
  
### Prerequisites
Following packages should be installed.
```
$ sudo apt-get install ros-kinetic-dynamixel-sdk
$ sudo apt-get install ros-kinetic-qt-build
$ sudo apt-get install ros-kinetic-dynamixel-workbench
$ sudo apt-get install ros-kinetic-dynamixel-workbench-msgs
```  
  
### Download & compile the source
1. Clone this repository to your catkin workspace,  
```
$ cd catkin_ws/src
$ git clone https://github.com/james-yoo/swerve_drive.git
```
  
2. Compile the source,  
```
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```
  
3. Launch the swerve_drive_control node,  
```
$ roslaunch swerve_drive swerve_drive_control.launch
```
  
4. Open the new terminal and start teleop_twist_keybord script,
```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
  
5. Control with twist keybord command:  
- 'i' = forward
- ',' = backward
- 'j' = left turn
- 'l' = right turn
  
6. (Optional) To visualize swerve_drive in Rviz, open the new terminal,
```
$ roslaunch swerve_drive swerve_drive_urdf.launch
```
  
