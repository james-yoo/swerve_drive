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
The off-the-shelf components for building and testing swerve-drive system are listed below. You can also download 2D/3D models from 'model' folder.  

| Parts                         | Model             | Qty    |
|-------------------------------|-------------------|--------|
| Dynamixel Servo Motor         | XL430-W250-T      | 2      | 
| Communication Interface       | U2D2              | 1      |
| Wheel+Tire Set                | TB3_WHEEL-ISW-01  | 1      | 
| SMPS                          | SMPS2Dynamixel    | 1      |
| Converting Cable(Molex-TTL)   | ROBOT CABLE-X3P   | 1      |
| Communication Calbe(TTL)      | ROBOT CABLE-X3P   | 2      |
  
  
## How to use
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
  
