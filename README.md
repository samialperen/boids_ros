## About
forked from https://github.com/mkrizmancic/sphero_formation .

## Prerequisities
Tested on Ubuntu 16.04 and ROS Kinetic
* Install Pandas library for python
```
$ sudo apt-get install python-pandas
```
* Install ROS map-server
```
$ sudo apt-get install ros-kinetic-map-server
```
* Install teleop-key (Optional) (Install only if you want to move leader via keyboard)
```
$ sudo apt-get install ros-kinetic-teleop-twist-keyboard
```
## Installation

Simply clone this repository inside ROS workspace and run `catkin_make` in workspace root.

## Usage

**Move Leader via Keyboard**:
```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot_0/cmd_vel
```


