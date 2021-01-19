## About
forked from https://github.com/mkrizmancic/sphero_formation and modified.

[Here](https://youtu.be/RTcC8k2Nvyw) is the link for the project implemented in this project.

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
## Installation

Simply clone this repository inside ROS workspace and run `catkin_make` in workspace root.

## Usage

**Move Leader via Keyboard**:
```
$ rosrun boids_ros boids_teleop.py
```


