glviewer
========

## Description

Visualization of robot(quadcopter/bicopter) in 3D using OpenGL. The ROS node was initially designed to be used in IARC 7th mission. So it still contains a lot of the circular ground robot in the code but commented out now. It is now been use in DARC Lab, University of Utah Robotics Center, by Jack. If there's anyone using the node and has some question, feel free to put it in 'issue'.

## Installation

Clone the package into your catkin workspace:
```
git clone https://github.com/JJJJJJJack/glviewer.git
cd ~/catkin_ws
sudo apt-get install liftgl-dev
cd ~/catkin_ws/src/glviewer/include/utility/
git submodule init
```

## Fonts

Font file is stored in 'font' folder, and the path name needs to be changed at line 35 in glviewer.cpp.

## Change vehicle

`VEHICLE_TYPE` is defined in *src/glviewer.cpp*.