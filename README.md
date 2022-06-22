# Description
Implementation of A* Graph Search Algorithm for a 2D Path Planning Problem with ROS

# Installation
This implementation is done as a catkin_ws, so it can be easly compiled with
```
  catkin_make
  catkin build
```
I strongly recommend building the package with "-DCMAKE_BUILD_TYPE=Release" flag to increase the efficiency. 

# Usage
For the demostarition a ros node that subscribes to 
```
/map
/initialpose
/move_base_simple/goal
```
and publishes 
```
/inflated_map
/optimal_path
``` 

is created. One can use the rviz navigation tools to define "/initialpose" and "/move_base_simple/goal" on rviz interface.

# Example


