# Description
Implementation of A* Graph Search Algorithm for a 2D Path Planning Problem with ROS

# Installation
This implementation is done as a catkin_ws, so it can be easly compiled with
```
  catkin_make
  catkin build
```
I strongly recommend building the package with "-DCMAKE_BUILD_TYPE=Release" build flag to create improve efficiency.

# Usage
For the demo, a ros node is created. This node subscribes to 
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
One can use the rviz tools to define "/initialpose" and "/move_base_simple/goal"

# Example


