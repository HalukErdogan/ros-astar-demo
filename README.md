# ros-astar-demo
Implementation of A* Graph Search Algorithm for a 2D Path Planning Problem with ROS

For the demo, a ros node is created. This node subscribes to 
```/map
  /initialpose
  /move_base_simple/goal
```
and publishes 
```/inflated_map
  /optimal_path
``` 
