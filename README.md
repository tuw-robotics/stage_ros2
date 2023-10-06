# stage_ros2
This is a  ROS bridge for the robot simulator [Stage](https://github.com/rtv/Stage).
It supports multiple robots with one or multiple tf-trees.

* [install](res/install.md)
* [run demos](res/demos.md)
* [handling multiple vehickes](res/multi_robot_setup.md)

## Single Robot Support
This stage ROS bridge can be configured to use a namespace even for one robot.
```
## with /tf
ros2 launch stage_ros2 stage.launch.py world:=cave enforce_prefixes:=false one_tf_tree:=true
## with /robot_0/tf
ros2 launch stage_ros2 stage.launch.py world:=cave enforce_prefixes:=true one_tf_tree:=false
```

## Multi Robot Support
This stage ROS bridge supports multiple robots with one or multiple tf-Trees.
### multiple tf-trees
<div align="center">
<img src="res/cave_three_robots_multiple_tf_trees_depth_info.jpg" alt="stage two rviz widowns showing two robot views" width="800px" /><br>
<table style="width:800px;"><td>
Multiple TF-trees after with two rviz nodes for robot_0 and robot_1: <br># <b>ros2 launch stage_ros2 stage.launch.py world:=cave_three_robots one_tf_tree:=false</b> 
<br># <b>ros2 launch stage_ros2 rviz_ns.launch.py config:=robot_ns namespace:=robot_0</b>
<br># <b>ros2 launch stage_ros2 rviz_ns.launch.py config:=robot_ns namespace:=robot_1</b>
<br>on <i>Ubuntu 22.04</i> with ros2 <i>humble</i>
</td></table> 
</div>

### one tf-tree
<div align="center">
<img src="res/cave_three_robots_one_tf_tree.jpg" alt="stage and rviz with laser, tf, and cameras" width="800px" /><br>
<table style="width:800px;"><td>
One TF-tree after: <br># <b>ros2 launch stage_ros2 demo.launch.py world:=cave_three_robots one_tf_tree:=true</b> 
<br>on <i>Ubuntu 22.04</i> with ros2 <i>humble</i>
</td></table> 
</div>




## Thanks
I like to thank all people behind [Stage](https://github.com/rtv/Stage) and [ros-simulation/stage_ros](https://github.com/ros-simulation) for the simulation and the first ROS bridge as well as the people for the initial ROS2 implementations [ShengliangD/stage_ros2](https://github.com/ShengliangD/stage_ros2) and [n0nzzz/stage_ros2](https://github.com/n0nzzz/stage_ros2). The work was used as base for my implementation. Which differs to the previous ROS interfaces in its modularity.
