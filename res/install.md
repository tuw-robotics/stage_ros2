# stage_ros2 Install
## Install
```
cd YOUR_ROS2_WORKSPACE
mkdir src
cd src
git clone git@github.com:tuw-robotics/Stage.git
git clone git@github.com:tuw-robotics/stage_ros2.git
rosdep update
rosdep install --from-paths ./Stage --ignore-src -r -y  # install dependencies for Stage
rosdep install --from-paths ./stage_ros2 --ignore-src -r -y  # install dependencies for stage_ros2
cd YOUR_ROS2_WORKSPACE
colcon build --symlink-install 
```