# stage_ros2


```
cd YOUR_ROS2_WORKSPACE
mkdir src
cd src
git clone --branch ros2 git@github.com:tuw-robotics/Stage.git
git clone git@github.com:tuw-robotics/stage_ros2.git
cd YOUR_ROS2_WORKSPACE
colcon build --symlink-install --cmake-args -DOpenGL_GL_PREFERENCE=LEGACY 
colcon build --symlink-install --packages-select stage_ros2        
```
