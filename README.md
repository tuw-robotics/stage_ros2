# stage_ros2

## missing dependencies
```
rosdep install -i --from-path src --rosdistro galactic -y

```

## missing dependencies

```
sudo apt-get install git cmake g++ libjpeg8-dev libpng-dev libglu1-mesa-dev libltdl-dev libfltk1.1-dev
cd YOUR_ROS2_WORKSPACE
mkdir src
cd src
git clone --branch ros2 git@github.com:tuw-robotics/Stage.git
git clone --branch humble git@github.com:tuw-robotics/stage_ros2.git
cd YOUR_ROS2_WORKSPACE
colcon build --symlink-install --cmake-args -DOpenGL_GL_PREFERENCE=LEGACY 
colcon build --symlink-install --packages-select stage_ros2        
```

```
cd YOUR_ROS2_WORKSPACE
cd src/stage_ros2/world
run stage_ros2 stage_ros2  --ros-args --ros-args -p world_file:=line.world 
```

SoftReset

```
ros2 service call /reset_positions std_srvs/srv/Empty
```