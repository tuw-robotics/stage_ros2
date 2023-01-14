# stage_ros2
## Examples

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
## launch files

### stage
```
ros2 launch stage_ros2 example.launch.py
```
### stage and rviz
an example world with different robot configurations
```
ros2 launch stage_ros2 example.launch.py
```
### teleop joy

```
ros2 launch stage_ros2 f710.launch.py namespace:='/'       # for a single vehicle world like cave.world
ros2 launch stage_ros2 f710.launch.py namespace:='robot_0' # for a world like cave_multi.world or example.world
```


## parameters on stage_ros2
### without parameters
it will start the cave.world
```
run stage_ros2 stage_ros2 
```
### without parameters
```
run stage_ros2 stage_ros2  --ros-args --ros-args \
    -p world_file:=line.world \ 
    -p use_static_transformations:=true
```

## SoftReset

```
ros2 service call /reset_positions std_srvs/srv/Empty
```