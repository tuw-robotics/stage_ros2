# stage_ros2 Demos

## launch files

### stage
```
ros2 launch stage_ros2 example.launch.py
```
### stage and rviz
demo worlds with different robot configurations and rviz configs
```
ros2 launch stage_ros2 demo.launch.py world:=cave
ros2 launch stage_ros2 demo.launch.py world:=lines
ros2 launch stage_ros2 demo.launch.py world:=cave_three_robots
ros2 launch stage_ros2 demo.launch.py world:=cave_seven_robots
```
<table>
  <tr>
    <th><img src="cave.png" width="200px"/> <br>cave</th>
    <th><img src="lines.png" width="200px"/> <br>lines</th>
    <th><img src="cave_three_robots.png" width="200px"/> <br>cave_three_robots</th>
    <th><img src="cave_seven_robots.png" width="200px"/> <br>cave_seven_robots</th>
  </tr> 
</table>

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