How to run navigation

Connections

| Pins  | Arduino |
| ------------- | ------------- |
| Left Motor IN1  | Content Cell  |
| Left Motor IN2  | Content Cell  |
| Right Motor IN1  | Content Cell  |
| Right Motor IN2  | Content Cell  |
| Left Motor ENCA  | Content Cell  |
| Left Motor ENCB  | Content Cell  |
| Right Motor ENCA  | Content Cell  |
| Right Motor ENCB  | Content Cell  |
| Left Motor PWR/EN  | Content Cell  |
| Right Motor PWR/EN  | Content Cell  |

Run Navigation From Scratch:

simulation:

1. ros2 launch robot gazebo.launch.py

2. ros2 launch slam_toolbox online_async_launch.py params_file:=./src/robot/config
mapper_params_online_async.yaml use_sim_time:=true

3. ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=new_map_save.yaml -p  use_sim_time:=true

4. ros2 run nav2_util lifecycle_bringup map_server

5. ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true

6. ros2 run nav2_util lifecycle_bringup amcl

real:

1. ros2 launch robot robot.launch.py

2. ros2 launch slam_toolbox online_async_launch.py params_file:=./src/robot/config
mapper_params_online_async.yaml use_sim_time:=false

3. ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=new_map_save.yaml -p  use_sim_time:=false

4. ros2 run nav2_util lifecycle_bringup map_server

5. ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=false

6. ros2 run nav2_util lifecycle_bringup amcl
