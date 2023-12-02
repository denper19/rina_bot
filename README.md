### References
1. [The Construct Sim](https://www.youtube.com/results?sp=mAEB&search_query=the+construct+sim)
2. [HBRobotics](https://github.com/hbrobotics/ros_arduino_bridge)
3. [LidarBot](https://github.com/TheNoobInventor/lidarbot)
4. [JoshNewans](https://github.com/joshnewans/articubot_one)

### Progress:
- [ ] Port to Humble
- [ ] IMU interface
- [ ] IMU URDF simulation
- [ ] EKF for fusion
- [ ] Add Dockers

### Connections

| Pins   | Arduino |
| ------------- | ------------- |
| Left Motor IN1  | Content Cell  |
| Left Motor IN2  | Content Cell  |
| Right Motor IN1  | Content Cell  |
| Right Motor IN2  | Content Cell  |
| Left Motor ENCA  | Content Cell  |
| Left Motor ENCB  | Content Cell  |
| Right Motor ENCA  | Content Cell  |
| Right Motor ENCB  | Content Cell  |
| Left Motor PWR/EN  | D12  |
| Right Motor PWR/EN  | D13  |

### Package Setup:

1. First git clone the package into your workspace, with `git clone https://github.com/denper19/rina_bot.git`
2. Now do `rosdep init && rosdep update --include-eol-distros`
3. Then install dependencies with `rosdep install --from-path src --ignore-src -r -y`
4. Build the package with `colcon build --symlink-install`
   1. If running on the ***raspberry pi*** then run `colcon build --symlink-install --executor sequential`
  
### Modifying discovery mechanism:

In case your router crashes while running the package (due to being flooded by messages), please follow the steps below
1. Open a new terminal and run `fastdds discovery --server-id 0`
2. Run `ifconfig` and determine your IP address
3. Run `export ROS_DISCOVERY_SERVER=127.0.0.1:11811` in each terminal. In case you are running it over ssh on the raspberry pi, then run `export ROS_DISCOVERY_SERVER=YOUR_IP_ADDRESS:11811`

### Running the package:

1. ***Whenever you open a new cmd tab, make sure to source the package with `source install/local_setup.bash`***
2. If running in simulation, then run `ros2 launch robot gazebo.launch.py`
   1. If running on real robot then ssh into the raspberry pi (assuming you have cloned, built and sourced the package) then run `ros2 launch robot robot.launch.py`
   2. In a new tab on the raspberry pi, to start the YDLidar, run `ros2 launch ydlidar_ros2_driver ydlidar_launch.py`
   3. All subsequent steps after this are to be executed on your laptor that is connected to the same wifi as the raspberry pi
4. Next, if you have a bluetooth controller, then connect it to your laptop and run `ros2 launch robot joystick.launch.py` in a new tab.
   1. If you don't, then run `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy`
5. Now run twist mux to merge the cmd velocity topics with `ros2 run twist_mux twist_mux --ros-args --params-file /home/jermito/rina_ws/src/rina_bot/robot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped`
6. Run slam toolbox with `ros2 launch slam_toolbox online_async_launch.py params_file:=./src/robot/config/mapper_params_online_async.yaml use_sim_time:=true` (if running on real bot, set `use_sim_time:=false`)
   1. When running slam toolbox, set the world to the `map` frame
   2. Make sure to add the SlamToolBox panel in RVIZ, and add the map display plugin
   3. Drive the robot around in the world until you are satisfied with the map created
   4. Save the map by typing a name into the `save map` and `serialize map` box and clicking their respective buttons.
   5. Once done, you can `CTRL-C` out of the cmd line.
7. Now run localization with `ros2 launch nav2_bringup localization_launch.py map:=/home/jermito/rina_ws/test_map.yaml`
   1. Set the frame to `map` in rviz
   2. Click the set pose option in rviz and then select a point on the map that best matches where the robot is physically.
9. Finally, you can run navigation. In a new tab run `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true`
