### References
1. [The Construct Sim](https://www.youtube.com/results?sp=mAEB&search_query=the+construct+sim)
2. [HBRobotics](https://github.com/hbrobotics/ros_arduino_bridge)
3. [LidarBot](https://github.com/TheNoobInventor/lidarbot)
4. [JoshNewans](https://github.com/joshnewans/articubot_one)

### Progress:
- [x] Port to Humble
- [x] Simplify/Reduce Steps
- [x] Make IMU reads faster
- [x] IMU interface
- [x] IMU URDF simulation
- [x] EKF for IMU + Odom fusion
- [x] Add GPS Simulation
- [ ] Look into adding Camera drivers
- [ ] Look into GPS drivers
- [ ] Look into ESP32 Camera
- [ ] Add Dockers

### Connections

| Pins   | Arduino |
| ------------- | ------------- |
| Left Motor IN1  | D10  |
| Left Motor IN2  | D6  |
| Right Motor IN1  | D9  |
| Right Motor IN2  | D5  |
| Left Motor ENCA  | D2  |
| Left Motor ENCB  | D3  |
| Right Motor ENCA  | A4  |
| Right Motor ENCB  | A5  |
| Left Motor PWR/EN  | D13  |
| Right Motor PWR/EN  | D12  |
| IMU SDA| SDA/PC4  |
| IMU SCL| SCL/PC5  |
| IMU PWR  | 5V  |
| IMU GND  | GND  |

### Package Setup:

1. `git clone https://github.com/denper19/rina_bot.git`
2. `cd rina_bot && git branch humble`
3. `sudo apt install python3-rosdep2`
2. `rosdep init && rosdep update --include-eol-distros`
3. `rosdep install --from-path src --ignore-src -r -y`
4. `colcon build --symlink-install`
   1. If running on the ***raspberry pi*** then run `colcon build --symlink-install --executor sequential`
  
### Modifying discovery mechanism:

In case your router crashes while running the package (due to being flooded by messages), please follow the steps below
1. Open a new terminal and run `fastdds discovery --server-id 0`
2. Run `ifconfig` and determine your IP address
3. Run `export ROS_DISCOVERY_SERVER=127.0.0.1:11811` in each terminal. In case you are running it over ssh on the raspberry pi, then run `export ROS_DISCOVERY_SERVER=YOUR_IP_ADDRESS:11811`

### Running the package:

1. ***Whenever you open a new cmd tab, make sure to source the package with `source install/local_setup.bash`***
2. If running in simulation, *open a new tab in the workspace* run `ros2 launch robot gazebo.launch.py`
   1. If running on real robot then ssh into the raspberry pi (assuming you have cloned, built and sourced the package) then run `ros2 launch robot robot.launch.py`
   2. In a new tab on the raspberry pi, to start the YDLidar, run `ros2 launch ydlidar_ros2_driver ydlidar_launch.py`. 
   3. All subsequent steps after this are to be executed on your laptop that is connected to the same wifi as the raspberry pi
4. Next, if you have a bluetooth controller, then connect it to your laptop. Otherwise if you don't, then run `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy`
5. Run slam toolbox with `ros2 launch robot online_async_launch.py use_sim_time:=true` (if running on real bot, set `use_sim_time:=false`)
   1. When running slam toolbox, set the world to the `map` frame
   2. Make sure to add the SlamToolBox panel in RVIZ, and add the map display plugin
   3. Drive the robot around in the world until you are satisfied with the map created
   4. Save the map by typing a name into the `save map` and `serialize map` box and clicking their respective buttons.
   5. Once done, you can `CTRL-C` out of the cmd line.
7. Now run localization with `ros2 launch robot localization_launch.py map:=<path_to_map>.yaml`
   1. Set the frame to `map` in rviz
   2. Click the *set pose* option in rviz and then select a point on the map that best matches where the robot is physically.
9. Finally, you can run navigation. In a new tab run `ros2 launch robot navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true`. If using real robot then set use_sim_time to false.

**Additionally, if you do not want to use AMCL for localization, then you simply follow step 9 immediately after step 5.**
