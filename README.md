# ROS 2 SLAM on Raspberry Pi 5

2D SLAM for a mobile robot with ROS 2 Jazzy and `slam_toolbox` (graph-based, with loop closure), on Ubuntu 24.04 and a Raspberry Pi 5.

- **Simulation:** TurtleBot3 in Gazebo. The `wander` node drives the robot around and avoids obstacles using the LiDAR, so the map builds without manual driving.
- **Real robot:** a Raspberry Pi 5 with an RPLIDAR runs `slam_toolbox`. RViz runs on a laptop on the same network.

## Architecture
```
 Gazebo / RPLIDAR ──/scan──► slam_toolbox ──/map, TF map→odom──► RViz2
        ▲                         ▲
        │ /cmd_vel                │ TF odom→base_link (odometry)
   wander node ◄──/scan───────────┘
```

## Setup (Ubuntu 24.04, ROS 2 Jazzy)
```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-turtlebot3-gazebo \
                 ros-jazzy-turtlebot3-description ros-jazzy-rplidar-ros
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/MinhHieu15/ros2-slam-rpi5.git
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

## Simulation
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch slam_bot sim_slam.launch.py                      # robot explores on its own
ros2 launch slam_bot sim_slam.launch.py auto_explore:=false  # drive it with teleop instead
```
Save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/tb3_world
```

## Raspberry Pi 5
```bash
ros2 launch slam_bot rpi5_slam.launch.py serial_port:=/dev/ttyUSB0
# on the laptop, same ROS_DOMAIN_ID:
rviz2 -d $(ros2 pkg prefix slam_bot)/share/slam_bot/config/slam.rviz
```
`config/slam_toolbox.yaml` updates the map less often than the defaults to keep the Pi's CPU load down. The real robot must publish its own `odom → base_link` transform from wheel odometry.

## `wander` parameters
| Parameter | Default | Meaning |
|---|---|---|
| `linear_speed` | 0.15 m/s | forward speed |
| `angular_speed` | 0.8 rad/s | turn rate when avoiding |
| `safe_distance` | 0.45 m | minimum clearance ahead |
| `stamped` | true | publish `TwistStamped` (TurtleBot3 on Jazzy); set `false` for robots that take `Twist` |

## Status
The package has not yet been built and run on hardware. Launch file names for `turtlebot3_gazebo` and `rplidar_ros` may need adjusting for your installed versions.

## Next steps
- Add Nav2 to navigate to goals on a saved map.
- Fuse a camera and IMU (visual-inertial SLAM).
