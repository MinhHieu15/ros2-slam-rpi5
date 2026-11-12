# ROS 2 SLAM on Raspberry Pi 5

2D SLAM for a mobile robot with ROS 2 Jazzy and `slam_toolbox` on Ubuntu 24.04, running on a Raspberry Pi 5.

In simulation, a TurtleBot3 in Gazebo builds the map while the `wander` node drives it around and avoids obstacles with the LiDAR. On the real robot, the Pi runs an RPLIDAR and `slam_toolbox`, and RViz runs on a laptop on the same network.

## Architecture
```
 Gazebo / RPLIDAR ──/scan──► slam_toolbox ──/map, TF map→odom──► RViz2
        ▲                         ▲
        │ /cmd_vel                │ TF odom→base_link (odometry)
   wander node ◄──/scan───────────┘
```

## Setup
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
ros2 launch slam_bot sim_slam.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/maps/tb3_world
```

## Raspberry Pi 5
```bash
ros2 launch slam_bot rpi5_slam.launch.py serial_port:=/dev/ttyUSB0
```
On the laptop, with the same `ROS_DOMAIN_ID`:
```bash
rviz2 -d $(ros2 pkg prefix slam_bot)/share/slam_bot/config/slam.rviz
```
`config/slam_toolbox.yaml` updates the map less often than the defaults to keep the CPU load down on the Pi. The robot's base driver publishes `odom → base_link` from wheel odometry.

## `wander` parameters
| Parameter | Default | |
|---|---|---|
| `linear_speed` | 0.15 m/s | forward speed |
| `angular_speed` | 0.8 rad/s | turn rate when avoiding |
| `safe_distance` | 0.45 m | clearance needed ahead |
| `stamped` | true | publish `TwistStamped` instead of `Twist` |
