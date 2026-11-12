# ROS 2 SLAM on Raspberry Pi 5

Mô phỏng và triển khai **SLAM 2D** cho robot di động với **ROS 2 Jazzy** trên **Ubuntu 24.04 / Raspberry Pi 5**, dùng `slam_toolbox` (graph-based SLAM, có loop closure).

- **Mô phỏng:** TurtleBot3 trong Gazebo, robot tự đi khám phá và né vật cản bằng LiDAR (node `wander`) để xây bản đồ.
- **Robot thật:** Raspberry Pi 5 + RPLIDAR chạy `slam_toolbox`; RViz chạy trên laptop qua mạng (DDS).

## Kiến trúc
```
 Gazebo / RPLIDAR ──/scan──► slam_toolbox ──/map, TF map→odom──► RViz2
        ▲                         ▲
        │ /cmd_vel                │ TF odom→base_link (odometry)
   wander node ◄──/scan───────────┘
```

## Cài đặt (Ubuntu 24.04 + ROS 2 Jazzy)
```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-turtlebot3-gazebo \
                 ros-jazzy-turtlebot3-description ros-jazzy-rplidar-ros
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/MinhHieu15/ros2-slam-rpi5.git
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

## Chạy mô phỏng
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch slam_bot sim_slam.launch.py            # tự khám phá
ros2 launch slam_bot sim_slam.launch.py auto_explore:=false   # điều khiển tay bằng teleop
```
Lưu bản đồ:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/tb3_world
```

## Chạy trên Raspberry Pi 5
```bash
ros2 launch slam_bot rpi5_slam.launch.py serial_port:=/dev/ttyUSB0
# Trên laptop (cùng ROS_DOMAIN_ID):
rviz2 -d $(ros2 pkg prefix slam_bot)/share/slam_bot/config/slam.rviz
```
Cấu hình `config/slam_toolbox.yaml` đã giảm tần suất cập nhật bản đồ để phù hợp với CPU của Pi 5.

## Node `wander`
| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `linear_speed` | 0.15 m/s | vận tốc tiến |
| `angular_speed` | 0.8 rad/s | vận tốc quay khi né |
| `safe_distance` | 0.45 m | khoảng cách an toàn phía trước |
| `stamped` | true | publish `TwistStamped` (TurtleBot3 Jazzy); đặt `false` nếu robot dùng `Twist` |

## Hướng phát triển
- Thêm Nav2 để điều hướng tới điểm đích trên bản đồ đã lưu.
- Kết hợp camera và IMU (Visual-Inertial SLAM).
