"""
Node "wander": robot tự đi khám phá và né vật cản dựa trên LiDAR (/scan),
giúp slam_toolbox xây dựng bản đồ mà không cần điều khiển tay.

Logic đơn giản:
- Chia vùng quét phía trước thành 3 sector: trái / giữa / phải.
- Phía trước thoáng  -> đi thẳng.
- Có vật cản         -> quay về phía sector thoáng hơn.
"""
import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Wander(Node):
    def __init__(self):
        super().__init__('wander')
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('safe_distance', 0.45)
        # TurtleBot3 trên ROS 2 Jazzy dùng TwistStamped cho /cmd_vel
        self.declare_parameter('stamped', True)

        self.v = self.get_parameter('linear_speed').value
        self.w = self.get_parameter('angular_speed').value
        self.d_safe = self.get_parameter('safe_distance').value
        self.stamped = self.get_parameter('stamped').value

        msg_type = TwistStamped if self.stamped else Twist
        self.pub = self.create_publisher(msg_type, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.get_logger().info('Wander node started')

    def sector_min(self, scan, start_deg, end_deg):
        """Khoảng cách nhỏ nhất trong sector [start_deg, end_deg] (0 deg = phía trước)."""
        vals = []
        for i, r in enumerate(scan.ranges):
            ang = math.degrees(scan.angle_min + i * scan.angle_increment)
            ang = (ang + 180.0) % 360.0 - 180.0
            if start_deg <= ang <= end_deg and scan.range_min < r < scan.range_max:
                vals.append(r)
        return min(vals) if vals else float('inf')

    def on_scan(self, scan):
        front = self.sector_min(scan, -20, 20)
        left = self.sector_min(scan, 20, 70)
        right = self.sector_min(scan, -70, -20)

        twist = Twist()
        if front > self.d_safe:
            twist.linear.x = self.v
            # bám nhẹ tránh tường hai bên
            if left < self.d_safe:
                twist.angular.z = -0.3 * self.w
            elif right < self.d_safe:
                twist.angular.z = 0.3 * self.w
        else:
            twist.angular.z = self.w if left > right else -self.w

        self.publish(twist)

    def publish(self, twist):
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist = twist
            self.pub.publish(msg)
        else:
            self.pub.publish(twist)

    def stop(self):
        self.publish(Twist())


def main():
    rclpy.init()
    node = Wander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
