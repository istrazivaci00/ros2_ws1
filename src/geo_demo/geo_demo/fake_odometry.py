import math

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class FakeOdometry(Node):
    """Simulira kretanje po kruznici: objavljuje odom -> base_link."""

    def __init__(self):
        super().__init__('fake_odometry')

        self.declare_parameter('radius', 5.0)
        self.declare_parameter('period_s', 20.0)
        self.declare_parameter('rate_hz', 20.0)

        self.br = TransformBroadcaster(self)
        self.t0 = self.get_clock().now()

        rate = self.get_parameter('rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.publish_tf)
        self.get_logger().info('fake_odometry pokrenut')

    def publish_tf(self):
        r = self.get_parameter('radius').value
        period = self.get_parameter('period_s').value

        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        theta = 2.0 * math.pi * t / period

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'

        tf.transform.translation.x = r * math.cos(theta)
        tf.transform.translation.y = r * math.sin(theta)
        tf.transform.translation.z = 0.0

        # cista rotacija oko z: kvaternion (0, 0, sin(yaw/2), cos(yaw/2))
        yaw = theta + math.pi / 2.0
        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)

        self.br.sendTransform(tf)


def main():
    rclpy.init()
    node = FakeOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()