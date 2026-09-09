import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class DistancePublisher(Node):
    def __init__(self):
        super().__init__('distance_publisher')

        self.declare_parameter('true_distance', 100.0)
        self.declare_parameter('sigma', 0.002)
        self.declare_parameter('rate_hz', 5.0)

        self.pub = self.create_publisher(Float64, 'distance', 10)

        period = 1.0 / self.get_parameter('rate_hz').value
        self.timer = self.create_timer(period, self.publish_measurement)
        self.get_logger().info('distance_publisher pokrenut')

    def publish_measurement(self):
        d = self.get_parameter('true_distance').value
        s = self.get_parameter('sigma').value
        msg = Float64()
        msg.data = random.gauss(d, s)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = DistancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()