import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class DistanceSubscriber(Node):
    def __init__(self):
        super().__init__('distance_subscriber')

        self.declare_parameter('report_every', 20)

        self.sub = self.create_subscription(
            Float64, 'distance', self.callback, 10)

        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0

        self.get_logger().info('distance_subscriber pokrenut')

    def callback(self, msg):
        x = msg.data

        # Welfordov algoritam: tekuća sredina i varijansa u jednom prolazu
        self.n += 1
        delta = x - self.mean
        self.mean += delta / self.n
        self.m2 += delta * (x - self.mean)

        every = self.get_parameter('report_every').value
        if self.n % every == 0:
            sigma = math.sqrt(self.m2 / (self.n - 1)) if self.n > 1 else 0.0
            self.get_logger().info(
                f'n={self.n:4d}  sredina={self.mean:.5f} m  '
                f'sigma={sigma * 1000:.2f} mm'
            )


def main():
    rclpy.init()
    node = DistanceSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()