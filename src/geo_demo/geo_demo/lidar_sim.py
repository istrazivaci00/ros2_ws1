import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class LidarSim(Node):
    """Simulira 2D lidar u poznatoj sobi i odometriju sa driftom."""

    def __init__(self):
        super().__init__('lidar_sim')

        self.declare_parameter('scan_hz', 10.0)
        self.declare_parameter('speed', 0.6)
        self.declare_parameter('path_radius', 4.5)
        self.declare_parameter('beams', 360)
        self.declare_parameter('range_max', 15.0)
        self.declare_parameter('range_sigma', 0.02)
        self.declare_parameter('odom_scale_error', 0.02)
        self.declare_parameter('odom_yaw_error', 0.01)

        r = self.get_parameter('path_radius').value
        cy = r  # centar kruznice, tako da robot krece iz koordinatnog pocetka

        # zidovi sobe i unutrasnja prepreka, kao duzi (x1, y1, x2, y2)
        self.segs = self._rect(-8.0, cy - 8.0, 8.0, cy + 8.0)
        self.segs += self._rect(-1.5, cy - 1.0, 1.5, cy + 1.0)
        self.segs = np.array(self.segs, dtype=float)

        n = self.get_parameter('beams').value
        self.angles = np.linspace(-math.pi, math.pi, n, endpoint=False)

        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.path_pub = self.create_publisher(Path, 'ground_truth_path', 10)
        self.br = TransformBroadcaster(self)

        self.phi = -math.pi / 2.0          # ugaona pozicija na kruznici
        self.cy = cy
        self.odom = np.zeros(3)            # x, y, theta procenjeni odometrijom
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.rng = np.random.default_rng(42)

        hz = self.get_parameter('scan_hz').value
        self.dt = 1.0 / hz
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info('lidar_sim pokrenut')

    @staticmethod
    def _rect(x0, y0, x1, y1):
        return [(x0, y0, x1, y0), (x1, y0, x1, y1),
                (x1, y1, x0, y1), (x0, y1, x0, y0)]

    def true_pose(self):
        r = self.get_parameter('path_radius').value
        x = r * math.cos(self.phi)
        y = self.cy + r * math.sin(self.phi)
        th = self.phi + math.pi / 2.0
        return x, y, th

    def raycast(self, px, py, th):
        rmax = self.get_parameter('range_max').value
        ang = self.angles + th
        d = np.stack([np.cos(ang), np.sin(ang)], axis=1)        # (B,2)

        a = self.segs[:, 0:2]                                   # (S,2)
        rv = self.segs[:, 2:4] - a                              # (S,2)
        ap = a - np.array([px, py])                             # (S,2)

        denom = d[:, None, 0] * rv[None, :, 1] - d[:, None, 1] * rv[None, :, 0]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (ap[None, :, 0] * rv[None, :, 1]
                 - ap[None, :, 1] * rv[None, :, 0]) / denom
            u = (ap[None, :, 0] * d[:, None, 1]
                 - ap[None, :, 1] * d[:, None, 0]) / denom

        ok = (np.abs(denom) > 1e-12) & (t > 0.05) & (u >= 0.0) & (u <= 1.0)
        t = np.where(ok, t, np.inf)
        rngs = t.min(axis=1)

        sigma = self.get_parameter('range_sigma').value
        rngs = rngs + self.rng.normal(0.0, sigma, rngs.shape)
        return np.where(np.isfinite(rngs) & (rngs < rmax), rngs, np.inf)

    def step_odometry(self, ds, dth):
        ks = self.get_parameter('odom_scale_error').value
        kt = self.get_parameter('odom_yaw_error').value
        ds_o = ds * (1.0 + ks) + self.rng.normal(0.0, 0.002)
        dth_o = dth * (1.0 + kt) + self.rng.normal(0.0, 0.0005)

        self.odom[2] += dth_o / 2.0
        self.odom[0] += ds_o * math.cos(self.odom[2])
        self.odom[1] += ds_o * math.sin(self.odom[2])
        self.odom[2] += dth_o / 2.0

    def tick(self):
        stamp = self.get_clock().now().to_msg()

        v = self.get_parameter('speed').value
        r = self.get_parameter('path_radius').value
        ds = v * self.dt
        dphi = ds / r

        x, y, th = self.true_pose()
        self.step_odometry(ds, dphi)
        self.phi += dphi

        # odom -> base_link (sa driftom)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = float(self.odom[0])
        tf.transform.translation.y = float(self.odom[1])
        qz, qw = yaw_to_quat(self.odom[2])
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.br.sendTransform(tf)

        # sken iz TACNE poze
        rngs = self.raycast(x, y, th)
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser'
        scan.angle_min = float(self.angles[0])
        scan.angle_max = float(self.angles[-1])
        scan.angle_increment = float(self.angles[1] - self.angles[0])
        scan.time_increment = 0.0
        scan.scan_time = float(self.dt)
        scan.range_min = 0.1
        scan.range_max = float(self.get_parameter('range_max').value)
        scan.ranges = [float(v) for v in rngs]
        self.scan_pub.publish(scan)

        # tacna putanja, za vizuelno poredjenje
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = 'map'
        ps.pose.position.x = x
        ps.pose.position.y = y
        qz, qw = yaw_to_quat(th)
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.path.poses.append(ps)
        self.path.header.stamp = stamp
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = LidarSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()