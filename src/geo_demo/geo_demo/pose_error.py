import csv
import math
import os
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from tf2_ros import (Buffer, TransformListener, LookupException,
                     ConnectivityException, ExtrapolationException)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PoseError(Node):
    """Poredi map->base_link sa tacnom pozom i pise CSV."""

    def __init__(self):
        super().__init__('pose_error')

        self.declare_parameter('out_csv', '~/ros2_ws/results/run.csv')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('delay_s', 0.5)
        self.last_rx = self.get_clock().now()
        self.finished = False

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pending = deque()

        self.create_subscription(
            PoseStamped, 'ground_truth_pose', self.on_pose, 50)
        self.timer = self.create_timer(0.2, self.process)

        path = os.path.expanduser(self.get_parameter('out_csv').value)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.fh = open(path, 'w', newline='')
        self.csv = csv.writer(self.fh)
        self.csv.writerow(['t', 'gt_x', 'gt_y', 'gt_yaw',
                           'slam_x', 'slam_y', 'slam_yaw',
                           'e_pos', 'e_yaw_deg'])

        self.n = 0
        self.sum_sq = 0.0
        self.max_e = 0.0
        self.get_logger().info(f'pose_error pise u {path}')

    def on_pose(self, msg):
        self.pending.append(msg)
        self.last_rx = self.get_clock().now()

    def process(self):
        now = self.get_clock().now()
        delay = Duration(seconds=self.get_parameter('delay_s').value)

        while self.pending:
            msg = self.pending[0]
            if (now - Time.from_msg(msg.header.stamp)) < delay:
                break
            self.record(self.pending.popleft())

        if (not self.finished and not self.pending and self.n > 0
                and (now - self.last_rx) > Duration(seconds=2.0)):
            self.finished = True
            rms = math.sqrt(self.sum_sq / self.n)
            self.fh.flush()
            self.get_logger().info(
                f'GOTOVO  n={self.n}  RMS={rms * 100:.2f} cm  '
                f'max={self.max_e * 100:.2f} cm')

    def record(self, msg):
        mf = self.get_parameter('map_frame').value
        bf = self.get_parameter('base_frame').value
        try:
            tr = self.buf.lookup_transform(mf, bf, Time.from_msg(msg.header.stamp))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        sx = tr.transform.translation.x
        sy = tr.transform.translation.y
        syaw = yaw_of(tr.transform.rotation)

        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gyaw = yaw_of(msg.pose.orientation)

        e_pos = math.hypot(sx - gx, sy - gy)
        e_yaw = wrap(syaw - gyaw)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.csv.writerow([f'{t:.3f}', f'{gx:.4f}', f'{gy:.4f}', f'{gyaw:.5f}',
                           f'{sx:.4f}', f'{sy:.4f}', f'{syaw:.5f}',
                           f'{e_pos:.4f}', f'{math.degrees(e_yaw):.3f}'])

        self.n += 1
        self.sum_sq += e_pos * e_pos
        self.max_e = max(self.max_e, e_pos)
        if self.n % 50 == 0:
            rms = math.sqrt(self.sum_sq / self.n)
            self.get_logger().info(
                f'n={self.n}  APE RMS={rms * 100:.1f} cm  max={self.max_e * 100:.1f} cm')

    def destroy_node(self):
        self.fh.flush()
        self.fh.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PoseError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()