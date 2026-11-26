"""Score one fixed-layout lap per fresh ROS 2 launch."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ackermann_msgs.msg import AckermannDriveStamped
from f110_msgs.msg import WpntArray
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
import numpy as np
from race_manager.track import Track, crossed_forward
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parent
LATERAL_ACCEL = 2.5
BRAKING_ACCEL = 2.5
PREVIEW_M = 2.0


class Trial(Node):
    def __init__(self, mode):
        super().__init__('ab_trial')
        self.mode = mode
        map_dir = Path(os.environ.get('AB_MAP_DIR', ROOT.parent / 'berlin_analysis')).resolve()
        self.track = Track(str(map_dir / 'global_waypoints.json'))
        self.line = self.track.finish_line()
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(AckermannDriveStamped, '/ab/controller_drive', self.drive, 10)
        self.create_subscription(WpntArray, '/local_waypoints', self.path, 10)
        self.create_subscription(Odometry, '/car_state/odom_GT', self.odom, 10)
        self.create_subscription(String, '/sim/collision', self.collision, 10)
        self.create_subscription(PoseArray, '/sim/obstacles', self.obstacles,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.layout = None
        self.layout_at = None
        self.position = None
        self.last_position = None
        self.last_stamp = None
        self.start_at = None
        self.line_at = None
        self.finished_at = None
        self.points = None
        self.state = None
        self.create_subscription(String, '/state', self.update_state, 10)
        self.overtaking_samples = 0
        self.collision_kind = None
        self.collision_position = None
        self.distance = 0.0
        self.max_speed = 0.0
        self.limited = 0
        self.drive_samples = 0
        self.cap_min = None
        self.max_progress = 0.0
        self.previous_s = None
        self.progress = 0.0
        self.min_obstacle_center_m = None

    def update_state(self, msg):
        self.state = msg.data

    def path(self, msg):
        self.points = np.array([(p.x_m, p.y_m) for p in msg.wpnts])

    def obstacles(self, msg):
        layout = [(p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w))
                  for p in msg.poses]
        if len(layout) == 4 and self.layout is None:
            self.layout = layout
            self.layout_at = time.monotonic()
        elif self.start_at is not None and self.layout != layout:
            self.collision_kind = 'layout_changed'

    def collision(self, msg):
        if msg.data != 'none' and self.collision_kind is None:
            self.collision_kind = msg.data
            self.collision_position = self.position

    def speed_cap(self):
        points = self.points
        i = int(np.argmin(np.sum((points - self.position) ** 2, axis=1)))
        candidates = points[max(0, i - 2):]
        if len(candidates) < 5:
            return math.inf
        # Use a 0.2 m chord on each side to reduce sensitivity to individual waypoints.
        a = candidates[2:-2] - candidates[:-4]
        b = candidates[4:] - candidates[2:-2]
        chord_product = np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1)
        chord_product *= np.linalg.norm(a + b, axis=1)
        cross = np.abs(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])
        curvature = 2 * cross / np.maximum(chord_product, 1e-8)
        arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(candidates, axis=0), axis=1))]
        ahead = np.maximum(arc[2:-2] - arc[i - max(0, i - 2)], 0)
        valid = (ahead <= PREVIEW_M) & (chord_product > 1e-8)
        if not valid.any():
            return math.inf
        caps = LATERAL_ACCEL / np.maximum(curvature[valid], 1e-6)
        caps += 2 * BRAKING_ACCEL * ahead[valid]
        return math.sqrt(float(caps.min()))

    def drive(self, msg):
        if self.collision_kind or self.finished_at is not None:
            self.publisher.publish(AckermannDriveStamped())
            return
        if self.layout_at is None or time.monotonic() - self.layout_at < 2.1:
            return
        if self.position is None or self.points is None or len(self.points) < 5:
            return
        if self.start_at is None:
            if msg.drive.speed <= 0.05:
                return
            self.start_at = time.monotonic()
            self.previous_s = self.track.nearest_s(*self.position)
        self.drive_samples += 1
        self.overtaking_samples += int(self.state == 'StateType.OVERTAKE')
        if self.mode == 'B' and msg.drive.speed > 0:
            cap = self.speed_cap()
            if cap < msg.drive.speed:
                self.limited += 1
                self.cap_min = cap if self.cap_min is None else min(self.cap_min, cap)
                msg.drive.speed = cap
        self.publisher.publish(msg)

    def odom(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_position is None or self.start_at is None:
            self.last_position, self.last_stamp = self.position, stamp
            return
        if self.finished_at is not None:
            return
        self.distance += math.dist(self.position, self.last_position)
        self.max_speed = max(self.max_speed, msg.twist.twist.linear.x)
        center_distance = min(math.hypot(p.x - x, p.y - y) for x, y, _ in self.layout)
        self.min_obstacle_center_m = (center_distance if self.min_obstacle_center_m is None
                                      else min(center_distance, self.min_obstacle_center_m))
        s = self.track.nearest_s(*self.position)
        self.progress += self.track.arc_delta(self.previous_s, s)
        self.previous_s = s
        self.max_progress = max(self.max_progress, self.progress)
        if self.collision_kind is None and crossed_forward(self.last_position, self.position, *self.line):
            # Interpolate the timestamp between the two ground-truth odometry messages.
            a, _, heading = self.line
            before = float(np.dot(np.array(self.last_position) - a, heading))
            after = float(np.dot(np.array(self.position) - a, heading))
            crossing = self.last_stamp + (stamp - self.last_stamp) * (-before / (after - before))
            if self.line_at is None:
                self.line_at = crossing
            elif self.progress > self.track.length * 0.95:
                self.finished_at = crossing
                self.publisher.publish(AckermannDriveStamped())
        self.last_position, self.last_stamp = self.position, stamp


def run(seed, mode, log_dir, drive_timeout):
    name = f'{seed}_{mode}'
    env = dict(os.environ, AB_SEED=str(seed), ROS_LOG_DIR=str(log_dir / 'ros' / name))
    node = Trial(mode)
    with (log_dir / f'{name}.log').open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', str(ROOT / 'berlin_trial.launch.py')],
            stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        status = 'startup_timeout'
        deadline = time.monotonic() + 30
        try:
            while process.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.collision_kind:
                    status = node.collision_kind if node.start_at else 'startup_collision'
                    break
                if node.finished_at is not None:
                    status = 'clean_lap'
                    break
                if node.start_at is None and time.monotonic() > deadline:
                    break
                if node.start_at and time.monotonic() - node.start_at > drive_timeout:
                    status = 'drive_timeout'
                    break
            else:
                status = 'launch_exit'
            node.publisher.publish(AckermannDriveStamped())
            return {
                'seed': seed, 'mode': mode, 'status': status, 'obstacles': node.layout,
                'lap_s': round(node.finished_at - node.line_at, 3) if status == 'clean_lap' else None,
                'collision': node.collision_kind, 'collision_xy': node.collision_position,
                'track_length_m': round(node.track.length, 3),
                'progress_m': round(node.max_progress, 3), 'distance_m': round(node.distance, 3),
                'max_speed_mps': round(node.max_speed, 3),
                'min_obstacle_center_m': (round(node.min_obstacle_center_m, 3)
                                          if node.min_obstacle_center_m is not None else None),
                'drive_commands': node.drive_samples, 'limited_commands': node.limited,
                'min_applied_cap_mps': round(node.cap_min, 3) if node.cap_min is not None else None,
                'overtake_samples': node.overtaking_samples, 'last_state': node.state,
            }
        finally:
            node.destroy_node()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seeds', nargs='+', type=int, required=True)
    parser.add_argument('--modes', nargs='+', choices=('A', 'B'), default=['A', 'B'])
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=55)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    try:
        with (args.out / 'runs.jsonl').open('w') as output:
            for index, seed in enumerate(args.seeds):
                modes = args.modes if index % 2 == 0 else list(reversed(args.modes))
                for mode in modes:
                    result = run(seed, mode, args.out, args.timeout)
                    row = json.dumps(result)
                    print(row, flush=True)
                    output.write(row + '\n')
                    output.flush()
                    if result['status'] in ('startup_timeout', 'launch_exit', 'startup_collision'):
                        return 1
    finally:
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

