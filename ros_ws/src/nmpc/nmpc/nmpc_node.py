#!/usr/bin/env python3
"""ROS node: follow the target with NMPC, escape moving obstacles, stay safe.

Every control period:
  1. Read the latest odometry, obstacles (from the lidar package) and target.
  2. Work out how dangerous the moving obstacles are (threat 0..1).
     High threat -> "escape mode": faster, harder acceleration allowed.
  3. Solve the NMPC (plus once more from an escape guess when threatened).
  4. Send the first command only if "this command, then brake" is safe.
     Otherwise use the best escape manoeuvre. Brake only as the last resort.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path

from obstacle_msgs.msg import ObstacleArray

from .safety import best_escape, check_command, threat_level
from .solver import NmpcSolver


# Every setting, with its default. ros_ws/config/nmpc.yaml can override them.
DEFAULTS = {
    # topics
    'odom_topic': '/AGV/odometry',
    'obstacle_topic': '/obstacles',
    'target_topic': '/target/relative_pose',
    'cmd_topic': '/AGV/cmd_vel',
    # horizon and solver
    'horizon': 3.0, 'dt': 0.1, 'max_obstacles': 12,
    'solver_max_iter': 100, 'solver_max_time': 0.08,
    # vehicle
    'wheel_base': 0.1046, 'steering_limit': 0.5, 'max_steer_rate': 2.0,
    'robot_radius': 0.11, 'safety_margin': 0.03,
    # follow mode
    'max_speed': 0.15, 'max_accel': 0.25, 'brake_decel': 0.5,
    'desired_distance': 0.50,
    # escape mode
    'escape_speed': 0.25, 'escape_accel': 0.5, 'escape_threat': 0.3,
    'escape_time': 1.5, 'escape_effort_scale': 0.1,
    'escape_gap_scale': 0.2, 'escape_view_scale': 0.2,
    # safety
    'hard_horizon': 1.0, 'prediction_growth': 0.05, 'ttc_threshold': 1.5,
    'min_speed': 0.05, 'sensor_timeout': 0.5,
    # target
    'target_radius': 0.14, 'target_center_offset': 0.10, 'target_clearance': 0.10,
    'target_velocity_filter': 0.35, 'target_loss_timeout': 0.8,
    'camera_half_fov': 0.85,
    # cost weights
    'w_gap': 4.0, 'w_view': 1.0, 'w_clear': 2.0, 'clear_scale': 0.08,
    'w_stop': 3.0, 'w_accel': 0.2, 'w_steer': 0.1,
    'w_accel_change': 0.5, 'w_steer_change': 0.5,
    'w_slack': 1000.0, 'w_terminal': 5.0,
    # escape manoeuvre scoring
    'w_dir': 1.0, 'w_escape_view': 0.5, 'w_escape_clear': 1.0,
}


def stamp_to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class NmpcNode(Node):
    def __init__(self):
        super().__init__('nmpc_controller')
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.s = {name: self.get_parameter(name).value for name in DEFAULTS}
        s = self.s

        self.get_logger().info('Building the NMPC problem...')
        self.solver = NmpcSolver(s)

        self.state = None            # (x, y, yaw, v) of the AGV
        self.odom_time = None
        self.odom_frame = 'odom'
        self.obstacles = []          # latest ObstacleArray contents
        self.obstacle_time = None
        self.target = None           # [x, y, vx, vy] in the odom frame
        self.target_time = None
        self.previous_command = (0.0, 0.0)   # (a, steer)
        self.previous_plan = []
        self.last_mode = ''

        self.create_subscription(Odometry, s['odom_topic'], self.on_odom, 10)
        self.create_subscription(ObstacleArray, s['obstacle_topic'], self.on_obstacles, 10)
        self.create_subscription(PoseStamped, s['target_topic'], self.on_target, 10)
        self.cmd_publisher = self.create_publisher(Twist, s['cmd_topic'], 10)
        self.path_publisher = self.create_publisher(Path, '/nmpc/path', 10)
        self.create_timer(s['dt'], self.control_loop)
        self.get_logger().info(f'NMPC ready: {s["horizon"]} s horizon, {self.solver.N} steps')

    # ------------------------------------------------------------------ inputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.state = (p.x, p.y, yaw, max(0.0, message.twist.twist.linear.x))
        self.odom_time = self.now()
        if message.header.frame_id:
            self.odom_frame = message.header.frame_id

    def on_obstacles(self, message):
        self.obstacles = list(message.obstacles)
        self.obstacle_time = self.now()

    def on_target(self, message):
        """Checkerboard position (AGV frame) -> odom frame, plus a velocity estimate."""
        if self.state is None:
            return
        x, y, yaw, _ = self.state
        bx, by = message.pose.position.x, message.pose.position.y
        tx = x + math.cos(yaw) * bx - math.sin(yaw) * by
        ty = y + math.sin(yaw) * bx + math.cos(yaw) * by
        stamp = stamp_to_seconds(message.header.stamp)

        vx = vy = 0.0
        if self.target is not None and self.target_time is not None:
            dt = stamp - self.target_time
            if 0.0 < dt < 1.0:
                measured_vx = (tx - self.target[0]) / dt
                measured_vy = (ty - self.target[1]) / dt
                a = self.s['target_velocity_filter']
                if math.hypot(measured_vx, measured_vy) < 0.5:   # ignore jumps
                    vx = a * measured_vx + (1 - a) * self.target[2]
                    vy = a * measured_vy + (1 - a) * self.target[3]
                else:
                    vx, vy = self.target[2], self.target[3]
        self.target = [tx, ty, vx, vy]
        self.target_time = stamp

    # ------------------------------------------------------------------ helpers
    def make_obstacle_list(self):
        """Lidar obstacles + the target itself, nearest first, as dictionaries."""
        s = self.s
        x, y = self.state[0], self.state[1]
        result = []
        for o in self.obstacles:
            result.append({'x': o.x, 'y': o.y, 'vx': o.vx, 'vy': o.vy,
                           'R': o.radius + s['robot_radius'] + s['safety_margin'],
                           'moving': bool(o.is_moving)})

        # The target is too low for the LiDAR, so add it from the camera.
        # Its centre is a bit behind the checkerboard (as seen from the AGV).
        if self.target is not None:
            tx, ty, tvx, tvy = self.target
            ux = tx - x
            uy = ty - y
            length = max(math.hypot(ux, uy), 1e-6)
            offset = s['target_center_offset']
            result.append({'x': tx + offset * ux / length, 'y': ty + offset * uy / length,
                           'vx': tvx, 'vy': tvy,
                           # never closer than target_clearance to the target body
                           'R': s['target_radius'] + s['robot_radius'] + s['target_clearance'],
                           'moving': math.hypot(tvx, tvy) > 0.04})

        for o in result:
            o['distance'] = math.hypot(o['x'] - x, o['y'] - y)
            # Already too close (sensor noise, start position)? Then only ask
            # "do not get any closer", otherwise the problem has no solution.
            if o['distance'] < o['R'] + 0.005:
                o['R'] = max(o['distance'] - 0.005, 0.01)
        result.sort(key=lambda o: o['distance'] - o['R'])
        return result[:int(s['max_obstacles'])]

    def send(self, a, steer, v_max):
        """(acceleration, steering) -> Twist for the Ackermann plugin."""
        v = self.state[3]
        speed = max(0.0, min(v_max, v + a * self.s['dt']))
        command = Twist()
        command.linear.x = speed
        command.angular.z = speed * math.tan(steer) / self.s['wheel_base']
        self.cmd_publisher.publish(command)
        self.previous_command = (a, steer)

    def brake(self, reason):
        self.log_mode('BRAKE: ' + reason)
        if self.state is None:
            self.cmd_publisher.publish(Twist())
            return
        self.send(-self.s['brake_decel'], self.previous_command[1], self.s['escape_speed'])
        self.previous_plan = []

    def log_mode(self, mode):
        if mode != self.last_mode:
            self.get_logger().info(mode)
            self.last_mode = mode

    def publish_path(self, states):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.odom_frame
        for x, y in states:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_publisher.publish(path)

    # ------------------------------------------------------------------ main loop
    def control_loop(self):
        s = self.s
        now = self.now()

        # 1. Are the sensors alive?
        if self.state is None or now - self.odom_time > s['sensor_timeout']:
            return self.brake('no odometry')
        if self.obstacle_time is None or now - self.obstacle_time > s['sensor_timeout']:
            return self.brake('no obstacle data')
        if self.target is None:
            return self.brake('target not seen yet')

        # Target lost: keep the last seen position as goal, but stop predicting
        # motion. The AGV drives up to it and waits there.
        target = list(self.target)
        if now - self.target_time > s['target_loss_timeout']:
            target[2] = target[3] = 0.0
            lost = ' (target lost, going to last seen position)'
        else:
            lost = ''

        obstacles = self.make_obstacle_list()

        # 2. Threat and mode.
        threat, threat_obstacle = threat_level(self.state, obstacles, s)
        escaping = threat > s['escape_threat']
        if escaping:
            mode = {'v_max': s['escape_speed'], 'a_max': s['escape_accel'],
                    'effort': s['escape_effort_scale'], 'gap': s['escape_gap_scale'],
                    'view': s['escape_view_scale']}
        else:
            mode = {'v_max': s['max_speed'], 'a_max': s['max_accel'],
                    'effort': 1.0, 'gap': 1.0, 'view': 1.0}

        # Escape manoeuvre: fallback, and a second starting guess for the NMPC.
        escape_plan = best_escape(self.state, target[:2], obstacles, threat_obstacle,
                                  max(threat, 0.5), s)

        # 3. Solve from one or two starting guesses.
        guesses = [self.previous_plan[1:] if self.previous_plan else []]
        if escaping and escape_plan is not None:
            guesses.append(escape_plan)

        best = None
        for guess in guesses:
            result = self.solver.solve(self.state, target, obstacles,
                                       self.previous_command, threat, mode, guess)
            a, steer = result['plan'][0]
            if not check_command(self.state, a, steer, obstacles, s):
                continue
            if best is None or result['cost'] < best['cost']:
                best = result

        # 4. Send the safest useful command.
        if best is not None:
            a, steer = best['plan'][0]
            self.send(a, steer, mode['v_max'])
            self.previous_plan = best['plan']
            self.publish_path(best['states'])
            self.log_mode(('NMPC escape' if escaping else 'NMPC follow') + lost)
        elif escape_plan is not None:
            a, steer = escape_plan[0]
            self.send(a, steer, s['escape_speed'])
            self.previous_plan = escape_plan
            self.log_mode('ESCAPE manoeuvre (NMPC answer was not safe)')
        else:
            self.brake('no safe option')


def main(args=None):
    rclpy.init(args=args)
    node = NmpcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_publisher.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
