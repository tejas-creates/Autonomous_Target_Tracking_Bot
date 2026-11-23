"""Follow CSV waypoints with bounded speed changes for Target_AGV."""

import csv
import math
import time

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node


def load_waypoints(filename):
    with open(filename, newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not {'x', 'y'} <= set(reader.fieldnames):
            raise ValueError('Waypoint CSV needs x and y columns')
        points = [(float(row['x']), float(row['y'])) for row in reader]
    if not points:
        raise ValueError('Waypoint CSV is empty')
    return points


class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')
        self.declare_parameter('csv_file', '')
        self.declare_parameter('max_speed', 0.10)
        self.declare_parameter('max_acceleration', 0.25)
        self.declare_parameter('max_deceleration', 0.25)
        self.declare_parameter('waypoint_radius', 0.15)

        filename = self.get_parameter('csv_file').value
        self.waypoints = load_waypoints(filename)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.acceleration = float(self.get_parameter('max_acceleration').value)
        self.deceleration = float(self.get_parameter('max_deceleration').value)
        self.radius = float(self.get_parameter('waypoint_radius').value)
        if min(self.max_speed, self.acceleration, self.deceleration, self.radius) <= 0:
            raise ValueError('Speed, acceleration, deceleration and radius must be positive')

        self.index = 0
        self.speed_command = 0.0
        self.last_stamp = None
        self.last_pose_wall = time.monotonic()
        self.publisher = self.create_publisher(Twist, '/Target_AGV/cmd_vel', 10)
        self.create_subscription(
            Pose, '/Target_AGV/ground_truth_pose', self.on_pose, 10
        )
        self.create_timer(0.2, self.stop_if_pose_stale)
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {filename}')

    def publish(self, speed, yaw_rate):
        command = Twist()
        command.linear.x = speed
        command.angular.z = yaw_rate
        self.publisher.publish(command)

    def on_pose(self, message):
        now = time.monotonic()
        self.last_pose_wall = now
        dt = 0.0 if self.last_stamp is None else min(now - self.last_stamp, 0.2)
        self.last_stamp = now
        if dt <= 0.0:
            return

        pose = message
        x, y = pose.position.x, pose.position.y
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))

        while self.index < len(self.waypoints):
            goal_x, goal_y = self.waypoints[self.index]
            distance = math.hypot(goal_x - x, goal_y - y)
            if distance > self.radius:
                break
            self.index += 1

        if self.index == len(self.waypoints):
            desired_speed, steering = 0.0, 0.0
        else:
            goal_x, goal_y = self.waypoints[self.index]
            distance = math.hypot(goal_x - x, goal_y - y)
            heading = math.atan2(goal_y - y, goal_x - x)
            error = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
            steering = max(-0.5, min(0.5, error))
            stopping_distance = max(0.0, distance - self.radius)
            desired_speed = min(self.max_speed,
                                math.sqrt(2 * self.deceleration * stopping_distance))

        change = (self.acceleration if desired_speed > self.speed_command
                  else self.deceleration) * dt
        self.speed_command += max(-change, min(change, desired_speed - self.speed_command))
        yaw_rate = self.speed_command * math.tan(steering) / 0.1046
        self.publish(self.speed_command, yaw_rate)

    def stop_if_pose_stale(self):
        if time.monotonic() - self.last_pose_wall > 1.0:
            self.speed_command = 0.0
            self.last_stamp = None
            self.publish(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointController()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.publish(0.0, 0.0)
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
