"""Move the cylinder obstacles back and forth in the Gazebo world."""

import math
import random
import time

import rclpy
from rclpy.node import Node

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GazeboNode


# Each item contains the start point, end point and initial point of one obstacle.
OBSTACLE_PATHS = {
    'Moving_Cylinder_01': ((1.0, 4.8), (1.0, 3.2), (1.0, 4.06746940175069)),
    'Moving_Cylinder_02': ((1.6, 3.2), (2.4, 4.8), (2.0, 4.0)),
    'Moving_Cylinder_03': ((2.6, 4.8), (3.0, 3.2), (2.7641495217722, 4.14340191291121)),
    'Moving_Cylinder_04': ((4.0, 3.0), (5.2, 4.4), (4.58368182869024, 3.68096213347195)),
    'Moving_Cylinder_05': ((4.2, 2.0), (6.6, 1.6), (5.67757135101035, 1.75373810816494)),
    'Moving_Cylinder_06': ((5.2, -0.2), (3.4, 1.2), (4.39444506236884, 0.42654272926868)),
    'Moving_Cylinder_07': ((2.4, -0.8), (2.4, 0.8), (2.4, 0.0)),
    'Moving_Cylinder_08': ((1.4, -0.8), (0.19962894637163, 1.02546032166582), (0.84925437257673, 0.03754459685878)),
    'Moving_Cylinder_09': ((-0.6, 1.0), (-1.67815116982149, -0.31495481308763), (-1.2428006979732, 0.21601543891485)),
    'Moving_Cylinder_10': ((-3.0, 2.0), (-1.0, 2.0), (-1.77305438444233, 2.0)),
    'Moving_Cylinder_11': ((-1.5, 5.0), (-0.5, 2.5), (-0.56251075657829, 2.65627689144573)),
}


class DynamicObstacleController(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_controller')

        self.declare_parameter('world_name', 'Autonomous_Target_Tracking_Bot')
        world_name = self.get_parameter('world_name').value

        self.service_name = f'/world/{world_name}/set_pose_vector'
        self.gazebo = GazeboNode()
        self.start_time = None
        self.last_warning = 0.0

        self.speeds = {
            name: random.uniform(0.1, 0.3) for name in OBSTACLE_PATHS
        }

        self.create_timer(0.05, self.move_obstacles)
        self.get_logger().info('Dynamic obstacle controller started')

    def get_position(self, start, end, initial, speed, elapsed):
        """Return a point which repeatedly travels between two endpoints."""
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        path_length = math.hypot(dx, dy)
        direction_x = dx / path_length
        direction_y = dy / path_length

        initial_distance = (
            (initial[0] - start[0]) * direction_x
            + (initial[1] - start[1]) * direction_y
        )

        distance = (initial_distance + speed * elapsed) % (2 * path_length)
        if distance > path_length:
            distance = 2 * path_length - distance

        x = start[0] + distance * direction_x
        y = start[1] + distance * direction_y
        return x, y

    def move_obstacles(self):
        if self.service_name not in self.gazebo.service_list():
            now = time.monotonic()
            if now - self.last_warning > 3.0:
                self.get_logger().warning('Waiting for the Gazebo world')
                self.last_warning = now
            return

        if self.start_time is None:
            self.start_time = time.monotonic()

        elapsed = time.monotonic() - self.start_time
        request = Pose_V()

        for name, (start, end, initial) in OBSTACLE_PATHS.items():
            x, y = self.get_position(
                start, end, initial, self.speeds[name], elapsed
            )

            pose = request.pose.add()
            pose.name = name
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.15
            pose.orientation.w = 1.0

        sent, reply = self.gazebo.request(
            self.service_name, request, Pose_V, Boolean, 100
        )

        if not sent or not reply.data:
            self.get_logger().warning('Could not update the obstacle positions')


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
