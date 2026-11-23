#!/usr/bin/env python3
"""Turn LiDAR scans into a list of circular obstacles (moving or static).

Steps done for every scan:
  1. Convert the scan ranges into (x, y) points around the AGV.
  2. Group neighbouring points into clusters (a gap bigger than cluster_gap
     starts a new cluster).
  3. Small cluster  -> one circle (cylinders, poles, ...).
     Long cluster   -> a chain of small circles (walls, sides of a car).
  4. Move the circles from the AGV frame into the odometry frame, so that
     a static obstacle keeps the same position while the AGV drives.
  5. Match the small circles with the ones from the last scan to estimate
     their velocity, and call them moving if they are fast enough.
  6. Publish everything as an ObstacleArray (plus RViz markers).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from obstacle_msgs.msg import Obstacle, ObstacleArray


def mean_point(points):
    x = sum(p[0] for p in points) / len(points)
    y = sum(p[1] for p in points) / len(points)
    return x, y


def farthest_distance(points, centre):
    """Largest distance from centre to any point of the cluster."""
    return max(math.hypot(p[0] - centre[0], p[1] - centre[1]) for p in points)


def make_clusters(points, cluster_gap):
    """Split an ordered list of points wherever two neighbours are far apart.

    points is a list of (x, y, ray_index). Missing rays (no return) also
    split clusters, because we do not know what is in between.
    """
    clusters = []
    current = []
    for point in points:
        if current:
            last = current[-1]
            gap = math.hypot(point[0] - last[0], point[1] - last[1])
            if gap > cluster_gap or point[2] != last[2] + 1:
                clusters.append(current)
                current = []
        current.append(point)
    if current:
        clusters.append(current)
    return clusters


def split_long_cluster(cluster, piece_length):
    """Cut a long cluster (wall) into pieces about piece_length long."""
    pieces = []
    current = [cluster[0]]
    for point in cluster[1:]:
        start = current[0]
        if math.hypot(point[0] - start[0], point[1] - start[1]) > piece_length:
            pieces.append(current)
            current = []
        current.append(point)
    pieces.append(current)
    return pieces


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # ---- Parameters (can be changed from the launch file) ----
        self.scan_topic = self.declare_parameter('scan_topic', '/AGV/lidar/scan').value
        self.odom_topic = self.declare_parameter('odom_topic', '/AGV/odometry').value
        self.output_topic = self.declare_parameter('output_topic', '/obstacles').value
        self.max_range = self.declare_parameter('max_range', 3.0).value
        self.cluster_gap = self.declare_parameter('cluster_gap', 0.12).value
        self.min_points = self.declare_parameter('min_points', 3).value
        self.max_circle_size = self.declare_parameter('max_circle_size', 0.35).value
        self.wall_piece_length = self.declare_parameter('wall_piece_length', 0.15).value
        self.radius_margin = self.declare_parameter('radius_margin', 0.03).value
        self.match_distance = self.declare_parameter('match_distance', 0.20).value
        self.velocity_filter = self.declare_parameter('velocity_filter', 0.65).value
        self.moving_speed = self.declare_parameter('moving_speed', 0.04).value
        self.min_hits = self.declare_parameter('min_hits', 3).value
        self.track_timeout = self.declare_parameter('track_timeout', 0.5).value
        self.publish_markers = self.declare_parameter('publish_markers', True).value

        # Latest AGV pose from odometry: x, y, yaw.
        self.pose = None
        self.odom_frame = 'odom'

        # Tracked small obstacles. Each one is a dictionary.
        self.tracks = []
        self.next_id = 0

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data
        )
        self.obstacle_publisher = self.create_publisher(ObstacleArray, self.output_topic, 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.output_topic + '/markers', 10
        )
        self.get_logger().info(
            f'Reading {self.scan_topic} and {self.odom_topic}, '
            f'publishing {self.output_topic}'
        )

    # ------------------------------------------------------------------
    def on_odom(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        if message.header.frame_id:
            self.odom_frame = message.header.frame_id

    def to_odom_frame(self, x, y):
        """Convert a point from the AGV (LiDAR) frame to the odometry frame."""
        agv_x, agv_y, yaw = self.pose
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        return (agv_x + cos_yaw * x - sin_yaw * y,
                agv_y + sin_yaw * x + cos_yaw * y)

    # ------------------------------------------------------------------
    def on_scan(self, scan):
        if self.pose is None:
            self.get_logger().warning('Waiting for odometry...', throttle_duration_sec=3.0)
            return
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9

        # 1. Ranges -> points in the LiDAR frame (LiDAR sits right above base_link).
        points = []
        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > min(scan.range_max, self.max_range):
                continue
            angle = scan.angle_min + i * scan.angle_increment
            points.append((distance * math.cos(angle), distance * math.sin(angle), i))

        # 2. Group neighbouring points.
        clusters = make_clusters(points, self.cluster_gap)

        # A 360 degree scan starts and ends behind the AGV, so one object can be
        # cut in two there. Join the first and last cluster if they touch.
        if len(clusters) > 1:
            first, last = clusters[0][0], clusters[-1][-1]
            full_circle = first[2] == 0 and last[2] == len(scan.ranges) - 1
            close = math.hypot(first[0] - last[0], first[1] - last[1]) <= self.cluster_gap
            if full_circle and close:
                clusters[0] = clusters.pop() + clusters[0]

        # 3. Clusters -> circles.
        small_circles = []   # (x, y, radius) in odom frame, will be tracked
        wall_circles = []    # (x, y, radius) in odom frame, always static
        for cluster in clusters:
            if len(cluster) < self.min_points:
                continue  # probably noise
            centre = mean_point(cluster)
            half_size = farthest_distance(cluster, centre)

            if 2 * half_size <= self.max_circle_size:
                # The LiDAR only sees the front of an object, so the mean of
                # the points is in front of the real centre. Push it back a bit
                # (for a circle the arc mean is ~0.64 * radius in front).
                distance = math.hypot(centre[0], centre[1])
                push = 0.64 * half_size
                x = centre[0] + push * centre[0] / distance
                y = centre[1] + push * centre[1] / distance
                radius = max(half_size, 0.03) + self.radius_margin
                small_circles.append((*self.to_odom_frame(x, y), radius))
            else:
                for piece in split_long_cluster(cluster, self.wall_piece_length):
                    piece_centre = mean_point(piece)
                    radius = farthest_distance(piece, piece_centre) + self.radius_margin
                    wall_circles.append((*self.to_odom_frame(*piece_centre), radius))

        # 4. + 5. Track small circles and decide moving / static.
        self.update_tracks(small_circles, stamp)

        # 6. Publish.
        self.publish(scan.header.stamp, wall_circles)

    # ------------------------------------------------------------------
    def update_tracks(self, circles, stamp):
        # Forget obstacles we have not seen for a while.
        self.tracks = [t for t in self.tracks if stamp - t['stamp'] <= self.track_timeout]

        # List every (track, circle) pair that is close enough, nearest first.
        pairs = []
        for t_index, track in enumerate(self.tracks):
            dt = stamp - track['stamp']
            predicted_x = track['x'] + track['vx'] * dt
            predicted_y = track['y'] + track['vy'] * dt
            for c_index, (x, y, _) in enumerate(circles):
                error = math.hypot(x - predicted_x, y - predicted_y)
                if error <= self.match_distance:
                    pairs.append((error, t_index, c_index))
        pairs.sort()

        used_tracks = set()
        used_circles = set()
        for _, t_index, c_index in pairs:
            if t_index in used_tracks or c_index in used_circles:
                continue
            used_tracks.add(t_index)
            used_circles.add(c_index)

            track = self.tracks[t_index]
            x, y, radius = circles[c_index]
            dt = stamp - track['stamp']
            if dt > 0:
                # Smooth the velocity, because single measurements are noisy.
                a = self.velocity_filter
                track['vx'] = a * (x - track['x']) / dt + (1 - a) * track['vx']
                track['vy'] = a * (y - track['y']) / dt + (1 - a) * track['vy']
            track['x'], track['y'], track['radius'] = x, y, radius
            track['stamp'] = stamp
            track['hits'] += 1

        # Circles with no match are new obstacles.
        for c_index, (x, y, radius) in enumerate(circles):
            if c_index not in used_circles:
                self.tracks.append({
                    'id': self.next_id, 'x': x, 'y': y, 'vx': 0.0, 'vy': 0.0,
                    'radius': radius, 'stamp': stamp, 'hits': 1,
                })
                self.next_id += 1

    def is_moving(self, track):
        speed = math.hypot(track['vx'], track['vy'])
        return track['hits'] >= self.min_hits and speed > self.moving_speed

    # ------------------------------------------------------------------
    def publish(self, stamp, wall_circles):
        message = ObstacleArray()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame

        for track in self.tracks:
            obstacle = Obstacle()
            obstacle.id = track['id']
            obstacle.x, obstacle.y = track['x'], track['y']
            obstacle.radius = track['radius']
            obstacle.hits = track['hits']
            obstacle.is_moving = self.is_moving(track)
            if obstacle.is_moving:
                obstacle.vx, obstacle.vy = track['vx'], track['vy']
            message.obstacles.append(obstacle)

        for x, y, radius in wall_circles:
            obstacle = Obstacle()
            obstacle.id = -1
            obstacle.x, obstacle.y, obstacle.radius = x, y, radius
            obstacle.is_wall = True
            message.obstacles.append(obstacle)

        self.obstacle_publisher.publish(message)
        if self.publish_markers:
            self.publish_marker_array(message)

    def publish_marker_array(self, message):
        """Draw the obstacles in RViz: red = moving, blue = static, grey = wall."""
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for number, obstacle in enumerate(message.obstacles):
            marker = Marker()
            marker.header = message.header
            marker.ns = 'obstacles'
            marker.id = number
            marker.type = Marker.CYLINDER
            marker.pose.position.x = obstacle.x
            marker.pose.position.y = obstacle.y
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 2 * obstacle.radius
            marker.scale.z = 0.1
            marker.color.a = 0.6
            if obstacle.is_moving:
                marker.color.r = 1.0
            elif obstacle.is_wall:
                marker.color.r = marker.color.g = marker.color.b = 0.5
            else:
                marker.color.b = 1.0
            markers.markers.append(marker)

        self.marker_publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
