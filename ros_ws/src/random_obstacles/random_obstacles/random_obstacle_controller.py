#!/usr/bin/env python3
"""Motion for the moving obstacles (garden robots).

Replaces the back-and-forth motion of ros_ws/src/dynamic_obstacle_controller.
Two modes (parameter motion):

crossing (default) - like the original controller, every obstacle shuttles back and
forth on a straight segment, but each segment crosses the Target_AGV trajectory
(docs/trajectory.csv).  The crossing points are spread along the whole route, so the
target drives through the obstacle traffic (it does not collide with the obstacles)
and the AGV behind it has to find its own way.  Each segment crosses the route at
90 +/- cross_max_angle_deg, keeps clear of the walls, the pots, the other segments and
the robot start spots, and the speed is drawn again on every pass.

random - each obstacle drives at a constant speed along a straight line segment
towards a random endpoint.  When it gets there, it picks the next endpoint within
+/-max_turn_deg (45 deg) of its current direction and keeps going, so it never stops.
An endpoint is valid only if it lies inside the garden with a margin to the walls,
and the straight leg does not pass a static obstacle (pots) too closely.
If no endpoint within +/-45 deg is valid (wall or pot ahead), or the obstacle is
about to collide with another moving obstacle, it picks any random valid point in
the whole garden (heading away from the other obstacle) and drives there.

Like the old controller, it moves the static models by teleporting them with
the /world/<name>/set_pose_vector service.  Time comes from the Gazebo clock,
so the robots pause when the simulation pauses and move at their real speed
when the real-time factor is below 1.

Run (Gazebo must be running; normally started by simulation/launch/world.launch.py):
    ros2 run random_obstacles random_obstacle_controller --ros-args -p seed:=7
    ros2 run random_obstacles random_obstacle_controller --ros-args -p motion:=random
"""

import csv
import math
import random
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GazeboNode
from gz.transport13 import SubscribeOptions


# Start positions if they cannot be read from Gazebo (same as both world files).
DEFAULT_OBSTACLES = {
    'Moving_Cylinder_01': (1.0, 4.06746940175069),
    'Moving_Cylinder_02': (2.0, 4.0),
    'Moving_Cylinder_03': (2.7641495217722, 4.14340191291121),
    'Moving_Cylinder_04': (4.58368182869024, 3.68096213347195),
    'Moving_Cylinder_05': (5.67757135101035, 1.75373810816494),
    'Moving_Cylinder_06': (4.39444506236884, 0.42654272926868),
    'Moving_Cylinder_07': (2.4, 0.0),
    'Moving_Cylinder_08': (0.84925437257673, 0.03754459685878),
    'Moving_Cylinder_09': (-1.2428006979732, 0.21601543891485),
    'Moving_Cylinder_10': (-1.77305438444233, 2.0),
    'Moving_Cylinder_11': (-0.56251075657829, 2.65627689144573),
    'Moving_Cylinder_12': (3.0, 2.2),
    'Moving_Cylinder_13': (0.8, 1.6),
    'Moving_Cylinder_14': (6.8, 4.6),
    'Moving_Cylinder_15': (6.9, -0.6),
    'Moving_Cylinder_16': (-3.0, 5.0),
    'Moving_Cylinder_17': (-3.0, -1.0),
    'Moving_Cylinder_18': (6.5, 2.6),
    'Moving_Cylinder_19': (1.0, 5.5),
    'Moving_Cylinder_20': (4.2, -1.3),
}



def find_trajectory():
    """docs/trajectory.csv of the project (searched upwards, works from src and install)."""
    for folder in Path(__file__).resolve().parents:
        candidate = folder / 'docs' / 'trajectory.csv'
        if candidate.is_file():
            return str(candidate)
    return ''


# AGV and Target_AGV spawn points (simulation/launch/world.launch.py).
DEFAULT_KEEP_CLEAR = [-0.8, 4.0, 0.0, 4.0]

# Static obstacles (x, y) the legs keep clear of.
DEFAULT_STATIC = [
    3.5, 0.0,
    3.91646629383458, 3.95615843583788,
    5.29579574182807, 2.88040766331977,
    5.21739651040941, 0.97503696357694,
    1.567105558238, 0.0,
    -0.2984938219409, 0.02240008134529,
    -1.7432329733961, 2.98037686654909,
]


class Obstacle:
    def __init__(self, name, x, y, speed):
        self.name = name
        self.x = x
        self.y = y
        self.speed = speed
        self.heading = 0.0
        self.goal_x = x
        self.goal_y = y
        self.ends = None            # crossing mode: the two segment end points
        self.goal_index = 0


def point_segment_distance(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def segments_intersect(a, b, c, d):
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return side(a, b, c) * side(a, b, d) < 0 and side(c, d, a) * side(c, d, b) < 0


def segment_distance(a, b, c, d):
    if segments_intersect(a, b, c, d):
        return 0.0
    return min(point_segment_distance(*a, *c, *d), point_segment_distance(*b, *c, *d),
               point_segment_distance(*c, *a, *b), point_segment_distance(*d, *a, *b))


def load_trajectory(filename):
    with open(filename, newline='', encoding='utf-8-sig') as stream:
        points = [(float(row['x']), float(row['y'])) for row in csv.DictReader(stream)]
    if len(points) < 2:
        raise ValueError(f'{filename}: need at least two waypoints')
    return points


class Polyline:
    def __init__(self, points):
        self.points = points
        self.starts = [0.0]
        for (ax, ay), (bx, by) in zip(points, points[1:]):
            self.starts.append(self.starts[-1] + math.hypot(bx - ax, by - ay))
        self.length = self.starts[-1]

    def point(self, s):
        s = min(max(s, 0.0), self.length)
        for i in range(len(self.points) - 1):
            seg = self.starts[i + 1] - self.starts[i]
            if s <= self.starts[i + 1] and seg > 1e-9:
                t = (s - self.starts[i]) / seg
                (ax, ay), (bx, by) = self.points[i], self.points[i + 1]
                return ax + t * (bx - ax), ay + t * (by - ay)
        return self.points[-1]

    def direction(self, s, span=0.25):
        """Heading of the route around s (smoothed over the waypoint corners)."""
        ax, ay = self.point(s - span)
        bx, by = self.point(s + span)
        return math.atan2(by - ay, bx - ax)


class RandomObstacleController(Node):
    def __init__(self):
        super().__init__('random_obstacle_controller')

        p = self.declare_parameter
        self.world_name = p('world_name', 'Autonomous_Target_Tracking_Bot').value
        self.name_prefix = p('name_prefix', 'Moving_Cylinder_').value
        # Garden operating area [x_min, x_max, y_min, y_max] (the hedge is just outside).
        self.area = list(p('area', [-4.1, 7.9, -1.9, 6.1]).value)
        self.radius = p('obstacle_radius', 0.10).value
        # Gap between the obstacle edge and the wall at any endpoint.
        self.wall_margin = p('wall_margin', 0.20).value
        self.min_leg = p('min_leg', 1.0).value
        self.max_leg = p('max_leg', 3.0).value
        self.min_speed = p('min_speed', 0.10).value
        self.max_speed = p('max_speed', 0.30).value
        self.max_turn = math.radians(p('max_turn_deg', 45.0).value)
        static = list(p('static_obstacles', DEFAULT_STATIC).value)
        self.static = list(zip(static[0::2], static[1::2]))
        # Centre distance kept from static obstacles (0.1 + 0.1 + gap).
        self.static_clearance = p('static_clearance', 0.35).value
        # random mode: two moving obstacles closer than this (centre to centre) and
        # approaching each other are "colliding"; both pick a new point in the garden.
        self.collision_distance = p('collision_distance', 0.35).value
        self.update_rate = p('update_rate', 20.0).value
        self.use_gazebo_clock = p('use_gazebo_clock', True).value
        self.face_direction = p('face_direction', True).value
        self.motion = p('motion', 'crossing').value
        if self.motion not in ('crossing', 'random'):
            raise ValueError("motion must be 'crossing' or 'random'")
        self.trajectory_file = p('trajectory_file', '').value or find_trajectory()
        # Distance of each end point from the crossing point [m] (random in this range).
        self.cross_min = p('cross_min_half_length', 0.6).value
        self.cross_max = p('cross_max_half_length', 1.2).value
        self.cross_max_angle = math.radians(p('cross_max_angle_deg', 30.0).value)
        # No crossings on the first / last metres of the route (robots start and stop there).
        self.cross_skip_start = p('cross_skip_start', 1.2).value
        self.cross_skip_end = p('cross_skip_end', 1.0).value
        # Minimum gap between two crossing segments (obstacle diameter + margin).
        self.segment_gap = p('segment_gap', 0.30).value
        keep = list(p('keep_clear_points', DEFAULT_KEEP_CLEAR).value)
        self.keep_clear = list(zip(keep[0::2], keep[1::2]))
        self.keep_clear_radius = p('keep_clear_radius', 0.6).value
        seed = p('seed', -1).value
        self.rng = random.Random(None if seed < 0 else seed)

        if self.motion == 'crossing' and not self.trajectory_file:
            raise ValueError('docs/trajectory.csv not found: set the trajectory_file parameter')
        self.route = Polyline(load_trajectory(self.trajectory_file)) if self.motion == 'crossing' else None

        self.service_name = f'/world/{self.world_name}/set_pose_vector'
        self.gazebo = GazeboNode()
        self.lock = threading.Lock()
        self.sim_time = None
        self.start_poses = {}
        # The clock is published at 1 kHz; throttle it so the Python callback does not
        # starve the service replies (that made ~30 % of the pose requests time out).
        clock_options = SubscribeOptions()
        clock_options.msgs_per_sec = 50
        if self.use_gazebo_clock:
            self.gazebo.subscribe(Clock, f'/world/{self.world_name}/clock', self.on_clock, clock_options)
        self.pose_topic = f'/world/{self.world_name}/pose/info'
        self.gazebo.subscribe(Pose_V, self.pose_topic, self.on_pose_info)

        self.obstacles = []
        self.last_time = None
        self.ready_since = None
        self.last_warning = 0.0
        self.last_old_check = 0.0
        self.failed_requests = 0
        self.total_requests = 0
        self.last_failure_report = 0.0

        self.create_timer(1.0 / self.update_rate, self.update)
        self.get_logger().info(f'Obstacle controller started ({self.motion} motion)')

    # ----------------------------------------------------------- Gazebo inputs
    def on_clock(self, msg):
        with self.lock:
            self.sim_time = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_pose_info(self, msg):
        if self.start_poses:
            return
        poses = {pose.name: (pose.position.x, pose.position.y)
                 for pose in msg.pose if pose.name.startswith(self.name_prefix)}
        with self.lock:
            if poses and not self.start_poses:
                self.start_poses = poses

    def now(self):
        if not self.use_gazebo_clock:
            return time.monotonic()
        with self.lock:
            return self.sim_time

    # ----------------------------------------------------------- geometry
    def inside(self, x, y, margin):
        x0, x1, y0, y1 = self.area
        return x0 + margin <= x <= x1 - margin and y0 + margin <= y <= y1 - margin

    def wall_distance(self, x, y):
        x0, x1, y0, y1 = self.area
        return min(x - x0, x1 - x, y - y0, y1 - y) - self.radius

    def leg_is_valid(self, sx, sy, ex, ey):
        if not self.inside(ex, ey, self.radius + self.wall_margin):
            return False
        for px, py in self.static:
            if math.hypot(ex - px, ey - py) < self.static_clearance:
                return False
            start = math.hypot(sx - px, sy - py)
            # Never closer than the clearance (or than now, if already closer).
            limit = min(self.static_clearance, start - 1e-6)
            if point_segment_distance(px, py, sx, sy, ex, ey) < limit:
                return False
        return True

    def try_legs(self, ob, centre, spread, min_leg, max_leg, tries=60):
        for _ in range(tries):
            angle = centre + self.rng.uniform(-spread, spread)
            length = self.rng.uniform(min_leg, max_leg)
            ex = ob.x + length * math.cos(angle)
            ey = ob.y + length * math.sin(angle)
            if self.leg_is_valid(ob.x, ob.y, ex, ey):
                return ex, ey, angle
        return None

    # ----------------------------------------------------------- crossing mode
    def crossing_gap(self, a, b, others):
        """Smallest gap to the other segments, or None if the segment is not allowed."""
        margin = self.radius + self.wall_margin
        if not (self.inside(*a, margin) and self.inside(*b, margin)):
            return None
        if any(point_segment_distance(px, py, *a, *b) < self.static_clearance for px, py in self.static):
            return None
        if any(point_segment_distance(px, py, *a, *b) < self.keep_clear_radius for px, py in self.keep_clear):
            return None
        return min((segment_distance(a, b, c, d) for c, d in others), default=math.inf)

    def make_crossings(self, count):
        """One segment per obstacle, crossing points spread evenly along the route."""
        first = self.cross_skip_start
        last = self.route.length - self.cross_skip_end
        spacing = (last - first) / max(count, 1)
        segments = []                              # (a, b, s) or None, one per obstacle
        placed = []                                # (a, b) of the segments found so far
        for i in range(count):
            centre = first + (i + 0.5) * spacing
            best, best_gap = None, -1.0
            for attempt in range(300):
                jitter = spacing * (0.3 if attempt < 100 else 0.5)   # widen the search later
                s = min(max(centre + self.rng.uniform(-jitter, jitter), first), last)
                px, py = self.route.point(s)
                angle = (self.route.direction(s) + math.pi / 2
                         + self.rng.uniform(-self.cross_max_angle, self.cross_max_angle))
                ca, sa = math.cos(angle), math.sin(angle)
                shrink = 1.0 if attempt < 150 else 0.6               # tight spots: shorter
                back = shrink * self.rng.uniform(self.cross_min, self.cross_max)
                front = shrink * self.rng.uniform(self.cross_min, self.cross_max)
                a = (px - back * ca, py - back * sa)
                b = (px + front * ca, py + front * sa)
                gap = self.crossing_gap(a, b, placed)
                if gap is not None and gap > best_gap:
                    best, best_gap = (a, b, s), gap
                    if gap >= self.segment_gap:
                        break
            # If no segment keeps segment_gap, the one with the largest gap is used.
            if best is not None:
                placed.append(best[:2])
            segments.append(best)
        return segments

    def choose_goal(self, ob, keep_direction):
        """Next endpoint; within +/-max_turn of the heading if keep_direction."""
        if ob.ends is not None:                    # crossing: turn round, new speed
            ob.goal_index = 1 - ob.goal_index
            ob.goal_x, ob.goal_y = ob.ends[ob.goal_index]
            ob.heading = math.atan2(ob.goal_y - ob.y, ob.goal_x - ob.x)
            ob.speed = self.rng.uniform(self.min_speed, self.max_speed)
            return
        short = (min(0.4, self.min_leg), self.min_leg)
        attempts = []
        if keep_direction:
            attempts += [(ob.heading, self.max_turn, self.min_leg, self.max_leg),
                         (ob.heading, self.max_turn, *short)]
        for centre, spread, lo, hi in attempts:
            result = self.try_legs(ob, centre, spread, lo, hi)
            if result:
                ob.goal_x, ob.goal_y, ob.heading = result
                return
        # Blocked within +/-45 deg (wall, corner or pot): any point in the garden.
        if self.random_point_goal(ob):
            return
        # Boxed in (should not happen): head for the middle of the garden.
        x0, x1, y0, y1 = self.area
        ob.goal_x, ob.goal_y = (x0 + x1) / 2, (y0 + y1) / 2
        ob.heading = math.atan2(ob.goal_y - ob.y, ob.goal_x - ob.x)
        self.get_logger().warning(f'{ob.name}: no valid leg found, heading to the centre')

    def random_point_goal(self, ob, away_x=None, away_y=None, tries=200):
        """Goal = random valid point anywhere in the garden.

        With away_x/away_y the new direction must point away from that position
        (the obstacle we are about to hit)."""
        x0, x1, y0, y1 = self.area
        m = self.radius + self.wall_margin
        for _ in range(tries):
            ex = self.rng.uniform(x0 + m, x1 - m)
            ey = self.rng.uniform(y0 + m, y1 - m)
            if math.hypot(ex - ob.x, ey - ob.y) < self.min_leg:
                continue
            if away_x is not None and (ex - ob.x) * (ob.x - away_x) + (ey - ob.y) * (ob.y - away_y) <= 0:
                continue
            if self.leg_is_valid(ob.x, ob.y, ex, ey):
                ob.goal_x, ob.goal_y = ex, ey
                ob.heading = math.atan2(ey - ob.y, ex - ob.x)
                return True
        return False

    def avoid_collisions(self):
        """random mode: obstacles about to touch each other both take a new random point."""
        free = [ob for ob in self.obstacles if ob.ends is None]
        for i, a in enumerate(free):
            for b in free[i + 1:]:
                dx, dy = b.x - a.x, b.y - a.y
                if math.hypot(dx, dy) >= self.collision_distance:
                    continue
                # Closing speed along the line between them (> 0: getting closer).
                closing = (math.cos(a.heading) * a.speed - math.cos(b.heading) * b.speed) * dx \
                    + (math.sin(a.heading) * a.speed - math.sin(b.heading) * b.speed) * dy
                if closing <= 0:
                    continue
                for ob, other in ((a, b), (b, a)):
                    if not self.random_point_goal(ob, other.x, other.y):
                        self.random_point_goal(ob)

    # ----------------------------------------------------------- motion
    def advance(self, ob, distance):
        for _ in range(10):                       # carries over to the next leg, no stop
            dx, dy = ob.goal_x - ob.x, ob.goal_y - ob.y
            remaining = math.hypot(dx, dy)
            if remaining > distance:
                ob.x += dx / remaining * distance
                ob.y += dy / remaining * distance
                break
            ob.x, ob.y = ob.goal_x, ob.goal_y
            distance -= remaining
            self.choose_goal(ob, keep_direction=True)
        # Wall reached / approached while heading into it (e.g. after a manual move).
        if ob.ends is None and self.wall_distance(ob.x, ob.y) < self.wall_margin * 0.5:
            x0, x1, y0, y1 = self.area
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            if (ob.goal_x - ob.x) * (cx - ob.x) + (ob.goal_y - ob.y) * (cy - ob.y) < 0:
                self.choose_goal(ob, keep_direction=False)
        # Hard guarantee: stay inside the garden.
        x0, x1, y0, y1 = self.area
        ob.x = min(max(ob.x, x0 + self.radius), x1 - self.radius)
        ob.y = min(max(ob.y, y0 + self.radius), y1 - self.radius)

    def create_obstacles(self):
        poses = self.start_poses or DEFAULT_OBSTACLES
        if not self.start_poses:
            self.get_logger().warning('No poses from Gazebo, using the default start positions')
        self.obstacles = []
        names = sorted(poses)
        crossings = self.make_crossings(len(names)) if self.motion == 'crossing' else [None] * len(names)
        for name, crossing in zip(names, crossings):
            x, y = poses[name]
            ob = Obstacle(name, x, y, self.rng.uniform(self.min_speed, self.max_speed))
            ob.heading = self.rng.uniform(-math.pi, math.pi)
            if crossing is not None:
                a, b, s = crossing
                t = self.rng.uniform(0.0, 1.0)     # random start point on the segment
                ob.x, ob.y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
                ob.ends = (a, b)
                ob.goal_index = self.rng.choice((0, 1))
                ob.goal_x, ob.goal_y = ob.ends[ob.goal_index]
                ob.heading = math.atan2(ob.goal_y - ob.y, ob.goal_x - ob.x)
                self.get_logger().info(
                    f'{name}: crosses the route at {s:.1f} m ({a[0]:.2f}, {a[1]:.2f}) -> '
                    f'({b[0]:.2f}, {b[1]:.2f})')
            else:
                if self.motion == 'crossing':
                    self.get_logger().warning(f'{name}: no free crossing found, moving randomly')
                self.choose_goal(ob, keep_direction=False)
            self.obstacles.append(ob)
        self.get_logger().info(
            f'Moving {len(self.obstacles)} obstacles ({self.motion}): '
            + ', '.join(f'{o.name[-2:]}@{o.speed:.2f}m/s' for o in self.obstacles))

    def send_poses(self):
        request = Pose_V()
        for ob in self.obstacles:
            pose = request.pose.add()
            pose.name = ob.name
            pose.position.x = ob.x
            pose.position.y = ob.y
            pose.position.z = 0.15
            yaw = ob.heading if self.face_direction else 0.0
            pose.orientation.z = math.sin(yaw / 2)
            pose.orientation.w = math.cos(yaw / 2)
        sent, reply = self.gazebo.request(self.service_name, request, Pose_V, Boolean, 50)
        self.total_requests += 1
        # A lost update is harmless: the next one sets the correct pose.
        if not sent or not reply.data:
            self.failed_requests += 1
        wall = time.monotonic()
        if self.failed_requests and wall - self.last_failure_report > 10.0:
            self.get_logger().warning(
                f'{self.failed_requests}/{self.total_requests} pose updates were not acknowledged '
                'in the last 10 s')
            self.failed_requests = self.total_requests = 0
            self.last_failure_report = wall

    def warn_about_old_controller(self, wall):
        if wall - self.last_old_check < 10.0:
            return
        self.last_old_check = wall
        if 'dynamic_obstacle_controller' in self.get_node_names():
            self.get_logger().warning(
                'dynamic_obstacle_controller is also running and moves the same models; '
                'remove it from ros_ws/launch/simulation.launch.py')

    def update(self):
        wall = time.monotonic()
        self.warn_about_old_controller(wall)
        now = self.now()
        if now is None or self.service_name not in self.gazebo.service_list():
            if wall - self.last_warning > 3.0:
                self.get_logger().warning('Waiting for the Gazebo world')
                self.last_warning = wall
            return

        if not self.obstacles:
            # Give pose/info a moment so the robots start where the world put them.
            if self.ready_since is None:
                self.ready_since = wall
            if not self.start_poses and wall - self.ready_since < 3.0:
                return
            self.create_obstacles()
            self.gazebo.unsubscribe(self.pose_topic)
            self.last_time = now
            self.last_failure_report = wall
            self.send_poses()
            return

        dt = now - self.last_time
        if dt < 0:                                 # world was reset
            self.get_logger().info('Simulation time went back, restarting from the start positions')
            self.last_time = now
            self.create_obstacles()
            self.send_poses()
            return
        if dt == 0:                                # paused
            return
        self.last_time = now
        dt = min(dt, 0.25)                         # no jumps after a stall
        self.avoid_collisions()
        for ob in self.obstacles:
            self.advance(ob, ob.speed * dt)
        self.send_poses()


def main(args=None):
    rclpy.init(args=args)
    node = RandomObstacleController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
