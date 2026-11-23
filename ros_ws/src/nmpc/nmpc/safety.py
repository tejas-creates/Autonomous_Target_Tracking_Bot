"""Plain-Python helpers: vehicle model, braking check, threat and escape.

Nothing here uses CasADi, so these functions are quick and easy to test.

An "obstacle" in this file is a dictionary:
    {'x', 'y', 'vx', 'vy', 'R', 'moving'}
where R is the distance the AGV centre must keep from the obstacle centre
(obstacle radius + robot radius + safety margin).
A "state" is a tuple (x, y, yaw, v) in the odometry frame.
"""

import math


def model_step(state, a, steer, dt, wheel_base):
    """Kinematic bicycle model, one Euler step (same model as the solver)."""
    x, y, yaw, v = state
    x += v * math.cos(yaw) * dt
    y += v * math.sin(yaw) * dt
    yaw += v * math.tan(steer) / wheel_base * dt
    v = max(0.0, v + a * dt)
    return (x, y, yaw, v)


def simulate(state, plan, s):
    """Drive the plan [(a, steer), ...], then brake to a full stop.

    Returns a list of (time, state). The braking tail is what makes a plan
    "passively safe": we only accept plans after which we can still stop.
    """
    dt = s['dt']
    points = []
    t = 0.0
    steer = 0.0
    for a, steer in plan:
        state = model_step(state, a, steer, dt, s['wheel_base'])
        t += dt
        points.append((t, state))
    while state[3] > 1e-6:
        state = model_step(state, -s['brake_decel'], steer, dt, s['wheel_base'])
        t += dt
        points.append((t, state))
    return points


def clearances(points, obstacles, s):
    """Smallest clearance [m] to static and to moving obstacles along a path.

    Negative clearance = collision. Moving obstacles are predicted with
    constant velocity and their circle grows with time (prediction error).
    """
    static_min = moving_min = float('inf')
    for t, (x, y, _, _) in points:
        for o in obstacles:
            ox = o['x'] + o['vx'] * t
            oy = o['y'] + o['vy'] * t
            clearance = math.hypot(x - ox, y - oy) - o['R']
            if o['moving']:
                clearance -= s['prediction_growth'] * t
                moving_min = min(moving_min, clearance)
            else:
                static_min = min(static_min, clearance)
    return static_min, moving_min


def check_command(state, a, steer, obstacles, s):
    """Is "do (a, steer) for one step, then brake" collision free?"""
    points = simulate(state, [(a, steer)], s)
    static_min, moving_min = clearances(points, obstacles, s)
    return static_min >= 0.0 and moving_min >= 0.0


def threat_level(state, obstacles, s):
    """How dangerous are the moving obstacles? Returns (0..1, worst obstacle).

    For each moving obstacle we find when it will be closest to the AGV
    (time of closest approach). If it gets too close soon, the threat is high.
    """
    x, y, yaw, v = state
    agv_vx, agv_vy = v * math.cos(yaw), v * math.sin(yaw)
    threat, worst = 0.0, None
    for o in obstacles:
        if not o['moving']:
            continue
        px, py = o['x'] - x, o['y'] - y                 # relative position
        wx, wy = o['vx'] - agv_vx, o['vy'] - agv_vy     # relative velocity
        speed2 = wx * wx + wy * wy
        t_close = 0.0 if speed2 < 1e-9 else max(0.0, -(px * wx + py * wy) / speed2)
        d_close = math.hypot(px + wx * t_close, py + wy * t_close)
        if d_close < o['R'] + 0.05 and t_close < s['ttc_threshold']:
            level = 1.0 - t_close / s['ttc_threshold']
            if level > threat:
                threat, worst = level, o
    return threat, worst


def normalize(x, y):
    length = math.hypot(x, y)
    if length < 1e-9:
        return 0.0, 0.0
    return x / length, y / length


def escape_direction(state, target_xy, obstacle, weight):
    """Blend "away from the obstacle" with "towards the target".

    weight = 1 -> purely away, weight = 0 -> purely towards the target.
    """
    x, y = state[0], state[1]
    tx, ty = normalize(target_xy[0] - x, target_xy[1] - y)

    if obstacle['moving'] and math.hypot(obstacle['vx'], obstacle['vy']) > 1e-3:
        # Leave the obstacle's lane: go sideways to its path, on the side we
        # are already on, and a little towards where it came from (pass behind).
        ux, uy = normalize(obstacle['vx'], obstacle['vy'])
        nx, ny = -uy, ux
        side = 1.0 if (x - obstacle['x']) * nx + (y - obstacle['y']) * ny >= 0 else -1.0
        ax, ay = normalize(side * nx - 0.3 * ux, side * ny - 0.3 * uy)
    else:
        ax, ay = normalize(x - obstacle['x'], y - obstacle['y'])

    ex, ey = normalize(weight * ax + (1 - weight) * tx, weight * ay + (1 - weight) * ty)

    if ex == 0.0 and ey == 0.0:
        # Target is right behind the obstacle: go around it, on the target side.
        px, py = -ay, ax
        if px * tx + py * ty < 0:
            px, py = -px, -py
        ex, ey = px, py

    if ex * ax + ey * ay < 0:
        # Never point back into the obstacle: keep only the sideways part.
        dot = ex * ax + ey * ay
        ex, ey = normalize(ex - dot * ax, ey - dot * ay)
    return ex, ey


def best_escape(state, target_xy, obstacles, threat_obstacle, weight, s):
    """Try a fan of steering angles at full acceleration; return the best plan.

    Returns a plan [(a, steer), ...] or None if every option hits a static
    obstacle. Plans that may touch a moving obstacle are only used when no
    fully safe plan exists (then the one with the most clearance wins).
    """
    direction = None
    if threat_obstacle is not None:
        direction = escape_direction(state, target_xy, threat_obstacle, weight)
    steps = int(round(s['escape_time'] / s['dt']))
    limit = s['steering_limit']

    best_safe, best_safe_score = None, -float('inf')
    best_risky, best_risky_clear = None, -float('inf')
    for i in range(7):
        steer = -limit + i * limit / 3          # -0.5 ... +0.5
        plan, v = [], state[3]
        for _ in range(steps):
            a = (s['escape_speed'] - v) / s['dt']
            a = max(-s['brake_decel'], min(s['escape_accel'], a))
            v = max(0.0, v + a * s['dt'])
            plan.append((a, steer))
        points = simulate(state, plan, s)
        static_min, moving_min = clearances(points, obstacles, s)
        if static_min < 0.0:
            continue                            # never hit walls / static things

        end_x, end_y, end_yaw, _ = points[-1][1]
        score = s['w_escape_clear'] * min(moving_min, 0.3) / 0.3
        if direction is not None:
            score += s['w_dir'] * (math.cos(end_yaw) * direction[0]
                                   + math.sin(end_yaw) * direction[1])
        bearing = math.atan2(target_xy[1] - end_y, target_xy[0] - end_x) - end_yaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        if abs(bearing) < s['camera_half_fov']:
            score += s['w_escape_view']          # target still in the camera

        if moving_min >= 0.0 and score > best_safe_score:
            best_safe, best_safe_score = plan, score
        if moving_min > best_risky_clear:
            best_risky, best_risky_clear = plan, moving_min

    # Braking is also an option; it wins if it keeps more clearance.
    brake_static, brake_moving = clearances(simulate(state, [], s), obstacles, s)
    if best_safe is not None:
        return best_safe
    if brake_static >= 0.0 and brake_moving >= best_risky_clear:
        return None
    return best_risky
