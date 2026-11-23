#!/usr/bin/env python3
"""Time the NMPC solver without Gazebo, sensors or ROS topics.

Runs the real solver (nmpc/solver.py) for a few made-up scenes, the same way the
node does: warm start from the last plan, AGV/target/obstacles moved every cycle.
Use it to check that the Raspberry Pi is fast enough (Phase 1, Step 9).

    source ros_ws/install/setup.bash
    ros_ws/.venv_nmpc/bin/python scripts/nmpc_benchmark.py
    ros_ws/.venv_nmpc/bin/python scripts/nmpc_benchmark.py --cycles 200 --config ros_ws/config/nmpc.yaml
"""

import argparse
import math
import random
import statistics
from pathlib import Path

import yaml

from nmpc.nmpc_node import DEFAULTS
from nmpc.safety import model_step
from nmpc.solver import NmpcSolver

REPO = Path(__file__).resolve().parents[1]


def load_settings(path):
    s = dict(DEFAULTS)
    if path and Path(path).exists():
        data = yaml.safe_load(Path(path).read_text()) or {}
        s.update(data.get('nmpc_controller', {}).get('ros__parameters', {}))
    return s


def obstacle(s, x, y, vx=0.0, vy=0.0, radius=0.08):
    return {'x': x, 'y': y, 'vx': vx, 'vy': vy,
            'R': radius + s['robot_radius'] + s['safety_margin'],
            'moving': math.hypot(vx, vy) > 0.04}


def target_obstacle(s, state, target):
    """The target body as an obstacle, placed as the node does it."""
    tx, ty, tvx, tvy = target
    ux, uy = tx - state[0], ty - state[1]
    length = max(math.hypot(ux, uy), 1e-6)
    offset = s['target_center_offset']
    return {'x': tx + offset * ux / length, 'y': ty + offset * uy / length,
            'vx': tvx, 'vy': tvy,
            'R': s['target_radius'] + s['robot_radius'] + s['target_clearance'],
            'moving': math.hypot(tvx, tvy) > 0.04}


def scenes(s):
    """name -> (target [x, y, vx, vy], obstacle list, escape mode?)"""
    static6 = [obstacle(s, 1.2, 0.6), obstacle(s, 1.5, -0.5), obstacle(s, 2.2, 0.2),
               obstacle(s, 0.6, 0.9), obstacle(s, 0.8, -0.9), obstacle(s, 2.8, -0.3)]
    crowded = static6 + [obstacle(s, 1.4, 1.2, 0.0, -0.2), obstacle(s, 2.0, -1.2, 0.0, 0.25),
                         obstacle(s, 2.6, 1.0, -0.1, -0.15), obstacle(s, 3.0, 0.0, -0.2, 0.0),
                         obstacle(s, 1.0, 1.6), obstacle(s, 1.0, -1.6)]
    attacker = [obstacle(s, 1.2, 0.1, -0.25, 0.0)] + static6[:3]
    return {
        'follow, open floor': ([0.8, 0.0, 0.10, 0.0], [], False),
        'follow, 6 static obstacles': ([0.8, 0.0, 0.10, 0.0], static6, False),
        'crowded, 10 static + 4 moving': ([0.8, 0.0, 0.10, 0.0], crowded, False),
        'escape mode, obstacle driving at AGV': ([0.8, 0.4, 0.10, 0.0], attacker, True),
    }


def run_scene(solver, s, target, obstacles, escaping, cycles, rng):
    dt = s['dt']
    state = (0.0, 0.0, 0.0, 0.05)
    prev, plan = (0.0, 0.0), []
    if escaping:
        mode = {'v_max': s['escape_speed'], 'a_max': s['escape_accel'],
                'effort': s['escape_effort_scale'], 'gap': s['escape_gap_scale'],
                'view': s['escape_view_scale']}
        threat = 0.8
    else:
        mode = {'v_max': s['max_speed'], 'a_max': s['max_accel'],
                'effort': 1.0, 'gap': 1.0, 'view': 1.0}
        threat = 0.0
    target = list(target)
    obstacles = [dict(o) for o in obstacles]
    times, solved = [], 0

    for _ in range(cycles):
        # Sensor noise, like a real LiDAR / camera.
        seen = [dict(o, x=o['x'] + rng.gauss(0, 0.01), y=o['y'] + rng.gauss(0, 0.01))
                for o in obstacles]
        seen.append(target_obstacle(s, state, target))
        seen.sort(key=lambda o: math.hypot(o['x'] - state[0], o['y'] - state[1]) - o['R'])

        result = solver.solve(state, target, seen[:s['max_obstacles']], prev, threat, mode,
                              plan[1:] if plan else [])
        times.append(result['time'] * 1000.0)
        solved += result['ok']
        plan = result['plan']
        prev = plan[0]

        # Move everything one step.
        state = model_step(state, prev[0], prev[1], dt, s['wheel_base'])
        target[0] += target[2] * dt
        target[1] += target[3] * dt
        for o in obstacles:
            o['x'] += o['vx'] * dt
            o['y'] += o['vy'] * dt
    return times, solved


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--cycles', type=int, default=100, help='solves per scene')
    parser.add_argument('--config', default=str(REPO / 'ros_ws/config/nmpc.yaml'))
    args = parser.parse_args()

    s = load_settings(args.config)
    print(f"Building the NMPC problem ({s['horizon']} s horizon, dt {s['dt']} s, "
          f"{s['max_obstacles']} obstacles, time limit {s['solver_max_time'] * 1000:.0f} ms)...")
    solver = NmpcSolver(s)
    rng = random.Random(1)
    budget = s['dt'] * 1000.0

    print(f"\n{'scene':40s} {'mean':>7s} {'p95':>7s} {'max':>7s} {'solved':>7s}")
    all_times = []
    for name, (target, obstacles, escaping) in scenes(s).items():
        times, solved = run_scene(solver, s, target, obstacles, escaping, args.cycles, rng)
        all_times += times
        p95 = sorted(times)[int(0.95 * (len(times) - 1))]
        print(f"{name:40s} {statistics.mean(times):6.1f}ms {p95:6.1f}ms {max(times):6.1f}ms "
              f"{100.0 * solved / len(times):6.0f}%")

    mean, worst = statistics.mean(all_times), max(all_times)
    print(f"\nAll scenes: mean {mean:.1f} ms, worst {worst:.1f} ms (control period {budget:.0f} ms)")
    if mean < 50 and worst < 100:
        print('OK: fast enough with room for the camera and LiDAR nodes.')
    elif mean < budget:
        print('TIGHT: works, but check again with the full system running (Phase 7).')
    else:
        print('TOO SLOW: reduce steps (larger dt) or max_obstacles (Phase 7).')


if __name__ == '__main__':
    main()
