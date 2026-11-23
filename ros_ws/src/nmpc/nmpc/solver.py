"""The NMPC optimisation problem, written with casadi.Opti.

The problem is built once. Every control cycle only the parameters change
(AGV state, target, obstacles, threat, mode), so solving stays fast.

State   X = [x, y, yaw, v]       (odometry frame)
Control U = [a, steer]          (acceleration, steering angle)
"""

import math
import time

import casadi as ca

from .safety import model_step


class NmpcSolver:
    def __init__(self, s):
        self.s = s
        self.N = N = int(round(s['horizon'] / s['dt']))
        self.K = K = int(s['max_obstacles'])
        dt, L = s['dt'], s['wheel_base']

        opti = ca.Opti()
        self.opti = opti

        # ---- Decision variables ----
        self.X = X = opti.variable(4, N + 1)
        self.U = U = opti.variable(2, N)
        self.S = S = opti.variable(N)        # slack for "soft" moving-obstacle rules

        # ---- Parameters (set before every solve) ----
        self.p_state = x0 = opti.parameter(4)
        self.p_target = target = opti.parameter(4)            # x, y, vx, vy
        self.p_obstacles = obs = opti.parameter(6, K)         # x, y, vx, vy, R, moving
        self.p_prev = prev = opti.parameter(2)                # last command (a, steer)
        self.p_threat = threat = opti.parameter()
        self.p_vmax = v_max = opti.parameter()
        self.p_amax = a_max = opti.parameter()
        self.p_effort = effort_scale = opti.parameter()      # mode weight scales
        self.p_gap = gap_scale = opti.parameter()
        self.p_view = view_scale = opti.parameter()

        d = s['desired_distance']
        cost = 0
        opti.subject_to(X[:, 0] == x0)

        for k in range(N):
            t = (k + 1) * dt
            a, steer = U[0, k], U[1, k]
            state = X[:, k]

            # 1. Vehicle model (kinematic bicycle, Euler step).
            opti.subject_to(X[:, k + 1] == ca.vertcat(
                state[0] + state[3] * ca.cos(state[2]) * dt,
                state[1] + state[3] * ca.sin(state[2]) * dt,
                state[2] + state[3] * ca.tan(steer) / L * dt,
                state[3] + a * dt,
            ))

            # 2. Limits. If we start faster than v_max (end of escape mode),
            #    allow it while braking down, otherwise the problem is infeasible.
            opti.subject_to(opti.bounded(-s['brake_decel'], a, a_max))
            opti.subject_to(opti.bounded(-s['steering_limit'], steer, s['steering_limit']))
            last_steer = prev[1] if k == 0 else U[1, k - 1]
            max_change = s['max_steer_rate'] * dt
            opti.subject_to(opti.bounded(-max_change, steer - last_steer, max_change))
            opti.subject_to(X[3, k + 1] >= 0)
            opti.subject_to(X[3, k + 1] <= ca.fmax(v_max, x0[3] - s['brake_decel'] * t))
            opti.subject_to(S[k] >= 0)

            # 3. Obstacles.
            #    Static ones: hard rule for the whole horizon.
            #    Moving ones: hard for the first hard_horizon seconds, soft later
            #    (far-future predictions are only guesses).
            position = X[0:2, k + 1]
            for j in range(K):
                centre = obs[0:2, j] + obs[2:4, j] * t
                R = obs[4, j] + obs[5, j] * s['prediction_growth'] * t
                distance2 = ca.sumsqr(position - centre)
                if t <= s['hard_horizon'] + 1e-9:
                    opti.subject_to(distance2 >= R ** 2)
                else:
                    opti.subject_to(distance2 + obs[5, j] * S[k] >= R ** 2)
                # Clearance cost: steer away early instead of braking late.
                # (Uses the plain radius: the growth is only for the hard rules.)
                clearance = ca.sqrt(distance2 + 1e-6) - obs[4, j]
                cost += dt * s['w_clear'] * ca.exp(-ca.fmax(clearance, -0.05) / s['clear_scale'])

            # 4. Follow the target (predicted with constant velocity).
            target_pos = target[0:2] + target[2:4] * t
            relative = target_pos - position
            distance = ca.sqrt(ca.sumsqr(relative) + 1e-6)
            heading = ca.vertcat(ca.cos(X[2, k + 1]), ca.sin(X[2, k + 1]))
            cost += dt * s['w_gap'] * gap_scale * ((distance - d) / d) ** 2
            # 0 when the camera looks straight at the target, up to 2 when behind.
            cost += dt * s['w_view'] * view_scale * (1 - ca.dot(heading, relative) / distance)

            # 5. Do not stop while a moving obstacle is coming.
            too_slow = ca.fmax(0, s['min_speed'] - X[3, k + 1]) / s['min_speed']
            cost += dt * s['w_stop'] * threat * too_slow ** 2

            # 6. Smooth driving, and a big price for using the slack.
            last_a = prev[0] if k == 0 else U[0, k - 1]
            cost += dt * effort_scale * (s['w_accel'] * (a / s['max_accel']) ** 2
                                         + s['w_steer'] * (steer / s['steering_limit']) ** 2)
            cost += effort_scale * (s['w_accel_change'] * ((a - last_a) / s['max_accel']) ** 2
                                    + s['w_steer_change'] * ((steer - last_steer) / s['steering_limit']) ** 2)
            cost += s['w_slack'] * (S[k] + S[k] ** 2)

        # End of the horizon: be at the right distance.
        cost += s['w_terminal'] * gap_scale * ((distance - d) / d) ** 2

        # 7. Braking tail: from the last planned state we must be able to brake
        #    to a stop without hitting a static obstacle (passive safety).
        tail = X[:, N]
        tail_steps = int(math.ceil(s['escape_speed'] / (s['brake_decel'] * dt))) + 1
        for i in range(tail_steps):
            v = tail[3]
            new_v = ca.fmax(0, v - s['brake_decel'] * dt)
            tail = ca.vertcat(
                tail[0] + v * ca.cos(tail[2]) * dt,
                tail[1] + v * ca.sin(tail[2]) * dt,
                tail[2] + v * ca.tan(U[1, N - 1]) / L * dt,
                new_v,
            )
            for j in range(K):
                distance2 = ca.sumsqr(tail[0:2] - obs[0:2, j])
                # (1 - moving) switches the rule off for moving obstacles.
                opti.subject_to((1 - obs[5, j]) * (distance2 - obs[4, j] ** 2) >= 0)

        self.cost = cost
        opti.minimize(cost)
        opti.solver('ipopt', {'expand': True, 'print_time': False}, {
            'print_level': 0, 'sb': 'yes',
            'max_iter': int(s['solver_max_iter']),
            'max_cpu_time': float(s['solver_max_time']),
            'tol': 1e-3, 'acceptable_tol': 1e-2,
        })

    def rollout_guess(self, state, plan):
        """Turn a list of controls into states, used as the starting guess."""
        plan = list(plan)[:self.N]
        while len(plan) < self.N:
            plan.append(plan[-1] if plan else (0.0, 0.0))
        states = [state]
        for a, steer in plan:
            states.append(model_step(states[-1], a, steer, self.s['dt'], self.s['wheel_base']))
        return states, plan

    def solve(self, state, target, obstacles, prev, threat, mode, guess_plan):
        """Solve once. Returns a dict with ok, plan [(a, steer)], states, cost, time.

        obstacles: list of dicts (see safety.py), at most max_obstacles long.
        mode: dict with v_max, a_max, effort, gap, view.
        """
        opti = self.opti
        opti.set_value(self.p_state, state)
        opti.set_value(self.p_target, target)
        opti.set_value(self.p_prev, prev)
        opti.set_value(self.p_threat, threat)
        opti.set_value(self.p_vmax, mode['v_max'])
        opti.set_value(self.p_amax, mode['a_max'])
        opti.set_value(self.p_effort, mode['effort'])
        opti.set_value(self.p_gap, mode['gap'])
        opti.set_value(self.p_view, mode['view'])

        # Unused obstacle slots: a tiny static obstacle far away.
        table = [[state[0] + 100.0, state[1] + 100.0, 0.0, 0.0, 0.01, 0.0]] * self.K
        for j, o in enumerate(obstacles[:self.K]):
            table[j] = [o['x'], o['y'], o['vx'], o['vy'], o['R'], 1.0 if o['moving'] else 0.0]
        opti.set_value(self.p_obstacles, ca.DM(table).T)

        states, plan = self.rollout_guess(state, guess_plan)
        opti.set_initial(self.X, ca.DM(states).T)
        opti.set_initial(self.U, ca.DM(plan).T)
        opti.set_initial(self.S, 0)

        start = time.monotonic()
        try:
            solution = opti.solve()
            ok, value = True, solution.value
        except RuntimeError:
            # Not converged (or time limit). The last iterate may still be usable;
            # the safety check in the node decides.
            ok, value = False, opti.debug.value
        elapsed = time.monotonic() - start

        U = value(self.U)
        X = value(self.X)
        return {
            'ok': ok,
            'plan': [(float(U[0, k]), float(U[1, k])) for k in range(self.N)],
            'states': [(float(X[0, k]), float(X[1, k])) for k in range(self.N + 1)],
            'cost': float(value(self.cost)),
            'time': elapsed,
        }
