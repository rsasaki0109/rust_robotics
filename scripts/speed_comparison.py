#!/usr/bin/env python3
"""Speed comparison benchmark for PythonRobotics implementations.

Outputs CSV to stdout: algorithm,python_ms,runs

Usage:
    python3 scripts/speed_comparison.py
"""

import math
import sys
import time

import numpy as np

# ---------------------------------------------------------------------------
# A* benchmark (same grid as Rust: 100x100 with internal wall)
# ---------------------------------------------------------------------------

sys.path.insert(0, "/tmp/PythonRobotics")

# We inline the PythonRobotics A* to disable animation / print
from PathPlanning.AStar.a_star import AStarPlanner as _AStarPlanner  # noqa: E402


class AStarPlannerQuiet(_AStarPlanner):
    """A* planner with prints and animation suppressed."""

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        self.obstacle_map = [
            [False for _ in range(self.y_width)] for _ in range(self.x_width)
        ]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    def planning(self, sx, sy, gx, gy):
        start_node = self.Node(
            self.calc_xy_index(sx, self.min_x),
            self.calc_xy_index(sy, self.min_y),
            0.0,
            -1,
        )
        goal_node = self.Node(
            self.calc_xy_index(gx, self.min_x),
            self.calc_xy_index(gy, self.min_y),
            0.0,
            -1,
        )
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node
        while True:
            if len(open_set) == 0:
                break
            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost
                + self.calc_heuristic(goal_node, open_set[o]),
            )
            current = open_set[c_id]
            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break
            del open_set[c_id]
            closed_set[c_id] = current
            for i, _ in enumerate(self.motion):
                node = self.Node(
                    current.x + self.motion[i][0],
                    current.y + self.motion[i][1],
                    current.cost + self.motion[i][2],
                    c_id,
                )
                n_id = self.calc_grid_index(node)
                if not self.verify_node(node):
                    continue
                if n_id in closed_set:
                    continue
                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node
        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry


def create_grid_obstacles():
    ox, oy = [], []
    for i in range(101):
        ox.append(float(i)); oy.append(0.0)
        ox.append(float(i)); oy.append(100.0)
        ox.append(0.0); oy.append(float(i))
        ox.append(100.0); oy.append(float(i))
    for i in range(20, 80):
        ox.append(50.0); oy.append(float(i))
    return ox, oy


def bench_a_star(runs):
    ox, oy = create_grid_obstacles()
    planner = AStarPlannerQuiet(ox, oy, 1.0, 0.5)
    t0 = time.perf_counter()
    for _ in range(runs):
        planner.planning(10.0, 10.0, 90.0, 90.0)
    elapsed = (time.perf_counter() - t0) * 1000.0
    return elapsed / runs


# ---------------------------------------------------------------------------
# RRT benchmark (same obstacles/parameters as Rust)
# ---------------------------------------------------------------------------

from PathPlanning.RRT.rrt import RRT  # noqa: E402


def bench_rrt(runs):
    obstacle_list = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1),
    ]
    t0 = time.perf_counter()
    for _ in range(runs):
        rrt = RRT(
            start=[0, 0],
            goal=[6.0, 10.0],
            rand_area=[-2, 15],
            obstacle_list=obstacle_list,
            expand_dis=3.0,
            path_resolution=0.5,
            goal_sample_rate=5,
            max_iter=500,
            robot_radius=0.8,
        )
        rrt.planning(animation=False)
    elapsed = (time.perf_counter() - t0) * 1000.0
    return elapsed / runs


# ---------------------------------------------------------------------------
# EKF benchmark (same model as Rust)
# ---------------------------------------------------------------------------

from PathPlanning.CubicSpline.cubic_spline_planner import CubicSpline2D  # noqa: E402


def ekf_motion_model(x, u, dt=0.1):
    yaw = x[2, 0]
    return np.array(
        [
            [x[0, 0] + dt * u[0, 0] * math.cos(yaw)],
            [x[1, 0] + dt * u[0, 0] * math.sin(yaw)],
            [x[2, 0] + dt * u[1, 0]],
            [u[0, 0]],
        ]
    )


def ekf_jacob_f(x, u, dt=0.1):
    yaw = x[2, 0]
    v = u[0, 0]
    return np.array(
        [
            [1.0, 0.0, -dt * v * math.sin(yaw), 0.0],
            [0.0, 1.0, dt * v * math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        ]
    )


def ekf_observation_model(x):
    H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
    return H @ x


def ekf_jacob_h():
    return np.array([[1, 0, 0, 0], [0, 1, 0, 0]])


def ekf_estimation(xEst, PEst, z, u, Q, R, dt=0.1):
    xPred = ekf_motion_model(xEst, u, dt)
    jF = ekf_jacob_f(xEst, u, dt)
    PPred = jF @ PEst @ jF.T + Q
    jH = ekf_jacob_h()
    zPred = ekf_observation_model(xPred)
    y = z - zPred
    S = jH @ PPred @ jH.T + R
    K = PPred @ jH.T @ np.linalg.inv(S)
    xEst = xPred + K @ y
    PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
    return xEst, PEst


def bench_ekf(steps):
    Q = np.diag([0.1, np.deg2rad(1.0), 0.1, 0.1]) ** 2
    R = np.diag([1.0, 1.0]) ** 2
    xEst = np.zeros((4, 1))
    PEst = np.eye(4)
    u = np.array([[1.0], [0.1]])
    dt = 0.1

    t0 = time.perf_counter()
    for i in range(steps):
        t = i * dt
        # Simulated GPS measurement: forward motion along x + small drift
        z = np.array([[t + 0.01 * i], [0.005 * i]])
        xEst, PEst = ekf_estimation(xEst, PEst, z, u, Q, R, dt)
    elapsed = (time.perf_counter() - t0) * 1000.0
    return elapsed


# ---------------------------------------------------------------------------
# Cubic spline benchmark (same waypoints as Rust)
# ---------------------------------------------------------------------------


def bench_cubic_spline(runs):
    x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    y = [0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0]
    ds = 0.1

    t0 = time.perf_counter()
    for _ in range(runs):
        sp = CubicSpline2D(x, y)
        s = np.arange(0, sp.s[-1], ds)
        for i_s in s:
            sp.calc_position(i_s)
            sp.calc_yaw(i_s)
            sp.calc_curvature(i_s)
    elapsed = (time.perf_counter() - t0) * 1000.0
    return elapsed / runs


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    print("Running Python speed benchmarks...", file=sys.stderr)

    a_star_runs = 100
    rrt_runs = 100
    ekf_steps = 1000
    spline_runs = 1000

    a_star_ms = bench_a_star(a_star_runs)
    print(f"  A* done: {a_star_ms:.3f} ms/run", file=sys.stderr)

    rrt_ms = bench_rrt(rrt_runs)
    print(f"  RRT done: {rrt_ms:.3f} ms/run", file=sys.stderr)

    ekf_ms = bench_ekf(ekf_steps)
    print(f"  EKF done: {ekf_ms:.3f} ms total ({ekf_steps} steps)", file=sys.stderr)

    spline_ms = bench_cubic_spline(spline_runs)
    print(f"  CubicSpline done: {spline_ms:.3f} ms/run", file=sys.stderr)

    # CSV output
    print("algorithm,python_ms,runs")
    print(f"a_star,{a_star_ms:.6f},{a_star_runs}")
    print(f"rrt,{rrt_ms:.6f},{rrt_runs}")
    print(f"ekf,{ekf_ms:.6f},{ekf_steps}")
    print(f"cubic_spline,{spline_ms:.6f},{spline_runs}")


if __name__ == "__main__":
    main()
