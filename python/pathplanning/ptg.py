#!/usr/bin/env python3


import csv
import pprint
import enum
import copy
import time
import logging
import os
import math
import numpy as np

from pathlib import Path
from scipy import interpolate
from scipy.spatial import ConvexHull
from IPython import embed
from numpy.polynomial import Polynomial
from multiprocessing import Pool

import matplotlib.pyplot as plt

log = logging.getLogger(__name__)

np.set_printoptions(suppress=True, precision=5, sign=" ", floatmode="fixed")

COLLISION_RADIUS = 1.5
MAX_MPH = 50.0
SIGMA_S = [1.0, 1.0, 1.0]

from dataclasses import dataclass


@dataclass
class CostWeights:
    time: float = 1.0
    pos: float = 1.0
    vel: float = 1.0
    acc: float = 1.0
    jek: float = 1.0
    efficiency: float = 1.0


def plot_trajectory(m, traj):
    """Plot trajectory on map"""
    curr_time = 0.0
    points = []
    times = np.linspace(0.0, traj.time)
    points = m.get_all_xy(traj(times)[:, 0, :])
    m.plot_points(points, plt)


def plot_collision(
    m, traj, traj_points, vid, vehicle_points, vehicle_collision_points, dist
):
    vehicle_points = np.asarray(vehicle_points)

    plt.plot(
        vehicle_points[:, 0],
        vehicle_points[:, 1],
        "mx",
        markersize=4,
        label=f"{vid}: {dist:5.3f}",
        linewidth=2,
    )

    for p in vehicle_collision_points:
        plt.gca().add_artist(
            plt.Circle(p, radius=COLLISION_RADIUS, alpha=0.3, color="r")
        )
        plt.gca().add_artist(plt.Circle(p, radius=0.05, alpha=0.5, color="r"))

    for p in traj_points:
        plt.gca().add_artist(
            plt.Circle(p, radius=COLLISION_RADIUS, alpha=0.3, color="b")
        )
        plt.gca().add_artist(plt.Circle(p, radius=0.05, alpha=0.5, color="b"))

    plot_trajectory(m, traj)


def create_convex_hull_from_kinematics(x, y, heading, length=3.0, width=2.0):
    """Create convex hull around point for collision detection"""
    import itertools

    vertices = np.asarray(
        list(
            itertools.product(
                [-length / 2.0, length / 2.0], [-width / 2.0, width / 2.0]
            )
        )
    )

    R = np.array(
        [
            [math.cos(heading), -math.sin(heading)],
            [math.sin(heading), math.cos(heading)],
        ]
    )

    vertices = vertices @ R.T + [x, y]
    return ConvexHull(vertices)


class Utils:
    @staticmethod
    def to_mps(mph):
        return mph * 0.44704

    @staticmethod
    def to_mph(mps):
        return mps * 2.2369362920544


def logistic(x):
    """
    A function that returns a value between 0 and 1 for x in the
    range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

    Useful for cost functions.
    """
    return 2.0 / (1 + np.exp(-x)) - 1.0


def gaussian_1d(mu, sigma, x):
    return (
        1.0
        / (sigma * math.sqrt(2.0 * math.pi))
        * math.exp(-0.5 * ((x - mu) / sigma) ** 2)
    )


def gaussian_loss_1d(mu, sigma, x):
    return gaussian_1d(mu, sigma, mu) - gaussian_1d(mu, sigma, x)


class JMTTrajectory1d:
    """1D JMT"""

    def __init__(self, pos_poly, start_cond, end_cond, t):
        self._pos_poly = pos_poly
        self._vel_poly = self._pos_poly.deriv()
        self._acc_poly = self._vel_poly.deriv()
        self._jek_poly = self._acc_poly.deriv()
        self._snp_poly = self._jek_poly.deriv()
        self._crc_poly = self._snp_poly.deriv()
        self._start_cond = start_cond
        self._end_cond = end_cond
        self._time = t
        self._is_valid = False
        self._cost = None
        self._behavior = None

    def __repr__(self):
        return (
            f"<JMTTrajectory1d poly={self._pos_poly.coef}, "
            f"start_cond={self._start_cond}, "
            f"end_cond={self._end_cond}, "
            f"time={self._time:7.3f}, "
            f"cost={self._cost}>"
        )

    @property
    def behavior(self):
        return self._behavior

    @behavior.setter
    def behavior(self, behavior):
        self._behavior = behavior

    @property
    def cost(self):
        return self._cost

    @property
    def is_valid(self):
        return self._is_valid

    @property
    def time(self):
        return self._time

    def eval(self, t):
        return np.asarray(
            [
                self._pos_poly(t),
                self._vel_poly(t),
                self._acc_poly(t),
                self._jek_poly(t),
                self._snp_poly(t),
                self._crc_poly(t),
            ]
        )

    def __call__(self, t):
        return self.eval(t)

    @property
    def pos_poly(self):
        return self._pos_poly

    @property
    def start_cond(self):
        return self._start_cond

    @property
    def end_cond(self):
        return self._end_cond

    def get_position(self, t):
        return self._pos_poly(t)

    def get_velocity(self, t):
        return self._vel_poly(t)

    def get_acceleration(self, t):
        return self._acc_poly(t)

    def get_jerk(self, t):
        return self._jek_poly(t)

    def get_snap(self, t):
        return self._snp_poly(t)

    def get_crackle(self, t):
        return self._crc_poly(t)

    def validate(
        self,
        max_vel=None,
        max_acc=10.0,
        max_jerk=10.0,
        total_acc=2.0,
        total_jerk=1.0,
        num_points=100,
        force_velocity_dir=False,
        pos_bounds=None,
    ):
        start = time.time()
        if max_vel is None:
            max_vel = Utils.to_mps(MAX_MPH)

        self._is_valid = True
        time_resolution = self.time / num_points
        start = time.time()
        all_points = np.asarray(
            [
                self.eval(curr_time)
                for curr_time in np.linspace(0.0, self.time, num_points)
            ]
        )
        self._all_points = all_points

        if pos_bounds is not None:
            out_of_bound = (
                (pos_bounds[0] > all_points[:, 0]) | (all_points[:, 0] > pos_bounds[1])
            ).any()
            if out_of_bound:
                self._is_valid = False
                return self._is_valid

        # Check maximums
        maximums = np.abs(all_points[:, 1:4]).max(axis=0)
        is_maximuns_valid = (maximums < [max_vel, max_acc, max_jerk]).all()

        # Check total values per seconds
        averages = (np.abs(all_points[:, 1:4]) * time_resolution).sum(
            axis=0
        ) / self.time

        self._total_vel = averages[0]
        self._total_acc = averages[1]
        self._total_jerk = averages[2]

        is_total_valid = (averages[1:] < [total_acc, total_jerk]).all()

        self._is_valid = is_maximuns_valid & is_total_valid
        # print(f"validation took {time.time() - start}:7.3f secs.")

        return self._is_valid

    def compute_cost(self, weights: CostWeights):
        """Compute cost for the trajectory"""

        # Time cost
        time_cost = logistic(self.time) * weights.time

        # Displacement cost
        displacement = self.end_cond[0] - self.start_cond[0]
        pos_cost = logistic(displacement ** 2) * weights.pos

        # Total (squared) jerk
        jek_cost = (
            logistic((self.get_jerk(np.arange(0.0, self.time, 0.02)) ** 2).sum())
            * weights.jek
        )

        efficiency_cost = (
            logistic(1.0 - self._end_cond[1] / Utils.to_mps(MAX_MPH))
            * weights.efficiency
        )

        # Monkey patch for now
        self._time_cost = time_cost
        self._pos_cost = pos_cost
        self._jek_cost = jek_cost
        self._efficiency_cost = efficiency_cost

        self._cost = time_cost + pos_cost + jek_cost + efficiency_cost
        return self._cost

    def compute_cost_without_position(self, weights: CostWeights):
        """Compute cost for the trajectory"""

        # Time cost
        time_cost = logistic(self.time) * weights.time

        # Displacement cost
        if len(self.end_cond) == 2:
            delta_velocity = self.end_cond[0] - self.start_cond[1]
        else:
            delta_velocity = self.end_cond[1] - self.start_cond[1]

        vel_cost = logistic(delta_velocity ** 2) * weights.vel

        # Total (squared) jerk
        jek_cost = (
            logistic((self.get_jerk(np.arange(0.0, self.time, 0.02)) ** 2).sum())
            * weights.jek
        )

        efficiency_cost = (
            logistic(1.0 - self._end_cond[0] / Utils.to_mps(MAX_MPH))
            * weights.efficiency
        )

        # Monkey patch for now
        self._time_cost = time_cost
        self._vel_cost = vel_cost
        self._jek_cost = jek_cost
        self._efficiency_cost = efficiency_cost

        self._cost = time_cost + vel_cost + jek_cost + efficiency_cost
        return self._cost

    @staticmethod
    def _plot(traj, plt):
        times = np.linspace(0.0, traj.time, 200)
        titles = ["Position", "Velocity", "Acceleration", "Jerk", "Snap", "Crackle"]
        funcs = [
            traj._pos_poly,
            traj._vel_poly,
            traj._acc_poly,
            traj._jek_poly,
            traj._snp_poly,
            traj._crc_poly,
        ]
        for i, func in enumerate(funcs):
            plt.subplot(3, 3, i + 1)
            plt.plot(times, func(times), color="g" if traj.is_valid else "r", alpha=0.5)
            plt.title(titles[i])

    @staticmethod
    def plot(traj):
        plt.figure(0)
        JMTTrajectory1d._plot(traj, plt)
        plt.show(block=True)

    @staticmethod
    def plot_all(trajs):
        plt.figure(0)
        for traj in trajs:
            JMTTrajectory1d._plot(traj, plt)
            plt.subplot(3, 3, 7)
            plt.title("Costs")
            plt.plot(
                [traj.cost for traj in trajs],
                color="g" if traj.is_valid else "r",
                marker="x",
                alpha=0.5,
                linewidth=2,
            )
        plt.show(block=True)


class JMTTrajectory2d:
    def __init__(self, lon_traj, lat_traj):
        self._lon_traj = lon_traj
        self._lat_traj = lat_traj

    @property
    def time(self):
        return min(self._lon_traj.time, self._lat_traj.time)

    def eval(self, t):
        return np.asarray([self._lon_traj(t), self._lat_traj(t)]).T

    def __call__(self, t):
        return self.eval(t)

    @property
    def behavior(self):
        return (self._lon_traj.behavior, self._lat_traj.behavior)

    @property
    def cost(self):
        k_lon = 1.0
        k_lat = 2.0
        return k_lon * self._lon_traj.cost + k_lat * self._lat_traj.cost

    @property
    def lon_traj(self):
        return self._lon_traj

    @property
    def lat_traj(self):
        return self._lat_traj

    def eval_frenet_points(self, num_points=100):
        timesteps = np.linspace(0.0, self.time, num_points)
        return self.eval(timesteps)[:, 0, :]

    def eval_points(self, num_points=100):
        return m.get_all_xy(self.eval_frenet_points(num_points))

    def validate_heading(self, initial_heading, max_heading=10.0):
        points = self.eval_points()
        diff = points[1:, :] - points[:-1, :]
        diff /= np.linalg.norm(diff, axis=1)[..., -1]
        headings = np.rad2deg(np.arctan2(diff[:, 1], diff[:, 0]))
        is_initial_heading_valid = abs(headings[0] - initial_heading) < max_heading
        is_traj_heading_valid = (
            np.abs(headings[1:] - headings[:-1]) < max_heading
        ).all()
        print(f"is_initial_heading_valid={is_initial_heading_valid}")
        print(f"is_traj_heading_valid={is_initial_heading_valid}")
        return is_traj_heading_valid and is_initial_heading_valid

    @staticmethod
    def _plot(traj, m, plt, color, pos_only=True):

        time = min(traj._lon_traj.time, traj._lat_traj.time)
        times = np.linspace(0.0, time, 100)
        if not pos_only:
            plt.subplot(3, 2, 1)
        points = traj(times)[:, 0, :]

        traj_xy = m.get_xy(points[:, 0], points[:, 1])

        padding = 1.0
        s_range = [points[:, 0].min() - padding, points[:, 0].max() + padding]
        plt.plot(
            traj_xy[:, 0],
            traj_xy[:, 1],
            linewidth=2,
            color=[0.0, 1.0 - color, 0.0],
            label=f"Cost: {traj.cost:7.3f}",
        )

        plt.xlabel("Longitudinal (m)")
        plt.ylabel("Lateral (m)")
        plt.title("Position trajectory in Cartesian frame")
        plt.legend()
        plt.axis("equal")

        if not pos_only:
            plt.subplot(3, 2, 2)
            points = traj(times)[:, 1, :]
            plt.plot(points[:, 0], alpha=0.5)
            # plt.plot(points[:, 0], points[:, 1])
            # plt.xlabel("Longitudinal (m/s)")
            # plt.ylabel("Lateral (m/s)")
            plt.title("Velocity trajectory in Frenet frame")
            # plt.axis("equal")

            plt.subplot(3, 2, 3)
            points = traj(times)[:, 2, :]
            plt.plot(points[:, 0])
            # plt.plot(points[:, 0], points[:, 1])
            # plt.xlabel("Longitudinal (m/s^2)")
            # plt.ylabel("Lateral (m/s^2)")
            plt.title("Acceleration trajectory in Frenet frame")
            # plt.axis("equal")

            plt.subplot(3, 2, 4)
            points = traj(times)[:, 3, :]
            plt.plot(points[:, 0], points[:, 1])
            plt.xlabel("Longitudinal (m/s^3)")
            plt.ylabel("Lateral (m/s^3)")
            plt.title("Jerk trajectory in Frenet frame")
            plt.axis("equal")

            plt.subplot(3, 2, 5)
            points = traj(times)[:, 4, :]
            plt.plot(points[:, 0], points[:, 1])
            plt.xlabel("Longitudinal (m/s^4)")
            plt.ylabel("Lateral (m/s^4)")
            plt.title("Snap trajectory in Frenet frame")
            plt.axis("equal")

            plt.subplot(3, 2, 6)
            points = traj(times)[:, 5, :]
            plt.plot(points[:, 0], points[:, 1], marker="x")
            plt.xlabel("Longitudinal (m/s^5)")
            plt.ylabel("Lateral (m/s^5)")
            plt.title("Crackle trajectory in Frenet frame")
            plt.axis("equal")

    @staticmethod
    def plot(traj):
        plt.figure(0, figsize=(12.8, 6.4))
        endpoint = traj.lon_traj.end_cond
        if len(traj.lon_traj.end_cond) == 2:
            endpoint = traj.lon_traj(traj.lon_traj.time)
        min_s = traj.lon_traj.start_cond[0]
        max_s = endpoint[0]
        s_range = [min_s, max_s]
        m.plot(s_range, plt=plt)
        JMTTrajectory2d._plot(traj, m, plt, 0.5)
        plt.show()

    @staticmethod
    def plot_all(trajs, pos_only=False):
        if len(trajs) == 0:
            print("Nothing to plot, length of trajs is 0.")
            return

        plt.figure(0, figsize=(12.8, 6.4))
        min_s = np.min([traj.lon_traj.start_cond[0] for traj in trajs])
        max_s = np.max([traj.lon_traj.end_cond[0] for traj in trajs])

        m = Map(Path(__file__).parent / "../data/highway_map.csv")
        if not pos_only:
            plt.subplot(3, 2, 1)
        m.plot([min_s, max_s], plt=plt)

        costs = [traj.cost for traj in trajs]
        min_cost = np.min(costs)
        max_cost = np.max(costs)

        def _color(cost):
            return (cost - min_cost) / (max_cost - min_cost)

        from tqdm import tqdm

        for traj in tqdm(trajs, desc="Plotting..."):
            JMTTrajectory2d._plot(traj, m, plt, _color(traj.cost), pos_only)

        plt.tight_layout()
        plt.show(block=True)


class JMTSolver:
    @staticmethod
    def solve_1d(s_i, s_f, T):
        T2 = T * T
        T3 = T2 * T
        T4 = T3 * T
        T5 = T4 * T

        A = np.array(
            [[T3, T4, T5], [3 * T2, 4 * T3, 5 * T4], [6 * T, 12 * T2, 20 * T3]]
        )

        c012 = [s_i[0], s_i[1], s_i[2] * 0.5]

        M = np.array([[1.0, T, 0.5 * T2], [0.0, 1.0, T], [0.0, 0.0, 1.0]])

        c345 = np.linalg.solve(A, s_f - M @ s_i)
        return JMTTrajectory1d(Polynomial(np.r_[c012, c345]), s_i, s_f, T)

    @staticmethod
    def solve_1d_without_endpos_constraint(s_i, s_f, T):
        """Solve polynomial without end position constraints

        The coefficient of crackle 5th order term is set to 0 (transversality condition).

        s_f = [s_f dot, s_f ddot]
        """
        assert len(s_i) == 3 and len(s_f) == 2
        T2 = T * T
        T3 = T2 * T
        T4 = T3 * T
        T5 = T4 * T

        c012 = [s_i[0], s_i[1], s_i[2] * 0.5]

        A = np.array([[3 * T2, 4 * T3], [6 * T, 12 * T2]])
        M = np.array([[1.0, T], [0.0, 1.0]])

        c34 = np.linalg.solve(A, s_f - M @ c012[1:])

        # Constraint the time
        traj = JMTTrajectory1d(Polynomial(np.r_[c012, c34, 0.0]), s_i, s_f, T)
        cropped_time = min(0.02 * 2000, T)
        cropped_s_f = traj(cropped_time)
        traj._time = cropped_time
        traj._end_cond = cropped_s_f[:3]
        return traj


class Kinematics:
    def __init__(self, kinematics):
        if len(kinematics) == 3:
            kinematics = np.r_[kinematics, 0.0, 0.0, 0.0]
        self._kinematics = Polynomial(kinematics)
        self._pos_poly = self._kinematics
        self._vel_poly = self._pos_poly.deriv()
        self._acc_poly = self._vel_poly.deriv()
        self._jek_poly = self._acc_poly.deriv()
        self._snp_poly = self._jek_poly.deriv()
        self._crc_poly = self._snp_poly.deriv()

    def get_kinematics(self, t=0.0):
        return np.array(
            [
                self._pos_poly(t),
                self._vel_poly(t),
                self._acc_poly(t),
                self._jek_poly(t),
                self._snp_poly(t),
                self._crc_poly(t),
            ]
        )

    @property
    def kinematics(self):
        return self._kinematics

    @property
    def coef(self):
        return self._kinematics.coef


def compute_start_state(ego, prevTraj, prevPath):

    # print(f"len(prevPath)={len(prevPath)}")
    ego_kinematics_s = Kinematics(ego["kinematics"][:3])
    ego_kinematics_d = Kinematics(ego["kinematics"][3:])
    if len(prevPath) == 0:
        start_state = np.array(
            [
                ego_kinematics_s.get_kinematics(0.0)[:3],
                ego_kinematics_d.get_kinematics(0.0)[:3],
            ]
        ).T
        # print(f"initial start_state={start_state}")
        return start_state

    lon_time = prevTraj["lonTraj"]["time"]
    lon_poly = Kinematics(prevTraj["lonTraj"]["position"])
    lon_start_cond = lon_poly.get_kinematics(0.0)[:3]
    lon_end_cond = lon_poly.get_kinematics(lon_time)[:3]
    lon_traj = JMTTrajectory1d(
        lon_poly.kinematics, lon_start_cond, lon_end_cond, lon_time
    )

    lat_time = prevTraj["latTraj"]["time"]
    lat_poly = Kinematics(prevTraj["latTraj"]["position"])
    lat_start_cond = lat_poly.get_kinematics(0.0)[:3]
    lat_end_cond = lat_poly.get_kinematics(lat_time)[:3]
    lat_traj = JMTTrajectory1d(
        lat_poly.kinematics, lat_start_cond, lon_end_cond, lat_time
    )

    prev_traj = JMTTrajectory2d(lon_traj, lat_traj)
    simulator_time_step = 0.02  # TODO
    executed_time = (100 - len(prevPath)) * simulator_time_step
    start_state = prev_traj(executed_time)[:3, :]

    return start_state


class Map:
    MAX_FRENET_S = 6945.554
    LANE_WIDTH = 4.0
    HALF_LANE_WIDTH = LANE_WIDTH / 2.0
    NUM_LANES = 3

    def __init__(self, filename):
        data = []
        with open(filename) as csvfile:
            spamreader = csv.reader(csvfile, delimiter=" ")
            for row in spamreader:
                data.append([float(num) for num in row])
        self.data = np.asarray(data)
        self.x = self.data[:, 0]
        self.y = self.data[:, 1]
        self.s = self.data[:, 2]
        self.dx = self.data[:, 3]
        self.dy = self.data[:, 4]
        self.sx_spline = interpolate.CubicSpline(self.s, self.x)
        self.sdx_spline = interpolate.CubicSpline(self.s, self.dx)
        self.sy_spline = interpolate.CubicSpline(self.s, self.y)
        self.sdy_spline = interpolate.CubicSpline(self.s, self.dy)

    @classmethod
    def get_lane_id(cls, d):
        for i in range(cls.NUM_LANES):
            if cls.is_in_lane(d, i):
                return i
        return -1

    @classmethod
    def get_lane_center_d(cls, lane_id):
        return cls.HALF_LANE_WIDTH + cls.LANE_WIDTH * lane_id

    @classmethod
    def get_road_boundaries(cls):
        return (0.0, cls.NUM_LANES * cls.LANE_WIDTH)

    @classmethod
    def get_lane_boundaries(cls, lane_id):
        center_d = cls.get_lane_center_d(lane_id)
        return (center_d - cls.HALF_LANE_WIDTH, center_d + cls.HALF_LANE_WIDTH)

    @classmethod
    def is_in_lane(cls, d, lane_id):
        return lane_id * cls.LANE_WIDTH < d < (lane_id + 1) * cls.LANE_WIDTH

    MAX_S = 6945.554

    def get_next_waypoint_index(self, x, y, theta):
        index = self.get_closest_waypoint_index(x, y)
        point = self.data[index]
        heading = math.arctan2(point[1] - y, point[0] - x)
        angle = abs(theta - heading)
        angle = min(2 * math.pi - angle, angle)

        if angle > (math.pi / 2.0):
            index += 1
            if index == len(self.data):
                index = 0
        return index

    def get_closest_waypoint_index(self, x, y):
        return np.argmin(np.linalg.norm(self.data[:, :2] - [x, y], axis=1))

    def get_sd(self, x, y, theta):
        next_index = self.get_next_waypoint_index(x, y, theta)
        prev_index = next_index - 1

        if next_index == 0:
            prev_index = len(self.data) - 1

        vec_n = self.data[next_index][:2] - self.data[prev_index][:2]
        vec_x = np.asarray([x, y]) - self.data[prev_index][:2]
        proj_norm = np.dot(vec_x, vec_n) / np.dot(vec_n, vec_n)
        proj = vec_n * proj_norm

        d = np.linalg.norm(vec_x - proj)

        center = [1000.0 - self.x[prev_index], 2000.0 - self.y[prev_index]]
        center_to_pos = np.linalg.norm(center - vec_x)
        center_to_ref = np.linalg.norm(center - proj)

        if center_to_pos < center_to_ref:
            d = -d

        s = np.sum(
            np.linalg.norm(
                self.data[1 : prev_index + 1][:2] - self.data[:prev_index][:2]
            )
        )
        s += np.linalg.norm(prej)

        return np.array([s, d]).T

    def get_xy(self, s, d):
        return np.array(
            [
                self.sx_spline(s) + d * self.sdx_spline(s),
                self.sy_spline(s) + d * self.sdy_spline(s),
            ]
        ).T

    def get_all_xy(self, points_sd):
        points = np.empty_like(points_sd)
        for i in range(len(points)):
            points[i] = self.get_xy(points_sd[i][0], points_sd[i][1])
        return points

    def plot(self, s_range=None, plt=None):
        if plt is None:
            import matplotlib.pyplot as plt

            plt.figure(0)

        if s_range is None:
            s_range = [self.s[0], self.s[-1]]

        for lane in self._crop_map_using_s(s_range):
            plt.plot(lane[:, 0], lane[:, 1], "r-", linewidth=1, alpha=0.5)
        plt.axis("equal")

    def _crop_map_using_s(self, s_range, num=100):
        if s_range is None:
            s_range = [self.s[0], self.s[-1]]

        s = np.linspace(s_range[0], s_range[1], num)
        lanes = []
        for lane_id in range(self.NUM_LANES):
            lane_xy = self.get_xy(
                s,
                np.ones(shape=s.shape)
                * (lane_id * self.LANE_WIDTH + self.HALF_LANE_WIDTH),
            )
            lanes.append(lane_xy)
        return lanes

    def _crop_map_using_xy(self, min_xy, max_xy):
        lanes = self._crop_map_using_s(
            [self.s[0], self.s[-1]], int(round(self.MAX_FRENET_S))
        )

        cropped_lanes = []
        for lane in lanes:
            cropped_lanes.append(
                lane[((min_xy < lane) & (lane < max_xy)).all(axis=1), :]
            )
        return cropped_lanes

    def plot_points(self, points, plt=None):
        for lane in self._crop_map_using_xy(
            points.min(axis=0) - 10, points.max(axis=0) + 10
        ):
            plt.plot(lane[:, 0], lane[:, 1], "r-", alpha=0.5)
            plt.plot(points[:, 0], points[:, 1], markersize=3, marker="x")
        plt.axis("equal")


m = Map(Path(os.environ.get("MAP_DATA", "")))


def _group_vehicles(tracked_vehicles):
    grouped_vehicles = dict()
    for vehicle_id, vehicle in tracked_vehicles.items():
        lane_id = Map.get_lane_id(vehicle["kinematics"][3])
        if grouped_vehicles.get(lane_id) is None:
            grouped_vehicles[lane_id] = []
        grouped_vehicles[lane_id].append(vehicle)
    return grouped_vehicles


class LonManeuverType(enum.IntEnum):
    kCrusing = 0
    kStopping = 1
    kFollowing = 2


class LatManeuverType(enum.IntEnum):
    kLaneKeeping = 0
    kLeftLaneChanging = 1
    kRightLaneChanging = 2


class PTG:
    @staticmethod
    def generate_lon_trajectories(lon_behavior, ego, leading_vehicle):
        if lon_behavior == LonManeuverType.kCrusing:
            return PTG._generate_crusing_trajectory(ego)
        elif lon_behavior == LonManeuverType.kStopping:
            return PTG._generate_stopping_trajectory(ego)
        elif lon_behavior == LonManeuverType.kFollowing:
            return PTG._generate_following_trajectory(ego, leading_vehicle)
        return []

    class LatTrajTask:
        def __init__(self, ego_kin, lat_behavior):
            self._ego_kin = ego_kin
            ego_d = ego_kin.coef[0]
            self._target_d = Map.get_lane_center_d(Map.get_lane_id(ego_d))
            if lat_behavior == LatManeuverType.kLeftLaneChanging:
                self._target_d = Map.get_lane_center_d(Map.get_lane_id(ego_d) - 1)
            elif lat_behavior == LatManeuverType.kRightLaneChanging:
                self._target_d = Map.get_lane_center_d(Map.get_lane_id(ego_d) + 1)
            self._lat_behavior = lat_behavior

        def __call__(self, T):
            s_i = self._ego_kin.coef[:3]
            s_f = [self._target_d, 0, 0]
            traj = JMTSolver.solve_1d(s_i, s_f, T)
            traj.behavior = self._lat_behavior
            is_valid = traj.validate(pos_bounds=Map.get_road_boundaries())
            traj.compute_cost(CostWeights(time=1.0, pos=1.0, jek=3.0))
            if is_valid:
                return [traj]

            return []

    @staticmethod
    def generate_lat_trajectories(lat_behavior, ego):
        ego_kin = Kinematics(ego["kinematics"][3:])
        task = PTG.LatTrajTask(ego_kin, lat_behavior)
        with Pool() as pool:
            result = pool.map(task, np.linspace(5.0, 10.0, 20))
        trajs = []
        for traj in result:
            trajs.extend(traj)
        trajs = [traj for traj in trajs if traj.is_valid]
        return np.asarray(trajs)

    @staticmethod
    def get_optimal_combination(lon_trajs, lat_trajs, k_lon=1.0, k_lat=2.0):
        costs = []
        for lon_traj in lon_trajs:
            for lat_traj in lat_trajs:
                costs.append(k_lon * lon_traj.cost + k_lat * lat_traj.cost)

        costs = np.reshape(np.asarray(costs), (len(lon_trajs), len(lat_trajs)))

        lon_id, lat_id = np.unravel_index(np.argmin(costs), costs.shape)
        return (lon_id, lat_id, costs.min())

    @staticmethod
    def check_collision(traj, tracked_vehicles):
        if len(tracked_vehicles):
            timesteps = np.linspace(0.0, traj.time, 50)
            traj_points = traj(timesteps)[:, 0, :]

            for vid, vehicle in tracked_vehicles.items():
                lon_kin = Kinematics(vehicle["kinematics"][:3])
                lat_kin = Kinematics(vehicle["kinematics"][3:])

                lons = lon_kin.get_kinematics(timesteps)[0, :]
                lats = lat_kin.get_kinematics(timesteps)[0, :]

                v_points = np.vstack([lons, lats]).T

                distances = np.linalg.norm(traj_points - v_points, axis=1)

                if (distances < COLLISION_RADIUS).any():
                    # plot_collision(
                    #     m,
                    #     traj,
                    #     [m.get_all_xy(traj_points)[distances.argmin()]],
                    #     vid,
                    #     m.get_all_xy(v_points),
                    #     [m.get_all_xy(v_points)[distances.argmin()]],
                    #     distances.min(),
                    # )
                    return True, vid, distances.min()
        return False, -1, -1.0

    class CrusingTrajTask:
        def __init__(self, ego_kin):
            self._ego_kin = ego_kin

        def __call__(self, T):
            trajs = []
            for dsdot in np.linspace(5.0, 20.0, 15):
                s_i = self._ego_kin.coef[:3]
                s_f = [
                    min(self._ego_kin.coef[3] + dsdot, Utils.to_mps(MAX_MPH)),
                    0.0,
                ]
                traj = JMTSolver.solve_1d_without_endpos_constraint(s_i, s_f, T)
                traj.behavior = LonManeuverType.kCrusing
                is_valid = traj.validate()
                traj.compute_cost_without_position(
                    CostWeights(time=5.0, vel=2.0, jek=1.0)
                )
                trajs.append(traj)
            return trajs

    @staticmethod
    def _generate_crusing_trajectory(ego):
        start = time.time()
        ego_kin = Kinematics(ego["kinematics"][:3])

        task = PTG.CrusingTrajTask(ego_kin)
        with Pool() as pool:
            result = pool.map(task, np.linspace(2.0, 15.0, 10))

        trajs = []
        for traj in result:
            trajs.extend(traj)

        # print(f" - velocity keeping took {time.time() - start:7.3f} secs")
        trajs = sorted(
            [traj for traj in trajs if traj.is_valid], key=lambda x: x._total_vel
        )
        if len(trajs):
            return [trajs[-1]]
        return []

    class StoppingTask:
        def __init__(self, ego_kin):
            self._ego_kin = ego_kin

        def __call__(self, T):
            trajs = []
            for dsdot in np.linspace(-1.0, -10.0, 10):
                s_i = self._ego_kin.coef[:3]
                s_f = [
                    max(self._ego_kin.coef[1] + dsdot, 0.0),
                    0.0,
                ]
                traj = JMTSolver.solve_1d_without_endpos_constraint(s_i, s_f, T)
                traj.behavior = LonManeuverType.kStopping
                is_valid = traj.validate()
                traj.compute_cost_without_position(
                    CostWeights(time=3.0, vel=2.0, jek=1.0)
                )
                if is_valid:
                    trajs.append(traj)
            return trajs

    @staticmethod
    def _generate_stopping_trajectory(ego):
        start = time.time()

        ego_kin = Kinematics(ego["kinematics"][:3])
        task = PTG.StoppingTask(ego_kin)

        with Pool() as pool:
            result = pool.map(task, np.linspace(10.0, 20.0, 10))

        trajs = []
        for traj in result:
            trajs.extend(traj)

        # print(f" - stopping took {time.time() - start:7.3f} secs")
        return trajs

    class FollowingTrajTask:
        def __init__(self, ego_kin, lead_kin):
            self._ego_kin = ego_kin
            self._lead_kin = lead_kin

        def __call__(self, T):
            tau = 0.1
            offset = 30.0
            trajs = []
            lead_kinematics = self._lead_kin.get_kinematics(T)
            s_i = self._ego_kin.coef[:3]
            s_f = [
                lead_kinematics[0] - tau * lead_kinematics[1] - offset,
                lead_kinematics[1] - tau * lead_kinematics[2],
                lead_kinematics[2],
            ]
            traj = JMTSolver.solve_1d(s_i, s_f, T)
            traj.behavior = LonManeuverType.kFollowing
            is_valid = traj.validate()
            traj.compute_cost(CostWeights(time=10.0, pos=2.0, jek=1.0))
            if is_valid:
                trajs.append(traj)
            return np.asarray(trajs)

    @staticmethod
    def _generate_following_trajectory(ego, leading_vehicle, offset=30.0, tau=0.1):
        start = time.time()

        ego_kin = Kinematics(ego["kinematics"][:3])
        lead_kin = Kinematics(leading_vehicle["kinematics"][:3])
        task = PTG.FollowingTrajTask(ego_kin, lead_kin)

        with Pool() as pool:
            result = pool.map(task, np.linspace(5.0, 10.0, 20))

        trajs = []
        for traj in result:
            trajs.extend(traj)

        # print(f" - follow took {time.time() - start:7.3f} secs")
        return trajs


def find_leading_vehicle(ego, vehicles):
    leading_vehicle = None
    min_dist = None
    for v in vehicles:
        dist = v["kinematics"][0] - ego["kinematics"][0]
        if (dist > 0.0) and (min_dist is None or dist < min_dist):
            min_dist = dist
            leading_vehicle = v
    return leading_vehicle, min_dist


def generate_trajectory(ego, tracked_vehicles):

    print("------------------ start " + "-" * 80)
    start_time = time.time()
    grouped_vehicles = _group_vehicles(tracked_vehicles)
    ego_lane_id = Map.get_lane_id(ego["kinematics"][3])

    left_lane_id = ego_lane_id - 1
    right_lane_id = ego_lane_id + 1

    lane_ids_for_planning = dict(current=ego_lane_id)

    if 0 <= left_lane_id < ego_lane_id:
        lane_ids_for_planning["left"] = left_lane_id
    if ego_lane_id < right_lane_id <= Map.NUM_LANES:
        lane_ids_for_planning["right"] = right_lane_id

    lon_behaviors = dict()
    leading_vehicles = dict()  # TODO

    for lane_name, lane_id in lane_ids_for_planning.items():
        lon_behaviors[lane_id] = []

        vehicles = grouped_vehicles.get(lane_id)
        if vehicles is not None and len(vehicles) > 0:
            leading_vehicle, min_dist = find_leading_vehicle(ego, vehicles)
            if min_dist is not None:
                if 0.0 < min_dist < 10.0:
                    lon_behaviors[lane_id].append(LonManeuverType.kStopping)
                elif 10.0 < min_dist < 50.0 and leading_vehicle is not None:
                    leading_vehicles[lane_id] = leading_vehicle
                    lon_behaviors[lane_id].append(LonManeuverType.kFollowing)
                else:
                    lon_behaviors[lane_id].append(LonManeuverType.kCrusing)
            else:
                lon_behaviors[lane_id].append(LonManeuverType.kCrusing)
        else:
            lon_behaviors[lane_id].append(LonManeuverType.kCrusing)

    # pprint.pprint(lon_behaviors)

    lat_behaviors = dict()
    lat_behaviors[ego_lane_id] = LatManeuverType.kLaneKeeping
    if "left" in lane_ids_for_planning:
        lat_behaviors[left_lane_id] = LatManeuverType.kLeftLaneChanging
    if "right" in lane_ids_for_planning:
        lat_behaviors[right_lane_id] = LatManeuverType.kRightLaneChanging

    best_traj = None
    best_cost = None
    trajs = []
    all_trajs = []

    for lane_name, lane_id in lane_ids_for_planning.items():
        lon_trajs = []
        lat_trajs = []

        if lane_id not in lon_behaviors or lane_id not in lat_behaviors:
            print(f"Skipping lane {lane_id}...")
            continue

        print(f"Planning for lane {lane_id}...")

        for lon_behavior in lon_behaviors[lane_id]:
            # print(f" - Generating behavior for {str(lon_behavior)}.")
            lon_trajs_per_behavior = PTG.generate_lon_trajectories(
                lon_behavior, ego, leading_vehicles.get(lane_id)
            )
            # if len(lon_trajs_per_behavior) > 0:
            #     print(
            #         f" - lane_id={lane_id}: {str(lon_behavior)} = {min([traj.cost for traj in lon_trajs_per_behavior])}"
            #     )
            # else:
            #     print(f" - lane_id={lane_id}: {str(lon_behavior)} failed.")
            lon_trajs.extend(lon_trajs_per_behavior)

        lat_trajs.extend(PTG.generate_lat_trajectories(lat_behaviors[lane_id], ego))

        if len(lon_trajs) == 0:
            print(" - len(lon_trajs) == 0, planning failed!")
            continue

        if len(lat_trajs) == 0:
            print(" - len(lat_trajs) == 0, planning failed!")
            continue

        lon_id, lat_id, cost = PTG.get_optimal_combination(lon_trajs, lat_trajs)

        traj = JMTTrajectory2d(lon_trajs[lon_id], lat_trajs[lat_id])

        print(
            f" - Planned for behavior: {traj._lon_traj.behavior.name}, {traj._lat_traj.behavior.name}, "
            f"lon_cost={lon_trajs[lon_id].cost:.3f}, lat_cost={lat_trajs[lat_id].cost:.3f}, cost={traj.cost:.3f}"
        )
        trajs.append(traj)

    import functools

    with Pool() as pool:
        result = pool.map(
            functools.partial(PTG.check_collision, tracked_vehicles=tracked_vehicles),
            trajs,
        )

    for index, collision_result in enumerate(result):
        is_in_collision, vid, distance = collision_result
        if is_in_collision:
            v_lane_id = m.get_lane_id(tracked_vehicles[vid]["kinematics"][3])
            print(
                f" - In collision with {vid} on lane {v_lane_id}, distance={distance:6.3f}"
            )
        traj = trajs[index]

        if best_cost is None or traj.cost < best_cost:
            best_cost = cost
            best_traj = traj

    # if best_traj is None:
    #     embed()

    print(f"Trajectory generation took {time.time() - start_time:.3f} seconds")
    assert best_traj is not None, "No best_traj found!"

    # if best_traj._lat_traj.behavior in (
    #     LatManeuverType.kLeftLaneChanging,
    #     LatManeuverType.kRightLaneChanging,
    # ):
    #     embed()

    return dict(
        # lon
        lon_coef=best_traj.lon_traj.pos_poly.coef,
        lon_start_cond=best_traj.lon_traj.start_cond,
        lon_end_cond=best_traj.lon_traj.end_cond,
        lon_time=best_traj.lon_traj.time,
        # lat
        lat_coef=best_traj.lat_traj.pos_poly.coef,
        lat_start_cond=best_traj.lat_traj.start_cond,
        lat_end_cond=best_traj.lat_traj.end_cond,
        lat_time=best_traj.lat_traj.time,
    )


if __name__ == "__main__":
    from IPython import embed

    embed()
