#!/usr/bin/env python3


import matplotlib.pyplot as plt
from numpy.polynomial import Polynomial
import enum
import copy
import numpy as np
import math
import time
import logging

log = logging.getLogger(__name__)

np.set_printoptions(suppress=True, precision=5, sign=" ", floatmode="fixed")


def mph_to_mps(mph):
    return mph * 0.44704


def mps_to_mph(mps):
    return mps * 2.2369362920544


def gaussian_1d(mu, sigma, x):
    return (
        1.0
        / (sigma * math.sqrt(2.0 * math.pi))
        * math.exp(-0.5 * ((x - mu) / sigma) ** 2)
    )


def gaussian_loss_1d(mu, sigma, x):
    return gaussian_1d(mu, sigma, mu) - gaussian_1d(mu, sigma, x)


class JMTTrajectory1d:
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

    def __repr__(self):
        return (
            f"<JMTTrajectory1d poly={self._pos_poly.coef}, "
            f"start_cond={self._start_cond}, "
            f"end_cond={self._end_cond}, "
            f"time={self._time:7.3f}, "
            f"cost={self._cost}>"
        )

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

    def get_velocoty(self, t):
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
        self, max_vel, max_acc, max_jerk, time_resolution=0.02, force_velocity_dir=False
    ):
        curr_time = 0.0
        total_acc = 0.0
        total_jerk = 0.0
        self._is_valid = True
        while curr_time < self._time:
            values = self.eval(curr_time)
            if max_vel < values[1]:
                self._is_valid = False
                # print(f" - speed={values[1]:7.3f} > {max_vel:7.3f}")
                break

            if force_velocity_dir and values[1] < -0.1:
                # print(f" - speed={values[1]:7.3f} < -0.1")
                self._is_valid = False
                break

            total_acc += abs(values[2] * time_resolution)
            total_jerk += abs(values[3] * time_resolution)
            curr_time += time_resolution

        if self._is_valid:
            if total_acc > 10.0:
                # print(f" - total_acc={total_acc:7.3f} > 10.0")
                self._is_valid = False
                return self._is_valid

            if total_jerk > 10.0:
                # print(f" - total_jerk={total_jerk:7.3f} > 10.0")
                self._is_valid = False
                return self._is_valid

        return self._is_valid

    def compute_cost(self, k_time, k_pos, k_vel, k_acc, k_jek):
        time_cost = self._time * k_time
        pos_cost = (self.get_position(self._time) - self.get_position(0.0)) ** 2 * k_pos
        vel_cost = gaussian_loss_1d(20.0, 5.0, self.get_velocoty(self._time)) * k_vel
        jek_cost = self.get_jerk(self._time) * k_jek
        self._cost = time_cost + pos_cost + vel_cost + jek_cost
        return self._cost

    @staticmethod
    def _plot(traj, plt):
        times = np.linspace(0.0, traj.time, 200)
        plt.subplot(3, 3, 1)
        plt.plot(times, traj._pos_poly(times), color="g" if traj.is_valid else "r")
        plt.title("Position")
        plt.subplot(3, 3, 2)
        plt.plot(times, traj._vel_poly(times), color="g" if traj.is_valid else "r")
        plt.title("Velocity")
        plt.subplot(3, 3, 3)
        plt.plot(times, traj._acc_poly(times), color="g" if traj.is_valid else "r")
        plt.title("Acceleration")
        plt.subplot(3, 3, 4)
        plt.plot(times, traj._jek_poly(times), color="g" if traj.is_valid else "r")
        plt.title("Jerk")
        plt.subplot(3, 3, 5)
        plt.plot(times, traj._snp_poly(times), color="g" if traj.is_valid else "r")
        plt.title("Snap")
        plt.subplot(3, 3, 6)
        plt.plot(times, traj._crc_poly(times), color="g" if traj.is_valid else "r")
        plt.title("Crackle")

    @staticmethod
    def plot(traj):

        plt.figure(0)
        # plt.ioff()
        JMTTrajectory1d._plot(traj, plt)
        # plt.ion()
        plt.show()

    @staticmethod
    def plot_all(trajs):

        plt.figure(0)
        # plt.ioff()
        for traj in trajs:
            JMTTrajectory1d._plot(traj, plt)
        plt.subplot(3, 3, 7)
        plt.title("Costs")
        plt.plot(
            [traj.cost for traj in trajs],
            color="g" if traj.is_valid else "r",
            marker="x",
        )
        # plt.ion()
        plt.show()


class JMTTrajectory2d:
    def __init__(self, lon_traj, lat_traj):
        self._lon_traj = lon_traj
        self._lat_traj = lat_traj

    def eval(self, t):
        return np.asarray([self._lon_traj(t), self._lat_traj(t)]).T

    def __call__(self, t):
        return self.eval(t)

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

    @staticmethod
    def _plot(traj, plt):
        time = min(traj._lon_traj.time, traj._lat_traj.time)
        times = np.linspace(0.0, time, 200)
        plt.subplot(3, 2, 1)
        points = traj(times)[:, 0, :]
        plt.plot(points[:, 0], points[:, 1], "r.")
        plt.plot(points[:, 0], points[:, 1])
        plt.xlabel("Longitudinal (m)")
        plt.ylabel("Lateral (m)")
        plt.title("Position trajectory in Frenet frame")
        plt.axis("equal")

        plt.subplot(3, 2, 2)
        points = traj(times)[:, 1, :]
        plt.plot(points[:, 0], points[:, 1], "r.")
        plt.plot(points[:, 0], points[:, 1])
        plt.xlabel("Longitudinal (m/s)")
        plt.ylabel("Lateral (m/s)")
        plt.title("Velocity trajectory in Frenet frame")
        plt.axis("equal")

        plt.subplot(3, 2, 3)
        points = traj(times)[:, 2, :]
        plt.plot(points[:, 0], points[:, 1], "r.")
        plt.plot(points[:, 0], points[:, 1])
        plt.xlabel("Longitudinal (m/s^2)")
        plt.ylabel("Lateral (m/s^2)")
        plt.title("Acceleration trajectory in Frenet frame")
        plt.axis("equal")

        plt.subplot(3, 2, 4)
        points = traj(times)[:, 3, :]
        plt.plot(points[:, 0], points[:, 1], "r.")
        plt.plot(points[:, 0], points[:, 1])
        plt.xlabel("Longitudinal (m/s^3)")
        plt.ylabel("Lateral (m/s^3)")
        plt.title("Jerk trajectory in Frenet frame")
        plt.axis("equal")

        plt.subplot(3, 2, 5)
        points = traj(times)[:, 4, :]
        plt.plot(points[:, 0], points[:, 1], "r.")
        plt.plot(points[:, 0], points[:, 1])
        plt.xlabel("Longitudinal (m/s^4)")
        plt.ylabel("Lateral (m/s^4)")
        plt.title("Snap trajectory in Frenet frame")
        plt.axis("equal")

        plt.subplot(3, 2, 6)
        points = traj(times)[:, 5, :]
        plt.plot(points[:, 0], points[:, 1], "r.")
        plt.plot(points[:, 0], points[:, 1])
        plt.xlabel("Longitudinal (m/s^5)")
        plt.ylabel("Lateral (m/s^5)")
        plt.title("Crackle trajectory in Frenet frame")
        plt.axis("equal")

        plt.tight_layout()

    @staticmethod
    def plot(traj):
        plt.figure(0, figsize=(12.8, 6.4))
        JMTTrajectory2d._plot(traj, plt)
        plt.show()

    @staticmethod
    def plot_all(trajs):
        plt.figure(0, figsize=(12.8, 6.4))
        for traj in trajs:
            JMTTrajectory2d._plot(traj, plt, color="g")
        plt.show()


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

        coeffs = [s_i[0], s_i[1], s_i[2] * 0.5]

        b = [
            s_f[0] - (s_i[0] + s_i[1] * T + 0.5 * s_i[2] * T2),
            s_f[1] - (s_i[1] + s_i[2] * T),
            s_f[2] - s_i[2],
        ]

        x = np.linalg.inv(A) @ b
        return JMTTrajectory1d(Polynomial(np.r_[coeffs, x]), s_i, s_f, T)


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


def compute_start_state(ego, prevTraj, prevPath, numPointsToPreserve):
    ego_kinematics_s = Kinematics(ego["kinematics"][:3])
    ego_kinematics_d = Kinematics(ego["kinematics"][3:])
    if len(prevPath) == 0:
        start_state = np.array(
            [
                ego_kinematics_s.get_kinematics(0.0)[:3],
                ego_kinematics_d.get_kinematics(0.0)[:3],
            ]
        ).T
        print(f"initial start_state={start_state}")
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
    executed_time = (100 - len(prevPath) + numPointsToPreserve) * simulator_time_step

    start_state = prev_traj(executed_time)[:3, :]
    print(f"start_state={start_state}")
    return start_state


class Map:
    MAX_FRENET_S = 6945.554
    LANE_WIDTH = 4.0
    HALF_LANE_WIDTH = LANE_WIDTH / 2.0
    NUM_LANES = 3

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


def _group_vehicles(tracked_vehicles):
    grouped_vehicles = dict()
    for vehicle_id, vehicle in tracked_vehicles.items():
        lane_id = Map.get_lane_id(vehicle["kinematics"][3])
        if lane_id not in grouped_vehicles:
            grouped_vehicles[lane_id] = []
        grouped_vehicles[lane_id].append(vehicle)
    return grouped_vehicles


class LongitudinalManeuverType(enum.IntEnum):
    kVelocityKeeping = 0
    kStopping = 1
    kFollowing = 2


class LateralManeuverType(enum.IntEnum):
    kLaneKeeping = 0
    kLeftLaneChanging = 1
    kRightLaneChanging = 2


class PTG:
    @staticmethod
    def generate_lon_trajectories(lon_behavior, ego, leading_vehicle):
        if lon_behavior == LongitudinalManeuverType.kVelocityKeeping:
            return PTG._generate_velocity_keeping_trajectory(ego)
        elif lon_behavior == LongitudinalManeuverType.kStopping:
            return PTG._generate_stopping_trajectory(ego)
        elif lon_behavior == LongitudinalManeuverType.kFollowing:
            return PTG._generate_following_trajectory(ego, leading_vehicle)
        return []

    @staticmethod
    def generate_lat_trajectories(lat_behavior, ego):
        ego_lat_kinematics = Kinematics(ego["kinematics"][3:])
        ego_d = ego_lat_kinematics.kinematics.coef[0]
        target_d = Map.get_lane_center_d(Map.get_lane_id(ego_d))
        if lat_behavior == LateralManeuverType.kLeftLaneChanging:
            target_d = Map.get_lane_center_d(Map.get_lane_id(ego_d) - 1)
        elif lat_behavior == LateralManeuverType.kRightLaneChanging:
            target_d = Map.get_lane_center_d(Map.get_lane_id(ego_d) + 1)

        candidate_times = np.linspace(3.0, 5.0, 4)

        trajs, valid_indices = PTG.solve_full_constraints_1d(
            s_i=ego_lat_kinematics.kinematics.coef[:3],
            s_f=[target_d, 0.0, 0.0],
            candidate_times=candidate_times,
            candidate_delta_s=[0.0],
            k_time=1.0,
            k_pos=1.0,
            k_vel=1.0,
            k_acc=1.0,
            k_jek=1.0,
        )
        return trajs[valid_indices]

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
        return False, -1.0

    @staticmethod
    def solve_full_constraints_1d(
        s_i,
        s_f,
        candidate_times,
        candidate_delta_s,
        k_time,
        k_pos,
        k_vel,
        k_acc,
        k_jek,
        force_velocity_dir=False,
    ):
        max_vel = mph_to_mps(49.0)
        max_acc = 3
        max_jerk = 3

        trajs = []
        valid_indices = []
        for ds in candidate_delta_s:
            s_f_copy = copy.deepcopy(s_f)
            s_f_copy[0] += ds
            for t in candidate_times:
                traj = JMTSolver.solve_1d(s_i, s_f_copy, t)
                if traj.validate(
                    max_vel, max_acc, max_jerk, force_velocity_dir=force_velocity_dir
                ):
                    valid_indices.append(True)
                else:
                    valid_indices.append(False)
                traj.compute_cost(k_time, k_pos, k_vel, k_acc, k_jek)
                trajs.append(traj)
        return np.asarray(trajs), valid_indices

    @staticmethod
    def _generate_velocity_keeping_trajectory(ego):
        try:
            ego_lon_kinematics = Kinematics(ego["kinematics"][:3])
        except Exception as e:
            print(e)
            from IPython import embed

            embed()
        lon_vel = ego_lon_kinematics.kinematics.coef[1]
        # if lon_vel < mph_to_mps(10.0):
        #     target_vel = mph_to_mps(10.0)
        #     candidate_delta_s = np.linspace(10.0, 25.0, 10)
        #     candidate_times = np.linspace(5.0, 10.0, 10)
        if lon_vel < mph_to_mps(20.0):
            target_vel = mph_to_mps(20.0)
            candidate_delta_s = np.linspace(10.0, 25.0, 10)
            candidate_times = np.linspace(2.0, 7.0, 10)
        else:
            target_vel = mph_to_mps(49.0)
            candidate_delta_s = np.linspace(25.0, 40.0, 10)
            candidate_times = np.linspace(2.0, 7.0, 10)

        # print(
        #     f"generating for candidate_times={candidate_times}, candidate_delta_s={candidate_delta_s}"
        # )

        trajs, valid_indices = PTG.solve_full_constraints_1d(
            s_i=ego_lon_kinematics.kinematics.coef[:3],
            s_f=[ego_lon_kinematics.kinematics.coef[0], target_vel, 0.0],
            candidate_times=candidate_times,
            candidate_delta_s=candidate_delta_s,
            k_time=1.0,
            k_pos=1.0,
            k_vel=1.0,
            k_acc=1.0,
            k_jek=1.0,
            force_velocity_dir=True,
        )

        # JMTTrajectory1d.plot_all(trajs)

        return trajs[valid_indices]

    @staticmethod
    def _generate_stopping_trajectory(ego):
        # TODO
        return []

    @staticmethod
    def _generate_following_trajectory(ego, leading_vehicle):
        # TODO
        return []


def generate_trajectory(ego, tracked_vehicles):

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
        lon_behaviors[lane_id] = [LongitudinalManeuverType.kVelocityKeeping]
        # if lane_id in grouped_vehicles:
        #     vehicles = grouped_vehicles[lane_id]

    lat_behaviors = dict()
    lat_behaviors[ego_lane_id] = LateralManeuverType.kLaneKeeping
    if "left" in lane_ids_for_planning:
        lat_behaviors[left_lane_id] = LateralManeuverType.kLeftLaneChanging
    if "right" in lane_ids_for_planning:
        lat_behaviors[right_lane_id] = LateralManeuverType.kRightLaneChanging

    best_traj = None
    best_cost = None
    trajs = []
    for lane_name, lane_id in lane_ids_for_planning.items():
        lon_trajs = []
        lat_trajs = []

        if lane_id not in lon_behaviors or lane_id not in lat_behaviors:
            print(f"skipping lane {lane_id}")
            continue

        print(f"planning for lane {lane_id}")

        for lon_behavior in lon_behaviors[lane_id]:
            print(f"generating behavior for {str(lon_behavior)}.")
            lon_trajs.extend(
                PTG.generate_lon_trajectories(
                    lon_behavior, ego, leading_vehicles.get(lane_id)
                )
            )

        lat_trajs.extend(PTG.generate_lat_trajectories(lat_behaviors[lane_id], ego))

        if len(lon_trajs) == 0:
            print("len(lon_trajs) == 0, planning failed!")
            continue

        if len(lat_trajs) == 0:
            print("len(lat_trajs) == 0, planning failed!")
            continue

        lon_id, lat_id, cost = PTG.get_optimal_combination(lon_trajs, lat_trajs)

        traj = JMTTrajectory2d(lon_trajs[lon_id], lat_trajs[lat_id])
        trajs.append(traj)

        # TODO
        # is_in_collision, distance = PTG.check_collision(traj, tracked_vehicles)
        # if is_in_collision:
        #     print("In collision!")
        #     continue

        if best_cost is None or cost < best_cost:
            best_cost = cost
            best_traj = traj

    # JMTTrajectory2d.plot_all(trajs)

    print(f"Trajectory generation tool {time.time() - start_time:.3f} seconds")

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
