#!/usr/bin/env python3
import csv
import math
import numpy as np
from scipy import interpolate


class Map:
    max_s = 6945.554

    def __init__(self, filename):
        data = []
        with open(filename) as csvfile:
            spamreader = csv.reader(csvfile, delimiter=' ')
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

        return [s, d]

    def get_xy(self, s, d):
        return [
            self.sx_spline(s) + d * self.sdx_spline(s),
            self.sy_spline(s) + d * self.sdy_spline(s),
        ]


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    m = Map('../../data/highway_map.csv')

    plt.figure(0)
    plt.plot(m.x, m.y, color='r', label='center')

    num_points = 1000
    ds = Map.max_s / num_points
    lane_left_boundary = []
    lane_right_boundary = []
    for i in range(num_points):
        x, y = m.get_xy(ds * i, -12)
        lane_left_boundary.append([x, y])
        x, y = m.get_xy(ds * i, 12)
        lane_right_boundary.append([x, y])

    lane_left_boundary = np.asarray(lane_left_boundary)
    lane_right_boundary = np.asarray(lane_right_boundary)

    plt.plot(
        lane_left_boundary[:, 0],
        lane_left_boundary[:, 1],
        color='g',
        label='left boundary',
    )
    plt.plot(
        lane_right_boundary[:, 0],
        lane_right_boundary[:, 1],
        color='b',
        label='right boundary',
    )

    plt.axis('equal')
    plt.ion()
    plt.show(block=False)

    from IPython import embed

    embed()
