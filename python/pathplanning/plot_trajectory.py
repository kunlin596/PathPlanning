#!/usr/bin/env python3
import matplotlib

matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import numpy as np
import json
import argparse
import os

NAMES = ["position", "velocity", "accelaration", "jerk", "snap", "crackle"]
np.set_printoptions(suppress=True, precision=6)


def evaluate_polynomial(coeffs, time, time_step=0.01):
    poly = np.polynomial.Polynomial(coeffs)
    curr_time = 0.0
    times = []
    values = []
    while curr_time < time + 1e-6:
        values.append(poly(curr_time))
        times.append(curr_time)
        curr_time += time_step
    return times, values


def plot2d(data):

    fig = plt.figure(0)
    gs = fig.add_gridspec(6, 4)
    axis2d = fig.add_subplot(gs[:, :2])
    axes1 = [fig.add_subplot(gs[i, 2]) for i in range(6)]
    for index, axis in enumerate(axes1):
        axis.set_title(NAMES[index])

    axes2 = [fig.add_subplot(gs[i, 3]) for i in range(6)]
    for index, axis in enumerate(axes2):
        axis.set_title(NAMES[index])

    plt.ioff()

    for traj_data in data:
        points = []
        for key, traj_data_1d in traj_data.items():
            dim = int(key[-1])
            times, values0 = evaluate_polynomial(
                traj_data_1d["func0"], traj_data_1d["time"]
            )
            times, values1 = evaluate_polynomial(
                traj_data_1d["func1"], traj_data_1d["time"]
            )
            times, values2 = evaluate_polynomial(
                traj_data_1d["func2"], traj_data_1d["time"]
            )
            times, values3 = evaluate_polynomial(
                traj_data_1d["func3"], traj_data_1d["time"]
            )
            times, values4 = evaluate_polynomial(
                traj_data_1d["func4"], traj_data_1d["time"]
            )
            times, values5 = evaluate_polynomial(
                traj_data_1d["func5"], traj_data_1d["time"]
            )

            if dim == 1:
                for index, values in enumerate(
                    [values5, values4, values3, values2, values1, values0]
                ):
                    axes1[index].plot(times, values)
            elif dim == 2:
                for index, values in enumerate(
                    [values5, values4, values3, values2, values1, values0]
                ):
                    axes2[index].plot(times, values)

            points.append(values5)
        points = np.array(points).T

        axis2d.plot(points[:, 0], points[:, 1])
        axis2d.axis('equal')

    plt.ion()
    plt.tight_layout()
    plt.show(block=True)


def main(data):
    plot2d(data)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Visualize Trajectory File")
    parser.add_argument(
        "filename", type=str, nargs='+', help="JMT trajectory file names"
    )
    data = []

    args = parser.parse_args()
    for filename in args.filename:
        if os.path.isdir(filename):
            for filename2 in sorted(os.listdir(filename)):
                abspath = os.path.join(filename, filename2)
                print(abspath)
                with open(abspath, "r") as file:
                    data.append(json.load(file))

    main(data)
