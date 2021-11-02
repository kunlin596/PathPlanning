#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import json
import argparse

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


def plot1d(data):
    plt.figure()
    plt.ioff()

    for i in range(6):
        plt.subplot(6, 1, i + 1)
        plt.title(NAMES[i])
        times, values = evaluate_polynomial(data[f"func{5 - i}"], data["time"])
        plt.plot(times, values)

    plt.ion()
    plt.show(block=False)


def plot2d(data):
    plt.figure(0)
    plt.ion()

    points = []
    for dim, traj_data in enumerate(data):
        times, values = evaluate_polynomial(traj_data["func5"], traj_data["time"])
        points.append(values)

    points = np.array(points).T

    plt.plot(points[:, 0], points[:, 1], "r.", markersize=1)
    plt.show(block=False)

    # plot1d(data[0])
    # plot1d(data[1])

    from IPython import embed

    embed()


def main(filename):
    with open(filename, "r") as f:
        data = json.load(f)

    if isinstance(data, list):
        plot2d(data)
    else:
        plot1d(data)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Visualize Trajectory File")
    parser.add_argument("--filename", "-f", type=str, help="JMT trajectory file names")
    args = parser.parse_args()

    main(args.filename)
