#!/usr/bin/env python3
import matplotlib

matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
import numpy as np
import json
import argparse
import os
from map import Map
from common import evaluate_polynomial


NAMES = ["position", "velocity", "acceleration", "jerk", "snap", "crackle"]

np.set_printoptions(suppress=True, precision=6)


def plot2d(data):
    m = Map("../data/highway_map.csv")

    fig0 = plt.figure(0)
    fig1 = plt.figure(1)

    gs = fig1.add_gridspec(6, 2)
    axes1 = [fig1.add_subplot(gs[i, 0]) for i in range(6)]
    for index, axis in enumerate(axes1):
        axis.set_title(NAMES[index])

    axes2 = [fig1.add_subplot(gs[i, 1]) for i in range(6)]
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

        xy = m.get_xy(points[:, 0], points[:, 1])
        # fig0.gca().plot(m.x, m.y, color='g')
        fig0.gca().plot(xy[:, 0], xy[:, 1])
        fig0.gca().axis("equal")

    plt.ion()
    plt.tight_layout()
    plt.show(block=True)
    from IPython import embed

    embed()


def main(data):

    plt.figure(0)
    plt.ioff()
    i = 1
    for j in range(6):
        for stepid, stepdata in data.items():
            plt.subplot(6, 3, i)
            if 0 in stepdata:
                for lanedata in stepdata[0]:
                    times, values = evaluate_polynomial(
                        lanedata[NAMES[j]], lanedata["time"]
                    )
                    plt.plot(times, values, label=lanedata["cost"])
                    plt.legend()
            plt.subplot(6, 3, i + 1)
            if 1 in stepdata:
                for lanedata in stepdata[1]:
                    times, values = evaluate_polynomial(
                        lanedata[NAMES[j]], lanedata["time"]
                    )
                    plt.plot(times, values, label=lanedata["cost"])
                    plt.legend()
            plt.subplot(6, 3, i + 2)
            if 2 in stepdata:
                for lanedata in stepdata[2]:
                    times, values = evaluate_polynomial(
                        lanedata[NAMES[j]], lanedata["time"]
                    )
                    plt.plot(times, values, label=lanedata["cost"])
                    plt.legend()
            i += 3
    plt.ion()
    plt.show(block=True)


# from IPython import embed

# embed()

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Visualize Trajectory File")
    parser.add_argument(
        "filename", type=str, nargs="+", help="JMT trajectory file names"
    )

    data = dict()
    args = parser.parse_args()
    for filename in args.filename:
        if os.path.isdir(filename):
            for filename2 in sorted(os.listdir(filename)):
                abspath = os.path.join(filename, filename2)
                components = filename2.split("_")
                step = int(components[0])
                laneid = int(components[1])
                sampleid = int(components[2])
                with open(abspath, "r") as file:
                    if step not in data:
                        data[step] = dict()
                    if laneid not in data[step]:
                        data[step][laneid] = []
                    data[step][laneid].append(json.load(file))

    print(f"Found {len(data)}")
    key = next(iter(data.keys()))
    main({key: data[key]})
