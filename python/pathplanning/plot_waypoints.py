#!/usr/bin/env python3
import matplotlib

# matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import numpy as np
import json
import argparse
import os
import time

from common import evaluate_polynomial, evaluate_polynomial_2d


def plot_xy(filename, gca):
    with open(filename, "r") as f:
        data = json.load(f)

    numPointsToPreserve = data.get('numPointsToPreserve', len(data['next_x']))
    color = np.random.random(3)
    gca.plot(data['next_x'], data['next_y'], linewidth=1.0)
    gca.plot(
        data['next_x'][:numPointsToPreserve],
        data['next_y'][:numPointsToPreserve],
        marker='.',
        color=color / 3,
        linewidth=0,
        markersize=20.0,
        alpha=0.7,
        label=f'prev_{id(filename)}_{len(data["next_x"][:numPointsToPreserve])}',
    )
    gca.plot(
        data['next_x'][numPointsToPreserve:],
        data['next_y'][numPointsToPreserve:],
        marker='.',
        color=color,
        linewidth=0,
        markersize=30.0,
        alpha=0.7,
        label=f'new_{id(filename)}_{len(data["next_x"][numPointsToPreserve:])}',
    )

    input()


def plot_sd(filename, gca):
    with open(filename, "r") as f:
        data = json.load(f)

    handles = []

    # for v in data.get('trackedVehicles', []):
    #     gca.plot(
    #         [v['conf'][0]],
    #         [v['conf'][3]],
    #         label=v['id'],
    #         marker='.',
    #         markersize=30,
    #         color='r',
    #         linewidth=0,
    #     )

    costs = np.array(data['allCosts'], dtype=float)
    costs[np.isnan(costs)] = 1e6
    # from IPython import embed

    # embed()
    colors = costs.copy()
    colors[colors > 100.0] = 0.0
    if len(colors[colors < 100.0]) > 0:
        colors[colors < 100.0] /= colors[colors < 100.0].max()
    for i, traj2d in enumerate(data.get('allTraj', [])):
        traj1 = traj2d['traj1']
        traj2 = traj2d['traj2']
        if np.isnan(np.array(traj1['func5'], dtype=float)).any():
            continue
        if np.isnan(np.array(traj2['func5'], dtype=float)).any():
            continue
        times, svalues, dvalues = evaluate_polynomial_2d(
            traj1['func5'], traj2['func5'], traj1['time']
        )
        if costs[i] < 1000:
            handles.extend(
                gca.plot(
                    svalues, dvalues, alpha=0.5, color=[0, colors[i], 0], label=costs[i]
                )
            )
        else:
            handles.extend(
                gca.plot(svalues, dvalues, alpha=0.2, color=[0, colors[i], 0])
            )

    for i, g in enumerate(data.get('allGoals', [])):
        handles.extend(
            gca.plot(
                g[0],
                g[3],
                marker='.',
                markersize=20,
                color=[0, colors[i], 0],
                linewidth=0,
                alpha=0.5,
            )
        )

    startConf = data['startConf']
    handles.extend(
        gca.plot(
            startConf[0],
            startConf[3],
            marker='.',
            markersize=30,
            color='g',
            linewidth=0,
        )
    )

    goalConf = data.get('goalConf')
    if goalConf is not None:
        handles.extend(
            gca.plot(
                goalConf[0],
                goalConf[3],
                marker='.',
                markersize=30,
                color='b',
                linewidth=0,
            )
        )

    traj1 = data['bestTrajectory']['traj1']
    traj2 = data['bestTrajectory']['traj2']
    # if np.isnan(np.array(traj1['func5'], dtype=float)).any():
    #     continue
    # if np.isnan(np.array(traj2['func5'], dtype=float)).any():
    #     continue

    times, svalues, dvalues = evaluate_polynomial_2d(
        traj1['func5'], traj2['func5'], traj1['time']
    )
    handles.append(
        gca.plot(svalues, dvalues, linewidth=1.0, color='b', markersize=2.0, marker='.')
    )

    gca.legend()
    # from IPython import embed
    # embed()

    input()

    return handles


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Visualize Trajectory File")
    parser.add_argument("filename", type=str, nargs='+', help="Waypoins file names")
    args = parser.parse_args()

    plt.figure(0)
    plt.axis('equal')
    plt.tight_layout()
    # plt.ioff()
    plt.ion()
    plt.show(block=False)
    for filename in args.filename:
        if os.path.isdir(filename):
            for f in sorted(os.listdir(filename)):
                handles = plot_sd(os.path.join(filename, f), plt.gca())
                plt.gca().clear()

    from IPython import embed

    embed()
