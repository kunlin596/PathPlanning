#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import json
import argparse


def main(filename):
    with open(filename, "r") as f:
        data = json.load(f)

    plt.plot(data['next_x'], data['next_y'], linewidth=1.0)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Visualize PTG File")
    parser.add_argument("filename", type=str, nargs='+', help="PTG filenames")
    args = parser.parse_args()

    plt.figure(0)
    plt.ioff()
    for filename in args.filename:
        if os.path.isdir(filename):
            for f in os.listdir(filename):
                main(os.path.join(filename, f))

    plt.ion()
    plt.show(block=False)
    from IPython import embed

    embed()
