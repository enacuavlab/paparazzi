#!/usr/bin/env python3

import sampleParser
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import argparse
import typing
import os

import gasSpiralModel

def show_raw_samples(samples: np.array) -> None:

    # Plot acquired data
    xs = samples[:, 0]
    ys = samples[:, 1]
    zs = samples[:, 2]
    vals = samples[:, 3]+1e-9

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    # Raw datapoints
    im = ax.scatter(xs, ys, zs, c=vals, marker='8', cmap='magma_r',
                    norm='log', plotnonfinite=True, alpha=0.7)
    cbar = ax.figure.colorbar(im, ax=ax)
    cbar.ax.set_ylabel("Gas concentration (a.u., log scale)",
                       rotation=-90, va="bottom")

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.legend()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        prog="Gas Samples displayer"
    )

    parser.add_argument("file", help="File to read gas samples from.\
        If it has extension '.pkl', use the pickle reader. Otherwise, assume TopoJSON.")
    parser.add_argument('--spiral',dest='spiral',action='store_true',default=False,help="Flag to activate modeling for spiral-like samples")

    args = parser.parse_args()

    _, ext = os.path.splitext(args.file)
    if ext == ".pkl":
        reader = sampleParser.SampleReader_pickle(args.file)
    else:
        reader = sampleParser.SampleReader_TopoJSON(args.file)

    samples = np.array(
        [np.array([s.x, s.y, s.z, s.val], dtype=float) for s in reader])

    if args.spiral:
        gasSpiralModel.show_samples(samples)
    else:
        show_raw_samples(samples)


if __name__ == "__main__":
    main()
