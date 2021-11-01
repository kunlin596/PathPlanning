#!/usr/bin/env python3
import numpy as np
from numpy.polynomial import Polynomial

np.set_printoptions(precision=5, suppress=True)


def JMT(start, end, T):
    T2 = T * T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T

    A = np.array([[T3, T4, T5], [3 * T2, 4 * T3, 5 * T4], [6 * T, 12 * T2, 20 * T3]])

    coeffs = [start[0], start[1], start[2] * 0.5]

    b = [
        end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T2),
        end[1] - (start[1] + start[2] * T),
        end[2] - start[2],
    ]

    x = np.linalg.inv(A) @ b
    # For poly1d the coeff start from the highest order term
    return np.r_[coeffs, x][::-1]


if __name__ == '__main__':
    time = 1.0
    startState = [124.834, 0.000, 0.000, 6.165, 0.000, 0.000]
    propoState = [134.834, 21.875, 0.000, 6.165, 0.000, 0.000]

    sPolyCoeffs = JMT(startState[:3], propoState[:3], time)
    dPolyCoeffs = JMT(startState[3:], propoState[3:], time)

    s = np.poly1d(sPolyCoeffs)
    d = np.poly1d(dPolyCoeffs)
    t = np.arange(0, time, 0.02)

    ss = [s(tt) for tt in t]
    dd = [d(tt) for tt in t]

    import matplotlib.pyplot as plt

    plt.plot(t, ss)
    plt.plot(t, dd)
    plt.ion()
    plt.show(block=False)

    from IPython import embed

    embed()
