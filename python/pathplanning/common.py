import numpy as np


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


def evaluate_polynomial_2d(xcoeffs, ycoeffs, time, time_step=0.01):
    poly1 = np.polynomial.Polynomial(xcoeffs)
    poly2 = np.polynomial.Polynomial(ycoeffs)
    curr_time = 0.0
    times = []
    xvalues = []
    yvalues = []
    while curr_time < time + 1e-6:
        xvalues.append(poly1(curr_time))
        yvalues.append(poly2(curr_time))
        times.append(curr_time)
        curr_time += time_step

    return times, xvalues, yvalues
