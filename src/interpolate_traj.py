#!/usr/bin/env python3
import numpy as np
from scipy import interpolate


def interpolate_traj(x_queue, x, y, interpolate_kind):
# if interpolate_kind == 'traj'
# interpolate the desired trajectory by time value
# x_queue is the interpolated point/points
# if x_queue is 1-D numpy array, then the output is 2-D numpy array
# if x_queue is a float, then the output is 2-D numpy array, but only has one column
# x is 1-D numpy array (1 by n), y is 2-D numpy array (m by n)
# default is linear interpolation
# when x_queue exceeds the range of x, function returns the boundary value of y

# if interpolate_kind == 'cmd'
# interpolate the commands by the machine time by Zero-Order Hold
# x_queue is the interpolated machine time
# assume x_queue is always 1-D numpy array, and the output is 2-D numpy array
# time before the first timestamp, commands are zeros
# time after the last timestamp, commands are the last ones.

    if interpolate_kind == 'traj':
        boundary = (y[:, 0], y[:, -1])
        f = interpolate.interp1d(x, y, kind='linear', bounds_error=False, fill_value=boundary)
        y_raw = f(x_queue)
        if isinstance(x_queue, float):
            y_queue = y_raw.reshape(y_raw.shape[0], -1)
        else:
            y_queue = y_raw

    elif interpolate_kind == 'cmd':
        boundary = (np.zeros(4), y[:, -1])
        f = interpolate.interp1d(x, y, kind='zero', bounds_error=False, fill_value=boundary)
        y_queue = f(x_queue)

    else:
        y_queue = []
        raise Exception("The interpolation type is wrong!")

    return y_queue


if __name__ == "__main__":
    x = np.array([0, 2, 4, 6, 8, 10])
    y = np.array([[0, 1, 2, 3, 4, 5], [0, -1, -2, -3, -4, -5], [10, 20, 30, 40, 50, 60], [-10, -20, -30, -40, -50, -60], [0.5, 1.5, 2.5, 3.5, 4.5, 5.5]])
    x_queue = 1
    print(x)
    print(y)
    y_queue = interpolate_traj(x_queue, x, y, 'traj')
    print(y_queue)
