#!/usr/bin/env python3
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def discrete_path_to_time_traj(path: list, dt: float, velocity_ave: float, interp_kind: str,
                               velocity_flag=False, ini_velocity_zero_flag=False):
    # if the path is not empty
    if path:
        dimension = len(path[0])
        # 1d array, each element is the distance between the current node position and the previou node position
        distance_vec = np.sum(np.abs(np.diff(path, axis=0))**2, axis=-1) ** 0.5
        # 1d array, each element is the duration between two adjacent nodes
        duration_vec = distance_vec / velocity_ave

        # time stamp vector for input path
        time_vec = [0.0]
        time_vec.extend(np.cumsum(duration_vec))

        # generate a queue time trajectory, the time interval is dt
        t_end = round(time_vec[-1] - time_vec[-1] % dt, 2)
        time_queue_vec = np.append(np.arange(time_vec[0], t_end, dt), round(time_vec[-1], 2))

        boundary = (path[0], path[-1])
        f = interpolate.interp1d(time_vec, path, kind=interp_kind,
            bounds_error=False, fill_value=boundary, axis=0)

        # each row is a instance of position
        position_traj = f(time_queue_vec)

        if velocity_flag:
            # compute the gradient of position as velocity
            velocity_traj = np.gradient(position_traj, time_queue_vec, axis=0, edge_order=2)
            # the velocity at start and goal are zeros
            if ini_velocity_zero_flag:
                velocity_traj[0] = np.zeros(dimension)
            velocity_traj[-1] = np.zeros(dimension)
            return time_queue_vec, position_traj, velocity_traj
        else:
            return time_queue_vec, position_traj
    else:
        print("This path is empty. The route is infeasible.")
        time_queue_vec = list()
        position_traj = list()
        if velocity_flag:
            velocity_traj = list()
            return time_queue_vec, position_traj, velocity_traj
        else:
            return time_queue_vec, position_traj


def plot_traj(path: list, time_queue_vec, position_traj, velocity_traj):
    dimension = len(path[0])
    # plot path, and position/velocity trajectories
    if dimension == 2:
        path_x, path_y = np.array(path).T
        position_x, position_y = np.array(position_traj).T

        fig1, ax1 = plt.subplots(1, 1)
        ax1.scatter(path_x, path_y, marker='o', color='red', label="Discrete path")
        ax1.plot(position_x, position_y, color='blue', label="Position trajectory")
        ax1.legend(loc="upper left")
        ax1.set_xlabel("x")
        ax1.set_ylabel("y")
    elif dimension == 3:
        path_x, path_y, path_z = np.array(path).T
        position_x, position_y, position_z = np.array(position_traj).T

        fig1 = plt.figure()
        ax1 = fig1.gca(projection="3d", title="Discrete path and position trajectory")
        ax1.scatter(path_x, path_y, path_z, marker='o', color='red', label="Discrete path")
        ax1.plot(position_x, position_y, position_z, color='blue', label="Position trajectory")
        ax1.legend(loc="upper left")
        ax1.set_xlabel("X Label")
        ax1.set_ylabel("Y Label")
        ax1.set_zlabel("Z Label")
        max_range = np.array([path_x.max()-path_x.min(), path_y.max()-path_y.min(), path_z.max()-path_z.min()]).max() / 2.0
        mid_x = (path_x.max() + path_x.min()) * 0.5
        mid_y = (path_y.max() + path_y.min()) * 0.5
        mid_z = (path_z.max() + path_z.min()) * 0.5
        ax1.set_xlim(mid_x - max_range, mid_x + max_range)
        ax1.set_ylim(mid_y - max_range, mid_y + max_range)
        ax1.set_zlim(mid_z - max_range, mid_z + max_range)
    else:
        raise Exception("Path dimension only support 2 and 3!")

    if dimension == 2:
        fig2, (ax2, ax3) = plt.subplots(2, 1)
        velocity_x, velocity_y = np.array(velocity_traj).T
    elif dimension == 3:
        fig2, (ax2, ax3, ax4) = plt.subplots(3, 1)
        velocity_x, velocity_y, velocity_z = np.array(velocity_traj).T
    else:
        raise Exception("Path dimension only support 2 and 3!")
    fig2.suptitle("Velocity trajectory")

    ax2.plot(time_queue_vec, velocity_x, color="blue", label="velocity in x")
    ax2.legend(loc="upper left")
    ax2.set_xlabel("time [sec]")
    ax2.set_ylabel("vx")

    ax3.plot(time_queue_vec, velocity_y, color="blue", label="velocity in y")
    ax3.legend(loc="upper left")
    ax3.set_xlabel("time [sec]")
    ax3.set_ylabel("vy")

    if dimension == 3:
        ax4.plot(time_queue_vec, velocity_z, color="blue", label="velocity in z")
        ax4.legend(loc="upper left")
        ax4.set_xlabel("time [sec]")
        ax4.set_ylabel("vz")
    plt.tight_layout()
    plt.show(block=False)


if __name__ == "__main__":
    path = [[1,2], [2,5], [3,8], [5,10], [12,12]]
    dt = 0.1
    velocity_ave = 0.5

    # call discrete_path_to_time_traj to generate trajectories
    time_queue_vec, position_traj, velocity_traj = discrete_path_to_time_traj(
        path, dt, velocity_ave, interp_kind='linear', velocity_flag=True,
        ini_velocity_zero_flag=True)

    # plot path, and position/velocity trajectories
    plot_traj(path, time_queue_vec, position_traj, velocity_traj)
    plt.show()
