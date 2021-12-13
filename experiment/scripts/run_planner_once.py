#!/usr/bin/env python3
import os
import sys
import time
import json
import matplotlib.pyplot as plt
import numpy as np
import pathmagic
with pathmagic.context(EXTERNAL_FLAG=True):
    import DrMaMP
    from SimulatorAimsLab import SimulatorAimsLab
    import csv_helper
    from discrete_path_to_time_traj import discrete_path_to_time_traj, plot_traj


if __name__ == "__main__":
    # load mambo index from command line arguments
    if len(sys.argv) == 2:
        mambo_idx = sys.argv[1]
    else:
        mambo_idx = 1

    # load configuration
    file_name = os.getcwd() + "/experiment/config_aimslab_ex_" + str(mambo_idx) + ".json"
    config_dict = json.load(open(file_name))

    # create a simulator
    MySimulator = SimulatorAimsLab(map_resolution=20)
    # convert 2D numpy array to 1D list
    world_map = MySimulator.map_array.flatten().tolist()

    # generate agent and targets manually in qualisys coordinates (meter)
    height = 1.0  # height is fixed during the flight

    agent_position_qualisys = [[-1.8, -0.9, height]]
    targets_position_qualisys = [[0.2, -0.4, height], [1.8, 0.9, height], [0.5, 0.8, height]]

    # agent_position_qualisys = [[-1.8, +0.9, height]]
    # targets_position_qualisys = [[0.2, +0.4, height], [1.8, -0.9, height], [0.5, -0.8, height]]

    # transform qualisys coordinates (meter) to map array (index)
    t0 = time.time()
    agent_position_index = MySimulator.qualisys_to_map_index_all(agent_position_qualisys)
    targets_position_index = MySimulator.qualisys_to_map_index_all(targets_position_qualisys)
    t1 = time.time()
    print("Qualisys coordinates to map index. Time used [sec]: " + str(t1 - t0))

    # high-level planning
    t0 = time.time()
    path_index, task_allocation_result = DrMaMP.SolveOneAgent(
        agent_position_index, targets_position_index, world_map, MySimulator.map_width, MySimulator.map_height)
    t1 = time.time()
    print("High-level planning. Time used [sec]: " + str(t1 - t0))

    # transform  map array (index) to qualisys coordinates (meter)
    t0 = time.time()
    path_qualisys = MySimulator.path_index_to_qualisys(path_index, height)
    t1 = time.time()
    print("Map index to qualisys coordinate. Time used [sec]: " + str(t1 - t0))

    # generate position and velocity trajectories
    dt = 0.1
    velocity_ave = 0.25
    # generate trajectories
    time_queue_vec, position_traj, velocity_traj = discrete_path_to_time_traj(
        path_qualisys, dt, velocity_ave, interp_kind='linear',
        velocity_flag=True, ini_velocity_zero_flag=True)

    # remove all the existing files in the trajectory directory
    directory_delete = os.path.expanduser("~") + "/Mambo-Tracking-Interface" + config_dict["DIRECTORY_TRAJ"] + "*"
    csv_helper.remove_traj_ref_lib(directory_delete)

    # output trajectories as a CSV file
    array_csv = np.vstack((time_queue_vec, np.array(position_traj).T, np.array(velocity_traj).T))
    filename_csv = os.path.expanduser("~") + "/Mambo-Tracking-Interface" + config_dict["DIRECTORY_TRAJ"] + "traj.csv"
    np.savetxt(filename_csv, array_csv, delimiter=",")

    # plot path, and position/velocity trajectories
    plot_traj(path_qualisys, time_queue_vec, position_traj, velocity_traj)
    plt.show()
