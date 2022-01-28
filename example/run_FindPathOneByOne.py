#!/usr/bin/env python3
import numpy as np
import time
from math import sqrt
import matplotlib.pyplot as plt
import pathmagic
with pathmagic.context():
    from Simulator import Simulator
    import DrMaMP


if __name__ == "__main__":
    # define the world
    map_width_meter = 25.0
    map_height_meter = 25.0
    map_resolution = 2
    value_non_obs = 0 # the cell is empty
    value_obs = 255 # the cell is blocked
    # create a simulator
    MySimulator = Simulator(map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)
    # number of obstacles
    num_obs = 150
    # [width, length] size of each obstacle [meter]
    size_obs = [1, 1]
    # generate random obstacles
    MySimulator.generate_random_obs(num_obs, size_obs)
    # convert 2D numpy array to 1D list
    world_map = MySimulator.map_array.flatten().tolist()

    # This is for an agent and a set of targets
    num_agents = 1
    num_targets = 4
    agent_position, targets_position = MySimulator.generate_agents_and_targets(num_agents, num_targets)
    t0 = time.time()
    # solve it
    path_many, distances_many = DrMaMP.FindPathOneByOne(agent_position, targets_position, world_map, MySimulator.map_width, MySimulator.map_height)
    t1 = time.time()
    print("These are all the paths. Time used [sec]:" + str(t1 - t0))
    for i in range(0,len(path_many),1):
        print("This is a path.")
        print("Total distance: " + str(distances_many[i]))
        distance_check = 0.0
        for j in range(0,len(path_many[i]),2):
            str_print = str(path_many[i][j]) + ', ' + str(path_many[i][j+1])
            print(str_print)

            if j > 0:
                distance_check = distance_check + sqrt((path_many[i][j]-path_many[i][j-2])**2 + (path_many[i][j+1]-path_many[i][j-1])**2)
        print("Distance computed afterwards: " + str(distance_check))

    # visualization
    # task_allocation_result is [0, 1, 2, 3, ..., num_targets]
    task_allocation_result = [*range(0, num_targets+1)]
    MySimulator.plot_paths([path_many], agent_position, targets_position, task_allocation_result, [], [])
    plt.show()
    