#!/usr/bin/env python3
import time
import matplotlib.pyplot as plt
import pathmagic
with pathmagic.context():
    from Simulator import Simulator
    import GA_Solver


if __name__ == "__main__":
    # define the world
    map_width_meter = 25.0
    map_height_meter = 25.0
    map_resolution = 2
    value_non_obs = 0  # the cell is empty
    value_obs = 255  # the cell is blocked
    # create a simulator
    MySimulator = Simulator(map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)
    # number of obstacles
    num_obs = 250
    # [width, length] size of each obstacle [meter]
    size_obs = [1, 1]
    # generate random obstacles
    MySimulator.generate_random_obs(num_obs, size_obs)
    # convert 2D numpy array to 1D list
    world_map = MySimulator.map_array.flatten().tolist()

    # This is for an agent and a set of targets
    num_agents = 1
    num_targets = 5
    agent_position, targets_position = MySimulator.generate_agents_and_targets(num_agents, num_targets)

    # some hyper-parameters for Genetic Algorithm
    population_size = 25
    max_iter = 25

    t0 = time.time()
    # solve it
    path_many_result, task_order, cost = GA_Solver.SolveSingleAgent(
        agent_position, targets_position,
        population_size, max_iter,
        world_map, MySimulator.map_width, MySimulator.map_height)
    t1 = time.time()
    print("Task allocation and collision-free path for a single agent. Time used [sec]:" + str(t1 - t0))
    # for i in range(len(path_many_result)):
    #     print("path")
    #     for j in range(0,len(path_many_result[i]),2):
    #         str_print = str(path_many_result[i][j]) + ', ' + str(path_many_result[i][j+1])
    #         print(str_print)

    print("task_order, 0 means the first task")
    for i in range(len(task_order)):
        print(task_order[i])

    # visualization
    MySimulator.plot_path_multi_agent([path_many_result], agent_position, targets_position, [task_order], [], [])
    plt.show()
