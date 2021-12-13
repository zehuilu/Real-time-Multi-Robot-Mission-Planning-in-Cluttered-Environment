#!/usr/bin/env python3
import os
import time
import matplotlib.pyplot as plt
import pathmagic
with pathmagic.context():
    from Simulator import Simulator
    import DrMaMP
    import CBBA_Path_Finding
    from compute_path_distance import compute_path_distance_one_agent, compute_path_distance_many_agents


if __name__ == "__main__":
    # a json configuration file for CBBA
    cbba_config_file_name = os.getcwd() + "/example/cbba_config.json"

    # define the world
    map_width_meter = 25.0
    map_height_meter = 25.0
    map_resolution = 2
    value_non_obs = 0  # the cell is empty
    value_obs = 255  # the cell is blocked
    # create a simulator
    Simulator = Simulator(map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)
    # number of obstacles
    num_obs = 50
    # [width, length] size of each obstacle [meter]
    size_obs = [1, 1]
    # generate random obstacles
    Simulator.generate_random_obs(num_obs, size_obs)
    # convert 2D numpy array to 1D list
    world_map = Simulator.map_array.flatten().tolist()

    # generate agents and targets randomly
    num_agents = 8
    num_targets = 40
    agent_position, targets_position = Simulator.generate_agents_and_targets(num_agents, num_targets)

    # parameters for my algorithm
    num_cluster = num_agents
    number_of_iterations = 300

    # my algorithm
    t0 = time.time()
    path_all_agents, task_allocation_all_agents, cluster_centers, points_idx_for_clusters, cluster_assigned_idx\
        = DrMaMP.MissionPlanning(agent_position, targets_position, num_cluster,
                                             number_of_iterations, world_map, Simulator.map_width,
                                             Simulator.map_height)
    t1 = time.time()
    print("My algorithm time used [sec]:" + str(t1 - t0))
    my_distance, my_distance_list = compute_path_distance_many_agents(path_all_agents)
    print("my total distance")
    print(my_distance)
    print("my_distance_list")
    print(my_distance_list)

    # visualization
    Simulator.plot_path_multi_agent(path_all_agents, agent_position, targets_position, task_allocation_all_agents, cluster_centers, points_idx_for_clusters)
    Simulator.plot_cluster_assign(agent_position, targets_position, points_idx_for_clusters, cluster_centers, cluster_assigned_idx)

    # CBBA + path finding
    t0 = time.time()
    agent_position_list, task_position_list, path_all_agents, task_allocation_list, AgentList, TaskList =\
        CBBA_Path_Finding.Solve(agent_position, targets_position, Simulator, cbba_config_file_name, plot_flag=False)
    t1 = time.time()
    print("CBBA + path finding time used [sec]:" + str(t1 - t0))
    cbba_distance, cbba_distance_list = compute_path_distance_many_agents(path_all_agents)
    print("CBBA distance_list")
    print(cbba_distance_list)
    print("CBBA total distance")
    print(cbba_distance)
    # For Simulator.plot_path_multi_agent(), each sub list of the input argument
    # task_allocation_result_many_agents is the task allocation order for each agent.
    # The first entry is the index of agent.
    for idx in range(len(task_allocation_list)):
        task_allocation_list[idx].insert(0, idx)
    # plot
    Simulator.plot_path_multi_agent(path_all_agents, agent_position, targets_position, task_allocation_list, [], [])

    plt.show()
