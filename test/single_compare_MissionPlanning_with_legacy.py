#!/usr/bin/env python3
import time
import matplotlib.pyplot as plt
import pathmagic
with pathmagic.context():
    import DrMaMP
    from Simulator import Simulator


if __name__ == "__main__":
    # define the world
    map_width_meter = 25.0
    map_height_meter = 25.0
    map_resolution = 2
    value_non_obs = 0  # the cell is empty
    value_obs = 255  # the cell is blocked
    # create a simulator
    Simulator = Simulator(map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)
    # number of obstacles
    num_obs = 20
    # [width, length] size of each obstacle [meter]
    size_obs = [1, 1]
    # generate random obstacles
    Simulator.generate_random_obs(num_obs, size_obs)
    # convert 2D numpy array to 1D list
    world_map = Simulator.map_array.flatten().tolist()

    # generate agents and targets randomly
    num_agents = 10
    num_targets = 50
    agent_position, targets_position = Simulator.generate_agents_and_targets(num_agents, num_targets)

    # parameters for k-means
    num_cluster = num_agents
    number_of_iterations = 200

    # legacy method
    t0 = time.time()
    path_all_agents, task_allocation_all_agents, cluster_centers, points_idx_for_clusters, cluster_assigned_idx\
        = DrMaMP.MissionPlanning_legacy(agent_position, targets_position, num_cluster,
                                             number_of_iterations, world_map, Simulator.map_width,
                                             Simulator.map_height)
    t1 = time.time()
    print("Legacy time [sec]:" + str(t1 - t0))
    # visualization
    Simulator.plot_path_multi_agent(path_all_agents, agent_position, targets_position, task_allocation_all_agents, cluster_centers, points_idx_for_clusters)
    Simulator.plot_cluster_assign(agent_position, targets_position, points_idx_for_clusters, cluster_centers, cluster_assigned_idx)

    # current method
    t0 = time.time()
    path_all_agents, task_allocation_all_agents, cluster_centers, points_idx_for_clusters, cluster_assigned_idx\
        = DrMaMP.MissionPlanning(agent_position, targets_position, num_cluster,
                                             number_of_iterations, world_map, Simulator.map_width,
                                             Simulator.map_height)
    t1 = time.time()
    print("Current method time [sec]:" + str(t1 - t0))
    # visualization
    Simulator.plot_path_multi_agent(path_all_agents, agent_position, targets_position, task_allocation_all_agents, cluster_centers, points_idx_for_clusters)
    Simulator.plot_cluster_assign(agent_position, targets_position, points_idx_for_clusters, cluster_centers, cluster_assigned_idx)
    plt.show()
