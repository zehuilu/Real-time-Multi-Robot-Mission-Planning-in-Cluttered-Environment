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
    map_width_meter = 50.0
    map_height_meter = 50.0
    map_resolution = 2
    value_non_obs = 0 # the cell is empty
    value_obs = 255 # the cell is blocked
    # create a simulator
    Simulator = Simulator(map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)
    # number of obstacles
    num_obs = 0
    # [width, length] size of each obstacle [meter]
    size_obs = [1, 1]
    # generate random obstacles
    Simulator.generate_random_obs(num_obs, size_obs)
    # convert 2D numpy array to 1D list
    world_map = Simulator.map_array.flatten().tolist()



    # This is for agents and a set of targets
    agent_position = [1,1, 99,99, 11,70]
    num_targets = 100
    # targets_position = Simulator.generate_targets_at_corner(num_targets)
    targets_position = Simulator.generate_targets(num_targets)

    num_cluster = 3
    num_iter = 300

    t0 = time.time()
    [cluster_centers, points_idx_for_clusters, cluster_assigned_idx] = DrMaMP.AssignCluster(agent_position, targets_position, num_cluster, num_iter)
    
    t1 = time.time()
    print("Time used [sec]:" + str(t1 - t0))


    print("Clusters:")
    for i in range(0,len(cluster_centers),2):
        print(str(cluster_centers[i]) + ", " + str(cluster_centers[i+1]))

    print("Targets for each cluster:")
    for i in range(0,len(points_idx_for_clusters)):
        print("Cluster: " + str(i))
        for j in range(0,len(points_idx_for_clusters[i])):
            print(points_idx_for_clusters[i][j])

    print("Assignment results:")
    for i in range(0,len(cluster_assigned_idx),1):
        print("Agent " + str(i) + " is assigned to Cluster " + str(cluster_assigned_idx[i]))

    # visualization
    Simulator.plot_cluster_assign(agent_position, targets_position, points_idx_for_clusters, cluster_centers, cluster_assigned_idx)
    plt.show()