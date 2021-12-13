#!/usr/bin/env python3
import time
import pathmagic
with pathmagic.context():
    import DrMaMP


if __name__ == "__main__":

    # define the world map
    map_width = 20
    map_height = 20

    world_map = [
    # 00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   # 00
        1,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,1,   # 01
        1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   # 02
        1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   # 03
        1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   # 04
        1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   # 05
        1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   # 06
        1,9,9,9,9,9,9,9,9,1,1,1,9,9,9,9,9,9,9,1,   # 07
        1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   # 08
        1,9,1,9,9,9,9,9,9,9,1,1,9,9,9,9,9,9,9,1,   # 09
        1,9,1,1,1,1,9,1,1,9,1,1,1,1,1,1,1,1,1,1,   # 10
        1,9,9,9,9,9,1,9,1,9,1,9,9,9,9,9,1,1,1,1,   # 11
        1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   # 12
        1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   # 13
        1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   # 14
        1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   # 15
        1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   # 16
        1,1,9,9,9,9,9,9,9,1,1,1,9,9,9,1,9,9,9,9,   # 17
        1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   # 18
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1    # 19
    ]

    # for LazyThetaStarPython, 0 for no obstacles; 255 for obstacles
    for idx in range(len(world_map)):
        if world_map[idx] == 9:
            world_map[idx] = 255
        else:
            world_map[idx] = 0


    # This is for an agent and a set of targets
    agent_position = [0, 0]
    targets_position = [11,10, 10,10, 19,17, 19,19, 18,18, 4,5, 5,5, 2,3, 3,3]

    num_cluster = 3
    num_iter = 300

    t0 = time.time()
    [means, assignments, points_idx_for_clusters] = DrMaMP.KMeans(targets_position, num_cluster, num_iter)
    
    t1 = time.time()
    print("Time used [sec]:" + str(t1 - t0))

    print("This is means")
    for i in range(0,len(means),2):
        print(str(means[i]) + ", " + str(means[i+1]))

    print("This is assignments")
    for i in range(0,len(assignments),1):
        print(assignments[i])

    print("This is points_idx_for_clusters")
    for i in range(0,len(points_idx_for_clusters),1):
        print("This is a cluster.")
        for j in range(0,len(points_idx_for_clusters[i]),1):
            print(points_idx_for_clusters[i][j])
