#!/usr/bin/env python3
import warnings
import math


def compute_path_distance_one_agent(path_list_2d: list):
    distance = 0.0
    for i in range(len(path_list_2d)):
        if len(path_list_2d[i]) > 0:
            for j in range(0, len(path_list_2d[i])-2, 2):
                distance = distance + math.sqrt((path_list_2d[i][j]-path_list_2d[i][j+2])**2 +\
                           (path_list_2d[i][j+1]-path_list_2d[i][j+3])**2)
        else:
            warnings.warn("The path is infeasible if is empty.")
    return distance

def compute_path_distance_many_agents(path_list_3d: list):
    distance = 0.0
    distance_list = []
    for idx in range(len(path_list_3d)):
        path_this_agent_2d = path_list_3d[idx]
        distance_this_agent = compute_path_distance_one_agent(path_this_agent_2d)
        distance = distance + distance_this_agent
        distance_list.append(distance_this_agent)
    return distance, distance_list
