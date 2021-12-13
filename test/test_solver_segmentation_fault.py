#!/usr/bin/env python3
import os
import matplotlib.pyplot as plt
import pathmagic
with pathmagic.context():
    from Simulator import Simulator
    import DrMaMP


if __name__ == "__main__":
    # a json configuration file for CBBA
    cbba_config_file_name = os.getcwd() + "/comparison/cbba_config_compare.json"

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

    # fix the average number of targets per agent
    num_targets_per_agent = 5
    max_num_agents = 20
    # for each number of agents, the number of algorithm running
    # each trial has a different set of agents and targets
    num_run = 1000
    num_agents = 5

    # parameters for my algorithm
    num_cluster = num_agents
    number_of_iterations = 200

    print("Running now")
    for idx_run in range(num_run):
        # generate agents and targets randomly
        agent_position, targets_position = Simulator.generate_agents_and_targets(
            num_agents, num_agents * num_targets_per_agent)

        # my algorithm
        path_all_agents_my, _, _, _, _ =\
            DrMaMP.MissionPlanning(agent_position, targets_position, num_cluster,
                                                number_of_iterations, world_map,
                                                Simulator.map_width, Simulator.map_height)
