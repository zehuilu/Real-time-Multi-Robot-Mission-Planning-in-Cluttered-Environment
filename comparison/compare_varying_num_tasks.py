#!/usr/bin/env python3
import os
import time
import pathmagic
import numpy as np
with pathmagic.context():
    from Simulator import Simulator
    import DrMaMP
    import CBBA_Path_Finding
    from compute_path_distance import compute_path_distance_many_agents


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
    MySimulator = Simulator(map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)
    # number of obstacles
    num_obs = 400
    # [width, length] size of each obstacle [meter]
    size_obs = [1/map_resolution, 1/map_resolution]
    # generate random obstacles
    MySimulator.generate_random_obs(num_obs, size_obs)
    # convert 2D numpy array to 1D list
    world_map = MySimulator.map_array.flatten().tolist()

    # fix the number of agents
    num_agents = 3

    max_num_tasks_per_agent = 21
    num_run = 100

    time_used_list_all_cases_my = []
    time_used_list_all_cases_cbba = []
    distance_list_all_cases_my = []
    distance_list_all_cases_cbba = []
    infeasible_list_all_cases_my = []
    infeasible_list_all_cases_cbba = []
    xticks_str_list = []
    for num_tasks_per_agent in range (2, max_num_tasks_per_agent+1, 3):
        time_used_list_single_case_my = []
        time_used_list_single_case_cbba = []
        distance_list_single_case_my = []
        distance_list_single_case_cbba = []
        infeasible_list_single_case_my = []
        infeasible_list_single_case_cbba = []

        # parameters for my algorithm
        num_cluster = num_agents
        number_of_iterations = 300

        for idx_run in range(num_run):
            # generate agents and targets randomly
            agent_position, targets_position = MySimulator.generate_agents_and_targets(
                num_agents, num_agents * num_tasks_per_agent)

            # my algorithm
            t0 = time.time()
            path_all_agents_my, _, _, _, _ =\
                DrMaMP.MissionPlanning(agent_position, targets_position, num_cluster,
                                       number_of_iterations, world_map,
                                       MySimulator.map_width, MySimulator.map_height)
            t1 = time.time()
            time_used_my = (t1 - t0) * 1000.0  # in millisecond
            time_used_list_single_case_my.append(time_used_my)
            this_distance_my, _, infeasible_flag_my = compute_path_distance_many_agents(path_all_agents_my)
            distance_list_single_case_my.append(this_distance_my)
            infeasible_list_single_case_my.append(infeasible_flag_my)\

            # CBBA
            t0 = time.time()
            _, _, path_all_agents_cbba, _, _, _ =\
            CBBA_Path_Finding.Solve(agent_position, targets_position, MySimulator,
                                    cbba_config_file_name, plot_flag=False)
            t1 = time.time()
            time_used_cbba = (t1 - t0) * 1000.0  # in millisecond
            time_used_list_single_case_cbba.append(time_used_cbba)
            this_distance_cbba, _, infeasible_flag_cbba = compute_path_distance_many_agents(path_all_agents_cbba)
            distance_list_single_case_cbba.append(this_distance_cbba)
            infeasible_list_single_case_cbba.append(infeasible_flag_cbba)

        time_used_list_all_cases_my.append(time_used_list_single_case_my)
        time_used_list_all_cases_cbba.append(time_used_list_single_case_cbba)
        distance_list_all_cases_my.append(distance_list_single_case_my)
        distance_list_all_cases_cbba.append(distance_list_single_case_cbba)
        infeasible_list_all_cases_my.append(infeasible_list_single_case_my)
        infeasible_list_all_cases_cbba.append(infeasible_list_single_case_cbba)
        xticks_str_list.append(str(num_agents * num_tasks_per_agent))
        print(num_tasks_per_agent)

    # save data
    prefix = "comparison/data/varying_num_tasks_"
    np.savetxt(prefix+"num_agents.csv", [num_agents], delimiter=",")
    np.savetxt(prefix+"algo_time_list_cbba.csv", time_used_list_all_cases_cbba, delimiter=",")
    np.savetxt(prefix+"algo_time_part1_list_my.csv", time_used_list_all_cases_my, delimiter=",")
    np.savetxt(prefix+"distance_list_all_cases_cbba.csv", distance_list_all_cases_cbba, delimiter=",")
    np.savetxt(prefix+"distance_list_all_cases_my.csv", distance_list_all_cases_my, delimiter=",")
    np.savetxt(prefix+"infeasible_list_all_cases_my.csv", infeasible_list_all_cases_my, delimiter=",")
    np.savetxt(prefix+"infeasible_list_all_cases_cbba.csv", infeasible_list_all_cases_cbba, delimiter=",")
    np.savetxt(prefix+"xticks_str_list.csv", xticks_str_list, delimiter =",", fmt ='% s')

    print("Completed!")
