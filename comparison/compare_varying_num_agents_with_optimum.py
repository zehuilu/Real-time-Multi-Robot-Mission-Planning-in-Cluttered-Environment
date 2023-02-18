#!/usr/bin/env python3
import os
import time
import pathmagic
import numpy as np
with pathmagic.context():
    from Simulator import Simulator
    import DrMaMP
    import CBBA_Path_Finding
    import OptimalSearch
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
    num_obs = 100
    # [width, length] size of each obstacle [meter]
    size_obs = [1, 1]
    # generate random obstacles
    MySimulator.generate_random_obs(num_obs, size_obs)
    # convert 2D numpy array to 1D list
    world_map = MySimulator.map_array.flatten().tolist()

    # fix the average number of targets per agent
    num_tasks_per_agent = 2
    max_num_agents = 3

    # num_run = 10
    num_run = 5

    time_used_list_all_cases_my = []
    time_used_list_all_cases_cbba = []
    time_used_list_all_cases_os = []
    distance_list_all_cases_my = []
    distance_list_all_cases_cbba = []
    distance_list_all_cases_os = []
    infeasible_list_all_cases_my = []
    infeasible_list_all_cases_cbba = []
    infeasible_list_all_cases_os = []
    xticks_str_list = []
    for num_agents in range (2, max_num_agents+1, 1):
        time_used_list_single_case_my = []
        time_used_list_single_case_cbba = []
        time_used_list_single_case_os = []
        distance_list_single_case_my = []
        distance_list_single_case_cbba = []
        distance_list_single_case_os = []
        infeasible_list_single_case_my = []
        infeasible_list_single_case_cbba = []
        infeasible_list_single_case_os = []

        # parameters for my algorithm
        num_cluster = num_agents
        number_of_iterations = 300

        for idx_run in range(num_run):
            # generate agents and targets randomly
            agent_position, targets_position = MySimulator.generate_agents_and_targets(
                num_agents, num_agents * num_tasks_per_agent)
            # print(targets_position)

            # my algorithm
            t0 = time.time()
            path_all_agents_my, _, _, _, _ =\
                DrMaMP.MissionPlanning(agent_position, targets_position, num_cluster,
                                       number_of_iterations, world_map,
                                       MySimulator.map_width, MySimulator.map_height)
            t1 = time.time()
            time_used_my = (t1 - t0) * 1000.0  # in millisecond
            time_used_list_single_case_my.append(time_used_my)
            this_distance_my, distance_list_my, infeasible_flag_my = compute_path_distance_many_agents(path_all_agents_my)
            distance_list_single_case_my.append(this_distance_my)
            infeasible_list_single_case_my.append(infeasible_flag_my)
            # print(path_all_agents_my)

            # CBBA
            t0 = time.time()
            _, _, path_all_agents_cbba, _, _, _ =\
            CBBA_Path_Finding.Solve(agent_position, targets_position, MySimulator,
                                    cbba_config_file_name, plot_flag=False)
            t1 = time.time()
            time_used_cbba = (t1 - t0) * 1000.0  # in millisecond
            time_used_list_single_case_cbba.append(time_used_cbba)
            this_distance_cbba, distance_list_cbba, infeasible_flag_cbba = compute_path_distance_many_agents(path_all_agents_cbba)
            distance_list_single_case_cbba.append(this_distance_cbba)
            infeasible_list_single_case_cbba.append(infeasible_flag_cbba)

            # optimal search
            t0 = time.time()
            _, optimal_cost, infeasible_flag_os = OptimalSearch.OptimalSearch(agent_position, targets_position, world_map, MySimulator.map_width, MySimulator.map_height)
            t1 = time.time()
            time_used_os = (t1 - t0) * 1000.0  # in millisecond
            time_used_list_single_case_os.append(time_used_os)
            distance_list_single_case_os.append(optimal_cost)
            infeasible_list_single_case_os.append(infeasible_flag_os)

        time_used_list_all_cases_my.append(time_used_list_single_case_my)
        time_used_list_all_cases_cbba.append(time_used_list_single_case_cbba)
        time_used_list_all_cases_os.append(time_used_list_single_case_os)
        distance_list_all_cases_my.append(distance_list_single_case_my)
        distance_list_all_cases_cbba.append(distance_list_single_case_cbba)
        distance_list_all_cases_os.append(distance_list_single_case_os)
        infeasible_list_all_cases_my.append(infeasible_list_single_case_my)
        infeasible_list_all_cases_cbba.append(infeasible_list_single_case_cbba)
        infeasible_list_all_cases_os.append(infeasible_list_single_case_os)
        xticks_str_list.append(str(num_agents))
        # print(num_agents)

    xticks_list = range(1, len(time_used_list_all_cases_my)+1)

    print("distance_list_all_cases_my")
    print(distance_list_all_cases_my)
    print("distance_list_all_cases_cbba")
    print(distance_list_all_cases_cbba)
    print("distance_list_all_cases_os")
    print(distance_list_all_cases_os)

    # save data
    prefix = "comparison/data/optimum_varying_num_agents_"

    np.savetxt(prefix+"num_tasks_per_agent.csv", [num_tasks_per_agent], delimiter=",")
    np.savetxt(prefix+"distance_list_all_cases_cbba.csv", distance_list_all_cases_cbba, delimiter=",")
    np.savetxt(prefix+"time_used_list_all_cases_cbba.csv", time_used_list_all_cases_cbba, delimiter=",")
    np.savetxt(prefix+"infeasible_list_all_cases_cbba.csv", infeasible_list_all_cases_cbba, delimiter=",")
    
    np.savetxt(prefix+"distance_list_all_cases_os.csv", distance_list_all_cases_os, delimiter=",")
    np.savetxt(prefix+"time_used_list_all_cases_os.csv", time_used_list_all_cases_os, delimiter=",")
    np.savetxt(prefix+"infeasible_list_all_cases_os.csv", infeasible_list_all_cases_os, delimiter=",")

    np.savetxt(prefix+"distance_list_all_cases_my.csv", distance_list_all_cases_my, delimiter=",")
    np.savetxt(prefix+"time_used_list_all_cases_my.csv", time_used_list_all_cases_my, delimiter=",")
    np.savetxt(prefix+"infeasible_list_all_cases_my.csv", infeasible_list_all_cases_my, delimiter=",")

    np.savetxt(prefix+"xticks_str_list.csv", xticks_str_list, delimiter =",", fmt ='% s')
    print("Completed!")
