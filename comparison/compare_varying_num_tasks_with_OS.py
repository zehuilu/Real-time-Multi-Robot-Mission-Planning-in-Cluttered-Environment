#!/usr/bin/env python3
import os
import time
import matplotlib.pyplot as plt
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

    # fix the average number of tasks per agent
    num_tasks_per_agent = 2
    max_num_agents = 3
    num_run = 10
    run_cbba_flag = True

    time_used_list_all_cases_my = []
    time_used_list_all_cases_cbba = []
    time_used_list_all_cases_os = []
    distance_list_all_cases_my = []
    distance_list_all_cases_cbba = []
    distance_list_all_cases_os = []
    xticks_str_list = []
    for num_agents in range (2, max_num_agents+1, 1):
        time_used_list_single_case_my = []
        time_used_list_single_case_cbba = []
        time_used_list_single_case_os = []
        distance_list_single_case_my = []
        distance_list_single_case_cbba = []
        distance_list_single_case_os = []

        # parameters for my algorithm
        num_cluster = num_agents
        number_of_iterations = 300

        for idx_run in range(num_run):
            # generate agents and tasks randomly
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
            this_distance_my, distance_list_my = compute_path_distance_many_agents(path_all_agents_my)

            distance_list_single_case_my.append(this_distance_my)

            # CBBA
            if run_cbba_flag:
                t0 = time.time()
                _, _, path_all_agents_cbba, _, _, _ =\
                CBBA_Path_Finding.Solve(agent_position, targets_position, MySimulator,
                                        cbba_config_file_name, plot_flag=False)
                t1 = time.time()
                time_used_cbba = (t1 - t0) * 1000.0  # in millisecond
                time_used_list_single_case_cbba.append(time_used_cbba)
                this_distance_cbba, distance_list_cbba = compute_path_distance_many_agents(path_all_agents_cbba)
                distance_list_single_case_cbba.append(this_distance_cbba)

            # optimal search
            t0 = time.time()
            _, optimal_cost = OptimalSearch.OptimalSearch(agent_position, targets_position, world_map, MySimulator.map_width, MySimulator.map_height)
            t1 = time.time()
            time_used_os = (t1 - t0) * 1000.0  # in millisecond
            time_used_list_single_case_os.append(time_used_os)
            distance_list_single_case_os.append(optimal_cost)

        time_used_list_all_cases_my.append(time_used_list_single_case_my)
        time_used_list_all_cases_cbba.append(time_used_list_single_case_cbba)
        time_used_list_all_cases_os.append(time_used_list_single_case_os)
        distance_list_all_cases_my.append(distance_list_single_case_my)
        distance_list_all_cases_cbba.append(distance_list_single_case_cbba)
        distance_list_all_cases_os.append(distance_list_single_case_os)
        xticks_str_list.append(str(num_agents))
        print(num_agents)

    xticks_list = range(1, len(time_used_list_all_cases_my)+1)

    mean_time_my = list()
    std_time_my = list()
    mean_distance_my = list()
    std_distance_my = list()
    mean_time_os = list()
    std_time_os = list()
    mean_distance_os = list()
    std_distance_os = list()
    for idx in range(len(time_used_list_all_cases_my)):
        mean_time_my.append(np.mean(time_used_list_all_cases_my[idx]))
        std_time_my.append(np.std(time_used_list_all_cases_my[idx]))
        mean_distance_my.append(np.mean(distance_list_all_cases_my[idx]))
        std_distance_my.append(np.std(distance_list_all_cases_my[idx]))
    for idx in range(len(time_used_list_all_cases_os)):
        mean_time_os.append(np.mean(time_used_list_all_cases_os[idx]))
        std_time_os.append(np.std(time_used_list_all_cases_os[idx]))
        mean_distance_os.append(np.mean(distance_list_all_cases_os[idx]))
        std_distance_os.append(np.std(distance_list_all_cases_os[idx]))
    if run_cbba_flag:
        mean_time_cbba = list()
        std_time_cbba = list()
        mean_distance_cbba = list()
        std_distance_cbba = list()
        for idx in range(len(time_used_list_all_cases_cbba)):
            mean_time_cbba.append(np.mean(time_used_list_all_cases_cbba[idx]))
            std_time_cbba.append(np.std(time_used_list_all_cases_cbba[idx]))
            mean_distance_cbba.append(np.mean(distance_list_all_cases_cbba[idx]))
            std_distance_cbba.append(np.std(distance_list_all_cases_cbba[idx]))

    print("distance_list_all_cases_my")
    print(distance_list_all_cases_my)
    print("distance_list_all_cases_cbba")
    print(distance_list_all_cases_cbba)
    print("distance_list_all_cases_os")
    print(distance_list_all_cases_os)


    if run_cbba_flag:
        # create box plot for both algorithm
        fig1, ax1 = plt.subplots()
        ax1.set_title('Computing time, num_tasks_per_agent = ' + str(num_tasks_per_agent))
        # create plot
        x_pos = np.array(range(len(time_used_list_all_cases_my)))*2.0-0.5
        ax1.bar(x_pos, mean_time_my, yerr=std_time_my, color='blue', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
        x_pos = np.array(range(len(time_used_list_all_cases_os)))*2.0+0.0
        ax1.bar(x_pos, mean_time_os, yerr=std_time_os, color='green', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
        x_pos = np.array(range(len(time_used_list_all_cases_cbba)))*2.0+0.5
        ax1.bar(x_pos, mean_time_cbba, yerr=std_time_cbba, color='red', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
        ax1.set_xlabel('Number of agents')
        ax1.set_ylabel('Computing time [ms]')
        ax1.yaxis.grid(True)
        plt.xticks(range(0, len(xticks_str_list)*2, 2), xticks_str_list)
        # set legends
        colors = ["blue", "green", "red"]
        labels = ["Proposed", "Optimal", "CBBA"]
        f = lambda c: plt.plot([],[], color=c)[0]
        handles = [f(colors[i]) for i in range(len(labels))]
        legend = plt.legend(handles, labels, loc='upper left', framealpha=1)


        # create box plot about total distance for both algorithm
        fig2, ax2 = plt.subplots()
        ax2.set_title('Total distance, num_tasks_per_agent = ' + str(num_tasks_per_agent))
        # create plot
        x_pos = np.array(range(len(distance_list_all_cases_my)))*2.0-0.5
        ax2.bar(x_pos, mean_distance_my, yerr=std_distance_my, color='blue', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
        x_pos = np.array(range(len(distance_list_all_cases_os)))*2.0+0.0
        ax2.bar(x_pos, mean_distance_os, yerr=std_distance_os, color='green', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
        x_pos = np.array(range(len(distance_list_all_cases_cbba)))*2.0+0.5
        ax2.bar(x_pos, mean_distance_cbba, yerr=std_distance_cbba, color='red', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
        ax2.set_xlabel('Number of agents')
        ax2.set_ylabel('Total distance')
        ax2.yaxis.grid(True)
        plt.xticks(range(0, len(xticks_str_list)*2, 2), xticks_str_list)
        # set legends
        colors = ["blue", "green", "red"]
        labels = ["Proposed", "Optimal", "CBBA"]
        f = lambda c: plt.plot([],[], color=c)[0]
        handles = [f(colors[i]) for i in range(len(labels))]
        legend = plt.legend(handles, labels, loc='upper left', framealpha=1)

    plt.show()
