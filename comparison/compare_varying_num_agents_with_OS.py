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
    import OptimalSearchV2
    import GA_Solver
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
    num_targets_per_agent = 2
    max_num_agents = 2
    num_run = 1
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
            # generate agents and targets randomly
            agent_position, targets_position = MySimulator.generate_agents_and_targets(
                num_agents, num_agents * num_targets_per_agent)

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
            # _, optimal_cost = OptimalSearchV2.OptimalSearch(agent_position, targets_position, world_map, MySimulator.map_width, MySimulator.map_height)
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

    print("distance_list_all_cases_my")
    print(distance_list_all_cases_my)
    print("distance_list_all_cases_cbba")
    print(distance_list_all_cases_cbba)
    print("distance_list_all_cases_os")
    print(distance_list_all_cases_os)

    if run_cbba_flag:
        # create box plot for both algorithm
        fig1, ax1 = plt.subplots()
        ax1.set_title('Computing time, num_targets_per_agent = ' + str(num_targets_per_agent))
        # create plot
        bp1 = ax1.boxplot(time_used_list_all_cases_my, positions=np.array(range(len(time_used_list_all_cases_my)))*2.0-0.5, widths=0.4, showfliers=False)
        bp2 = ax1.boxplot(time_used_list_all_cases_cbba, positions=np.array(range(len(time_used_list_all_cases_cbba)))*2.0-0.0, widths=0.4, showfliers=False)
        bp4 = ax1.boxplot(time_used_list_all_cases_os, positions=np.array(range(len(time_used_list_all_cases_os)))*2.0+0.5, widths=0.4, showfliers=False)
        plt.setp(bp1['boxes'], color='blue')
        plt.setp(bp1['whiskers'], color='blue')
        plt.setp(bp1['caps'], color='blue')
        plt.setp(bp1['medians'], color='blue')
        plt.setp(bp2['boxes'], color='red')
        plt.setp(bp2['whiskers'], color='red')
        plt.setp(bp2['caps'], color='red')
        plt.setp(bp2['medians'], color='red')
        plt.setp(bp4['boxes'], color='brown')
        plt.setp(bp4['whiskers'], color='brown')
        plt.setp(bp4['caps'], color='brown')
        plt.setp(bp4['medians'], color='brown')
        plt.xlabel('Number of agents')
        plt.ylabel('Computing time [ms]')
        plt.xticks(range(0, len(xticks_str_list)*2, 2), xticks_str_list)
        # set legends
        colors = ["blue", "red", "brown"]
        labels = ["Proposed", "CBBA", "Optimal Search"]
        f = lambda c: plt.plot([],[], color=c)[0]
        handles = [f(colors[i]) for i in range(len(labels))]
        legend = plt.legend(handles, labels, loc='upper left', framealpha=1)

        # create box plot about total distance for both algorithm
        fig2, ax2 = plt.subplots()
        ax2.set_title('Total distance, num_targets_per_agent = ' + str(num_targets_per_agent))
        # create plot
        bp5 = ax2.boxplot(distance_list_all_cases_my, positions=np.array(range(len(distance_list_all_cases_my)))*2.0-0.5, widths=0.4, showfliers=False)
        bp6 = ax2.boxplot(distance_list_all_cases_cbba, positions=np.array(range(len(distance_list_all_cases_cbba)))*2.0-0.0, widths=0.4, showfliers=False)
        bp8 = ax2.boxplot(distance_list_all_cases_os, positions=np.array(range(len(distance_list_all_cases_os)))*2.0+0.5, widths=0.4, showfliers=False)
        plt.setp(bp5['boxes'], color='blue')
        plt.setp(bp5['whiskers'], color='blue')
        plt.setp(bp5['caps'], color='blue')
        plt.setp(bp5['medians'], color='blue')
        plt.setp(bp6['boxes'], color='red')
        plt.setp(bp6['whiskers'], color='red')
        plt.setp(bp6['caps'], color='red')
        plt.setp(bp6['medians'], color='red')
        plt.setp(bp8['boxes'], color='brown')
        plt.setp(bp8['whiskers'], color='brown')
        plt.setp(bp8['caps'], color='brown')
        plt.setp(bp8['medians'], color='brown')
        plt.xlabel('Number of agents')
        plt.ylabel('Total distance')
        plt.xticks(range(0, len(xticks_str_list)*2, 2), xticks_str_list)
        # set legends
        colors = ["blue", "red", "brown"]
        labels = ["Proposed", "CBBA", "Optimal Search"]
        f = lambda c: plt.plot([],[], color=c)[0]
        handles = [f(colors[i]) for i in range(len(labels))]
        legend = plt.legend(handles, labels, loc='upper left', framealpha=1)

    plt.show()
