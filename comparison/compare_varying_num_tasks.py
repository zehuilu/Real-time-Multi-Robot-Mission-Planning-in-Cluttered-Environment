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
    from compute_path_distance import compute_path_distance_one_agent, compute_path_distance_many_agents


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

    # fix the number of agents
    num_agents = 5
    max_num_targets_per_agent = 20
    num_run = 300
    run_cbba_flag = False

    time_used_list_all_cases_my = []
    time_used_list_all_cases_cbba = []
    distance_list_all_cases_my = []
    distance_list_all_cases_cbba = []
    xticks_str_list = []
    for num_targets_per_agent in range (2, max_num_targets_per_agent+1, 3):
        time_used_list_single_case_my = []
        time_used_list_single_case_cbba = []
        distance_list_single_case_my = []
        distance_list_single_case_cbba = []

        # parameters for my algorithm
        num_cluster = num_agents
        number_of_iterations = 200

        for idx_run in range(num_run):
            # generate agents and targets randomly
            agent_position, targets_position = Simulator.generate_agents_and_targets(
                num_agents, num_agents * num_targets_per_agent)

            # my algorithm
            t0 = time.time()
            path_all_agents_my, _, _, _, _ =\
                DrMaMP.MissionPlanning(agent_position, targets_position, num_cluster,
                                                   number_of_iterations, world_map,
                                                   Simulator.map_width, Simulator.map_height)
            t1 = time.time()
            time_used_my = (t1 - t0) * 1000.0  # in millisecond
            time_used_list_single_case_my.append(time_used_my)
            this_distance_my, _ = compute_path_distance_many_agents(path_all_agents_my)
            distance_list_single_case_my.append(this_distance_my)

            if run_cbba_flag:
                # CBBA
                t0 = time.time()
                _, _, path_all_agents_cbba, _, _, _ =\
                CBBA_Path_Finding.Solve(agent_position, targets_position, Simulator,
                                        cbba_config_file_name, plot_flag=False)
                t1 = time.time()
                time_used_cbba = (t1 - t0) * 1000.0  # in millisecond
                time_used_list_single_case_cbba.append(time_used_cbba)
                this_distance_cbba, _ = compute_path_distance_many_agents(path_all_agents_cbba)
                distance_list_single_case_cbba.append(this_distance_cbba)

        time_used_list_all_cases_my.append(time_used_list_single_case_my)
        time_used_list_all_cases_cbba.append(time_used_list_single_case_cbba)
        distance_list_all_cases_my.append(distance_list_single_case_my)
        distance_list_all_cases_cbba.append(distance_list_single_case_cbba)
        xticks_str_list.append(str(num_agents * num_targets_per_agent))
        print(num_targets_per_agent)

    xticks_list = range(1, len(time_used_list_all_cases_my)+1)

    if run_cbba_flag:
        # create box plot for both algorithm
        fig1, ax1 = plt.subplots()
        ax1.set_title('Solving time, num_agents = ' + str(num_agents))
        # create plot
        bp1 = ax1.boxplot(time_used_list_all_cases_my, positions=np.array(range(len(time_used_list_all_cases_my)))*2.0-0.4, widths=0.6, showfliers=False)
        bp2 = ax1.boxplot(time_used_list_all_cases_cbba, positions=np.array(range(len(time_used_list_all_cases_cbba)))*2.0+0.4, widths=0.6, showfliers=False)
        plt.setp(bp1['boxes'], color='blue')
        plt.setp(bp1['whiskers'], color='blue')
        plt.setp(bp1['caps'], color='blue')
        plt.setp(bp1['medians'], color='blue')
        plt.setp(bp2['boxes'], color='red')
        plt.setp(bp2['whiskers'], color='red')
        plt.setp(bp2['caps'], color='red')
        plt.setp(bp2['medians'], color='red')
        plt.xlabel('Number of targets')
        plt.ylabel('Solving time [ms]')
        plt.xticks(range(0, len(xticks_str_list)*2, 2), xticks_str_list)
        # set legends
        colors = ["blue", "red"]
        labels = ["Proposed", "CBBA"]
        f = lambda c: plt.plot([],[], color=c)[0]
        handles = [f(colors[i]) for i in range(len(labels))]
        legend = plt.legend(handles, labels, loc='upper left', framealpha=1)

        # create box plot about total distance for both algorithm
        fig2, ax2 = plt.subplots()
        ax2.set_title('Total distance, num_agents = ' + str(num_agents))
        # create plot
        bp3 = ax2.boxplot(distance_list_all_cases_my, positions=np.array(range(len(distance_list_all_cases_my)))*2.0-0.4, widths=0.6, showfliers=False)
        bp4 = ax2.boxplot(distance_list_all_cases_cbba, positions=np.array(range(len(distance_list_all_cases_cbba)))*2.0+0.4, widths=0.6, showfliers=False)
        plt.setp(bp3['boxes'], color='blue')
        plt.setp(bp3['whiskers'], color='blue')
        plt.setp(bp3['caps'], color='blue')
        plt.setp(bp3['medians'], color='blue')
        plt.setp(bp4['boxes'], color='red')
        plt.setp(bp4['whiskers'], color='red')
        plt.setp(bp4['caps'], color='red')
        plt.setp(bp4['medians'], color='red')
        plt.xlabel('Number of targets')
        plt.ylabel('Total distance')
        plt.xticks(range(0, len(xticks_str_list)*2, 2), xticks_str_list)
        # set legends
        colors = ["blue", "red"]
        labels = ["Proposed", "CBBA"]
        f = lambda c: plt.plot([],[], color=c)[0]
        handles = [f(colors[i]) for i in range(len(labels))]
        legend = plt.legend(handles, labels, loc='upper left', framealpha=1)

    # create box plot for my algorithm
    fig3, ax3 = plt.subplots()
    ax3.set_title('Solving time for proposed, num_agents = ' + str(num_agents))
    # create plot
    bp5 = ax3.boxplot(time_used_list_all_cases_my, showfliers=False)
    plt.setp(bp5['boxes'], color='blue')
    plt.setp(bp5['whiskers'], color='blue')
    plt.setp(bp5['caps'], color='blue')
    plt.setp(bp5['medians'], color='blue')
    plt.xlabel('Number of targets')
    plt.ylabel('Solving time [ms]')
    plt.xticks(xticks_list, xticks_str_list)

    if run_cbba_flag:
        # create box plot for CBBA + path finding
        fig4, ax4 = plt.subplots()
        ax4.set_title('Solving time for CBBA, num_agents = ' + str(num_agents))
        # create plot
        bp6 = ax4.boxplot(time_used_list_all_cases_cbba, showfliers=False)
        plt.setp(bp6['boxes'], color='red')
        plt.setp(bp6['whiskers'], color='red')
        plt.setp(bp6['caps'], color='red')
        plt.setp(bp6['medians'], color='red')
        plt.xlabel('Number of targets')
        plt.ylabel('Solving time [ms]')
        plt.xticks(xticks_list, xticks_str_list)

    fig5, ax5 = plt.subplots()
    ax5.set_title('Total distance for proposed, num_agents = ' + str(num_agents))
    # create plot
    bp7 = ax5.boxplot(distance_list_all_cases_my, showfliers=False)
    plt.setp(bp7['boxes'], color='blue')
    plt.setp(bp7['whiskers'], color='blue')
    plt.setp(bp7['caps'], color='blue')
    plt.setp(bp7['medians'], color='blue')
    plt.xlabel('Number of targets')
    plt.ylabel('Total distance')
    plt.xticks(xticks_list, xticks_str_list)

    plt.show()
