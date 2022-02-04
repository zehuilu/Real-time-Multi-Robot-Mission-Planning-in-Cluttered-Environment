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
    num_targets_per_agent = 6
    max_num_agents = 17
    num_run = 50
    run_cbba_flag = False
    run_ga_flag = True

    # some hyper-parameters for Genetic Algorithm
    population_size = 25
    max_iter = 25

    time_used_list_all_cases_my = []
    time_used_list_all_cases_cbba = []
    time_used_list_all_cases_ga = []
    distance_list_all_cases_my = []
    distance_list_all_cases_cbba = []
    distance_list_all_cases_ga = []
    xticks_str_list = []
    for num_agents in range (2, max_num_agents+1, 3):
        time_used_list_single_case_my = []
        time_used_list_single_case_cbba = []
        time_used_list_single_case_ga = []
        distance_list_single_case_my = []
        distance_list_single_case_cbba = []
        distance_list_single_case_ga = []

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
            this_distance_my, _ = compute_path_distance_many_agents(path_all_agents_my)
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
                this_distance_cbba, _ = compute_path_distance_many_agents(path_all_agents_cbba)
                distance_list_single_case_cbba.append(this_distance_cbba)

            # Genetic Algorithm
            if run_ga_flag:
                t0 = time.time()
                _, _, cost, _, _, _\
                    = GA_Solver.MissionPlanning(
                        agent_position, targets_position, num_cluster, number_of_iterations,
                        population_size, max_iter, world_map, MySimulator.map_width, MySimulator.map_height)
                t1 = time.time()
                time_used_ga = (t1 - t0) * 1000.0  # in millisecond
                time_used_list_single_case_ga.append(time_used_ga)
                distance_list_single_case_ga.append(cost)

        time_used_list_all_cases_my.append(time_used_list_single_case_my)
        time_used_list_all_cases_cbba.append(time_used_list_single_case_cbba)
        time_used_list_all_cases_ga.append(time_used_list_single_case_ga)
        distance_list_all_cases_my.append(distance_list_single_case_my)
        distance_list_all_cases_cbba.append(distance_list_single_case_cbba)
        distance_list_all_cases_ga.append(distance_list_single_case_ga)
        xticks_str_list.append(str(num_agents))
        print(num_agents)

    xticks_list = range(1, len(time_used_list_all_cases_my)+1)

    if run_cbba_flag:
        # create box plot for both algorithm
        fig1, ax1 = plt.subplots()
        ax1.set_title('Computing time, num_targets_per_agent = ' + str(num_targets_per_agent))
        # create plot
        bp1 = ax1.boxplot(time_used_list_all_cases_my, positions=np.array(range(len(time_used_list_all_cases_my)))*2.0-0.5, widths=0.4, showfliers=False)
        bp2 = ax1.boxplot(time_used_list_all_cases_cbba, positions=np.array(range(len(time_used_list_all_cases_cbba)))*2.0+0.5, widths=0.4, showfliers=False)
        bp3 = ax1.boxplot(time_used_list_all_cases_ga, positions=np.array(range(len(time_used_list_all_cases_ga)))*2.0+0.0, widths=0.4, showfliers=False)
        plt.setp(bp1['boxes'], color='blue')
        plt.setp(bp1['whiskers'], color='blue')
        plt.setp(bp1['caps'], color='blue')
        plt.setp(bp1['medians'], color='blue')
        plt.setp(bp2['boxes'], color='red')
        plt.setp(bp2['whiskers'], color='red')
        plt.setp(bp2['caps'], color='red')
        plt.setp(bp2['medians'], color='red')
        plt.setp(bp3['boxes'], color='green')
        plt.setp(bp3['whiskers'], color='green')
        plt.setp(bp3['caps'], color='green')
        plt.setp(bp3['medians'], color='green')
        plt.xlabel('Number of agents')
        plt.ylabel('Computing time [ms]')
        plt.xticks(range(0, len(xticks_str_list)*2, 2), xticks_str_list)
        # set legends
        colors = ["blue", "red", "green"]
        labels = ["Proposed", "CBBA", "K-GA"]
        # colors = ["blue", "red"]
        # labels = ["Proposed", "CBBA"]
        f = lambda c: plt.plot([],[], color=c)[0]
        handles = [f(colors[i]) for i in range(len(labels))]
        legend = plt.legend(handles, labels, loc='upper left', framealpha=1)

        # create box plot about total distance for both algorithm
        fig2, ax2 = plt.subplots()
        ax2.set_title('Total distance, num_targets_per_agent = ' + str(num_targets_per_agent))
        # create plot
        bp4 = ax2.boxplot(distance_list_all_cases_my, positions=np.array(range(len(distance_list_all_cases_my)))*2.0-0.5, widths=0.4, showfliers=False)
        bp5 = ax2.boxplot(distance_list_all_cases_cbba, positions=np.array(range(len(distance_list_all_cases_cbba)))*2.0+0.5, widths=0.4, showfliers=False)
        bp6 = ax2.boxplot(distance_list_all_cases_ga, positions=np.array(range(len(distance_list_all_cases_ga)))*2.0+0.0, widths=0.4, showfliers=False)
        plt.setp(bp4['boxes'], color='blue')
        plt.setp(bp4['whiskers'], color='blue')
        plt.setp(bp4['caps'], color='blue')
        plt.setp(bp4['medians'], color='blue')
        plt.setp(bp5['boxes'], color='red')
        plt.setp(bp5['whiskers'], color='red')
        plt.setp(bp5['caps'], color='red')
        plt.setp(bp5['medians'], color='red')
        plt.setp(bp6['boxes'], color='green')
        plt.setp(bp6['whiskers'], color='green')
        plt.setp(bp6['caps'], color='green')
        plt.setp(bp6['medians'], color='green')
        plt.xlabel('Number of agents')
        plt.ylabel('Total distance')
        plt.xticks(range(0, len(xticks_str_list)*2, 2), xticks_str_list)
        # set legends
        colors = ["blue", "red", "green"]
        labels = ["Proposed", "CBBA", "K-GA"]
        # colors = ["blue", "red"]
        # labels = ["Proposed", "CBBA"]
        f = lambda c: plt.plot([],[], color=c)[0]
        handles = [f(colors[i]) for i in range(len(labels))]
        legend = plt.legend(handles, labels, loc='upper left', framealpha=1)

    # # create box plot for my algorithm
    # fig3, ax3 = plt.subplots()
    # ax3.set_title('Computing time for proposed, num_targets_per_agent = ' + str(num_targets_per_agent))
    # # create plot
    # bp7 = ax3.boxplot(time_used_list_all_cases_my, showfliers=False)
    # plt.setp(bp7['boxes'], color='blue')
    # plt.setp(bp7['whiskers'], color='blue')
    # plt.setp(bp7['caps'], color='blue')
    # plt.setp(bp7['medians'], color='blue')
    # plt.xlabel('Number of agents')
    # plt.ylabel('Computing time [ms]')
    # plt.xticks(xticks_list, xticks_str_list)

    # fig4, ax4 = plt.subplots()
    # ax4.set_title('Total distance for proposed, num_targets_per_agent = ' + str(num_targets_per_agent))
    # # create plot
    # bp8 = ax4.boxplot(distance_list_all_cases_my, showfliers=False)
    # plt.setp(bp8['boxes'], color='blue')
    # plt.setp(bp8['whiskers'], color='blue')
    # plt.setp(bp8['caps'], color='blue')
    # plt.setp(bp8['medians'], color='blue')
    # plt.xlabel('Number of agents')
    # plt.ylabel('Total distance')
    # plt.xticks(xticks_list, xticks_str_list)

    # if run_cbba_flag:
    #     # create box plot for CBBA + path finding
    #     fig5, ax5 = plt.subplots()
    #     ax5.set_title('Computing time for CBBA, num_targets_per_agent = ' + str(num_targets_per_agent))
    #     # create plot
    #     bp9 = ax5.boxplot(time_used_list_all_cases_cbba, showfliers=False)
    #     plt.setp(bp9['boxes'], color='red')
    #     plt.setp(bp9['whiskers'], color='red')
    #     plt.setp(bp9['caps'], color='red')
    #     plt.setp(bp9['medians'], color='red')
    #     plt.xlabel('Number of agents')
    #     plt.ylabel('Computing time [ms]')
    #     plt.xticks(xticks_list, xticks_str_list)

    if run_ga_flag:
        # # create box plot for GA
        # fig6, ax6 = plt.subplots()
        # ax6.set_title('Computing time for GA, num_targets_per_agent = ' + str(num_targets_per_agent))
        # # create plot
        # bp10 = ax6.boxplot(time_used_list_all_cases_ga, showfliers=False)
        # plt.setp(bp10['boxes'], color='green')
        # plt.setp(bp10['whiskers'], color='green')
        # plt.setp(bp10['caps'], color='green')
        # plt.setp(bp10['medians'], color='green')
        # plt.xlabel('Number of agents')
        # plt.ylabel('Computing time [ms]')
        # plt.xticks(xticks_list, xticks_str_list)


        # create box plot for both algorithm
        fig7, ax7 = plt.subplots()
        ax7.set_title('Computing time, num_targets_per_agent = ' + str(num_targets_per_agent))
        # create plot
        bp11 = ax7.boxplot(time_used_list_all_cases_my, positions=np.array(range(len(time_used_list_all_cases_my)))*2.0-0.25, widths=0.4, showfliers=False)
        bp12 = ax7.boxplot(time_used_list_all_cases_ga, positions=np.array(range(len(time_used_list_all_cases_ga)))*2.0+0.25, widths=0.4, showfliers=False)
        plt.setp(bp11['boxes'], color='blue')
        plt.setp(bp11['whiskers'], color='blue')
        plt.setp(bp11['caps'], color='blue')
        plt.setp(bp11['medians'], color='blue')
        plt.setp(bp12['boxes'], color='green')
        plt.setp(bp12['whiskers'], color='green')
        plt.setp(bp12['caps'], color='green')
        plt.setp(bp12['medians'], color='green')
        plt.xlabel('Number of agents')
        plt.ylabel('Computing time [ms]')
        plt.xticks(range(0, len(xticks_str_list)*2, 2), xticks_str_list)
        # set legends
        colors = ["blue", "green"]
        labels = ["Proposed", "K-GA"]
        f = lambda c: plt.plot([],[], color=c)[0]
        handles = [f(colors[i]) for i in range(len(labels))]
        legend = plt.legend(handles, labels, loc='upper left', framealpha=1)

        # create box plot for both algorithm
        fig8, ax8 = plt.subplots()
        ax8.set_title('Total distance, num_targets_per_agent = ' + str(num_targets_per_agent))
        # create plot
        bp13 = ax8.boxplot(distance_list_all_cases_my, positions=np.array(range(len(distance_list_all_cases_my)))*2.0-0.25, widths=0.4, showfliers=False)
        bp14 = ax8.boxplot(distance_list_all_cases_ga, positions=np.array(range(len(distance_list_all_cases_ga)))*2.0+0.25, widths=0.4, showfliers=False)
        plt.setp(bp13['boxes'], color='blue')
        plt.setp(bp13['whiskers'], color='blue')
        plt.setp(bp13['caps'], color='blue')
        plt.setp(bp13['medians'], color='blue')
        plt.setp(bp14['boxes'], color='green')
        plt.setp(bp14['whiskers'], color='green')
        plt.setp(bp14['caps'], color='green')
        plt.setp(bp14['medians'], color='green')
        plt.xlabel('Number of agents')
        plt.ylabel('Total distance')
        plt.xticks(range(0, len(xticks_str_list)*2, 2), xticks_str_list)
        # set legends
        colors = ["blue", "green"]
        labels = ["Proposed", "K-GA"]
        f = lambda c: plt.plot([],[], color=c)[0]
        handles = [f(colors[i]) for i in range(len(labels))]
        legend = plt.legend(handles, labels, loc='upper left', framealpha=1)


    plt.show()