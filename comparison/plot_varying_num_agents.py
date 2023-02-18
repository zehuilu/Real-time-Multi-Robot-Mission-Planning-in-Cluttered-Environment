#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import pathmagic
with pathmagic.context():
    import filter_comparison as helper


if __name__ == "__main__":
    plot_cbba_flag = True
    # plot_cbba_flag = False

    # load data
    prefix = "comparison/data/varying_num_agents_"

    num_tasks_per_agent = int(np.genfromtxt(
        prefix+"num_tasks_per_agent.csv", delimiter=","))
    _time_used_list_all_cases_cbba = np.genfromtxt(
        prefix+"algo_time_list_cbba.csv", delimiter=",")
    _time_used_list_all_cases_my = np.genfromtxt(
        prefix+"algo_time_part1_list_my.csv", delimiter=",")
    _distance_list_all_cases_cbba = np.genfromtxt(
        prefix+"distance_list_all_cases_cbba.csv", delimiter=",")
    _distance_list_all_cases_my = np.genfromtxt(
        prefix+"distance_list_all_cases_my.csv", delimiter=",")
    _infeasible_list_all_cases_my = np.genfromtxt(
        prefix+"infeasible_list_all_cases_my.csv", delimiter=",")
    _infeasible_list_all_cases_cbba = np.genfromtxt(
        prefix+"infeasible_list_all_cases_cbba.csv", delimiter=",")
    xticks_str_list = np.genfromtxt(
        prefix+"xticks_str_list.csv", delimiter=",", dtype="int")
    xticks_str_list.astype('str')

    # xticks of plots
    xticks_list = range(1, len(_time_used_list_all_cases_my)+1)

    # filter the data
    infeasible_list_all_cases = helper.find_infeasible_all(_infeasible_list_all_cases_my, _infeasible_list_all_cases_cbba)
    time_used_list_all_cases_cbba, distance_list_all_cases_cbba = \
        helper.find_other_lists(_time_used_list_all_cases_cbba, _distance_list_all_cases_cbba, infeasible_list_all_cases)
    time_used_list_all_cases_my, distance_list_all_cases_my = \
        helper.find_other_lists(_time_used_list_all_cases_my, _distance_list_all_cases_my, infeasible_list_all_cases)


    mean_time_my = list()
    std_time_my = list()
    mean_distance_my = list()
    std_distance_my = list()
    for idx in range(len(time_used_list_all_cases_my)):
        mean_time_my.append(np.mean(time_used_list_all_cases_my[idx]))
        std_time_my.append(np.std(time_used_list_all_cases_my[idx]))
        mean_distance_my.append(np.mean(distance_list_all_cases_my[idx]))
        std_distance_my.append(np.std(distance_list_all_cases_my[idx]))

    mean_time_cbba = list()
    std_time_cbba = list()
    mean_distance_cbba = list()
    std_distance_cbba = list()
    for idx in range(len(time_used_list_all_cases_cbba)):
        mean_time_cbba.append(np.mean(time_used_list_all_cases_cbba[idx]))
        std_time_cbba.append(np.std(time_used_list_all_cases_cbba[idx]))
        mean_distance_cbba.append(np.mean(distance_list_all_cases_cbba[idx]))
        std_distance_cbba.append(np.std(distance_list_all_cases_cbba[idx]))


    if plot_cbba_flag:
        # create box plot for both algorithm
        fig1, ax1 = plt.subplots()
        ax1.set_title('Computation time, num_tasks_per_agent = ' + str(num_tasks_per_agent))
        # create plot
        x_pos = np.array(range(len(time_used_list_all_cases_my)))*2.0-0.2
        ax1.bar(x_pos, mean_time_my, yerr=std_time_my, color='blue', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
        x_pos = np.array(range(len(time_used_list_all_cases_cbba)))*2.0+0.2
        ax1.bar(x_pos, mean_time_cbba, yerr=std_time_cbba, color='red', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
        ax1.set_xlabel('Number of agents')
        ax1.set_ylabel('Computation time [ms]')
        ax1.yaxis.grid(True)
        plt.xticks(range(0, len(xticks_str_list)*2, 2), xticks_str_list)
        # set legends
        colors = ["blue", "red"]
        labels = ["Proposed", "CBBA"]
        f = lambda c: plt.plot([],[], color=c)[0]
        handles = [f(colors[i]) for i in range(len(labels))]
        legend = plt.legend(handles, labels, loc='upper left', framealpha=1)


        # create box plot about total distance for both algorithm
        fig2, ax2 = plt.subplots()
        ax2.set_title('Total distance, num_tasks_per_agent = ' + str(num_tasks_per_agent))
        # create plot
        x_pos = np.array(range(len(distance_list_all_cases_my)))*2.0-0.2
        ax2.bar(x_pos, mean_distance_my, yerr=std_distance_my, color='blue', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
        x_pos = np.array(range(len(distance_list_all_cases_cbba)))*2.0+0.2
        ax2.bar(x_pos, mean_distance_cbba, yerr=std_distance_cbba, color='red', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
        ax2.set_xlabel('Number of agents')
        ax2.set_ylabel('Total distance')
        ax2.yaxis.grid(True)
        plt.xticks(range(0, len(xticks_str_list)*2, 2), xticks_str_list)
        # set legends
        colors = ["blue", "red"]
        labels = ["Proposed", "CBBA"]
        f = lambda c: plt.plot([],[], color=c)[0]
        handles = [f(colors[i]) for i in range(len(labels))]
        legend = plt.legend(handles, labels, loc='upper left', framealpha=1)


    # create box plot for my algorithm
    fig3, ax3 = plt.subplots()
    ax3.set_title('Computation time for proposed, num_tasks_per_agent = ' + str(num_tasks_per_agent))
    # create plot
    ax3.bar(xticks_list, mean_time_my, yerr=std_time_my, color='blue', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
    ax3.set_xlabel('Number of agents')
    ax3.set_ylabel('Computation time [ms]')
    ax3.yaxis.grid(True)
    plt.xticks(xticks_list, xticks_str_list)
    # set legends
    colors = ["blue"]
    labels = ["Proposed"]
    f = lambda c: plt.plot([],[], color=c)[0]
    handles = [f(colors[i]) for i in range(len(labels))]
    legend = plt.legend(handles, labels, loc='upper left', framealpha=1)


    fig4, ax4 = plt.subplots()
    ax4.set_title('Total distance for proposed, num_tasks_per_agent = ' + str(num_tasks_per_agent))
    # create plot
    ax4.bar(xticks_list, mean_distance_my, yerr=std_distance_my, color='blue', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
    ax4.set_xlabel('Number of agents')
    ax4.set_ylabel('Total distance')
    ax4.yaxis.grid(True)
    plt.xticks(xticks_list, xticks_str_list)
    # set legends
    colors = ["blue"]
    labels = ["Proposed"]
    f = lambda c: plt.plot([],[], color=c)[0]
    handles = [f(colors[i]) for i in range(len(labels))]
    legend = plt.legend(handles, labels, loc='upper left', framealpha=1)

    plt.show()
