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
    # load data
    prefix = "comparison/data/optimum_varying_num_agents_"

    num_tasks_per_agent = int(np.genfromtxt(
        prefix+"num_tasks_per_agent.csv", delimiter=","))

    mean_time_cbba = np.genfromtxt(
        prefix+"mean_time_cbba.csv", delimiter=",")
    std_time_cbba = np.genfromtxt(
        prefix+"std_time_cbba.csv", delimiter=",")
    mean_distance_cbba = np.genfromtxt(
        prefix+"mean_distance_cbba.csv", delimiter=",")
    std_distance_cbba = np.genfromtxt(
        prefix+"std_distance_cbba.csv", delimiter=",")
    distance_list_all_cases_cbba = np.genfromtxt(
        prefix+"distance_list_all_cases_cbba.csv", delimiter=",")
    time_used_list_all_cases_cbba = np.genfromtxt(
        prefix+"time_used_list_all_cases_cbba.csv", delimiter=",")

    mean_time_os = np.genfromtxt(
        prefix+"mean_time_os.csv", delimiter=",")
    std_time_os = np.genfromtxt(
        prefix+"std_time_os.csv", delimiter=",")
    mean_distance_os = np.genfromtxt(
        prefix+"mean_distance_os.csv", delimiter=",")
    std_distance_os = np.genfromtxt(
        prefix+"std_distance_os.csv", delimiter=",")
    distance_list_all_cases_os = np.genfromtxt(
        prefix+"distance_list_all_cases_os.csv", delimiter=",")
    time_used_list_all_cases_os = np.genfromtxt(
        prefix+"time_used_list_all_cases_os.csv", delimiter=",")

    mean_time_my = np.genfromtxt(
        prefix+"mean_time_my.csv", delimiter=",")
    std_time_my = np.genfromtxt(
        prefix+"std_time_my.csv", delimiter=",")
    mean_distance_my = np.genfromtxt(
        prefix+"mean_distance_my.csv", delimiter=",")
    std_distance_my = np.genfromtxt(
        prefix+"std_distance_my.csv", delimiter=",")
    distance_list_all_cases_my = np.genfromtxt(
        prefix+"distance_list_all_cases_my.csv", delimiter=",")
    time_used_list_all_cases_my = np.genfromtxt(
        prefix+"time_used_list_all_cases_my.csv", delimiter=",")

    xticks_str_list = np.genfromtxt(
        prefix+"xticks_str_list.csv", delimiter=",", dtype="int")
    xticks_str_list.astype('str')

    # xticks of plots
    xticks_list = range(1, len(time_used_list_all_cases_my)+1)


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
