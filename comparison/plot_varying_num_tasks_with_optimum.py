#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import pathmagic
with pathmagic.context():
    import filter_comparison as helper

if __name__ == "__main__":
    # load data
    prefix = "comparison/data/optimum_varying_num_tasks_"

    num_agents = int(np.genfromtxt(
        prefix+"num_agents.csv", delimiter=","))
    _time_used_list_all_cases_cbba = np.genfromtxt(
        prefix+"time_used_list_all_cases_cbba.csv", delimiter=",")
    _time_used_list_all_cases_my = np.genfromtxt(
        prefix+"time_used_list_all_cases_my.csv", delimiter=",")
    _time_used_list_all_cases_os = np.genfromtxt(
        prefix+"time_used_list_all_cases_os.csv", delimiter=",")
    _distance_list_all_cases_cbba = np.genfromtxt(
        prefix+"distance_list_all_cases_cbba.csv", delimiter=",")
    _distance_list_all_cases_my = np.genfromtxt(
        prefix+"distance_list_all_cases_my.csv", delimiter=",")
    _distance_list_all_cases_os = np.genfromtxt(
        prefix+"distance_list_all_cases_os.csv", delimiter=",")
    _infeasible_list_all_cases_cbba = np.genfromtxt(
        prefix+"infeasible_list_all_cases_cbba.csv", delimiter=",")
    _infeasible_list_all_cases_os = np.genfromtxt(
        prefix+"infeasible_list_all_cases_os.csv", delimiter=",")
    _infeasible_list_all_cases_my = np.genfromtxt(
        prefix+"infeasible_list_all_cases_my.csv", delimiter=",")

    xticks_str_list = np.genfromtxt(
        prefix+"xticks_str_list.csv", delimiter=",", dtype="int")
    xticks_str_list.astype('str')
    try:
        len(xticks_str_list)
    except:
        xticks_str_list = [xticks_str_list]


    # xticks of plots
    xticks_list = range(1, len(_time_used_list_all_cases_my)+1)


    # filter the data
    infeasible_list_all_cases = helper.find_infeasible_all(_infeasible_list_all_cases_my, \
        _infeasible_list_all_cases_cbba, _infeasible_list_all_cases_os)
    
    print(infeasible_list_all_cases)

    time_used_list_all_cases_cbba, distance_list_all_cases_cbba = \
        helper.find_other_lists(_time_used_list_all_cases_cbba, _distance_list_all_cases_cbba, infeasible_list_all_cases)
    time_used_list_all_cases_my, distance_list_all_cases_my = \
        helper.find_other_lists(_time_used_list_all_cases_my, _distance_list_all_cases_my, infeasible_list_all_cases)
    time_used_list_all_cases_os, distance_list_all_cases_os = \
        helper.find_other_lists(_time_used_list_all_cases_os, _distance_list_all_cases_os, infeasible_list_all_cases)


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

    mean_time_cbba = list()
    std_time_cbba = list()
    mean_distance_cbba = list()
    std_distance_cbba = list()
    for idx in range(len(time_used_list_all_cases_cbba)):
        mean_time_cbba.append(np.mean(time_used_list_all_cases_cbba[idx]))
        std_time_cbba.append(np.std(time_used_list_all_cases_cbba[idx]))
        mean_distance_cbba.append(np.mean(distance_list_all_cases_cbba[idx]))
        std_distance_cbba.append(np.std(distance_list_all_cases_cbba[idx]))


    # create box plot for both algorithm
    fig1, ax1 = plt.subplots()
    ax1.set_title('Computation time, num_agents = ' + str(num_agents))
    # create plot
    x_pos = np.array(range(len(time_used_list_all_cases_my)))*2.0-0.5
    ax1.bar(x_pos, mean_time_my, yerr=std_time_my, color='blue', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
    x_pos = np.array(range(len(time_used_list_all_cases_os)))*2.0+0.0
    ax1.bar(x_pos, mean_time_os, yerr=std_time_os, color='green', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
    x_pos = np.array(range(len(time_used_list_all_cases_cbba)))*2.0+0.5
    ax1.bar(x_pos, mean_time_cbba, yerr=std_time_cbba, color='red', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
    ax1.set_xlabel('Number of tasks')
    ax1.set_ylabel('Computation time [ms]')
    ax1.yaxis.grid(True)

    print(xticks_str_list)

    plt.xticks(range(0, len(xticks_str_list)*2, 2), xticks_str_list)
    # set legends
    colors = ["blue", "green", "red"]
    labels = ["Proposed", "Optimal", "CBBA"]
    f = lambda c: plt.plot([],[], color=c)[0]
    handles = [f(colors[i]) for i in range(len(labels))]
    legend = plt.legend(handles, labels, loc='upper left', framealpha=1)


    # create box plot about total distance for both algorithm
    fig2, ax2 = plt.subplots()
    ax2.set_title('Total distance, num_agents = ' + str(num_agents))
    # create plot
    x_pos = np.array(range(len(distance_list_all_cases_my)))*2.0-0.5
    ax2.bar(x_pos, mean_distance_my, yerr=std_distance_my, color='blue', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
    x_pos = np.array(range(len(distance_list_all_cases_os)))*2.0+0.0
    ax2.bar(x_pos, mean_distance_os, yerr=std_distance_os, color='green', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
    x_pos = np.array(range(len(distance_list_all_cases_cbba)))*2.0+0.5
    ax2.bar(x_pos, mean_distance_cbba, yerr=std_distance_cbba, color='red', width=0.4, align='center', alpha=0.5, ecolor='black', capsize=5)
    ax2.set_xlabel('Number of tasks')
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
