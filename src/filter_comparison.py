#!/usr/bin/env python3
import math
import numpy as np


def find_infeasible_all(*args):
    """
    Given infeasible list of all methods, return infeasible list where at least one method is infeasible.

    Element is 1 when infeasible, 0 when feasible.
    """
    infeasible_list_all = np.array(args[0], dtype="bool")
    for idx in range(len(args) - 1):
        infeasible_list_all = infeasible_list_all | np.array(args[idx + 1], dtype="bool")
    return infeasible_list_all


def find_other_lists(time_used_list_all_cases: list, distance_list_all_cases: list, infeasible_list_all: list):
    """
    Given infeasible list returned by the above function, return the list when all methods are feasible.
    """
    if len(infeasible_list_all.shape) == 2:
        new_time_used_list_all_cases = []
        new_distance_list_all_cases = []
        for i in range(len(time_used_list_all_cases)):
            new_time_used_list_this = []
            new_distance_used_list_this = []
            for j in range(len(time_used_list_all_cases[i])):
                if not infeasible_list_all[i][j]:
                    new_time_used_list_this.append(time_used_list_all_cases[i][j])
                    new_distance_used_list_this.append(distance_list_all_cases[i][j])
            new_time_used_list_all_cases.append(new_time_used_list_this)
            new_distance_list_all_cases.append(new_distance_used_list_this)
    elif len(infeasible_list_all.shape) == 1:
        new_time_used_list_all_cases = []
        new_distance_list_all_cases = []
        for i in range(len(time_used_list_all_cases)):
            if not infeasible_list_all[i]:
                new_time_used_list_all_cases.append(time_used_list_all_cases[i])
                new_distance_list_all_cases.append(distance_list_all_cases[i])
        new_time_used_list_all_cases = [new_time_used_list_all_cases]
        new_distance_list_all_cases = [new_distance_list_all_cases]
    else:
        raise Exception("Wrong data dimension!")

    return new_time_used_list_all_cases, new_distance_list_all_cases


if __name__ == "__main__":
    a = np.array([[1,0,0,0], [1,1,0,0]])
    b = np.array([[0,0,0,1], [1,1,0,0]])
    c = np.array([[0,0,0,1], [1,0,0,0]])

    re1 = find_infeasible_all(a, b, c)
    print(re1)

    prefix = "comparison/data/varying_num_agents_"
    x = np.genfromtxt(
        prefix+"infeasible_list_all_cases_my.csv", delimiter=",")
    y = np.genfromtxt(
        prefix+"infeasible_list_all_cases_cbba.csv", delimiter=",")

    re2 = find_infeasible_all(x, y)
    print(re2)
