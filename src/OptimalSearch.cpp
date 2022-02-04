#include <iostream>
#include <vector>
#include <iterator>
#include <tuple>
#include "../include/optimal_search.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

/*
Main function for Optimal Search.
*/
inline std::tuple< std::vector<std::vector<int>>, float > OptimalSearch(
    std::vector<int>& agent_position,
    std::vector<int>& targets_position,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    const int num_agent = static_cast<int>(agent_position.size() / 2);
    const int num_task = static_cast<int>(targets_position.size() / 2);

    int* agent_position_array = agent_position.data();
    int* targets_position_array = targets_position.data();

    std::vector<int> start_case_vec(num_agent * num_task, -1);
    int *solution = start_case_vec.data(); 

    float cost = permutation_num_task(num_agent, num_task, agent_position_array, targets_position_array, Map, mapSizeX, mapSizeY, solution);

    // each sub-vector is the indices of the assigned tasks, where the order is the execution order
    std::vector<std::vector<int>> allocation_result;
    for (int i = 0; i < num_agent; i++) {
        std::vector<int> result_this;
        for (int j = 0; j < num_task; j++) {
            if (solution[i*num_task+j]-1 >= 0) result_this.push_back(solution[i*num_task+j]-1);
        }
        allocation_result.push_back(result_this);
    }

    return {allocation_result, cost};
}


inline PYBIND11_MODULE(OptimalSearch, module) {
    module.doc() = "Python wrapper of Optimal Search";

    module.def("OptimalSearch", &OptimalSearch, "Search the optimal solution for multi-agent mission planning");
}
