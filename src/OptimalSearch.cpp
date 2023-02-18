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
inline std::tuple< std::vector<std::vector<int>>, float, bool> OptimalSearch(
    std::vector<int>& agent_position,
    std::vector<int>& targets_position,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    std::tuple<std::vector<std::vector<int>>, float, bool> result = run_optimal_search(agent_position, targets_position, Map, mapSizeX, mapSizeY);

    std::vector<std::vector<int>> allocation_result = std::get<0>(result);
    float cost = std::get<1>(result);
    bool infeasible = std::get<2>(result);
    // std::cout << "Minimum cost = " << cost << "\n";
    // std::cout << "Index of agent and task starts from 0 \n";
    // std::cout << "Path \n";
    // for (int i = 0; i < allocation_result.size(); i++) {
    //     std::cout << "Agent " << i << ":";
    //     for (int j = 0; j < allocation_result[i].size(); j++) {
    //         std::cout << " -> " << allocation_result[i][j];
    //     }
    //     std::cout << "\n";
    // }
    // std::cout << "Infeasible = " << infeasible << "\n";

    return {allocation_result, cost, infeasible};
}


inline PYBIND11_MODULE(OptimalSearch, module) {
    module.doc() = "Python wrapper of Optimal Search";

    module.def("OptimalSearch", &OptimalSearch, "Search the optimal solution for multi-agent mission planning");
}
