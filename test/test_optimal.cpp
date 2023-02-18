#include <iostream>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <tuple>
#include <chrono>
#include "../externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include/find_path.hpp"
#include "../include/optimal_search.hpp"

int main() {
    using namespace std::chrono;
    auto start = high_resolution_clock::now();
    int agent[] = {1,1, 1,2, 1,3};
    int targets[] = {2,1, 2,3, 1,4, 7,7, 8,1, 9,1};

    std::vector<int> agent_position;
    for (int i = 0; i < sizeof(agent)/sizeof(agent[0]); i++) {
        agent_position.push_back(agent[i]);
    }
    std::vector<int> targets_position;
    for (int i = 0; i < sizeof(targets)/sizeof(targets[0]); i++) {
        targets_position.push_back(targets[i]);
    }

    std::vector<int> map;
    int mapSizeX = 10;
    int mapSizeY = 10;
    for (int i = 0; i < mapSizeX * mapSizeY; i++) {
        map.push_back(0);
    }
    map[11] = 1;

    // path, cost, feasible
    std::tuple<std::vector<std::vector<int>>, float, bool> result = run_optimal_search(agent_position, targets_position, map, mapSizeX, mapSizeY);
  
    std::vector<std::vector<int>> path = std::get<0>(result);
    float cost = std::get<1>(result);
    bool infeasible = std::get<2>(result);
    std::cout << "Minimum cost = " << cost << "\n";
    std::cout << "Index of agent and task starts from 0 \n";
    std::cout << "Path \n";
    for (int i = 0; i < path.size(); i++) {
        std::cout << "Agent " << i << ":";
        for (int j = 0; j < path[i].size(); j++) {
            std::cout << " -> " << path[i][j];
        }
        std::cout << "\n";
    }
    std::cout << "Infeasible = " << infeasible << "\n";

    auto stop = high_resolution_clock::now();
    auto run_time = duration_cast<milliseconds>(stop - start);
    std::cout << "Run time: " << run_time.count() << "ms\n";

    return 0;
}
