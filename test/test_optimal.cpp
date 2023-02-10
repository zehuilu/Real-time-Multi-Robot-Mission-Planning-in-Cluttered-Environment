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
    int agent_position[] = {1,1, 1,2, 1,3};
    int targets_position[] = {2,1, 2,3, 1,4, 7,7, 8,1, 9,1};
    const int num_agent = sizeof(agent_position)/sizeof(agent_position[0]) / 2;
    const int num_task = sizeof(targets_position)/sizeof(targets_position[0]) / 2;
    std::vector<int> map;
    int mapSizeX = 10;
    int mapSizeY = 10;
    for (int i = 0; i < mapSizeX * mapSizeY; i++) {
        map.push_back(0);
    }

    int solution[num_agent * num_task] = {-1};
    float cost = permutation_num_task(num_agent, num_task, agent_position, targets_position, map, mapSizeX, mapSizeY, solution);
  
    std::cout << "Minimum cost = " << cost << "\n";
    std::cout << "Index of agent and task starts from 0 \n";
    std::cout << "Path \n";
    for (int i = 0; i < num_agent; i++) {
        std::cout << "Agent " << i << ":";
        for (int j = 0; j < num_task; j++) {
            if (solution[i*num_task+j]-1 >= 0) std::cout << " -> " << solution[i*num_task+j] - 1;  
        }
        std::cout << "\n";
    }

    auto stop = high_resolution_clock::now();
    auto run_time = duration_cast<milliseconds>(stop - start);
    std::cout << "Run time: " << run_time.count() << "ms\n";

    return 0;
}
