#include <limits>
#include <ctime>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <random>
#include <typeinfo>
#include "../include/ga_solver.hpp"


int main()
{
    std::clock_t time_start;
    double time_duration;

    time_start = std::clock();

    // Number of nodes in TSP
    const size_t num_nodes = 5;
    // Initial population size for the algorithm 
    const size_t pop_size = 50;
    // Number of Gene Iterations 
    const size_t gen_thres = 30;
    
    constexpr bool PRINT_FLAG = true; // true to print during Genetic Algorithm

    std::vector<std::vector<float>> distance_matrix{
        { 0, 2, 100, 12, 5 },
        { 2, 0, 4, 8, 20 },
        { 3, 4, 0, 3, 3 },
        { 12, 8, 3, 0, 10 },
        { 5, 1, 3, 10, 0 } };

    // std::vector<std::vector<float>> distance_matrix{
    //     { 0, 1, 6, 3, 7 },
    //     { 1, 0, 2, 10, 4 },
    //     { 6, 2, 0, 2, LARGE_NUM },
    //     { 3, 10, 2, 0, 13 },
    //     { 7, 4, LARGE_NUM, 13, 0 } };

    // std::vector<int> and float
    auto [index_route_vec, cost] = Run(num_nodes, pop_size, gen_thres, distance_matrix, PRINT_FLAG);

    time_duration = (std::clock() - time_start) / (double)CLOCKS_PER_SEC;
    std::cout << "Time used [sec]: " << time_duration << std::endl;

    std::cout << "Task allocation and its cost\n";
    std::cout << "0 is the initial position and 1 is the first task\n";
    for (auto ele : index_route_vec) std::cout << ele << "->";
    std::cout << "   " << cost << std::endl;
}