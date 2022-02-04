#ifndef SOLVE_TEST_HPP
#define SOLVE_TEST_HPP

#include <iostream>
#include <vector>
#include <array>
#include <tuple>
#include "../externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include/tileadaptor.hpp"
#include "../externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include/utility.hpp"
#include "../externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include/get_combination.hpp"


#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"


using namespace operations_research;


inline void PrintSolution(
    const RoutingIndexManager& manager,
    const RoutingModel& routing,
    const Assignment& solution)
{
    // Inspect solution.
    LOG(INFO) << "Objective: " << solution.ObjectiveValue() << " miles";
    int64 index = routing.Start(0);
    LOG(INFO) << "Route:";
    int64 distance{0};
    std::stringstream route;
    while (routing.IsEnd(index) == false) {
        route << manager.IndexToNode(index).value() << " -> ";
        int64 previous_index = index;
        index = solution.Value(routing.NextVar(index));
        distance += routing.GetArcCostForVehicle(previous_index, index, int64{0});
    }
    LOG(INFO) << route.str() << manager.IndexToNode(index).value();
    LOG(INFO) << "Route distance: " << distance << "miles";
    LOG(INFO) << "";
    LOG(INFO) << "Advanced usage:";
    LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
}


inline std::tuple< std::vector<std::vector<int>>, std::vector<float>, std::vector<std::vector<int64_t>> > solve_test(
    int *agent_position,
    std::vector<int> &targets_position,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    int num_nodes = targets_position.size()/2 + 1; // number of targets + one agent
    std::vector<int> start_goal_pair = get_combination(num_nodes, 2);
    std::vector<std::vector<int>> path_many;
    std::vector<float> distances_many;
    std::vector<std::vector<int64_t>> distance_matrix(num_nodes, std::vector<int64_t>(num_nodes, 0));
    int start[2];
    int goal[2];
    // about solving tsp
    constexpr int num_vehicles_tsp = 1;
    constexpr RoutingIndexManager::NodeIndex depot_tsp{0};

    //Instantiating our path adaptor
    //passing the map size, and the map
    Vectori mapSize(mapSizeX, mapSizeY);
    TileAdaptor adaptor(mapSize, Map);
    //This is a bit of an exageration here for the weight, but it did make my performance test go from 8s to 2s
    Pathfinder pathfinder(adaptor, 100.f /*weight*/);

    for (unsigned long idx = 0; idx < start_goal_pair.size(); idx = idx + 2)
    {
        int start_idx = start_goal_pair[idx];
        int goal_idx = start_goal_pair[idx+1];

        if (start_idx != 0)
        {
            start[0] = targets_position[2*(start_idx-1)];
            start[1] = targets_position[2*(start_idx-1)+1];
        }
        else
        {
            start[0] = agent_position[0];
            start[1] = agent_position[1];
        }

        if (goal_idx != 0)
        {
            goal[0] = targets_position[2*(goal_idx-1)];
            goal[1] = targets_position[2*(goal_idx-1)+1];

        }
        else
        {
            goal[0] = agent_position[0];
            goal[1] = agent_position[1];
        }

        //doing the search
        auto [Path, Distance] = pathfinder.search(start[1]*mapSizeX+start[0], goal[1]*mapSizeX+goal[0], mapSize);
        path_many.push_back(Path);
        distances_many.push_back(Distance);

        // assign distance to distance matrix
        int i = num_nodes - 2 - floor(sqrt(-8*(idx/2) + 4*num_nodes*(num_nodes-1)-7)/2.0 - 0.5);
        int j = (idx/2) + i + 1 - (num_nodes*(num_nodes-1))/2 + ((num_nodes-i)*((num_nodes-i)-1))/2;

        if (Path.size() > 0)
        {
            distance_matrix[i][j] = floor(Distance);
            distance_matrix[j][i] = floor(Distance);
        }
        else
        {
            distance_matrix[i][j] = INT64_MAX;
            distance_matrix[j][i] = INT64_MAX;
        }
        
        // Regenerate the neighbors for next run
        if (idx < start_goal_pair.size()-1)
        {
            pathfinder.generateNodes();
        }

    }

    std::cout << "This is distance_matrix: " << std::endl;
    for (unsigned long i = 0; i < distance_matrix.size(); i++)
    {
        for (unsigned long j = 0; j < distance_matrix[i].size(); j++)
        {
            std::cout << distance_matrix[i][j] << ", ";
        }
        std::cout << std::endl;
    }




    // Create Routing Index Manager
    RoutingIndexManager manager(distance_matrix.size(), num_vehicles_tsp, depot_tsp);
    // Create Routing Model.
    RoutingModel routing(manager);

    const int transit_callback_index = routing.RegisterTransitCallback(
        [&distance_matrix, &manager](int64 from_index, int64 to_index) -> int64 {
        // Convert from routing variable Index to distance matrix NodeIndex.
        auto from_node = manager.IndexToNode(from_index).value();
        auto to_node = manager.IndexToNode(to_index).value();
        return distance_matrix[from_node][to_node];
        });

    // Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
    // Setting first solution heuristic.
    RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
    searchParameters.set_first_solution_strategy(FirstSolutionStrategy::AUTOMATIC);

    // Solve the problem.
    const Assignment* solution = routing.SolveWithParameters(searchParameters);

    // Print solution on console.
    PrintSolution(manager, routing, *solution);

    return {path_many, distances_many, distance_matrix};
}

#endif