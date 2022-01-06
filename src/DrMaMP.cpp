#include <iostream>
#include <vector>
#include <tuple>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <math.h>
#include <limits>
#include <algorithm>
#include <future>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "tileadaptor.hpp"
#include "utility.hpp"
#include "get_combination.hpp"
#include "k_means.hpp"
#include "k_means_with_plus_plus.hpp"

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

#include "ortools/base/logging.h"
#include "ortools/linear_solver/linear_solver.h"


using namespace operations_research;
constexpr int SCALE_FACTOR = 100; // to scale the distance matrix for solving TSP


/*
PrintSolution: Print the solution of traveling salesman problem via or-tools on console.
Examples shown in SolveOneAgent() or https://developers.google.com/optimization/routing/tsp#c++

Input:
    manager: RoutingIndexManager
    routing: RoutingModel
    solution: Assignment

Output:
    void
*/
inline void PrintSolution(
    const RoutingIndexManager& manager,
    const RoutingModel& routing,
    const Assignment& solution)
{
    // Inspect solution.
    LOG(INFO) << "Objective: " << solution.ObjectiveValue() / SCALE_FACTOR << " [unit]";
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
    // Skip the last one, because in my case, the last one is a dummy variable
    // so that the agent doesn't have to go back to the starting point.
    // LOG(INFO) << route.str() << manager.IndexToNode(index).value();
    LOG(INFO) << route.str();
    LOG(INFO) << "Route distance: " << distance / SCALE_FACTOR << "[unit]";
    LOG(INFO) << "";
    LOG(INFO) << "Advanced usage:";
    LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
}


/*
FindPathOneByOne: Find all the collision-free paths consecutively, i.e. the paths from a0~t0, t0~t1, t1~t2, ...

Input:
    agent_position: 1D integer array [x, y] for the agent position
    targets_position: 1D integer array [x0,y0, x1,y1, x2,y2, ...] for the targets positions
    Map: 1D integer array for the map, flattened by a 2D array map; 0 for no obstacles, 255 for obstacles
    mapSizeX: integer for the width of the map
    mapSizeY: integer for the height of the map

Output:
    path: 2D integer array for all the index paths, [[idx_x_0,idx_y_0, idx_x_1,idx_y_1, ...], [idx_x_0,idx_y_0, idx_x_1,idx_y_1, ...], ...]
    distance: 1D float array for all the distances of the paths
*/
inline std::tuple<std::vector<std::vector<int>>, std::vector<float>> FindPathOneByOne(
    std::vector<int> &agent_position,
    std::vector<int> &targets_position,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    std::vector<std::vector<int>> path_many;
    std::vector<float> distances_many;
    int start[2];
    int goal[2];

    // Instantiating our path adaptor
    // passing the map size, and the map
    Vectori mapSize(mapSizeX, mapSizeY);
    TileAdaptor adaptor(mapSize, Map);
    // This is a bit of an exageration here for the weight, but it did make my performance test go from 8s to 2s
    Pathfinder pathfinder(adaptor, 100.f /*weight*/);

    if (targets_position.size() > 0) {
        // start is agent_position, goal is the first two elements of targets_position, doing the search
        // output: std::tuple<std::vector<int>, float>
        auto [Path, Distance] = pathfinder.search(agent_position[1]*mapSizeX+agent_position[0], targets_position[1]*mapSizeX+targets_position[0], mapSize);
        path_many.push_back(Path);
        distances_many.push_back(Distance);

        // Regenerate the neighbors for next run
        // if (idx < start_goal_pair.size()-1)
        pathfinder.generateNodes();

        for (size_t idx = 2; idx < targets_position.size(); idx = idx + 2) {
            goal[0] = targets_position[idx];
            goal[1] = targets_position[idx+1];

            start[0] = targets_position[idx-2];
            start[1] = targets_position[idx-1];

            // start is the previous target, goal is the current target, doing the search
            // output: std::tuple<std::vector<int>, float>
            auto [Path, Distance] = pathfinder.search(start[1]*mapSizeX+start[0], goal[1]*mapSizeX+goal[0], mapSize);
            path_many.push_back(Path);
            distances_many.push_back(Distance);

            // Regenerate the neighbors for next run
            pathfinder.generateNodes();
        }

    }

    // if targets_position is empty, return empty arrays
    return {path_many, distances_many};
}


/*
FindPathMany: Find all the collision-free paths from every element to another element in start+targets.

Input:
    agent_position: 1D integer array [x, y] for the agent position
    targets_position: 1D integer array [x0,y0, x1,y1, x2,y2, ...] for the targets positions
    Map: 1D integer array for the map, flattened by a 2D array map; 0 for no obstacles, 255 for obstacles
    mapSizeX: integer for the width of the map
    mapSizeY: integer for the height of the map

Output:
    path: 2D integer array for all the index paths, [[idx_x_0,idx_y_0, idx_x_1,idx_y_1, ...], [idx_x_0,idx_y_0, idx_x_1,idx_y_1, ...], ...]
    distance: 1D float array for all the distances of the paths
*/
inline std::tuple<std::vector<std::vector<int>>, std::vector<float>> FindPathMany(
    std::vector<int> &agent_position,
    std::vector<int> &targets_position,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    std::vector<int> start_goal_pair = get_combination(targets_position.size()/2 + 1, 2);
    std::vector<std::vector<int>> path_many;
    std::vector<float> distances_many;
    int start[2];
    int goal[2];

    // Instantiating our path adaptor
    // passing the map size, and the map
    Vectori mapSize(mapSizeX, mapSizeY);
    TileAdaptor adaptor(mapSize, Map);
    // This is a bit of an exageration here for the weight, but it did make my performance test go from 8s to 2s
    Pathfinder pathfinder(adaptor, 100.f /*weight*/);

    for (size_t idx = 0; idx < start_goal_pair.size(); idx = idx + 2) {
        int start_idx = start_goal_pair[idx];
        int goal_idx = start_goal_pair[idx+1];

        if (start_idx != 0) {
            start[0] = targets_position[2*(start_idx-1)];
            start[1] = targets_position[2*(start_idx-1)+1];
        }
        else {
            start[0] = agent_position[0];
            start[1] = agent_position[1];
        }

        if (goal_idx != 0) {
            goal[0] = targets_position[2*(goal_idx-1)];
            goal[1] = targets_position[2*(goal_idx-1)+1];

        }
        else {
            goal[0] = agent_position[0];
            goal[1] = agent_position[1];
        }

        // doing the search
        // output: std::tuple<std::vector<int>, float>
        // when infeasible, Path is empty, Distance = 0
        auto [Path, Distance] = pathfinder.search(start[1]*mapSizeX+start[0], goal[1]*mapSizeX+goal[0], mapSize);
        path_many.push_back(Path);
        distances_many.push_back(Distance);

        // Regenerate the neighbors for next run
        // if (idx < start_goal_pair.size()-1)
        pathfinder.generateNodes();
    }

    return {path_many, distances_many};
}


/*
FindPath: Find a collision-free path from a start to a target.

Input:
    startPoint: 1D integer array [x, y] for the start position
    endPoint: 1D integer array [x, y] for the goal position
    Map: 1D integer array for the map, flattened by a 2D array map; 0 for no obstacles, 255 for obstacles
    mapSizeX: integer for the width of the map
    mapSizeY: integer for the height of the map

Output:
    path: 1D integer array for the index path from startPoint to endPoint, [idx_x_0,idx_y_0, idx_x_1,idx_y_1, idx_x_2,idx_y_2, ...]
    distance: float for the total distance of the path
*/
inline std::tuple<std::vector<int>, float> FindPath(
    std::vector<int> &startPoint,
    std::vector<int> &endPoint,
    std::vector<int> &Map,
    int &mapSizeX,
    int &mapSizeY)
{
    // Instantiating our path adaptor
    // passing the map size, and the map
    Vectori mapSize(mapSizeX, mapSizeY);

    TileAdaptor adaptor(mapSize, Map);
    
    // This is a bit of an exageration here for the weight, but it did make my performance test go from 8s to 2s
    Pathfinder pathfinder(adaptor, 100.f /*weight*/);

    // The map was edited so we need to regenerate teh neighbors
    // pathfinder.generateNodes();

    // doing the search
    // merly to show the point of how it work
    // as it would have been way easier to simply transform the vector to id and pass it to search

    // output: std::tuple<std::vector<int>, float>
    // auto [Path, Distance] = pathfinder.search(startPoint[1]*mapSizeX+startPoint[0], endPoint[1]*mapSizeX+endPoint[0], mapSize);

    return pathfinder.search(startPoint[1]*mapSizeX+startPoint[0], endPoint[1]*mapSizeX+endPoint[0], mapSize);
}


inline std::tuple< std::vector<float>, std::vector<size_t>, std::vector<std::vector<size_t>>, std::vector<float> > KMeans(
    std::vector<int>& targets_position,
    const size_t& num_cluster,
    const size_t& number_of_iterations)
{
    std::vector<float> targets_position_float(targets_position.begin(), targets_position.end());
    return k_means_with_plus_plus(targets_position_float, num_cluster, number_of_iterations);
}


inline std::tuple< std::vector<float>, std::vector<std::vector<size_t>>, std::vector<size_t> > AssignCluster(
    std::vector<int>& agent_position,
    std::vector<int>& targets_position,
    const size_t& num_cluster,
    const size_t& number_of_iterations)
{
    // convert raw target positions to Points
    std::vector<float> targets_position_float(targets_position.begin(), targets_position.end());
    // K-means clustering
    auto [cluster_centers, assignments, points_idx_for_clusters, sum_distance_vec] = k_means_with_plus_plus(targets_position_float, num_cluster, number_of_iterations);
    size_t num_agents = agent_position.size()/2;

    // Solver
    // Create the mip solver with the SCIP backend.
    // *****************************************************************This line takes most of the time!!!
    MPSolver solver("assignment_mip", MPSolver::SCIP_MIXED_INTEGER_PROGRAMMING);

    // Create binary integer variables
    // x[i][j] is an array of 0-1 variables, which will be 1 if cluster i is assigned to agent j.
    std::vector<std::vector<const MPVariable*>> x(num_cluster, std::vector<const MPVariable*>(num_agents));

    // Create the objective function
    MPObjective* const objective = solver.MutableObjective();

    for (size_t i = 0; i < num_cluster; ++i) {
        // Create the constraints
        // Each cluster is assigned to at most one agent.
        LinearExpr cluster_sum;

        for (size_t j = 0; j < num_agents; ++j) {
            // for binary integer variables
            x[i][j] = solver.MakeIntVar(0, 1, "");
            
            // for constraints
            cluster_sum += x[i][j];

            // distance btw a cluster centroid and an agent plus the summation of distance btw this centroid and its associated points
            float cost = std::sqrt(std::pow(static_cast<float>(agent_position[2*j])-cluster_centers[2*i], 2) + 
                                   std::pow(static_cast<float>(agent_position[2*j+1])-cluster_centers[2*i+1], 2)) + 
                         sum_distance_vec[i];


            // // here estimate the distance by l1 norm
            // float cost = 1 * (abs( static_cast<float>(agent_position[2*j]) - cluster_centers[2*i] ) + 
            //     abs( static_cast<float>(agent_position[2*j+1]) - cluster_centers[2*i+1] ));

            // for the objective function
            objective->SetCoefficient(x[i][j], cost);
        }

        // for constraints
        solver.MakeRowConstraint(cluster_sum <= 1.0);
    }
    // for constraints
    objective->SetMinimization();

    // Each task is assigned to exactly one worker.
    for (size_t j = 0; j < num_agents; ++j) {
        LinearExpr agent_sum;
        for (size_t i = 0; i < num_cluster; ++i) {
            agent_sum += x[i][j];
        }
        solver.MakeRowConstraint(agent_sum == 1.0);
    }

    std::vector<size_t> cluster_assigned_idx(num_agents);

    // Solve
    const MPSolver::ResultStatus result_status = solver.Solve();

    // Print solution.
    // Check that the problem has a feasible solution.
    if ( (result_status != MPSolver::OPTIMAL) & (result_status != MPSolver::FEASIBLE) ) {
        LOG(FATAL) << "No solution found.";
    }

    // LOG(INFO) << "Total cost = " << objective->Value() << "\n\n";

    // std::cout << "Clusters and agents index start from 0" << std::endl;
    for (size_t i = 0; i < num_cluster; ++i) {
        for (size_t j = 0; j < num_agents; ++j) {
            // Test if x[i][j] is 0 or 1 (with tolerance for floating point arithmetic).
            if (x[i][j]->solution_value() > 0.5) {

                // float cost = 1 * ( std::pow( static_cast<float>(agent_position[2*j]-cluster_centers[2*i]), 2 ) + 
                // std::pow( static_cast<float>(agent_position[2*j+1]-cluster_centers[2*i+1]), 2 ) );

                // LOG(INFO) << "Cluster " << i << " assigned to agent " << j
                // << ".  Cost = " << cost;

            // cluster_assigned_idx stores the cluster index for each agent
            // cluster_assigned_idx[3] = 5 means Cluster 5 is assigned to Agent 3
            cluster_assigned_idx[j] = i;
            }
        }
    }

    return {cluster_centers, points_idx_for_clusters, cluster_assigned_idx};
}


/*
SolveOneAgent: Find all the collision-free paths from every element to another element in start+targets,
    and return an optimal order for the agent to explore all the targets, and the concatenated path given the order.

Input:
    agent_position: 1D integer array [x, y] for the agent position
    targets_position: 1D integer array [x0,y0, x1,y1, x2,y2, ...] for the targets positions
    Map: 1D integer array for the map, flattened by a 2D array map; 0 for no obstacles, 255 for obstacles
    mapSizeX: integer for the width of the map
    mapSizeY: integer for the height of the map

Output:
    path_many_result: 2D integer array for all the index paths given the task allocation order,
        [[x0,y0, ..., x1,y1], [x1,y1, ..., x2,y2], [x2,y2, ..., x3,y3], ...]
    target_idx_order: 1D integer array for task allocation order, 
        [0, 4, 3, 1, 2, 5] means the task allocation order is T0 -> T4 -> T3 -> T1 -> T2 -> T5,
        where T0 is the first task.
*/
inline std::tuple< std::vector<std::vector<int>>, std::vector<size_t> > SolveOneAgent(
    std::vector<int> &agent_position,
    std::vector<int> &targets_position,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    int num_nodes = targets_position.size()/2 + 1; // number of targets + one agent
    std::vector<int> start_goal_pair = get_combination(num_nodes, 2);
    std::vector<std::vector<int>> path_many;
    std::vector<std::vector<int64_t>> distance_matrix(num_nodes+1, std::vector<int64_t>(num_nodes+1, 0));
    int start[2];
    int goal[2];
    // about solving tsp
    constexpr int num_vehicles_tsp = 1;
    const std::vector<RoutingIndexManager::NodeIndex> start_tsp{ RoutingIndexManager::NodeIndex{0} };
    // for example: 1 agent, 5 targets, num_nodes=6 (#0~#5), then add one more dummy node #6
    const std::vector<RoutingIndexManager::NodeIndex> end_tsp{ RoutingIndexManager::NodeIndex{num_nodes} };

    // Instantiating our path adaptor
    // passing the map size, and the map
    Vectori mapSize(mapSizeX, mapSizeY);
    TileAdaptor adaptor(mapSize, Map);
    // This is a bit of an exageration here for the weight, but it did make my performance test go from 8s to 2s
    Pathfinder pathfinder(adaptor, 100.f /*weight*/);

    for (size_t idx = 0; idx < start_goal_pair.size(); idx = idx + 2) {
        int start_idx = start_goal_pair[idx];
        int goal_idx = start_goal_pair[idx+1];

        if (start_idx != 0) {
            start[0] = targets_position[2*(start_idx-1)];
            start[1] = targets_position[2*(start_idx-1)+1];
        }
        else {
            start[0] = agent_position[0];
            start[1] = agent_position[1];
        }

        if (goal_idx != 0) {
            goal[0] = targets_position[2*(goal_idx-1)];
            goal[1] = targets_position[2*(goal_idx-1)+1];

        }
        else {
            goal[0] = agent_position[0];
            goal[1] = agent_position[1];
        }

        // doing the search, when infeasible, Path is empty, Distance = 0
        // when start = goal, Path = start and Distance is empty
        // std::vector<int>, float
        auto [Path, Distance] = pathfinder.search(start[1]*mapSizeX+start[0], goal[1]*mapSizeX+goal[0], mapSize);

        // assign distance to distance matrix
        int i = num_nodes - 2 - floor(sqrt(-8*(idx/2) + 4*num_nodes*(num_nodes-1)-7)/2.0 - 0.5);
        int j = (idx/2) + i + 1 - (num_nodes*(num_nodes-1))/2 + ((num_nodes-i)*((num_nodes-i)-1))/2;

        if (Path.size() > 2) {
            distance_matrix[i][j] = static_cast<int64_t>( floor(SCALE_FACTOR * Distance) );
            distance_matrix[j][i] = distance_matrix[i][j];
        }
        else if (Path.size() == 2) {
            // when start = goal, let Path = {start, goal}, and Distance = 0
            Path.push_back(Path[0]);
            Path.push_back(Path[1]);
            distance_matrix[i][j] = static_cast<int64_t>( floor(SCALE_FACTOR * 0.0) );
            distance_matrix[j][i] = distance_matrix[i][j];
        }
        else {
            // if no feasible, set as a large number, but not the maximum of int64_t
            distance_matrix[i][j] = static_cast<int64_t>( std::numeric_limits<int>::max() / SCALE_FACTOR / SCALE_FACTOR );
            distance_matrix[j][i] = distance_matrix[i][j];
        }

        path_many.push_back(Path);
        
        // Regenerate the neighbors for next run
        // if (idx < start_goal_pair.size()-1)
        pathfinder.generateNodes();
    }

    // Create Routing Index Manager
    RoutingIndexManager manager(distance_matrix.size(), num_vehicles_tsp, start_tsp, end_tsp);
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

    // // Print solution on console.
    // PrintSolution(manager, routing, *solution);

    // std::cout << "This is distance_matrix: " << std::endl;
    // for (size_t i = 0; i < distance_matrix.size(); i++)
    // {
    //     for (size_t j = 0; j < distance_matrix[i].size(); j++)
    //     {
    //         std::cout << distance_matrix[i][j] << ", ";
    //     }
    //     std::cout << std::endl;
    // }

    // parse the solution, index_route_vec is a vector of route node index, 0 means the agent, 1 ~ n means the fitst ~ n-th target
    // the last node is a dummy node to relax the constraint that the agent must go back to its original location, so skip the last node
    // example: index_route_vec = [0, 4, 3, 1, 2, 5], 0 means the agent, 1 ~ 5 means the fitst ~ 5-th target,
    // the task allocation order is agent 0 -> T4 -> T3 -> T1 -> T2 -> T5
    std::vector<int> index_route_vec;
    int64 index_now = routing.Start(0);

    while (routing.IsEnd(index_now) == false) {
        index_route_vec.push_back(manager.IndexToNode(index_now).value());
        index_now = solution->Value(routing.NextVar(index_now));
    }

    // need to map index route to path index, if we have route x -> y, we want to map [x,y] to the corresponding index of path_many, 
    // which indicates the route x -> y.

    // start_goal_pair is the start-goal index pair, path_many stores paths along entries of start_goal_pair
    // For example: there is 1 agent and 5 targers, start_goal_pair (row-major) = 
    //    [0,1,  0,2,  0,3,  0,4,  0,5,
    //     1,2,  1,3,  1,4,  1,5,
    //     2,3,  2,4,  2,5,
    //     3,4,  3,5,
    //     4,5]

    // Assume Route [x,y], x < y, is located in x-row, (y-x-1)-column, so there are (x-1) rows before [x,y], denote N = num_targets.
    // So the total number of elements before x-row is N + (N-1) + (N-2) + ... + (N-(x-1)) = (( N + (N-(x-1)) ) * x) / 2 = ( (2*N-x+1)*x ) / 2
    // This formula also holds when x = 0.
    // And the number of elements in x-row before [x,y] is y - x - 1, note indices start from 0.
    // So the index of path_many corresponding to route [x,y] is (2*N-x+1)*x/2 + y-x-1
    // size_t index_x_to_y = (2*N-x+1)*x/2 + y-x-1;
    // std::vector<int> path_x_to_y = path_many[index_x_to_y];

    // When Route [x,y], x > y, locate the path index for route [y,x], then the path can be obtained by reversing the path of route [y,x].

    int x, y; // the route index [x,y]
    size_t idx_path; // the corresponding path index for route [x,y]
    std::vector<std::vector<int>> path_many_result; // the paths given route [x,y]
    for (size_t i = 0; i < index_route_vec.size()-1; ++i) {
        if (index_route_vec[i] < index_route_vec[i+1]) {
            x = index_route_vec[i];
            y = index_route_vec[i+1];
            idx_path = static_cast<size_t>( (2*(num_nodes-1)-x+1)*x/2 + y-x-1 );
            path_many_result.push_back(path_many[idx_path]);
        }
        else {
            x = index_route_vec[i+1];
            y = index_route_vec[i];
            idx_path = static_cast<size_t>( (2*(num_nodes-1)-x+1)*x/2 + y-x-1 );
            std::vector<int> path_now = path_many[idx_path];

            // if route [x,y], x > y, we need to reverse path_now
            // let path_now = [x0,y0, x1,y1, x2,y2], then path_reversed = [x2,y2, x1,y1, x0,y0]
            // so it's NOT an exact vector reverse

            // swap the first entry with the second to last, swap the second with the last
            // swap the third with the forth to last, swap the forth with the third to last
            // until met the central 2 entries, skip them

            // when the path is empty, do not swap
            for (int i = 0; i < static_cast<int>(path_now.size()/2) - 1; i = i + 2) {
                std::iter_swap(path_now.begin()+i, path_now.end()-i-2);
                std::iter_swap(path_now.begin()+i+1, path_now.end()-i-1);
            }
            path_many_result.push_back(path_now);
        }
    }

    // target_idx_order = [0, 3, 4, 2, 1] means the task allocation order is
    // T0 -> T3 -> T4 -> T2 -> T1, where T0 is the first task
    std::vector<size_t> target_idx_order;
    for (size_t i = 0; i < index_route_vec.size()-1; ++i) {
        target_idx_order.push_back(index_route_vec[i+1] - 1);
    }

    return {path_many_result, target_idx_order};
}


inline std::tuple< std::vector<std::vector<int>>, std::vector<size_t> > mission_planning_one_agent(
    const size_t &idx_agent, const std::vector<int> &agent_position, const std::vector<int> &targets_position,
    const std::vector<size_t> &cluster_assigned_idx, const std::vector<std::vector<size_t>> &points_idx_for_clusters,
    const std::vector<int> &Map, const int &mapSizeX, const int &mapSizeY)
{
    // the index of assigned cluster for every agent
    // [2, 0, 1] indicates that agent-0 with cluster-2, agent-1 with cluster-0, agent-2 with cluster-1
    size_t cluster_idx = cluster_assigned_idx[idx_agent];

    // each agent's position
    std::vector<int> agent_position_each_agent {agent_position[2*idx_agent], agent_position[2*idx_agent+1]};

    // number of targets for each agent
    size_t num_targets_each_agent = points_idx_for_clusters[cluster_idx].size();

    std::vector<int> targets_position_each_agent(2*num_targets_each_agent);

    for (size_t j = 0; j < num_targets_each_agent; ++j) {
        size_t target_id = points_idx_for_clusters[cluster_idx][j];
        targets_position_each_agent[2*j] = targets_position[2*target_id];
        targets_position_each_agent[2*j+1] = targets_position[2*target_id+1];
    }

    // remember to add other agents as obstacles
    std::tuple< std::vector<std::vector<int>>, std::vector<size_t> > result_tuple_one_agent;
    std::vector<std::vector<int>> path_many_each_agent;
    std::vector<size_t> target_idx_order;

    if (num_targets_each_agent > 0) {
        result_tuple_one_agent = SolveOneAgent(agent_position_each_agent, targets_position_each_agent, Map, mapSizeX, mapSizeY);
        std::tie(path_many_each_agent, target_idx_order) = result_tuple_one_agent;
    }
    // if no tasks assigned to this agent, target_idx_order is empty vectors
    return {path_many_each_agent, target_idx_order};
}


/*
MissionPlanning: for multiple agents, segment targets into clusters by K-means Clustering first,
    then assign clusters to agents, and run SolveOneAgent for each agent with multithreading to
    have the collision-free path and task allocation result.

Input:
    agent_position: 1D integer array [x0,y0, x1,y1, x2,y2, ...] for the agents positions
    targets_position: 1D integer array [x0,y0, x1,y1, x2,y2, ...] for the targets positions
    num_cluster: positive integer for number of clusters you want to have for K-means Clustering
    number_of_iterations: positive integer for number of iterations for K-means Clustering
    Map: 1D integer array for the map, flattened by a 2D array map; 0 for no obstacles, 255 for obstacles
    mapSizeX: integer for the width of the map
    mapSizeY: integer for the height of the map

Output:
    task_allocation_agents: 2D integer array, each sub-array is the task allocation for an agent
        exmaple: task_allocation_agents[1] = [0, 3, 2, 1] means the task allocation order for Agent 1 is
        T0 -> T3 -> T2 -> T1, where Agent 1 is the second agent, T0 is the first task
*/
inline std::tuple< std::vector<std::vector<std::vector<int>>>, std::vector<std::vector<size_t>>,
    std::vector<float>, std::vector<std::vector<size_t>>, std::vector<size_t> > MissionPlanning(
    std::vector<int>& agent_position,
    std::vector<int>& targets_position,
    const size_t& num_cluster,
    const size_t& number_of_iterations,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    size_t num_agents = agent_position.size()/2;
    // std::tuple< std::vector<float>, std::vector<std::vector<size_t>>, std::vector<size_t> >
    auto [cluster_centers, points_idx_for_clusters, cluster_assigned_idx] = AssignCluster(agent_position, targets_position, num_cluster, number_of_iterations);

    // a 3D vector, each sub 2D vector is the path for each agent
    std::vector<std::vector<std::vector<int>>> path_all_agents;

    // a 2D vector, each sub 1D vector indicates the task allocation order for each agent
    // note that this index is for local target set
    std::vector<std::vector<size_t>> task_allocation_agents_local;

    // a vector of result of asynchronous operations
    std::vector<std::future< std::tuple<std::vector<std::vector<int>>, std::vector<size_t>> >> vec_async_op;

    for (size_t idx_agent = 0; idx_agent < num_agents; ++idx_agent) {
        vec_async_op.push_back(std::async(std::launch::async, &mission_planning_one_agent, idx_agent, agent_position, targets_position,
            cluster_assigned_idx, points_idx_for_clusters, Map, mapSizeX, mapSizeY));
    }

    for (auto &async_op : vec_async_op) {
        std::tuple<std::vector<std::vector<int>>, std::vector<size_t>> result_tuple_one_agent = async_op.get();
        path_all_agents.push_back(std::get<0>(result_tuple_one_agent));
        task_allocation_agents_local.push_back(std::get<1>(result_tuple_one_agent));
    }

    // convert the local target index to the global target index
    std::vector<std::vector<size_t>> task_allocation_agents;
    for (size_t idx_agent = 0; idx_agent < num_agents; ++idx_agent) {
        // the cluster index for current agent
        size_t cluster_idx_this = cluster_assigned_idx[idx_agent];
        // the associated targets set/pool
        std::vector<size_t> targets_pool_this = points_idx_for_clusters[cluster_idx_this];
        // the global target index vector for current agent
        std::vector<size_t> task_allocation_this;
        for (size_t i = 0; i < targets_pool_this.size(); ++i) {
            // the global target index
            size_t task_id_this = task_allocation_agents_local[idx_agent][i];
            // bundle this index by a given sequence
            task_allocation_this.push_back(targets_pool_this[task_id_this]);
        }
        task_allocation_agents.push_back(task_allocation_this);
    }

    return {path_all_agents, task_allocation_agents, cluster_centers, points_idx_for_clusters, cluster_assigned_idx};
}


inline std::tuple< std::vector<std::vector<std::vector<int>>>, std::vector<std::vector<size_t>> > MissionPlanningWithClustering(
    std::vector<int>& agent_position,
    std::vector<int>& targets_position,
    std::vector<std::vector<size_t>>& points_idx_for_clusters,
    std::vector<size_t>& cluster_assigned_idx,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    size_t num_agents = agent_position.size()/2;

    // a 3D vector, each sub 2D vector is the path for each agent
    std::vector<std::vector<std::vector<int>>> path_all_agents;

    // a 2D vector, each sub 1D vector indicates the task allocation order for each agent
    // note that this index is for local target set
    std::vector<std::vector<size_t>> task_allocation_agents_local;

    // a vector of result of asynchronous operations
    std::vector<std::future< std::tuple<std::vector<std::vector<int>>, std::vector<size_t>> >> vec_async_op;

    for (size_t idx_agent = 0; idx_agent < num_agents; ++idx_agent) {
        vec_async_op.push_back(std::async(std::launch::async, &mission_planning_one_agent, idx_agent, agent_position, targets_position,
            cluster_assigned_idx, points_idx_for_clusters, Map, mapSizeX, mapSizeY));
    }

    for (auto &async_op : vec_async_op) {
        std::tuple<std::vector<std::vector<int>>, std::vector<size_t>> result_tuple_one_agent = async_op.get();
        path_all_agents.push_back(std::get<0>(result_tuple_one_agent));
        task_allocation_agents_local.push_back(std::get<1>(result_tuple_one_agent));
    }

    // convert the local target index to the global target index
    std::vector<std::vector<size_t>> task_allocation_agents;
    for (size_t idx_agent = 0; idx_agent < num_agents; ++idx_agent) {
        // the cluster index for current agent
        size_t cluster_idx_this = cluster_assigned_idx[idx_agent];
        // the associated targets set/pool
        std::vector<size_t> targets_pool_this = points_idx_for_clusters[cluster_idx_this];
        // the global target index vector for current agent
        std::vector<size_t> task_allocation_this;
        for (size_t i = 0; i < targets_pool_this.size(); ++i) {
            // the global target index
            size_t task_id_this = task_allocation_agents_local[idx_agent][i];
            // bundle this index by a given sequence
            task_allocation_this.push_back(targets_pool_this[task_id_this]);
        }
        task_allocation_agents.push_back(task_allocation_this);
    }

    return {path_all_agents, task_allocation_agents};
}


inline std::vector<std::vector<int>> path_planning_one_agent_many_tasks(
    const size_t &idx_agent,
    const std::vector<int> &agent_position,
    const std::vector<std::vector<int>> &targets_position,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    // copy map to revise it for each thread
    std::vector<int> MapNew = Map;
    // initialize start point as each agent's position
    int startPoint[2] = {agent_position[2*idx_agent], agent_position[2*idx_agent+1]};
    // each agent's target list [x0,y0, x1,y1, x2,y2, ...]
    std::vector<int> targets_position_this_agent = targets_position[idx_agent];

    std::vector<std::vector<int>> path_many;

    for (size_t idx_target = 0; idx_target < targets_position_this_agent.size()/2; ++idx_target) {
        int endPoint[2] = {targets_position_this_agent[2*idx_target], targets_position_this_agent[2*idx_target+1]};

        // remember to add other agents as obstacles (this is the buffered version)
        for (size_t idx_agent_ = 0; idx_agent_ < agent_position.size()/2; ++idx_agent_) {
            [[likely]] if (idx_agent_ != idx_agent) {

                // buffering the surrounding cells as well
                int x = agent_position[2*idx_agent_];
                int y = agent_position[2*idx_agent_+1];

                int half_length_x = 3;
                int half_length_y = 3;

                int x_min = std::max(0, x-half_length_x);
                int x_max = std::min(mapSizeX-1, x+half_length_x);
                int y_min = std::max(0, y-half_length_y);
                int y_max = std::min(mapSizeY-1, y+half_length_y);

                for (int idx_x = x_min; idx_x < x_max+1; ++idx_x )
                    {
                        for (int idx_y = y_min; idx_y < y_max+1; ++idx_y )
                            MapNew[idx_y * mapSizeX + idx_x] = 255;
                    }
            }
        }


        // // remember to add other agents as obstacles
        // for (size_t idx_agent_ = 0; idx_agent_ < agent_position.size()/2; ++idx_agent_) {
        //     MapNew[agent_position[2*idx_agent_+1] * mapSizeX + agent_position[2*idx_agent_]] = 255;
        // }
        // MapNew[agent_position[2*idx_agent+1] * mapSizeX + agent_position[2*idx_agent]] = 0;


        // Instantiating the path adaptor, passing the map size, and the map
        Vectori mapSize(mapSizeX, mapSizeY);
        TileAdaptor adaptor(mapSize, MapNew);
        Pathfinder pathfinder(adaptor, 100.f /*weight*/);
        // find the path, returned by std::tuple<std::vector<int>, float>
        auto [path, distance] = pathfinder.search(startPoint[1]*mapSizeX+startPoint[0], endPoint[1]*mapSizeX+endPoint[0], mapSize);


        // if the start == end, path only has [x0,y0], then make it empty
        if (path.size() < 4) {
            path = {};
        }


        // initialize the start point of next iteration as the end point of this iteration
        startPoint[0] = targets_position_this_agent[2*idx_target];
        startPoint[1] = targets_position_this_agent[2*idx_target+1];

        // path for this start point to this end point
        path_many.push_back(path);
    }

    return path_many;
}


inline std::vector<std::vector<std::vector<int>>> PathPlanningMultiAgent(
    std::vector<int>& agent_position,
    std::vector<std::vector<int>>& targets_position,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    size_t num_agents = agent_position.size()/2;

    // a 3D vector, each sub-vector is a path for each agent, from the agent position to target positions
    // path_all_agents[0] = [[x0,y0, ..., x1,y1], [x1,y1, ..., x2,y2], ...] is the path for agent 0 [x0,y0] to
    // task 0 [x1,y1], and task 0 [x1,y1] to task 1 [x2,y2], etc.
    std::vector<std::vector<std::vector<int>>> path_all_agents;

    // a vector of result of asynchronous operations
    std::vector<std::future< std::vector<std::vector<int>> >> vec_async_op;

    for (size_t idx_agent = 0; idx_agent < num_agents; ++idx_agent) {
        vec_async_op.push_back(std::async(std::launch::async, &path_planning_one_agent_many_tasks,
            idx_agent, agent_position, targets_position, Map, mapSizeX, mapSizeY));
    }

    for (auto &async_op : vec_async_op) {
        std::vector<std::vector<int>> path_current = async_op.get();
        path_all_agents.push_back(path_current);
    }

    return path_all_agents;
}


inline std::tuple< std::vector<std::vector<std::vector<int>>>, std::vector<std::vector<size_t>>, 
    std::vector<float>, std::vector<std::vector<size_t>>, std::vector<size_t> > MissionPlanning_legacy(
    std::vector<int>& agent_position,
    std::vector<int>& targets_position,
    const size_t& num_cluster,
    const size_t& number_of_iterations,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    size_t num_agents = agent_position.size()/2;
    // std::tuple< std::vector<float>, std::vector<std::vector<size_t>>, std::vector<size_t> >
    auto [cluster_centers, points_idx_for_clusters, cluster_assigned_idx] = AssignCluster(agent_position, targets_position, num_cluster, number_of_iterations);

    // a 3D vector, each sub 2D vector is the path for each agent
    std::vector<std::vector<std::vector<int>>> path_all_agents;

    // a 2D vector, each sub 1D vector indicates the task allocation order for each agent
    std::vector<std::vector<size_t>> task_allocation_agents;

    for (size_t idx_agent = 0; idx_agent < num_agents; ++idx_agent) {
        // the index of assigned cluster for every agent
        // [2, 0, 1] indicates that agent-0 with cluster-2, agent-1 with cluster-0, agent-2 with cluster-1
        size_t cluster_idx = cluster_assigned_idx[idx_agent];

        // each agent's position
        std::vector<int> agent_position_each_agent {agent_position[2*idx_agent], agent_position[2*idx_agent+1]};

        // number of targets for each agent
        size_t num_targets_each_agent = points_idx_for_clusters[cluster_idx].size();

        std::vector<int> targets_position_each_agent(2*num_targets_each_agent);
        
        for (size_t j = 0; j < num_targets_each_agent; ++j) {
            size_t target_id = points_idx_for_clusters[cluster_idx][j];
            targets_position_each_agent[2*j] = targets_position[2*target_id];
            targets_position_each_agent[2*j+1] = targets_position[2*target_id+1];
        }
        
        // remember to add other agents as obstacles
        std::tuple< std::vector<std::vector<int>>, std::vector<size_t> > result_SolveOneAgent;
        std::vector<std::vector<int>> path_many_each_agent;
        std::vector<size_t> target_idx_order;

        if (num_targets_each_agent > 0) {
            result_SolveOneAgent = SolveOneAgent(agent_position_each_agent, targets_position_each_agent, Map, mapSizeX, mapSizeY);
            std::tie(path_many_each_agent, target_idx_order) = result_SolveOneAgent;
        }
        // no tasks assigned to this agent, pass empty vectors

        // targets_position_assigned_agents.push_back(targets_position_each_agent);
        path_all_agents.push_back(path_many_each_agent);

        task_allocation_agents.push_back(target_idx_order);
    }

    return {path_all_agents, task_allocation_agents, cluster_centers, points_idx_for_clusters, cluster_assigned_idx};
}


inline PYBIND11_MODULE(DrMaMP, module) {
    module.doc() = "Python wrapper of DrMaMP Solver";

    module.def("FindPath", &FindPath, "Find a collision-free path from a start to a target");

    module.def("FindPathMany", &FindPathMany, "Find all the collision-free paths from every element to another element in start+targets");

    module.def("FindPathOneByOne", &FindPathOneByOne, "Find all the collision-free paths consecutively");

    module.def("SolveOneAgent", &SolveOneAgent, 
        "Find all the collision-free paths from every element to another element in start+targets, and return an optimal order for the agent to explore all the targets, and the concatenated path given the order.");

    module.def("KMeans", &KMeans, "K-means Clustering to segment all the targets to several clusters");

    module.def("AssignCluster", &AssignCluster, "K-means Clustering first, then assign clusters to agents, #clusters >= #agents");

    module.def("MissionPlanning", &MissionPlanning, "K-means Clustering, then assign clusters to agents, and run SolveOneAgent for each agent (with multithreading)");

    module.def("MissionPlanningWithClustering", &MissionPlanningWithClustering, "Run SolveOneAgent for each agent (with multithreading) given a clustering result");

    module.def("PathPlanningMultiAgent", &PathPlanningMultiAgent, "Planning path for multiple agents, each agent is associated with a target");

    module.def("MissionPlanning_legacy", &MissionPlanning_legacy, "Old version of MissionPlanning using for-loops (without multithreading)");
}
