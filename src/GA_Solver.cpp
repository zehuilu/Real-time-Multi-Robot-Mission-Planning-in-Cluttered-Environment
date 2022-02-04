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

#include "../externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include/tileadaptor.hpp"
#include "../externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include/utility.hpp"
#include "../externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include/get_combination.hpp"
#include "../include/ga_solver.hpp"
#include "../include/k_means_with_plus_plus.hpp"

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

#include "ortools/base/logging.h"
#include "ortools/linear_solver/linear_solver.h"


using namespace operations_research;


static constexpr float WEIGHT_PATH = 1E2; // weight for path finder
static constexpr bool PRINT_FLAG = false; // true to print during Genetic Algorithm


/*
_SolveSingle: Find all the collision-free paths from every element to another element in start+targets,
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
inline std::tuple< std::vector<std::vector<int>>, std::vector<size_t>, float > _SolveSingle(
    const std::vector<int> &agent_position,
    const std::vector<int> &targets_position,
    const size_t &population_size,
    const size_t &max_iter,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    int num_nodes = targets_position.size()/2 + 1; // number of targets + one agent
    std::vector<int> start_goal_pair = get_combination(num_nodes, 2);
    std::vector<std::vector<int>> path_many;
    std::vector<std::vector<float>> distance_matrix(num_nodes, std::vector<float>(num_nodes, 0));
    int start[2];
    int goal[2];

    // Instantiating our path adaptor
    // passing the map size, and the map
    Vectori mapSize(mapSizeX, mapSizeY);
    TileAdaptor adaptor(mapSize, Map);
    // This is a bit of an exageration here for the weight, but it did make my performance test go from 8s to 2s
    Pathfinder pathfinder(adaptor, WEIGHT_PATH);

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

        // doing the search, when infeasible, path is empty, distance = 0
        // when start = goal, path = start and distance is empty
        // std::vector<int>, float
        auto [path, distance] = pathfinder.search(start[1]*mapSizeX+start[0], goal[1]*mapSizeX+goal[0], mapSize);

        // assign distance to distance matrix
        int i = num_nodes - 2 - floor(sqrt(-8*(idx/2) + 4*num_nodes*(num_nodes-1)-7)/2.0 - 0.5);
        int j = (idx/2) + i + 1 - (num_nodes*(num_nodes-1))/2 + ((num_nodes-i)*((num_nodes-i)-1))/2;

        if (path.size() > 2) {
            distance_matrix[i][j] = distance;
            distance_matrix[j][i] = distance_matrix[i][j];
        }
        else if (path.size() == 2) {
            // when start = goal, let path = {start, goal}, and distance = 0
            path.push_back(path[0]);
            path.push_back(path[1]);
            distance_matrix[i][j] = 0.0;
            distance_matrix[j][i] = distance_matrix[i][j];
        }
        else {
            // if no feasible, set as a large number, but not the maximum of int64_t
            distance_matrix[i][j] = LARGE_NUM;
            distance_matrix[j][i] = distance_matrix[i][j];
        }

        path_many.push_back(path);
        
        // Regenerate the neighbors for next run
        // if (idx < start_goal_pair.size()-1)
        pathfinder.generateNodes();
    }

    // std::cout << "This is distance_matrix: " << std::endl;
    // for (size_t i = 0; i < distance_matrix.size(); i++)
    // {
    //     for (size_t j = 0; j < distance_matrix[i].size(); j++)
    //     {
    //         std::cout << distance_matrix[i][j] << ", ";
    //     }
    //     std::cout << std::endl;
    // }

    // run GA Solver to get the result, std::vector<int> and float
    auto [index_route_vec, cost] = Run(num_nodes, population_size, max_iter, distance_matrix, PRINT_FLAG);

    // index_route_vec is a vector of route node index, 0 means the agent, 1 ~ n means the fitst ~ n-th target
    // example: index_route_vec = [0, 4, 3, 1, 2, 5], 0 means the agent, 1 ~ 5 means the fitst ~ 5-th target,
    // the task allocation order is agent 0 -> T4 -> T3 -> T1 -> T2 -> T5

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

    return {path_many_result, target_idx_order, cost};
}


/*
SolveSingleAgent: Find all the collision-free paths from every element to another element in start+targets,
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
inline std::tuple< std::vector<std::vector<int>>, std::vector<size_t>, float > SolveSingleAgent(
    const std::vector<int> &agent_position,
    const std::vector<int> &targets_position,
    const size_t &population_size,
    const size_t &max_iter,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    size_t num_targets = targets_position.size() / 2;

    std::tuple< std::vector<std::vector<int>>, std::vector<size_t>, float > result_tuple;
    std::tuple<std::vector<int>, float> result_tuple_one_target;
    std::vector<int> path_single;
    std::vector<std::vector<int>> path_many;
    std::vector<size_t> target_idx_order;
    float cost;

    if (num_targets > 1) {
        result_tuple = _SolveSingle(agent_position, targets_position, population_size, max_iter, Map, mapSizeX, mapSizeY);
        std::tie(path_many, target_idx_order, cost) = result_tuple;
    }
    else if (num_targets > 0) {
        // Instantiating our path adaptor, passing the map size, and the map
        Vectori mapSize(mapSizeX, mapSizeY);

        TileAdaptor adaptor(mapSize, Map);
    
        // This is a bit of an exageration here for the weight, but it did make my performance test go from 8s to 2s
        Pathfinder pathfinder(adaptor, WEIGHT_PATH);

        // doing the search
        result_tuple_one_target = pathfinder.search(agent_position[1]*mapSizeX+agent_position[0], targets_position[1]*mapSizeX+targets_position[0], mapSize);
        std::tie(path_single, cost) = result_tuple_one_target;

        path_many.push_back(path_single);
        target_idx_order.push_back(0);
    }
    return {path_many, target_idx_order, cost};
}


inline std::tuple< std::vector<float>, std::vector<std::vector<size_t>>, std::vector<size_t> > AssignCluster(
    const std::vector<int>& agent_position,
    const std::vector<int>& targets_position,
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


inline std::tuple< std::vector<std::vector<int>>, std::vector<size_t>, float > mission_planning_one_agent(
    const size_t &idx_agent, const std::vector<int> &agent_position, const std::vector<int> &targets_position,
    const std::vector<size_t> &cluster_assigned_idx, const std::vector<std::vector<size_t>> &points_idx_for_clusters,
    const size_t &population_size, const size_t &max_iter,
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
    std::tuple< std::vector<std::vector<int>>, std::vector<size_t>, float > result_tuple_one_agent;
    std::tuple<std::vector<int>, float> result_tuple_one_target;
    std::vector<int> path_single;
    std::vector<std::vector<int>> path_many_each_agent;
    std::vector<size_t> target_idx_order;
    float cost;

    if (num_targets_each_agent > 1) {
        result_tuple_one_agent = _SolveSingle(agent_position_each_agent, targets_position_each_agent,
                                              population_size, max_iter, Map, mapSizeX, mapSizeY);
        std::tie(path_many_each_agent, target_idx_order, cost) = result_tuple_one_agent;
    }
    else if (num_targets_each_agent > 0) {
        // Instantiating our path adaptor, passing the map size, and the map
        Vectori mapSize(mapSizeX, mapSizeY);

        TileAdaptor adaptor(mapSize, Map);
    
        // This is a bit of an exageration here for the weight, but it did make my performance test go from 8s to 2s
        Pathfinder pathfinder(adaptor, WEIGHT_PATH);

        // doing the search
        result_tuple_one_target = pathfinder.search(agent_position_each_agent[1]*mapSizeX+agent_position_each_agent[0],
                                                    targets_position_each_agent[1]*mapSizeX+targets_position_each_agent[0], mapSize);
        std::tie(path_single, cost) = result_tuple_one_target;

        path_many_each_agent.push_back(path_single);
        target_idx_order.push_back(0);
    }
    // if no tasks assigned to this agent, target_idx_order is empty vectors
    return {path_many_each_agent, target_idx_order, cost};
}


/*
MissionPlanning: for multiple agents, segment targets into clusters by K-means Clustering first,
    then assign clusters to agents, and run SolveSingleAgent for each agent with multithreading to
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
    float, std::vector<float>, std::vector<std::vector<size_t>>, std::vector<size_t> > MissionPlanning(
    const std::vector<int>& agent_position,
    const std::vector<int>& targets_position,
    const size_t& num_cluster,
    const size_t& number_of_iterations,
    const size_t& population_size,
    const size_t& max_iter,
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

    // a 1D vector, each element is the cost for each agent
    std::vector<float> cost_vec;

    // a vector of result of asynchronous operations
    std::vector<std::future< std::tuple<std::vector<std::vector<int>>, std::vector<size_t>, float> >> vec_async_op;

    for (size_t idx_agent = 0; idx_agent < num_agents; ++idx_agent) {
        vec_async_op.push_back(std::async(std::launch::async, &mission_planning_one_agent, idx_agent, agent_position, targets_position,
            cluster_assigned_idx, points_idx_for_clusters, population_size, max_iter, Map, mapSizeX, mapSizeY));
    }

    for (auto &async_op : vec_async_op) {
        std::tuple<std::vector<std::vector<int>>, std::vector<size_t>, float> result_tuple_one_agent = async_op.get();
        path_all_agents.push_back(std::get<0>(result_tuple_one_agent));
        task_allocation_agents_local.push_back(std::get<1>(result_tuple_one_agent));
        cost_vec.push_back(std::get<2>(result_tuple_one_agent));
    }

    // std::cout << "cost_vec" << std::endl;
    // for (auto ele : cost_vec) std::cout << ele << "->";
    // std::cout << " " << std::endl;

    // total cost for all agents
    float cost_total = 0.0;
    for (auto& c : cost_vec) cost_total += c;

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

    return {path_all_agents, task_allocation_agents, cost_total, cluster_centers, points_idx_for_clusters, cluster_assigned_idx};
}


inline PYBIND11_MODULE(GA_Solver, module) {
    module.doc() = "Python wrapper of GA_Solver";

    module.def("AssignCluster", &AssignCluster, "K-means Clustering first, then assign clusters to agents, #clusters >= #agents");

    module.def("SolveSingleAgent", &SolveSingleAgent, "Solve the single-agent mission planning problem by GA algorithm");

    module.def("MissionPlanning", &MissionPlanning, "K-means Clustering, then assign clusters to agents, and run SolveSingleAgent for each agent (with multithreading)");

}
