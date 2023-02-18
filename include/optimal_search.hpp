#include <iostream>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <tuple>
#include "../externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include/find_path.hpp"

int tree_height = 0;
int feasible_path = 0;

inline void print_task_number(std::vector<int> &num_task_assign, const int &num_agent) {
    for (int i = 0; i < num_agent; i++) {
        std::cout << num_task_assign[i] << " ";
    }
    std::cout << "\n";
}

inline void move_one_task_to_next_agent(std::vector<int> &num_task_assign, const int &agent_index) {
    num_task_assign[agent_index]--;
    num_task_assign[agent_index + 1]++;
}

inline void move_all_task_back(std::vector<int> &num_task_assign, const int &num_agent) {
    int agent_index = num_agent - 1;
    while (num_task_assign[agent_index - 1] == 1) {
        agent_index--;
    }

    [[unlikely]] if (agent_index == num_agent - 1) {
        return;
    }

    num_task_assign[agent_index] += num_task_assign[num_agent - 1] - 1;
    num_task_assign[num_agent - 1] = 1;
}


inline void permutation_order_task(
    const std::vector<int> &tree, 
    const int &num_agent, 
    const int &num_task, 
    const std::vector<int> &agent_position, 
    const std::vector<int> &targets_position, 
    const std::vector<int> &map, 
    const int &mapSizeX, 
    const int &mapSizeY, 
    const std::vector<int> &num_task_assign, 
    std::vector<int> &solution, 
    float &cost_now) {

    float all_cost[tree_height];
    for (int k = 0; k < tree_height; k++) {
        int index = 0;
        float cost_total = 0;
        
        // all agents
        for (int i = 0; i < num_agent; i++) {
            // each agent
            int current_position[2] = {agent_position[2*i], agent_position[2*i+1]};
            for (int j = 0; j < num_task_assign[i]; j++) {              
                int next_position[2] = {targets_position[2*(tree[k*num_task + index]-1)], targets_position[2*tree[k*num_task + index]-1]};  
                std::vector<int> path;
                float new_cost;
                std::tie(path, new_cost) = find_path(current_position, next_position, map, mapSizeX, mapSizeY);
                // CHECK PATH INFEASIBLITY
                if (path.size() == 2) {
                    // when start = goal, let path = {start, goal}, and distance = 0
                    new_cost = 0;
                }
                else if (path.size() < 2) {
                    // if no feasible, set as a large number, but not the maximum of int64_t
                    new_cost = std::numeric_limits<int>::max() / num_task / num_agent / num_task;
                }
                else {
                    feasible_path++;
                }
                cost_total += new_cost;
                
                current_position[0] = next_position[0];
                current_position[1] = next_position[1];
                
                index++;
            }
        }

        all_cost[k] = cost_total;   
    }

    for (int k= 0; k < tree_height; k++) {
        if (all_cost[k] < cost_now) {
            cost_now = all_cost[k];
            int index = 0;
            for (int i = 0; i < num_agent; i++) {
                for (int j = 0; j < num_task; j++) {
                    if (j < num_task_assign[i]) {
                        solution[i*num_task+j] = tree[k*num_task + index];
                        index++;
                    }
                    else {
                        solution[i*num_task+j] = -1;
                    }
                }
            }

        }
    } 
}

inline void swap(int task_index[], const int &a, const int &b) {
    // int temp = task_index[a];
    // task_index[a]= task_index[b];
    // task_index[b] = temp;

    std::swap(task_index[a], task_index[b]);
}

inline void get_tree(std::vector<int> &tree, int task_index[], const int &start_idx, const int &end_idx, const int &num_task) {
    if (start_idx == end_idx) {
        for (int i = 0; i < num_task; i++) {
            tree.push_back(task_index[i]);
        }
    } else {
        for (int i = start_idx; i <= end_idx; i++) {
            swap(task_index, start_idx, i);
            get_tree(tree, task_index, start_idx + 1, end_idx, num_task);
            swap(task_index, start_idx, i);   
        }
    }
}

inline std::tuple<std::vector<std::vector<int>>, float, bool> run_optimal_search(
    std::vector<int>& agent_position,
    std::vector<int>& targets_position,
    const std::vector<int> &map, 
    const int &mapSizeX, 
    const int &mapSizeY) 
{
    // permutation_num_task
    const int num_agent = static_cast<int>(agent_position.size() / 2);
    const int num_task = static_cast<int>(targets_position.size() / 2);
    int solution[num_agent * num_task] = {-1};
    bool infeasible = true;

    tree_height = 1;
    for(int i = 1; i <= num_task; ++i) {
        tree_height *= i;
    }

    // int C_task = 1;
    // int C_agent = 1;
    // int C_diff = 1;
    // for(int i = 1; i < num_task; ++i) {
        // C_task *= i;
    // }
    // for(int i = 1; i < num_agent; ++i) {
        // C_agent *= i;
    // }
    // for(int i = 1; i <= num_task - num_agent; ++i) {
        // C_diff *= i;
    // }

    // int expect_case = factorial*C_task/C_agent/C_diff;
    float cost_now = 1E8;
    int agent_index = 0;
    std::vector<int> num_task_assign;
    for (int i = 0; i < num_agent; i++) {
        if (i == 0) {
            num_task_assign.push_back(num_task - num_agent + 1);
        }
        else {
            num_task_assign.push_back(1);
        }
    }

    int total_case_assign_num_task = 0;
    
    std::vector<int> task_index_vec(num_task);
    for (int i = 0; i < num_task; i++) {
        task_index_vec[i] = i + 1;
    }
    int* task_index = task_index_vec.data();

    std::vector<int> tree;
    get_tree(tree, task_index, 0, num_task-1, num_task);

    std::vector<int> solution_vec(num_agent*num_task, -1);

    // int percent = 0;
    while (true) {
        // move one task to tail
        while (agent_index != num_agent - 1) {
            permutation_order_task(tree, num_agent, num_task, agent_position, targets_position, map, mapSizeX, mapSizeY, num_task_assign, solution_vec, cost_now);
            total_case_assign_num_task++;
            move_one_task_to_next_agent(num_task_assign, agent_index);
            agent_index++;
        }
        // for last one
        permutation_order_task(tree, num_agent, num_task, agent_position, targets_position, map, mapSizeX, mapSizeY, num_task_assign, solution_vec, cost_now);
        total_case_assign_num_task++;
        // stop consition
        [[unlikely]] if (num_task_assign[num_agent - 1] == num_task - num_agent + 1) {
            break;
        }
        // move agent pointer back
        agent_index--;
        while (num_task_assign[agent_index] == 1) {
            agent_index--;
        }

        move_all_task_back(num_task_assign, num_agent);
        move_one_task_to_next_agent(num_task_assign, agent_index);
        agent_index++;
    }

    for (int i = 0; i < num_agent*num_task; i++) {
        solution[i] = solution_vec[i];
    }

    std::vector<std::vector<int>> allocation_result;
    for (int i = 0; i < num_agent; i++) {
        std::vector<int> result_this;
        for (int j = 0; j < num_task; j++) {
            if (solution[i*num_task+j]-1 >= 0) result_this.push_back(solution[i*num_task+j]-1);
        }
        allocation_result.push_back(result_this);
    }

    if (feasible_path != 0) {
        infeasible = false;
    }

    return {allocation_result, cost_now, infeasible};
    
}