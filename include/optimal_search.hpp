#include <stdio.h>
#include <iostream>
#include <tuple>
#include "../externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include/find_path.hpp"

// int case_count = 0;

inline void print_task_number(int num_task_assign[], int num_agent) {
    for (int i = 0; i < num_agent; i++) {
        std::cout << num_task_assign[i] << " ";
    }
    std::cout << "\n";
}

inline void move_one_task_to_next_agent(int num_task_assign[], int agent_index) {
    num_task_assign[agent_index]--;
    num_task_assign[agent_index + 1]++;
}

inline void move_all_task_back(int num_task_assign[], int num_agent) {
    int agent_index = num_agent - 1;
    while (num_task_assign[agent_index - 1] == 1) {
        agent_index--;
    }

    if (agent_index == num_agent - 1) {
        return;
    }

    num_task_assign[agent_index] += num_task_assign[num_agent - 1] - 1;
    num_task_assign[num_agent - 1] = 1;
}

inline void permutation_order_task(int task_index[], int size, int num_agent, int num_task, int agent_position[], int targets_position[], std::vector<int> map, int mapSizeX, int mapSizeY, int num_task_assign[], int solution[], float &cost_now) {

    if (size == 1) {
        // case_count++;
        // if (case_count % 1000 == 0) {
        //     std::cout << case_count << "\n";
        // }

        int index = 0;
        float cost_total = 0;
        // all agents
        for (int i = 0; i < num_agent; i++) {
            // each agent
            int current_position[2] = {agent_position[2*i], agent_position[2*i+1]};
            // std::cout << "[" << current_position[0] << ", " << current_position[1] << "]\n";
            for (int j = 0; j < num_task_assign[i]; j++) {
                
                int next_position[2] = {targets_position[2*(task_index[index]-1)], targets_position[2*task_index[index]-1]};  
                // std::cout << "(" << next_position[0] << ", " << next_position[1] << ")\n";
                std::vector<int> path;
                float new_cost;
                std::tie(path, new_cost) = find_path(current_position, next_position, map, mapSizeX, mapSizeY);
                cost_total += new_cost;
                
                current_position[0] = next_position[0];
                current_position[1] = next_position[1];
               
                index++;
                // std::cout << "path[";
                // for (int k = 0; k < path.size(); k++ )
		 // {
	             // std::cout << path[k] << ", ";
		 // }
		 // std::cout << "]";
            }

        }
        // std::cout<<cost_total << "\n";
        
        if (cost_total < cost_now) {
            cost_now = cost_total;
            int index = 0;
            for (int i = 0; i < num_agent; i++) {
                for (int j = 0; j < num_task; j++) {
                    if (j < num_task_assign[i]) {
                        solution[i*num_task+j] = task_index[index];
                        index++;
                    }
                    else {
                        solution[i*num_task+j] = -1;
                    }
                }
            }

        }
    }

    for (int i = 0; i < size; i++) {
        permutation_order_task(task_index, size - 1, num_agent, num_task, agent_position, targets_position, map, mapSizeX, mapSizeY, num_task_assign, solution, cost_now);
            
        if (size % 2 == 1) {
            int temp = task_index[0];
            task_index[0] = task_index[size - 1];
            task_index[size - 1] = temp;
        }
        else {
            int temp = task_index[i];
            task_index[i] = task_index[size - 1];
            task_index[size - 1] = temp;
        }  
    }
}

inline float permutation_num_task(const int num_agent, const int num_task, int agent_position[], int targets_position[], std::vector<int> map, int mapSizeX, int mapSizeY, int solution[]) {
    // int factorial = 1;
    // for(int i = 1; i <= num_task; ++i) {
        // factorial *= i;
    // }
    
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
    int num_task_assign[num_agent];
    for (int i = 0; i < num_agent; i++) {
        if (i == 0) {
            num_task_assign[i] = num_task - num_agent + 1;
        }
        else {
            num_task_assign[i] = 1;
        }
    }
    int total_case_assign_num_task = 0;
    
    std::vector<int> task_index_vec(num_task);
    for (int i = 0; i < num_task; i++) {
        task_index_vec[i] = i + 1;
    }
    int* task_index = task_index_vec.data();

    // int percent = 0;
    while (true) {
        // std::cout << case_count << "/" << expect_case << "\n";
        // move one task to tail
        while (agent_index != num_agent - 1) {
            // print_task_number(num_task_assign, num_agent);
            permutation_order_task(task_index, num_task, num_agent, num_task, agent_position, targets_position, map, mapSizeX, mapSizeY, num_task_assign, solution, cost_now);
            total_case_assign_num_task++;
            move_one_task_to_next_agent(num_task_assign, agent_index);
            agent_index++;
        }
        // for last one
        // print_task_number(num_task_assign, num_agent);
        permutation_order_task(task_index, num_task, num_agent, num_task, agent_position, targets_position, map, mapSizeX, mapSizeY, num_task_assign, solution, cost_now);
        total_case_assign_num_task++;
        // stop consition
        if (num_task_assign[num_agent - 1] == num_task - num_agent + 1) {
            break;
        }
        // move agent pointer back
        agent_index--;
        while (num_task_assign[agent_index] == 1) {
            agent_index--;
        }
        
        // // move all task back and move one forward
        move_all_task_back(num_task_assign, num_agent);
        move_one_task_to_next_agent(num_task_assign, agent_index);
        agent_index++;
    }

    // std::cout << "Total combination without ordering = " << total_case_assign_num_task << "\n";
    // std::cout << "Total cases = " << case_count << "\n";
    return cost_now;
    
}
