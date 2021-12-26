#include <stdio.h>
#include <iostream>
#include "../include/node.h"

int case_count = 0;

inline agent *initialize(int num_agent, int num_task) {
    // initialize first agent
    agent *head = (agent*)malloc(sizeof(agent));
    head->agent_index = 0;
    head->num_task = num_task;
    head->prev_agent = NULL;
    head->next_agent = NULL;
    head->next_task = NULL;

    agent *this_agent = head;
    // initialize rest of agents
    for (int i = 1; i < num_agent; i++) {
        agent *temp = (agent*)malloc(sizeof(agent));
        temp->agent_index = i;
        temp->num_task = 0;
        this_agent->next_agent = temp;
        temp->prev_agent = this_agent;
        temp->next_agent = NULL;
        temp->next_task = NULL;
        this_agent = this_agent->next_agent;
    }

    task *first_task = (task*)malloc(sizeof(task));
    first_task->task_index = 0;
    first_task->prev_task = NULL;
    first_task->next_task = NULL;
    first_task->next_in_list = NULL;
    head->next_task = first_task;

    task *task_ptr = first_task;

    for (int i = 1; i < num_task; i++) {
        task *temp = (task*)malloc(sizeof(task));
        temp->task_index = i;
        task_ptr->next_task = temp;
        temp->prev_task = task_ptr;
        temp->next_task = NULL;
        temp->next_in_list = NULL;
        task_ptr = task_ptr->next_task;   
    }
    return head;
}


inline void print_task_number(agent *head) {
    agent *agent_ptr = head;
    while (agent_ptr != NULL) {
        int num_task = 0;
        task *task_ptr = agent_ptr->next_task;
        while (task_ptr != NULL) {
            num_task++;
            task_ptr = task_ptr->next_task;
        }
        
        std::cout << "(" << agent_ptr->num_task << ", " << num_task << ") ";
        agent_ptr = agent_ptr->next_agent;
    }
    std::cout << "\n";
}

inline void move_one_task_to_next_agent(agent *agent_ptr) {
    task *task_to_move = agent_ptr->next_task;
    // only one task, set agent task to NULL
    if (task_to_move->next_task == NULL) {
        agent_ptr->next_task = NULL;
    }
    // multiple tasks, move to last one
    else {
        while (task_to_move->next_task != NULL) {
            task_to_move = task_to_move->next_task;
        }
    }
    // next agent has no task
    if (agent_ptr->next_agent->next_task == NULL) {
        // disconnect from prev task
        if (task_to_move->prev_task != NULL) {
            task_to_move->prev_task->next_task = NULL;
            task_to_move->prev_task = NULL;
        }
        agent_ptr->next_agent->next_task = task_to_move;
    }
    // next agent has tasks
    else {
        task *place_to_append = agent_ptr->next_agent->next_task;
        while (place_to_append->next_task != NULL) {
            place_to_append = place_to_append->next_task;
        }
        if (task_to_move->prev_task != NULL) {
            task_to_move->prev_task->next_task = NULL;
        }
        place_to_append->next_task = task_to_move;
        task_to_move->prev_task = place_to_append;
    }
    // change task number
    agent_ptr->num_task--;
    agent_ptr->next_agent->num_task++;
}

inline void move_all_task_back(agent *agent_ptr) {
    agent *place_to_move = agent_ptr;
    while (place_to_move->prev_agent->next_task == NULL) {
        place_to_move = place_to_move->prev_agent;
    }

    if (place_to_move->next_task == NULL) {
        place_to_move->next_task = agent_ptr->next_task;
        agent_ptr->next_task = NULL;
        place_to_move->num_task = agent_ptr->num_task;
        agent_ptr->num_task = 0;     
    }
}

inline void permutation_order_task(agent *head, int task_index[], int size, int num_task) {
    if (size == 1) {
        case_count++;

        agent *agent_ptr = head;
        int index = 0;
        // assign values to each node
        while (agent_ptr != NULL) {
            // go through agents
            task *task_ptr = agent_ptr->next_task;
            while (task_ptr != NULL) {
                // go through tasks
                task_ptr->task_index = task_index[index];
                index++;
                task_ptr = task_ptr->next_task;
            }
            agent_ptr = agent_ptr->next_agent;
        }
        agent_ptr = head;
        // add code in this loop to run for each case
        while (agent_ptr != NULL) {
            // for each agent
            std::cout << "Agent" << agent_ptr->agent_index + 1 << ": ";
            task *task_ptr = agent_ptr->next_task;
            while (task_ptr != NULL) {
                // for each task
                std::cout << task_ptr->task_index << "->";
                task_ptr = task_ptr->next_task;
            }
            std::cout << "end!   ";
            agent_ptr = agent_ptr->next_agent;
        }
        std::cout << "\n";
        return;
    }

    for (int i = 0; i < size; i++) {
        permutation_order_task(head, task_index, size - 1, num_task);
    
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

inline void permutation_num_task(agent *head, int num_agent, int num_task) {
    agent *agent_ptr = head;
    agent *tail = head;
    while (tail->next_agent != NULL) {
        tail = tail->next_agent;
    }
    int total_case_assign_num_task = 0;
    int task_index[num_task] = {};
    for (int i = 0; i < num_task; i++) {
        task_index[i] = i + 1;
    }

    while (true) {
        // move one task to tail
        while (agent_ptr->next_agent != NULL) {
            print_task_number(head);
            permutation_order_task(head, task_index, num_task, num_task);
            total_case_assign_num_task++;
            move_one_task_to_next_agent(agent_ptr);
            agent_ptr = agent_ptr->next_agent;
        }
        // for last one
        print_task_number(head);
        permutation_order_task(head, task_index, num_task, num_task);
        total_case_assign_num_task++;
        // stop consition
        if (tail->num_task == num_task) {
            break;
        }
        // move agent pointer back
        agent_ptr = agent_ptr->prev_agent;
        while (agent_ptr->next_task == NULL) {
            agent_ptr = agent_ptr->prev_agent;
        }
        
        // move all task back and move one forward
        move_all_task_back(tail);
        move_one_task_to_next_agent(agent_ptr);
        agent_ptr = agent_ptr->next_agent;
    }

    std::cout << "Total combination without ordering = " << total_case_assign_num_task << "\n";
    std::cout << "Total cases = " << case_count;
    
}

void BaseLine(int agent_position[], int targets_position[], int map[], int mapSizeX, int mapSizeY) {


}

int main() {
    int num_agent = 5;
    int num_task = 5;
    agent *head = initialize(num_agent, num_task);
    permutation_num_task(head, num_agent, num_task);
    return 0;
}
