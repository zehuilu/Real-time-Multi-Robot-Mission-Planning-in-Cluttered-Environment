/*==============
 * Define nodes
 *==============
 */
#include <vector>
typedef struct task
{
    int task_index;
    size_t position[2];
    task *prev_task;
    task *next_task;
    task *next_in_list;

} task;

typedef struct agent
{
    int agent_index;
    size_t position[2];
    int num_task;
    agent *prev_agent;
    agent *next_agent;
    task *next_task; 
    
} agent;

typedef struct task_list
{
    size_t position[2];
    task_list *next_task;
} task_list;


/*==================
 * Define Functions
 *==================
 */
agent *initialize(int num_agent, int num_task, int agent_position[]);
task_list *initialize_tast(int num_task, int targets_position[]);
void print_task_number(agent *head);
void move_one_task_to_next_agent(agent *this_agent);
void move_all_task_back(agent *this_agent);
int* permutation_order_task(agent *head, int task_index, int size, int num_task, int targets_position[], std::vector<int> map, int mapSizeX, int mapSizeY, int solution[]);
int* permutation_num_task(agent *head, int num_agent, int num_task, int targets_position[], std::vector<int> map, int mapSizeX, int mapSizeY,  int solution[]);


// DrMaMP.py MissionPlanning()
// return case, cost
void BaseLine(int agent_position[], int targets_position[], int map[], int mapSizeX, int mapSizeY);

// call functions FindPath() to get cost
