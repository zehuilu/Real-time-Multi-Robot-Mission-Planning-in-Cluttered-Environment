#!/usr/bin/env python3
import json
import matplotlib.pyplot as plt
import pathmagic
with pathmagic.context():
    from Simulator import Simulator
    import DrMaMP
    from CBBA import CBBA
    from Agent import Agent
    from Task import Task
    from WorldInfo import WorldInfo


def Solve(agent_position: list, targets_position: list, Simulator: Simulator,
          cbba_config_file_name: str, plot_flag: bool):
    """
    Solve the task allocation and path finding problem for multiple agents with CBBA as task allocation solver,
    and DrMaMP as the path finding solver.

    Input:
        agent_position: a 1D list for agent position in cells, [x0, y0]
        targets_position: a 1D list for targets positions in cells, [x0,y0, x1,y2, x2,y2, ...]
        Simulator: an instance of class Simulator
        cbba_config_data: a JSON configuration data
            cbba_config_file_name = "cbba_config.json"
            cbba_json_file = open(cbba_config_file_name)
            cbba_config_data = json.load(cbba_json_file)
        plot_flag: bool, True to plot results in cell map.

    Output:
        agent_position_list: a 2D list of agent positions in cells,
            i-th sub-list is the position of agent-i, [[x0,y0], [x1,y1], [x2,y2], ...]
        task_position_list: a 2D list of task positions in cells, 
            i-th sub-list is all the tasks positions of agent-i, [[x0,y0, x3,y3, x5,y5], [x2,y2, x1,y1, x4,y4], ...]
        path_all_agents: a 3D list of paths for every agent, i-th sub-list is a 2D list for paths of agent-i,
            path_all_agents[i] = [[x0,y0, ..., x2,y2], [x2,y2, ..., x5,y5], [x5,y5, ..., x7,y7], ...] means
            agent-i visits [x2,y2] from [x0,y0], then visits [x5,y5] from [x2,y2], then visits [x7,y7] from [x5,y5]
    """

    # read the configuration from the json file
    cbba_json_file = open(cbba_config_file_name)
    cbba_config_data = json.load(cbba_json_file)

    # CBBA + path finding
    # create a world for CBBA, each list is [min, max] coordinates for x,y,z axis in cells, not in meter
    WorldInfo_CBBA = WorldInfo([0, Simulator.map_width], [0, Simulator.map_height], [0, 0])
    # create a list of Agent(s) and Task(s) in cells
    AgentList, TaskList = load_agents_and_tasks(agent_position, targets_position, Simulator, cbba_config_data)

    # create a CBBA solver
    CBBA_solver = CBBA(cbba_config_data)
    # solve
    max_depth = int(len(targets_position)/2)
    # max_depth = int(len(targets_position))
    task_allocation_list, _ = CBBA_solver.solve(AgentList, TaskList, WorldInfo_CBBA, max_depth, time_window_flag=False)

    # Path Finding with CBBA
    agent_position_list, task_position_list, path_all_agents = FindPath(
        task_allocation_list, AgentList, TaskList, Simulator, cell_flag=True)

    if plot_flag:
        plot_in_cell(agent_position_list, task_position_list, AgentList, TaskList, Simulator, cell_flag=True)

    return agent_position_list, task_position_list, path_all_agents, task_allocation_list, AgentList, TaskList


def FindPath(CBBA_path: list, AgentList: list, TaskList: list, Simulator: Simulator, cell_flag: bool):
    """
    Find the collison-free path given the CBBA task allocation result.

    Input:
        CBBA_path: a 2D list returned by CBBA.solve() as CBBA task allocation result,
            i-th sub-list is a set of task indices for Agent-i to explore,
            [[3,5,1,0,2,4], [...], ...] means Agent-0 needs to visit Task 3 -> 5 -> 1-> 0 -> 2 -> 4
        AgentList: a 1D list of dataclass Agent() for agents
        TaskList: a 1D list of dataclass Task() for tasks
        Simulator: an instance of class Simulator()
        cell_flag: bool, True if the positions of agents and tasks are in cells, not in meters;
            False if the positions in meters.

    Output:
        agent_position_list: a 2D list of agent positions in cells,
            i-th sub-list is the position of agent-i, [[x0,y0], [x1,y1], [x2,y2], ...]
        task_position_list: a 2D list of task positions in cells, 
            i-th sub-list is all the tasks positions of agent-i, [[x0,y0, x3,y3, x5,y5], [x2,y2, x1,y1, x4,y4], ...]
        path_all_agents: a 3D list of paths for every agent, i-th sub-list is a 2D list for paths of agent-i,
            path_all_agents[i] = [[x0,y0, ..., x2,y2], [x2,y2, ..., x5,y5], [x5,y5, ..., x7,y7], ...] means
            agent-i visits [x2,y2] from [x0,y0], then visits [x5,y5] from [x2,y2], then visits [x7,y7] from [x5,y5]
    """

    # number of agents
    num_agents = len(AgentList)
    # number of tasks
    num_tasks = len(TaskList)

    # 2d list
    agent_position_list = []
    # 2d list
    task_position_list = [[] for _ in range(num_agents)]
    # 3d list
    path_all_agents = []

    if not cell_flag:
        # if the positions of AgentList and TaskList in meters, not in cells
        for idx_agent in range(num_agents):
            # the current agent position [x, y] in cells
            position_agent = Simulator.position_to_map_index([AgentList[idx_agent].x, AgentList[idx_agent].y])
            agent_position_list.append(position_agent)

            # the current task path returned by CBBA
            task_path_current = CBBA_path[idx_agent]

            if len(task_path_current) > 0:
                for j in range(len(task_path_current)):
                    goal = Simulator.position_to_map_index(
                        [TaskList[task_path_current[j]].x, TaskList[task_path_current[j]].y])
                    task_position_list[idx_agent].extend(goal)
            else:
                print("Agent " + str(idx_agent) + " is empty. Pass.")
                # pass

            # find the path
            path_many, distances_many = DrMaMP.FindPathOneByOne(position_agent, task_position_list[idx_agent],
                Simulator.map_array.flatten().tolist(), Simulator.map_width, Simulator.map_height)
            path_all_agents.append(path_many)

    else:
        # if the positions of AgentList and TaskList in cells, not in meters
        for idx_agent in range(0, num_agents):
            # the current task path returned by CBBA
            task_path_current = CBBA_path[idx_agent]

            # the current agent position
            agent_position_list.append([AgentList[idx_agent].x, AgentList[idx_agent].y])

            if len(task_path_current) > 0:
                for j in range(len(task_path_current)):
                    task_position_list[idx_agent].extend([TaskList[task_path_current[j]].x, TaskList[task_path_current[j]].y])
            else:
                # print("Agent " + str(idx_agent) + " is empty. Pass.")
                pass

            # find the path
            path_many, distances_many = DrMaMP.FindPathOneByOne(
                agent_position_list[idx_agent], task_position_list[idx_agent],
                Simulator.map_array.flatten().tolist(), Simulator.map_width, Simulator.map_height)
            path_all_agents.append(path_many)

    return agent_position_list, task_position_list, path_all_agents


def load_agents_and_tasks(agent_position: list, targets_position: list, Simulator: Simulator, cbba_config_data):
    """
    Load agents and tasks (homogeneous) as CBBA data type given agent positions and target positions.

    Input:
        agent_position: a 1D list for agent position, [x0, y0]
        targets_position: a 1D list for targets positions, [x0,y0, x1,y2, x2,y2, ...]
        Simulator: an instance of class Simulator
        cbba_config_data: a JSON configuration data
            cbba_config_file_name = "cbba_config.json"
            cbba_json_file = open(cbba_config_file_name)
            cbba_config_data = json.load(cbba_json_file)

    Output:
        AgentList: a 1D list of dataclass Agent()
        TaskList: a 1D list of dataclass Task()
    """

    # Create default agents
    agent_default = Agent()
    # agent type
    agent_default.agent_type = cbba_config_data["AGENT_TYPES"].index("quad")
    # agent cruise velocity (m/s) ==> cell/s
    agent_default.nom_velocity = float(cbba_config_data["QUAD_DEFAULT"]["NOM_VELOCITY"] * Simulator.resolution)

    # Create default tasks
    task_default = Task()
    # task type
    task_default.task_type = cbba_config_data["TASK_TYPES"].index("track")
    # task reward
    task_default.task_value = float(cbba_config_data["TRACK_DEFAULT"]["TASK_VALUE"])
    # task start time (sec)
    task_default.start_time = 0.0
    # task expiry time (sec)
    task_default.end_time = 0.0
    # task default duration (sec)
    task_default.duration = 0.0

    # create empty list, each element is a dataclass Agent() or Task()
    AgentList = []
    TaskList = []

    num_agents = int(len(agent_position)/2)
    num_tasks = int(len(targets_position)/2)

    # load agents
    for idx_agent in range(num_agents):
        # create a new instance of dataclass agent_default
        AgentList.append(Agent(**agent_default.__dict__))
        AgentList[idx_agent].agent_id = idx_agent
        AgentList[idx_agent].x = agent_position[2*idx_agent]
        AgentList[idx_agent].y = agent_position[2*idx_agent+1]
        AgentList[idx_agent].z = 0

    # load tasks
    for idx_task in range(num_tasks):
        # create a new instance of dataclass task_default
        TaskList.append(Task(**task_default.__dict__))
        TaskList[idx_task].task_id = idx_task
        TaskList[idx_task].x = targets_position[2*idx_task]
        TaskList[idx_task].y = targets_position[2*idx_task+1]
        TaskList[idx_task].z = 0
        # no time window
        TaskList[idx_task].start_time = 0.0
        TaskList[idx_task].duration = 0.0
        TaskList[idx_task].end_time = 0.0

    # for n in range(num_tasks):
    #     print("Task " + str(n))
    #     print(str(TaskList[n].x)+", "+str(TaskList[n].y)+", "+str(TaskList[n].z))
    #     print(str(TaskList[n].start_time)+" - "+str(TaskList[n].end_time))
    # for m in range(num_agents):
    #     print("Agent " + str(m))
    #     print(str(AgentList[m].x)+", "+str(AgentList[m].y)+", "+str(AgentList[m].z))

    return AgentList, TaskList


def plot_in_cell(agent_position_list: list, task_position_list: list, AgentList: list, TaskList: list,
                 Simulator: Simulator, cell_flag: bool):
    """
    Plots CBBA outputs in cell map when there is no time window for tasks.

    Input:
        agent_position_list: a 2D list of agent positions in cells,
            i-th sub-list is the position of agent-i, [[x0,y0], [x1,y1], [x2,y2], ...]
        task_position_list: a 2D list of task positions in cells, 
            i-th sub-list is all the tasks positions of agent-i, [[x0,y0, x3,y3, x5,y5], [x2,y2, x1,y1, x4,y4], ...]
        AgentList: a 1D list of dataclass Agent()
        TaskList: a 1D list of dataclass Task()
        Simulator: an instance of class Simulator

    Output:
        void
    """

    assert cell_flag, "Function plot_in_cell() only plots in cell map, cell_flag should be True."

    # number of agents
    num_agents = len(AgentList)
    # number of tasks
    num_tasks = len(TaskList)

    # 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111)
    # offset to plot text in 2D space
    offset = (Simulator.map_width-0) / 100

    # plot tasks
    for m in range(0, num_tasks):
        # track task is red
        if TaskList[m].task_type == 0:
            color_str = 'red'
        # rescue task is blue
        else:
            color_str = 'blue'

        task_position_in_cell = [TaskList[m].x, TaskList[m].y]
        ax.scatter(task_position_in_cell[0], task_position_in_cell[1], marker='x', color=color_str)
        ax.text(task_position_in_cell[0]+offset, task_position_in_cell[1]+offset, "T"+str(m))

    # plot agents
    for n in range(0, num_agents):
        # quad agent is red
        if AgentList[n].agent_type == 0:
            color_str = 'red'
        # car agent is blue
        else:
            color_str = 'blue'
        ax.scatter(agent_position_list[n][0], agent_position_list[n][1], marker='o', color=color_str)
        ax.text(agent_position_list[n][0]+offset, agent_position_list[n][1]+offset, "A"+str(n))

        # if the path is not empty
        if task_position_list[n]:
            ax.plot([agent_position_list[n][0], task_position_list[n][0]],
                [agent_position_list[n][1], task_position_list[n][1]], linewidth=2, color=color_str)

            for i in range(2, len(task_position_list[n]), 2):
                ax.plot([task_position_list[n][i], task_position_list[n][i-2]],
                    [task_position_list[n][i+1], task_position_list[n][i-1]], linewidth=2, color=color_str)

    plt.title('Agent Paths in cell map')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    # set legends
    colors = ["red", "blue", "red", "blue"]
    marker_list = ["o", "o", "x", "x"]
    labels = ["Agent type 1", "Agent type 2", "Task type 1", "Task type 2"]
    def f(marker_type, color_type): return plt.plot([], [], marker=marker_type, color=color_type, ls="none")[0]
    handles = [f(marker_list[i], colors[i]) for i in range(len(labels))]
    plt.legend(handles, labels, bbox_to_anchor=(1, 1), loc='upper left', framealpha=1)

    ax.set_aspect('equal')
    ax.set_xlim([0, Simulator.map_width])
    ax.set_ylim([0, Simulator.map_height])
    plt.show(block=False)
