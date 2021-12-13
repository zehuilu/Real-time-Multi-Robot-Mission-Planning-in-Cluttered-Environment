#!/usr/bin/env python3
import os
import matplotlib.pyplot as plt
import pathmagic
with pathmagic.context():
    from Simulator import Simulator
    import CBBA_Path_Finding


if __name__ == "__main__":
    # a json configuration file for CBBA
    cbba_config_file_name = os.getcwd() + "/example/cbba_config.json"

    # define the world
    map_width_meter = 25.0
    map_height_meter = 25.0
    map_resolution = 2
    value_non_obs = 0  # the cell is empty
    value_obs = 255  # the cell is blocked
    # create a simulator
    Simulator = Simulator(map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)
    # number of obstacles
    num_obs = 50
    # [width, length] size of each obstacle [meter]
    size_obs = [1, 1]
    # generate random obstacles
    Simulator.generate_random_obs(num_obs, size_obs)

    # randomly generate agents and targets
    num_agents = 3
    num_targets = 12
    agent_position, targets_position = Simulator.generate_agents_and_targets(num_agents, num_targets)

    # solve the task allocation + path finding problem
    agent_position_list, task_position_list, path_all_agents, task_allocation_list, AgentList, TaskList = CBBA_Path_Finding.Solve(
        agent_position, targets_position, Simulator, cbba_config_file_name, plot_flag=True)

    # plot for a single agent
    Simulator.plot_path_multi_agent(path_all_agents, agent_position, targets_position, task_allocation_list, [], [])

    # For Simulator.plot_path_multi_agent(), each sub list of the input argument
    # task_allocation_result_many_agents is the task allocation order for each agent.
    # The first entry is the index of agent.
    for idx in range(len(task_allocation_list)):
        task_allocation_list[idx].insert(0, idx)
    # plot for many agents
    Simulator.plot_path_multi_agent(path_all_agents, agent_position, targets_position, task_allocation_list, [], [])

    plt.show()
