#!/usr/bin/env python3
import asyncio
import random
import matplotlib.pyplot as plt
import pathmagic
with pathmagic.context():
    from Simulator import Simulator
    from PathPlanner import PathPlanner


def assign_targets(agents_position: list, targets_position_1d: list):
    """
    Assign targets to agents.

    Input:
        agents_position = [a0,b0, a1,b1, ...]
        targets_position_1d = [x0,y0, x1,y1, x2,y2, x3,y3, x4,y4, ...]

    Output:
        targets_position: 2D list, targets positions for multi-agent path planning,
            i-th sub-list is a list of targets for i-th agent

    Example:
        agents_position = [a0,b0, a1,b1, a2,b2]
        targets_position_1d = [x0,y0, x1,y1, x2,y2, x3,y3, x4,y4, x5,y5, x6,y6, x7,y7]
        targets_position = assign_targets(agents_position, targets_position_1d)

        ==> targets_position = [[x0,y0, x3,y3, x6,y6], [x1,y1, x4,y4, x7,y7], [x2,y2, x5,y5]]
    """
    num_agents = int(len(agents_position) / 2)
    num_targets = int(len(targets_position_1d) / 2)
    num_group = num_targets // num_agents
    num_reminder = num_targets % num_agents
    assert num_group > 0, "Each agent should have at least 1 task!"
    # 2D target list for multi-agent path planning
    targets_position = [[] for _ in range(num_agents)]
    for idx_group in range(num_group):
        for idx_agent in range(num_agents):
            idx_x = idx_group * num_agents * 2 + 2 * idx_agent
            idx_y = idx_x + 1
            targets_position[idx_agent].extend([targets_position_1d[idx_x], targets_position_1d[idx_y]])
    for idx_reminder in range(num_reminder):
        idx_x = num_group * num_agents * 2 + 2 * idx_reminder
        idx_y = idx_x + 1
        targets_position[idx_reminder].extend([targets_position_1d[idx_x], targets_position_1d[idx_y]])
    return targets_position


if __name__ == "__main__":
    # define the world
    map_width_meter = 25.0
    map_height_meter = 25.0
    map_resolution = 2
    value_non_obs = 0  # the cell is empty
    value_obs = 255  # the cell is blocked
    # create a simulator
    MySimulator = Simulator(map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)
    # number of obstacles
    num_obs = 150
    # [width, length] size of each obstacle [meter]
    size_obs = [1, 1]
    # generate random obstacles
    MySimulator.generate_random_obs(num_obs, size_obs)
    # randomly generate agents and targets
    num_agents = 3
    num_targets = 8
    agents_position, targets_position_1d = MySimulator.generate_agents_and_targets(num_agents, num_targets)
    # for path planning only, assume that tasks are already assigned
    targets_position = assign_targets(agents_position, targets_position_1d)

    # average agent velocity in cells
    agent_velocity_ave = [random.randint(4,8) for i in range(num_agents)]

    # planning and visualization frequency in Hz
    planning_frequency = 5

    # initialize a planner
    MyPlanner = PathPlanner(MySimulator)

    # run the planner online
    asyncio.run(MyPlanner.run_planner({"agents_position": agents_position,
                                       "targets_position": targets_position,
                                       "solver_mode": "PathPlanningMultiAgent",
                                       "agent_velocity_ave": agent_velocity_ave,
                                       "planning_frequency": planning_frequency}))
