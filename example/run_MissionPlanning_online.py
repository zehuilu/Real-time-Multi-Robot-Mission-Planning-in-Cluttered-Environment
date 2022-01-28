#!/usr/bin/env python3
import asyncio
import random
import matplotlib.pyplot as plt
import pathmagic
with pathmagic.context():
    from Simulator import Simulator
    from MissionPlannerMultiAgent import MissionPlannerMultiAgent


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
    num_obs = 200
    # [width, length] size of each obstacle [meter]
    size_obs = [1, 1]
    # generate random obstacles
    MySimulator.generate_random_obs(num_obs, size_obs)
    # randomly generate agents and targets
    num_agents = 5
    num_targets = 25
    agents_position, targets_position = MySimulator.generate_agents_and_targets(num_agents, num_targets)

    # number of clusters for task decomposition
    num_cluster = num_agents
    # number of iterations for task decomposition
    number_of_iterations = 300

    # average agent velocity in cells
    agent_velocity_ave = [random.randint(4,8) for i in range(num_agents)]

    # planning and visualization frequency in Hz
    planning_frequency = 5

    # initialize a planner
    MyPlanner = MissionPlannerMultiAgent(MySimulator)

    # run the planner online
    asyncio.run(MyPlanner.run_planner({"agents_position": agents_position,
                                       "targets_position": targets_position,
                                       "num_cluster": num_cluster,
                                       "number_of_iterations": number_of_iterations,
                                       "agent_velocity_ave": agent_velocity_ave,
                                       "planning_frequency": planning_frequency}))
