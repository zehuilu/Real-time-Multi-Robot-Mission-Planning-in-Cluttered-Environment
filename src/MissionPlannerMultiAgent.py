#!/usr/bin/env python3
import asyncio
import time
import math
from random import randint, uniform
from itertools import chain
import matplotlib.pyplot as plt
import numpy as np
import pathmagic
with pathmagic.context():
    import DrMaMP
    from discrete_path_to_time_traj import discrete_path_to_time_traj
    from interpolate_traj import interpolate_traj
    from AgentFSM import AgentFSM


# when distance between A and B < this number, we say A and B have same position
DISTANCE_THRESHOLD = 1.414
# True if some obstacles are dynamic
OBS_DYNAMIC_FLAG = False


class MissionPlannerMultiAgent:
    num_agents: int  # number of agents
    planning_frequency: int  # planning and visualization frequency in Hz
    list_AgentFSM: list  # a list of AgentFSM objects
    num_cluster: int  # number of clusters for task decomposition, mission planning
    number_of_iterations: int  # number of iterations for task decomposition, mission planning

    def __init__(self, MySimulator):
        """
        Initialize a Planner Object.
        """
        self.MySimulator = MySimulator
        self.list_AgentFSM = list()

    async def run_planner(self, input_dict: dict):
        """
        Run the planner online.

        Inputs:
            input_dict: a dictionary includes agent_velocity_ave,
                planning_frequency, and positions for agents and targets
        """
        self.planning_frequency = input_dict["planning_frequency"]
        self.num_obs = input_dict["num_obs"]

        if "num_cluster" in input_dict and "number_of_iterations" in input_dict:
            # number of clusters for task decomposition
            self.num_cluster = input_dict["num_cluster"]
            # number of iterations for task decomposition
            self.number_of_iterations = input_dict["number_of_iterations"]

        # a list includes the average velocity of each agent
        agent_velocity_ave = input_dict["agent_velocity_ave"]
        # the initial agents and targets positions
        if "agents_position" in input_dict and "targets_position" in input_dict:
            agents_position = input_dict["agents_position"]
            targets_position = input_dict["targets_position"]
            self.num_agents = int(len(agents_position) / 2)
        else:
            Exception("Need agents positions and targets positions")

        time_escape = 50  # shut down the iteration after this time [sec]
        time_begin = time.time()
        time_used = 0  # initialize the global time as 0
        # create a realtime plotting figure
        ax = self.MySimulator.create_realtime_plot(realtime_flag=True, cluster_legend_flag=True, path_legend_flag=True)

        # initialize task decomposition
        path_all_agents, task_order, cluster_centers, _, _ = \
            DrMaMP.MissionPlanning(agents_position, targets_position, self.num_cluster, self.number_of_iterations,
                                   self.MySimulator.map_array.flatten().tolist(), self.MySimulator.map_width,
                                   self.MySimulator.map_height)

        # rearrange the targets, 1D to 2D list
        targets_position_2d = self.rearrange_targets(targets_position, task_order)

        # initialize Finite State Machine for each agent
        for idx_agent in range(self.num_agents):
            self.list_AgentFSM.append(AgentFSM(agentIdx=idx_agent, distanceThreshold=DISTANCE_THRESHOLD))
            # initialize the FSM with the first target
            self.list_AgentFSM[idx_agent].initFSM(targetPosition=targets_position_2d[idx_agent][0:2])

        # initialize movement direction of dynamic obstacles randomly
        if OBS_DYNAMIC_FLAG:
            self.num_obs_dynamic = 15
            self.move_angle_list = list()
            self.obs_left_top_corner_list = list()
            for idx in range(self.num_obs_dynamic):
                    self.move_angle_list.append(uniform(0.0, 360.0))
                    self.obs_left_top_corner_list.append([self.MySimulator.obs_left_top_corner[idx][0],
                                                          self.MySimulator.obs_left_top_corner[idx][1]])

        idx_iter = 0
        while(time_used < time_escape):
            t_start = time.time()

            # do the planning
            path_all_agents, targets_position_2d, cluster_centers, time_algorithm_ms = \
                self.run_solver_once(agents_position, targets_position_2d, cluster_centers)

            # update the agents positions and target
            agents_position, targets_position_2d = self.update_agents_positions(
                path_all_agents, agents_position, targets_position_2d,
                agent_velocity_ave, dt_update=1/self.planning_frequency)

            # for visualization only
            targets_plot_list = list(chain.from_iterable(targets_position_2d))

            # update the figure
            self.MySimulator.update_realtime_plot(path_all_agents, agents_position, targets_plot_list,
                                                  [], cluster_centers, [], ax, plot_cluster_flag=True)

            # plot the algorithm time
            time_str = "Computing Time [ms]: " + str(time_algorithm_ms)
            plt.text(0.25, 0.9, time_str, fontsize=14, transform=plt.gcf().transFigure)

            plt.pause(1E-6)
            time_sleep = max(0, 1/self.planning_frequency - time.time() + t_start)
            time_used = time.time() - time_begin
            print("Current Time [sec]: " + str(time_used))
            idx_iter += 1

            # if True, move obstacles and then update map array
            if OBS_DYNAMIC_FLAG:
                # if idx_iter < 40:
                # if idx_iter % 2 <= 0.5:
                # update map
                self.update_obs(mangitude=1.0)

            await asyncio.sleep(time_sleep)

        # update the figure one more time
        targets_plot_list = list(chain.from_iterable(targets_position_2d))
        self.MySimulator.update_realtime_plot([], agents_position, targets_plot_list, [], [], [], ax)
        plt.pause(5)

    def run_solver_once(self, agents_position, targets_position_2d, cluster_centers):
        """
        Run the planning solver once.
        run multi-agent path planning or mission planning based on different frequency

        Input:
            agents_position: 1D list for agents positions, [x0,y0, x1,y1, ...]
            targets_position_2d: 2D list, each sub-list is the targets positions in an sequence for each agent,
                [  [x0,y0, x1,y1, ...] , [x2,y2, x3,y3, ...] , ... ]
            cluster_centers: 1D list for centroids positions, [x0,y0, x1,y1, ...]

        """
        # 2d target list to 1d
        targets_position_1d = list(chain.from_iterable(targets_position_2d))

        # do the planning
        t0 = time.time()
        path_all_agents, task_order, cluster_centers, _, _ = \
            DrMaMP.MissionPlanningIteratively(agents_position, targets_position_1d, cluster_centers,
                                              self.num_cluster, self.number_of_iterations,
                                              self.MySimulator.map_array.flatten().tolist(),
                                              self.MySimulator.map_width, self.MySimulator.map_height)
        t1 = time.time()
        time_algorithm_ms = round((t1-t0)*1000, 2)  # milliseconds

        # rearrange the targets
        targets_position_2d = self.rearrange_targets(targets_position_1d, task_order)

        return path_all_agents, targets_position_2d, cluster_centers, time_algorithm_ms

    def rearrange_targets(self, targets_position_1d: list, task_order: list):
        """
        Rearrange the 1D targets set to a 2D targets set given the task execution sequence.

        Input:
            targets_position_1d: 1D list, [x0,y0, x1,y1, x2,y2, ...]
            task_order: 2D list, each sub-list is the targets indices in an execution sequence
        
        Output:
            targets_position_2d: 2D list, each sub-list is the targets positions in an execution sequence
                for each agent

        Example:
            targets_position_1d = [x0,y0, x1,y1, x2,y2, x3,y3]
            task_order = [[2,0], [1,3]]
            targets_position_2d = rearrange_targets(targets_position_1d, task_order)
            ==> targets_position_2d = [[x2,y2, x0,y0], [x1,y1, x3,y3]]
        """
        targets_position_2d = [[] for _ in range(self.num_agents)]
        for idx_agent in range(self.num_agents):
            for idx_task in range(len(task_order[idx_agent])):
                task_id = task_order[idx_agent][idx_task]
                targets_position_2d[idx_agent].extend(targets_position_1d[2*task_id : 2*task_id+2])
        return targets_position_2d

    def update_agents_positions(self, path_all_agents: list, agents_position: list,
                                targets_position_2d: list, agent_velocity_ave: list,
                                dt_update: float):
        """
        Update the agents positions by moving forward along the previous planning trajectory, and
        update the targets list by updating finite state machine.

        Inputs:
            path_all_agents: a 3D list, each sub-list is a path of an agent.
                For example, path_all_agents[0] = [[x0,y0, ..., x1,y1], [x1,y1, ..., x2,y2], [x2,y2, ..., x3,y3], ...]
            agents_position: a 2D list for all the agents' positions, [x0,y0, x1,y1, x2,y2, ...]
            targets_position_2d: a 2D list, each sub-list is a target set of an agent.
                For example, targets_position_2d[1] = [x0,y0, x1,y1, ...] is for the second agent (Agent-1).
        
        Returns:
            agents_position_now: a 2D list for updated agents positions, [x0,y0, x1,y1, x2,y2, ...]
            targets_position_2d: a 2D list for updated targets positions
        """
        # initialize the output
        agents_position_now = list()

        for idx_agent in range(len(path_all_agents)):
            # get the current agent and targets
            agent_position_this = agents_position[2*idx_agent : 2*idx_agent+2]

            # 2d path to 1d path [x0,y0, ..., x1,y1, x1,y1, ..., x2,y2, x2,y2, ..., x3,y3, x3,y3, ...]
            path_agent_this = list(chain.from_iterable(path_all_agents[idx_agent]))

            # # 2d path to 1d path [x0,y0, ..., x1,y1, ..., x2,y2, ..., x3,y3]
            # if len(path_all_agents[idx_agent]) > 1:
            #     path_agent_this = list(chain.from_iterable(sub[:-2] for sub in path_all_agents[idx_agent][:-1]))
            #     path_agent_this.extend(path_all_agents[idx_agent][-1])
            # elif len(path_all_agents[idx_agent]) > 0:
            #     path_agent_this = copy.deepcopy(path_all_agents[idx_agent][0])
            # else:
            #     # empty list
            #     path_agent_this = list()

            # transit states and update Finite State Machine
            _, target_finish_flag_this = self.list_AgentFSM[idx_agent].transition(
                agentPositionNow=agents_position[2*idx_agent : 2*(idx_agent+1)],
                targetPosition=targets_position_2d[idx_agent][0:2])

            # different actions based on current states
            if self.list_AgentFSM[idx_agent].StateNow.stateName == "Unassigned":
                agents_position_now.extend(agent_position_this)

            elif self.list_AgentFSM[idx_agent].StateNow.stateName == "Assigned":

                # discrete path without time information converting to time trajectory
                time_queue_vec, position_traj = discrete_path_to_time_traj(
                    np.array(path_agent_this).reshape((-1,2)).tolist(),
                    dt=0.1, velocity_ave=agent_velocity_ave[idx_agent], interp_kind='linear')
                # position_traj is a numpy array
                if len(position_traj) > 0:
                    # if there exists feasible path, move one time step (dt_update) forward
                    try:
                        posi_now = interpolate_traj(dt_update, time_queue_vec, position_traj.T, interpolate_kind='traj')
                        agents_position_now.extend(posi_now.T.round().astype(int).tolist()[0])
                    except:
                        # in this case, path_agent_this = [[x0,y0, x0,y0]]
                        agents_position_now.extend(path_agent_this[-2:])
                else:
                    # there doesn't exist feasible path, don't move
                    agents_position_now.extend(agent_position_this)

            elif self.list_AgentFSM[idx_agent].StateNow.stateName == "Completed":
                if self.list_AgentFSM[idx_agent].targetPosition:
                    agents_position_now.extend(self.list_AgentFSM[idx_agent].targetPosition)
                else:
                    agents_position_now.extend(agent_position_this)

            else:
                Exception("AgentFSM only supports 4 states: Unassigned, Assigned, Completed!")

            # update new targets positions list
            if target_finish_flag_this:
                # if there are still targets, delete the first one; else, do nothing
                if targets_position_2d[idx_agent]:
                    del targets_position_2d[idx_agent][0:2]

        return agents_position_now, targets_position_2d

    def update_obs(self, mangitude=1.0):
        """
        Update obstacles positions
        """
        # empty map
        self.MySimulator.map_array = np.array([self.MySimulator.value_non_obs] * 
                                              (self.MySimulator.map_width * self.MySimulator.map_height)).\
                                     reshape(-1, self.MySimulator.map_width)
        for idx in range(self.num_obs):
            if idx < self.num_obs_dynamic:
                px = int(round(self.obs_left_top_corner_list[idx][0] + mangitude * math.cos(self.move_angle_list[idx])))
                py = int(round(self.obs_left_top_corner_list[idx][1] + mangitude * math.sin(self.move_angle_list[idx])))

                px = max(1, px)
                px = min(self.MySimulator.map_width-self.MySimulator.size_obs_width-1, px)
                py = max(1, py)
                py = min(randint(1, self.MySimulator.map_height-self.MySimulator.size_obs_height-1), py)

                self.obs_left_top_corner_list[idx][0] = px
                self.obs_left_top_corner_list[idx][1] = py
            else:
                px = self.MySimulator.obs_left_top_corner[idx][0]
                py = self.MySimulator.obs_left_top_corner[idx][1]
            obs_mat = self.MySimulator.map_array[py : py+self.MySimulator.size_obs_height][:, px : px+self.MySimulator.size_obs_width]

            self.MySimulator.map_array[py : py+self.MySimulator.size_obs_height][:, px : px+self.MySimulator.size_obs_width] \
                = self.MySimulator.value_obs * np.ones(obs_mat.shape)
