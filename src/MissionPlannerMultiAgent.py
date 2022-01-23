#!/usr/bin/env python3
import asyncio
import time
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


class MissionPlannerMultiAgent:
    solver_mode: str
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
            input_dict: a dictionary includes solver_mode, agent_velocity_ave,
                planning_frequency, and positions for agents and targets
        """
        self.solver_mode = input_dict["solver_mode"]
        self.planning_frequency = input_dict["planning_frequency"]

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
        end_flag = False  # initialize, False when at least one agent state is not "End"
        ax = self.MySimulator.create_realtime_plot()  # create a realtime plotting figure

        # initialize task decomposition
        path_all_agents, task_order, cluster_centers, points_idx_for_clusters, cluster_assigned_idx = \
            DrMaMP.MissionPlanning(agents_position, targets_position, self.num_cluster, self.number_of_iterations,
            self.MySimulator.map_array.flatten().tolist(), self.MySimulator.map_width, self.MySimulator.map_height)

        # rearrange the targets, 1D to 2D list
        targets_position_2d = self.rearrange_targets(targets_position, task_order)

        # initialize Finite State Machine for each agent
        for idx_agent in range(self.num_agents):
            self.list_AgentFSM.append(AgentFSM(agentIdx=idx_agent, distanceThreshold=DISTANCE_THRESHOLD))
            # initialize the FSM with a set of targets
            self.list_AgentFSM[idx_agent].initFSM(targetSetTotal=targets_position_2d[idx_agent])

        while((time_used < time_escape) and not end_flag):
            t_start = time.time()

            # update the map by MySimulator.map_array
            # convert 2D numpy array to 1D list
            # world_map = self.MySimulator.map_array.flatten().tolist()

            # update targets by Finite State Machine
            targets_position_2d, _ = self.update_targets_list(agents_position, targets_position_2d)

            # do the planning
            path_all_agents, time_algorithm_ms = self.run_solver_once(agents_position, targets_position_2d)

            # for visualization only
            targets_plot_list = list(chain.from_iterable(targets_position_2d))

            # update the figure
            self.MySimulator.update_realtime_plot(path_all_agents, agents_position, targets_plot_list, [], [], [], ax)

            # update the agents positions
            agents_position, targets_position_2d, end_flag = self.update_agents_positions(
                path_all_agents, agents_position, targets_position_2d, task_order,
                agent_velocity_ave, dt_update=1/self.planning_frequency)

            # plot the algorithm time
            time_str = "Computation Time [ms]: " + str(time_algorithm_ms)
            plt.text(0.25, 0.9, time_str, fontsize=14, transform=plt.gcf().transFigure)

            plt.pause(1E-6)
            time_sleep = max(0, 1/self.planning_frequency - time.time() + t_start)
            time_used = time.time() - time_begin
            print("Current Time [sec]: " + str(time_used))
            await asyncio.sleep(time_sleep)

        # update the figure one more time
        targets_plot_list = list(chain.from_iterable(targets_position_2d))
        self.MySimulator.update_realtime_plot([], agents_position, targets_plot_list, [], [], [], ax)
        plt.pause(5)

    def run_solver_once(self, agents_position, targets_position_2d):
        """
        Run the planning solver once.

        Input:
            agents_position: 1D list for agents positions, [x0,y0, x1,y1, ...]
            targets_position_2d: 2D list, each sub-list is the targets positions in an sequence for each agent,
                [  [x0,y0, x1,y1, ...] , [x2,y2, x3,y3, ...] , ... ]

        """
        # do the planning
        t0 = time.time()
        path_all_agents = DrMaMP.PathPlanningMultiAgent(
            agents_position, targets_position_2d,
            self.MySimulator.map_array.flatten().tolist(),
            self.MySimulator.map_width, self.MySimulator.map_height)
        t1 = time.time()
        time_algorithm_ms = round((t1-t0)*1000, 2)  # milliseconds
        # print("Solver time used [sec]:" + str(t1 - t0))

        return path_all_agents, time_algorithm_ms

    def update_targets_list(self, agents_position: list, targets_position_2d: list):
        """
        Update the targets list by updating finite state machine.

        Inputs:
            agents_position: a 1D list, [x0,y0, x1,y1, ...]
            targets_position_2d: a 2D list, each sub-list is a target set of an agent.
                For example, targets_position_2d[1] = [x0,y0, x1,y1] is for the second agent (Agent-1).

        Outputs:
            targets_position_2d_new: a 2D list, as same as targets_position_2d
            end_flag: boolean, True if all agents states are "End"
        """
        # initialize the output
        end_flag_list = list()
        targets_position_2d_new = list()

        for idx_agent in range(self.num_agents):
            # transit states
            _, targets_position_new_this = self.list_AgentFSM[idx_agent].transition(
                agentPositionNow=agents_position[2*idx_agent : 2*(idx_agent+1)],
                targetSetTotal=targets_position_2d[idx_agent])

            # if state is "End", this agent completed all assigned tasks
            if self.list_AgentFSM[idx_agent].StateNow.stateName == "End":
                end_flag_list.append(True)
            else:
                end_flag_list.append(False)

            # update new targets positions list
            targets_position_2d_new.append(targets_position_new_this)

        # True if all agents states are "End"
        end_flag = all(end_flag_list)
        return targets_position_2d_new, end_flag

    def rearrange_targets(self, targets_position_1d: list, task_allocation_result: list):
        """
        Rearrange the 1D targets set to a 2D targets set given the task execution sequence.

        Input:
            targets_position_1d: 1D list, [x0,y0, x1,y1, x2,y2, ...]
            task_allocation_result: 2D list, each sub-list is the targets indices in an execution sequence
        
        Output:
            targets_position_2d: 2D list, each sub-list is the targets positions in an execution sequence
                for each agent

        Example:
            targets_position_1d = [x0,y0, x1,y1, x2,y2, x3,y3]
            task_allocation_result = [[2,0], [1,3]]
            targets_position_2d = rearrange_targets(targets_position_1d, task_allocation_result)
            ==> targets_position_2d = [[x2,y2, x0,y0], [x1,y1, x3,y3]]
        """
        targets_position_2d = [[] for _ in range(self.num_agents)]
        for idx_agent in range(self.num_agents):
            for idx_task in range(len(task_allocation_result[idx_agent])):
                task_id = task_allocation_result[idx_agent][idx_task]
                targets_position_2d[idx_agent].extend(targets_position_1d[2*task_id : 2*task_id+2])
        return targets_position_2d

    def update_agents_positions(self, path_all_agents: list, agents_position: list,
                                targets_position: list, task_allocation_result: list,
                                agent_velocity_ave: list, dt_update: float):
        """
        Update the agents positions by moving forward along the previous planning trajectory, and
        update the targets list by updating finite state machine.

        Inputs:
            path_all_agents: a 3D list, each sub-list is a path of an agent.
                For example, path_all_agents[0] = [[x0,y0, ..., x1,y1], [x1,y1, ..., x2,y2], [x2,y2, ..., x3,y3], ...]
            targets_position: a 2D list, each sub-list is a target set of an agent.
                For example, targets_position[1] = [x0,y0, x1,y1, ...] is for the second agent (Agent-1).
        """
        # initialize the output
        agents_position_now = list()
        end_flag_list = list()
        targets_position_new = list()

        for idx_agent in range(len(path_all_agents)):
            # get the current agent and targets
            agent_position_this = agents_position[2*idx_agent : 2*idx_agent+2]
            targets_position_set_this = targets_position[idx_agent]
            task_allocation_this = task_allocation_result[idx_agent]

            # 2d path to 1d path [x0,y0, ..., x1,y1, x1,y1, ..., x2,y2, x2,y2, ..., x3,y3, x3,y3, ...]
            path_agent_this = list(chain.from_iterable(path_all_agents[idx_agent]))

            # transit states
            _, targets_position_new_this = self.list_AgentFSM[idx_agent].transition(
                agentPositionNow=agent_position_this, targetSetTotal=targets_position_set_this)

            # different actions based on current states
            if self.list_AgentFSM[idx_agent].StateNow.stateName == "Unassigned":
                agents_position_now.extend(agent_position_this)

            elif self.list_AgentFSM[idx_agent].StateNow.stateName == "End":
                # agents_position_now.extend(targets_position_set_this[-2:])
                agents_position_now.extend(agent_position_this)

            elif self.list_AgentFSM[idx_agent].StateNow.stateName == "Assigned":

                # discrete path without time information converting to time trajectory
                time_queue_vec, position_traj = discrete_path_to_time_traj(
                    np.array(path_agent_this).reshape((-1,2)).tolist(),
                    dt=0.1, velocity_ave=agent_velocity_ave[idx_agent], interp_kind='linear')
                # position_traj is a numpy array
                if len(position_traj) > 0:
                    # if there exists feasible path, move one time step (dt_update) forward
                    posi_now = interpolate_traj(dt_update, time_queue_vec, position_traj.T, interpolate_kind='traj')
                    agents_position_now.extend(posi_now.T.round().astype(int).tolist()[0])
                else:
                    # there doesn't exist feasible path, don't move
                    agents_position_now.extend(agent_position_this)

            elif self.list_AgentFSM[idx_agent].StateNow.stateName == "Completed":

                if not self.list_AgentFSM[idx_agent].targetSetOrder:
                    # if no task allocation order, when Completed, let the agent goes to the completed task's position
                    agents_position_now.extend(targets_position_set_this[2*self.list_AgentFSM[idx_agent].targetIdxNow : 
                        2*self.list_AgentFSM[idx_agent].targetIdxNow+2])
                else:
                    # if exists a task allocation order, when Completed,
                    # let the agent goes to the specific completed task's position, by the allocation order
                    targetIdx = self.list_AgentFSM[idx_agent].targetSetOrder[self.list_AgentFSM[idx_agent].targetIdxNow]
                    target_position_this = targets_position_new_this[2*targetIdx : 2*targetIdx+2]
                    agents_position_now.extend(target_position_this)

            else:
                Exception("AgentFSM only supports 4 states: Unassigned, Assigned, Completed, End!")

            # if state is "End", this agent completed all assigned tasks
            if self.list_AgentFSM[idx_agent].StateNow.stateName == "End":
                end_flag_list.append(True)
            else:
                end_flag_list.append(False)

            # update new targets positions list
            targets_position_new.append(targets_position_new_this)

        # True if all agents states are "End"
        end_flag = all(end_flag_list)
        return agents_position_now, targets_position_new, end_flag
