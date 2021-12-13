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


"""
NOTE: An improvement is that redo SolveOneAgent when "Completed"; otherwise, only run PathPlanning.
"""


# when distance between A and B < this number, we say A and B have same position
DISTANCE_THRESHOLD = 1.414


class MissionPlanner:
    solver_mode: str
    num_agents: int  # number of agents
    planning_frequency: int  # planning and visualization frequency in Hz
    list_AgentFSM: list  # a list of AgentFSM objects

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
        self.loading_planning_function()

        # a list includes the average velocity of each agent
        agent_velocity_ave = input_dict["agent_velocity_ave"]
        # the initial agents and targets positions
        if "agents_position" in input_dict and "targets_position" in input_dict:
            agents_position = input_dict["agents_position"]
            targets_position = input_dict["targets_position"]
            self.num_agents = int(len(agents_position) / 2)
        else:
            Exception("Need agents positions and targets positions")

        # initialize Finite State Machine for each agent
        for idx_agent in range(self.num_agents):
            self.list_AgentFSM.append(AgentFSM(agentIdx=idx_agent, distanceThreshold=DISTANCE_THRESHOLD))
            # initialize the FSM with a set of targets
            self.list_AgentFSM[idx_agent].initFSM(targetSetTotal=targets_position[2*idx_agent : 2*idx_agent+2])

        time_escape = 50  # shut down the iteration after this time [sec]
        time_begin = time.time()
        time_used = 0  # initialize the global time as 0
        end_flag_list = False  # initialize, False when at least one agent state is not "End"
        ax = self.MySimulator.create_realtime_plot()  # create a realtime plotting figure

        while((time_used < time_escape) and not end_flag_list):
            t_start = time.time()

            # update the map by MySimulator.map_array
            # convert 2D numpy array to 1D list
            # world_map = self.MySimulator.map_array.flatten().tolist()

            if targets_position:
                t0 = time.time()
                # do the planning
                path_all_agents, task_allocation_result = self.PlanningFunction({"agents_position": agents_position, "targets_position": targets_position})
                t1 = time.time()
                time_algorithm_ms = round((t1-t0)*1000, 2)  # milliseconds
                # print("Time used [sec]:" + str(t1 - t0))
            else:
                path_all_agents = [[]]
                task_allocation_result = [[]]
                time_algorithm_ms = None

            # update the figure
            self.MySimulator.update_realtime_plot(path_all_agents, agents_position, targets_position, task_allocation_result, [], [], ax)

            # update the agents positions
            agents_position, targets_position, end_flag_list = self.update_agents_positions(
                path_all_agents, agents_position, [targets_position], task_allocation_result,
                agent_velocity_ave, dt_update=1/self.planning_frequency)

            # 2d to 1d for calling planning function at next iteration
            targets_position = list(chain.from_iterable(targets_position))

            # plot the algorithm time
            time_str = "Computation Time [ms]: " + str(time_algorithm_ms)
            plt.text(0.25, 0.9, time_str, fontsize=14, transform=plt.gcf().transFigure)

            plt.pause(1E-6)
            time_sleep = max(0, 1/self.planning_frequency - time.time() + t_start)
            time_used = time.time() - time_begin
            print("Current Time [sec]: " + str(time_used))
            await asyncio.sleep(time_sleep)

        # update the figure one more time
        self.MySimulator.update_realtime_plot([], agents_position, targets_position, task_allocation_result, [], [], ax)
        plt.pause(5)

    def loading_planning_function(self):
        """
        Loading a planning function by self.solver_mode.
        """
        self.PlanningFunction = {
            "SolveOneAgent": lambda input_dict: self.SolveOneAgentOuter(input_dict)
        }[self.solver_mode]

    def SolveOneAgentOuter(self, input_dict: dict):
        """
        The outputs for SolveOneAgent are 2d lists. Need to convert them as 3d lists.
        """
        PlanningFunction = getattr(DrMaMP, self.solver_mode)
        path_all_agents, task_allocation_result = PlanningFunction(
            input_dict["agents_position"], input_dict["targets_position"],
            self.MySimulator.map_array.flatten().tolist(),
            self.MySimulator.map_width, self.MySimulator.map_height)
        return [path_all_agents], [task_allocation_result]

    def update_agents_positions(self, path_all_agents: list, agents_position: list,
                                targets_position: list, task_allocation_result: list,
                                agent_velocity_ave: list, dt_update: float):
        """
        Update the agents positions by moving forward along the previous planning trajectory.

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
            # get current agent and targets
            agent_position_this = agents_position[2*idx_agent : 2*idx_agent+2]
            targets_position_set_this = targets_position[idx_agent]
            task_allocation_this = task_allocation_result[idx_agent]

            # 2d path to 1d path [x0,y0, ..., x1,y1, x1,y1, ..., x2,y2, x2,y2, ..., x3,y3, x3,y3, ...]
            path_agent_this = list(chain.from_iterable(path_all_agents[idx_agent]))

            # transit states
            _, targets_position_new_this = self.list_AgentFSM[idx_agent].transition(
                agentPositionNow=agent_position_this, targetSetTotal=targets_position_set_this, targetSetOrder=task_allocation_this)

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
        end_flag_list = all(end_flag_list)
        return agents_position_now, targets_position_new, end_flag_list
