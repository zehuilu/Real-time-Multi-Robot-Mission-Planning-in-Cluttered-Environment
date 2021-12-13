#!/usr/bin/env python3
import os
import asyncio
import time
import json
from itertools import chain
import matplotlib.pyplot as plt
import numpy as np
import pathmagic
with pathmagic.context(EXTERNAL_FLAG=True):
    import DrMaMP
    import csv_helper
    from UdpProtocol import UdpProtocol
    from SimulatorAimsLab import SimulatorAimsLab
    from discrete_path_to_time_traj import discrete_path_to_time_traj
    from AgentFSMExp import AgentFSMExp


# when distance between A and B < this number, we say A and B have same position
DISTANCE_THRESHOLD = 0.2
BUFFER_BOUNDARY = 0.15
CASE_NUM = 6


class PlannerMocapMultiAgent:
    num_agents: int  # number of agents
    planning_frequency: int  # planning and visualization frequency in Hz
    list_AgentFSMExp: list  # a list of AgentFSMExp objects
    num_cluster: int  # number of clusters for task decomposition, mission planning
    number_of_iterations: int  # number of iterations for task decomposition, mission planning
    config_data_list: list  # a list of dictionaries for agents configurations
    height_fly: float

    def __init__(self):
        """
        Initialize a PlannerMocap Object. This is used with Motion Capture System.
        """
        self.num_agents = 2  # number of agents

        # create a simulator
        self.MySimulator = SimulatorAimsLab(map_resolution=20, buffer_bdy=BUFFER_BOUNDARY)

        # load multiple agents' configuration as a list of dictionaries
        self.config_data_list = list()
        for idx in range(self.num_agents):
            file_name = os.getcwd() + "/experiment/config_aimslab_ex_" + str(idx+1) + ".json"
            self.config_data_list.append(json.load(open(file_name)))

        self.mocap_type = "QUALISYS"
        # the address for publishing agents positions from mocap
        self.state_address_list = list()
        for idx in range(self.num_agents):
            self.state_address_list.append(
                (self.config_data_list[idx][self.mocap_type]["IP_STATES_ESTIMATION"],
                int(self.config_data_list[idx][self.mocap_type]["PORT_POSITION_PLANNER"])))

        # the address for publishing obstalce positions from mocap
        self.server_address_obs = (self.config_data_list[0][self.mocap_type]["IP_OBS_POSITION"],
                                   int(self.config_data_list[0][self.mocap_type]["PORT_OBS_POSITION"]))

    async def run_planner(self):
        """
        Run the planner online with Motion Capture System.
        """
        self.list_AgentFSMExp = list()  # a list for Agent Finite State Machine
        self.planning_frequency = 5  # planner frequency in Hz
        # number of clusters for task decomposition
        self.num_cluster = self.num_agents
        # number of iterations for task decomposition
        self.number_of_iterations = 500
        # each element is each agent's average velocity (m/s)
        velocity_ave_list = [0.20, 0.20]

        # remove all the existing files in the trajectory directory
        for idx in range(self.num_agents):
            directory_delete = os.path.expanduser("~") + "/Mambo-Tracking-Interface" + \
                               self.config_data_list[idx]["DIRECTORY_TRAJ"] + "*"
            csv_helper.remove_traj_ref_lib(directory_delete)

        time_escape = 100  # shut down the iteration after this time [sec]

        # create a customized UDP protocol for subscribing states from mocap
        loop_states_01 = asyncio.get_running_loop()
        transport_states_01, protocol_states_01 = await loop_states_01.create_datagram_endpoint(
            UdpProtocol, local_addr=self.state_address_list[0], remote_addr=None)

        loop_states_02 = asyncio.get_running_loop()
        transport_states_02, protocol_states_02 = await loop_states_02.create_datagram_endpoint(
            UdpProtocol, local_addr=self.state_address_list[1], remote_addr=None)

        self.transport_states_list = [transport_states_01, transport_states_02]
        self.protocol_states_list = [protocol_states_01, protocol_states_02]

        # create a customized UDP protocol for subscribing obstacles positions from mocap
        loop_obs = asyncio.get_running_loop()
        self.transport_obs, self.protocol_obs = await loop_obs.create_datagram_endpoint(
            UdpProtocol, local_addr=self.server_address_obs, remote_addr=None)

        # get the agent home position
        self.agents_home_qualisys_list = await self.update_states_mocap()

        self.height_fly = 0.8  # a constant fly height in meter
        # generate a scenario
        # generate targets manually
        targets_position_qualisys = self.generate_targets_manually(CASE_NUM)
        # generate obstacles manually
        self.MySimulator.generate_obs_manually(CASE_NUM)

        # transform qualisys coordinates (meter) to map array (index)
        agent_position_index = self.MySimulator.qualisys_to_map_index_all(self.agents_home_qualisys_list)
        targets_position_index = self.MySimulator.qualisys_to_map_index_all(targets_position_qualisys)

        # initialize task decomposition
        path_all_agents, task_allocation_result, cluster_centers, points_idx_for_clusters, cluster_assigned_idx = \
            DrMaMP.MissionPlanning(agent_position_index, targets_position_index, self.num_cluster, self.number_of_iterations,
            self.MySimulator.map_array.flatten().tolist(), self.MySimulator.map_width, self.MySimulator.map_height)

        # rearrange the targets by task allocation result, 2D to 3D list
        targets_position_3d = self.rearrange_targets(targets_position_qualisys, task_allocation_result)

        # initialize Finite State Machine for each agent
        for idx_agent in range(self.num_agents):
            self.list_AgentFSMExp.append(AgentFSMExp(agentIdx=idx_agent, distanceThreshold=DISTANCE_THRESHOLD))
            # initialize the FSM with a set of targets
            self.list_AgentFSMExp[idx_agent].initFSM(targetSetTotal=targets_position_3d[idx_agent])

        self.ax = self.MySimulator.create_realtime_plot()  # create a realtime plotting figure
        time_begin = time.time()
        time_used = 0  # initialize the global time as 0

        while(time_used < time_escape):
            t_start = time.time()

            # update the agent position
            agents_position_list = await self.update_states_mocap()

            # update targets by Finite State Machine
            targets_position_3d, _ = self.update_targets_list(agents_position_list, targets_position_3d)

            # do the planning
            position_traj_list, time_algorithm_ms = self.run_solver_once(agents_position_list, targets_position_3d, velocity_ave_list, time_begin)

            # for visualization only
            targets_plot_list = self.targets_transform_ordered(targets_position_3d)

            # update the figure
            self.MySimulator.update_realtime_plot(position_traj_list, agents_position_list, targets_plot_list, task_allocation_result, self.ax)

            # plot the algorithm time
            time_str = "Computation Time [ms]: " + str(time_algorithm_ms)
            plt.text(0.25, 0.9, time_str, fontsize=14, transform=plt.gcf().transFigure)

            plt.pause(1E-9)
            time_sleep = max(0, 1/self.planning_frequency - time.time() + t_start)
            time_used = time.time() - time_begin
            print("Current Time [sec]: " + str(time_used))
            await asyncio.sleep(time_sleep)

    def run_solver_once(self, agents_position_2d, targets_position_3d, velocity_ave_list, time_begin):
        """
        Run the planning solver once.

        Input:
            agents_position_2d: 2D list for agents positions (in Qualisys coordinates), [[x0,y0,z0], [x1,y1,z1], ...]
            targets_position_3d: 3D list, each sub-list is the targets positions (in Qualisys coordinates) in an execution
                sequence for each agent, [  [[x0,y0,z0], [x1,y1,z1], ...] , [[x2,y2,z2], [x3,y3,z3], ...] , ... ]
            velocity_ave_list: 1D list, each element is each agent's average velocity (m/s), [0.25, 0.3, ...]
            time_begin: the time stamp where the whole planner begins

        """
        # 3D targets (in Qualisys) to 2D targets (in map index) with order inherited
        targets_position_index_2d = self.targets_transform_index_ordered(targets_position_3d)

        # transform qualisys coordinates (meter) to map index
        agents_position_index = self.MySimulator.qualisys_to_map_index_all(agents_position_2d)

        # do the planning
        t0 = time.time()
        path_all_agents_index = DrMaMP.PathPlanningMultiAgent(
            agents_position_index, targets_position_index_2d,
            self.MySimulator.map_array.flatten().tolist(),
            self.MySimulator.map_width, self.MySimulator.map_height)
        t1 = time.time()
        time_algorithm_ms = round((t1-t0)*1000, 2)  # milliseconds
        # print("Solver time used [sec]:" + str(t1 - t0))

        # transform map array (index) to qualisys coordinates (meter)
        path_qualisys_list = list()
        for idx in range(len(path_all_agents_index)):
            path_qualisys_list.append(self.MySimulator.path_index_to_qualisys(path_all_agents_index[idx], self.height_fly))

        # generate position and velocity trajectories (as a motion planner)
        # t0 = time.time()
        dt = 0.1
        # generate trajectories
        time_now = time.time()

        position_traj_list = list()
        for idx in range(len(path_qualisys_list)):
            if path_qualisys_list[idx]:
                time_queue_vec, position_traj, velocity_traj = discrete_path_to_time_traj(
                    path_qualisys_list[idx], dt, velocity_ave_list[idx], interp_kind='linear',
                    velocity_flag=True, ini_velocity_zero_flag=False)

                # Mambo Tracking Controller uses the accumulated time, need to change the time trajectory here too
                time_queue_vec = time_queue_vec + max(time_now-time_begin-1.0, 0)

                # output trajectories as a CSV file
                array_csv = np.vstack((time_queue_vec, np.array(position_traj).T, np.array(velocity_traj).T))
                time_name = time.strftime("%Y%m%d%H%M%S")
                filename_csv = os.path.expanduser("~") + "/Mambo-Tracking-Interface" + self.config_data_list[idx]["DIRECTORY_TRAJ"] + time_name + ".csv"
                np.savetxt(filename_csv, array_csv, delimiter=",")
                position_traj_list.append(position_traj)
            else:
                position_traj_list.append(list())


                # time_queue_vec, position_traj, velocity_traj = discrete_path_to_time_traj(
                #     agents_position_2d, dt, velocity_ave_list[idx], interp_kind='linear',
                #     velocity_flag=True, ini_velocity_zero_flag=False)

                # # Mambo Tracking Controller uses the accumulated time, need to change the time trajectory here too
                # time_queue_vec = time_queue_vec + max(time_now-time_begin-1.0, 0)

                # # output trajectories as a CSV file
                # array_csv = np.vstack((time_queue_vec, position_traj.T, velocity_traj.T))
                # time_name = time.strftime("%Y%m%d%H%M%S")
                # filename_csv = os.path.expanduser("~") + "/Mambo-Tracking-Interface" + self.config_data_list[idx]["DIRECTORY_TRAJ"] + time_name + ".csv"
                # np.savetxt(filename_csv, array_csv, delimiter=",")
                # position_traj_list.append(position_traj)

        return position_traj_list, time_algorithm_ms

    def update_targets_list(self, agents_position: list, targets_position_3d: list):
        """
        Update the targets list by updating finite state machine.

        Inputs:
            agents_position: a 2D list, each sub-list is each agent's positions (in Qualisys), [[x0,y0,z0], [x1,y1,z1], ...]
            targets_position_3d: a 3D list, each sub-list is a target set of an agent.
                For example, targets_position[1] = [[x0,y0,z0], [x1,y1,z1]] is for the second agent (Agent-1).

        Outputs:
            targets_position_3d_new: a 3D list, as same as targets_position_3d
            end_flag: boolean, True if all agents states are "End"
        """
        # initialize the output
        end_flag_list = list()
        targets_position_3d_new = list()

        for idx_agent in range(self.num_agents):
            # transit states
            _, targets_position_new_this = self.list_AgentFSMExp[idx_agent].transition(
                agentPositionNow=agents_position[idx_agent], targetSetTotal=targets_position_3d[idx_agent],
                homePosition=self.agents_home_qualisys_list[idx_agent])

            # if state is "End", this agent completed all assigned tasks
            if self.list_AgentFSMExp[idx_agent].StateNow.stateName == "End":
                end_flag_list.append(True)
            else:
                end_flag_list.append(False)

            # update new targets positions list
            targets_position_3d_new.append(targets_position_new_this)

        # True if all agents states are "End"
        end_flag = all(end_flag_list)
        return targets_position_3d_new, end_flag

    def rearrange_targets(self, targets_position_input: list, task_allocation_result: list):
        """
        Rearrange the 2D targets set (in Qualisys coordinates) to a 3D targets set (in Qualisys coordinates)
        given the task execution sequence.

        Input:
            targets_position_input: 2D list, [[x0,y0,z0], [x1,y1,z1], [x2,y2,z2], ...]
            task_allocation_result: 2D list, each sub-list is the targets indices in an execution sequence
        
        Output:
            targets_position_output: 3D list, each sub-list is the targets positions in an execution sequence
                for each agent

        Example:
            targets_position_input = [[x0,y0,z0], [x1,y1,z1], [x2,y2,z2], [x3,y3,z3]]
            task_allocation_result = [[2,0], [1,3]]
            targets_position_output = self.rearrange_targets(targets_position_input, task_allocation_result)
            ==> targets_position_output = [  [[x2,y2,z2], [x0,y0,z0]] , [[x1,y1,z1], [x3,y3,z3]]  ]
        """
        targets_position_output = [[] for _ in range(self.num_agents)]
        for idx_agent in range(self.num_agents):
            for idx_task in range(len(task_allocation_result[idx_agent])):
                task_id = task_allocation_result[idx_agent][idx_task]
                targets_position_output[idx_agent].append(targets_position_input[task_id])
        return targets_position_output

    def targets_transform_index_ordered(self, targets_position_3d: list):
        """
        Transform a 3D targets positions (in Qualisys coordinates) list to a 2D targets positions (in map index) list,
        where the order of original targets list is inherited. Each sub-list is the targets list for each agent.

        Input:
            targets_position_3d: 3D list, each sub-list is the targets positions (in Qualisys coordinates) in an execution
                sequence for each agent, [  [[x0,y0,z0], [x1,y1,z1], ...] , [[x2,y2,z2], [x3,y3,z3], ...] , ... ]

        Output:
            targets_position_index_2d: 2D list, each sub-list is the targets positions (in Qualisys coordinates) in an execution
                sequence for each agent, [[x0,y0, x1,y1, ...], [x2,y2, x3,y3, ...], ...]

        Example:
            targets_position_3d = [  [[x2,y2,z2], [x0,y0,z0]] , [[x1,y1,z1], [x3,y3,z3]]  ]
            targets_position_index_2d = self.targets_transform_index_ordered(targets_position_3d)
            ==> targets_position_index_2d = [[x2,y2, x0,y0], [x1,y1, x3,y3]]
            NOTE: {xi,yi} in targets_position_index_2d doesn't equal to {xi,yi} in targets_position_3d
        """
        targets_position_index_2d = list()
        for idx_agent in range(self.num_agents):
            targets_position_index_2d.append(self.MySimulator.qualisys_to_map_index_all(targets_position_3d[idx_agent]))
        return targets_position_index_2d

    def targets_transform_ordered(self, targets_position_3d: list):
        """targets_transform_ordered
        Transform a 3D targets positions (in Qualisys coordinates) list to a 2D targets positions (in Qualisys coordinates) list.
        Each sub-list is the position for each target. The order of output list does not matter. This is only for visualization.

        Input:
            targets_position_3d: 3D list, each sub-list is the targets positions (in Qualisys coordinates) in an execution
                sequence for each agent, [  [[x0,y0,z0], [x1,y1,z1], ...] , [[x2,y2,z2], [x3,y3,z3], ...] , ... ]

        Output:
            targets_position_2d: 2D list, each sub-list is the target position (in Qualisys coordinates) for each target,
                [[x0,y0,z0], [x1,y1,z1], [x2,y2,z2], [x3,y3,z3], ...]

        Example:
            targets_position_3d = [  [[x2,y2,z2], [x0,y0,z0], [x4,y4,z4]] , [[x1,y1,z1], [x3,y3,z3]]  ]
            targets_position_2d = self.targets_transform_ordered(targets_position_3d)
            ==> targets_position_2d = [[x2,y2,z2], [x0,y0,z0], [x4,y4,z4], [x1,y1,z1], [x3,y3,z3]]
        """
        targets_position_2d = [target for targets_set in targets_position_3d for target in targets_set]
        return targets_position_2d

    async def update_states_mocap(self):
        """
        Update the positions from motion capture system.

        Output:
            positions_list: 2D list for agents positions (in Qualisys coordinates),
                [[x0,y0,z0], [x1,y1,z1], ...]
        """
        positions_list = list()
        for idx in range(self.num_agents):
            msg = await self.protocol_states_list[idx].recvfrom()
            positions_list.append(np.frombuffer(msg, dtype=np.float64).tolist())
        return positions_list

    async def update_obs_mocap(self):
        """
        Update the obstacle positions from motion capture system.
        """
        msg = await self.protocol_obs.recvfrom()
        position_obs = np.frombuffer(msg, dtype=np.float64)
        # print("position_obs")
        # print(position_obs.tolist())
        return position_obs.tolist()

    def generate_targets_manually(self, case_num: int):
        """
        Generate some targets in Qualisys coordinates (meter) manually.

        Output:
            targets_position: [[x0,y0,z0], [x1,y1,z1], ...]
        """
        if case_num == 5:
            targets_position = [[1.18, 0.0, self.height_fly], [1.7, 0.5, self.height_fly],
                                [1.8, 0.9, self.height_fly], [0.4, 0.9, self.height_fly],
                                [1.8, -0.9, self.height_fly], [1.86, -1.8, self.height_fly],
                                [1.9, -1.4, self.height_fly]]
        elif case_num == 6:
            targets_position = [[1.0, 0.0, self.height_fly], [1.7, 0.5, self.height_fly],
                                [1.5, 0.9, self.height_fly], [0.4, 0.9, self.height_fly],
                                [1.8, -0.5, self.height_fly], [1.86, -1.8, self.height_fly],
                                [1.9, -1.4, self.height_fly]]
        else:
            pass
        return targets_position
