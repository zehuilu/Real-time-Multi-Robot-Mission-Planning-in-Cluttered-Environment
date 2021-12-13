#!/usr/bin/env python3
import os
import asyncio
import time
import json
import matplotlib.pyplot as plt
import numpy as np
import pathmagic
with pathmagic.context(EXTERNAL_FLAG=True):
    import DrMaMP
    import csv_helper
    from UdpProtocol import UdpProtocol
    from SimulatorAimsLab import SimulatorAimsLab
    from discrete_path_to_time_traj import discrete_path_to_time_traj


CASE_NUM = 4
DISTANCE_SQUARE_RESOLUTION = 0.04
BUFFER_BOUNDARY = 0.15


class PlannerMocap:
    config_data: dict
    back_home_flag: bool
    stop_planning_flag: bool
    height_fly: float

    def __init__(self, mambo_idx: int):
        """
        Initialize a PlannerMocap Object. This is used with Motion Capture System.

        Inputs:
            mambo_idx: int, index for the mambo you want to fly
        """
        # create a simulator
        self.MySimulator = SimulatorAimsLab(map_resolution=20, buffer_bdy=BUFFER_BOUNDARY)

        # load the configuration as a dictionary
        json_file = open(os.getcwd() + "/experiment/config_aimslab_ex_" + str(mambo_idx) + ".json")
        self.config_data = json.load(json_file)

        self.mocap_type = "QUALISYS"
        # the address for publishing agent positions from mocap
        self.server_address_states = (self.config_data[self.mocap_type]["IP_STATES_ESTIMATION"],
                                      int(self.config_data[self.mocap_type]["PORT_POSITION_PLANNER"]))
        # the address for publishing obstalce positions from mocap
        self.server_address_obs = (self.config_data[self.mocap_type]["IP_OBS_POSITION"],
                                   int(self.config_data[self.mocap_type]["PORT_OBS_POSITION"]))

        # True if agent needs to go home, no more target checking
        # and once arrives home, no new trajectory csv files
        self.back_home_flag = False
        # True to stop the planning
        self.stop_planning_flag = False

    async def run_planner(self, *argv):
        """
        Run the planner online with Motion Capture System.
        NOTE: plt.pause() blocks the main thread.
        """
        # remove all the existing files in the trajectory directory
        directory_delete = os.path.expanduser("~") + "/Mambo-Tracking-Interface" + self.config_data["DIRECTORY_TRAJ"] + "*"
        csv_helper.remove_traj_ref_lib(directory_delete)

        frequency = 2  # planner frequency in Hz
        time_escape = 50  # shut down the planner after this time [sec]

        # create a customized UDP protocol for subscribing states from mocap
        loop_states = asyncio.get_running_loop()
        self.transport_states, self.protocol_states = await loop_states.create_datagram_endpoint(
            UdpProtocol, local_addr=self.server_address_states, remote_addr=None)
        
        # create a customized UDP protocol for subscribing obstacles positions from mocap
        loop_obs = asyncio.get_running_loop()
        self.transport_obs, self.protocol_obs = await loop_obs.create_datagram_endpoint(
            UdpProtocol, local_addr=self.server_address_obs, remote_addr=None)

        # get the agent home position
        self.agent_home_qualisys = await self.update_states_mocap()

        self.height_fly = 1.0  # a constant fly height in meter
        # generate targets manually
        targets_position_qualisys = self.generate_targets_manually(CASE_NUM)
        # generate obstacles manually
        self.MySimulator.generate_obs_manually(CASE_NUM)

        self.ax = self.MySimulator.create_realtime_plot()  # create a realtime plotting figure
        time_begin = time.time()
        time_used = 0  # initialize the global time as 0

        while(time_used < time_escape):
            t_start = time.time()

            # update the agent position
            agent_position_qualisys = await self.update_states_mocap()
            # print("agent_position_qualisys")
            # print(agent_position_qualisys)

            # update target list
            targets_position_qualisys = self.update_target_list(agent_position_qualisys, targets_position_qualisys)

            # # update the obstacle position
            # obs_position = await self.update_obs_mocap()
            # print("obs_position")
            # print(obs_position)
            # # update the map
            # self.update_obs_map(obs_position, size_obs_qualisys)

            # distance between the agent and home
            distance_square = (agent_position_qualisys[0]-self.agent_home_qualisys[0])**2 + \
                             (agent_position_qualisys[1]-self.agent_home_qualisys[1])**2
            # if agent is going back home, and close enough to the home position
            # no sending new trajectory csv files
            if self.back_home_flag and (distance_square <= DISTANCE_SQUARE_RESOLUTION):
                self.stop_planning_flag = True
                print("Arrived home!")
            else:
                if not self.stop_planning_flag:
                    # when mocap is unavailable, returns a list of numpy nan
                    if not np.isnan(agent_position_qualisys[0]):
                        # run the planning solver once
                        position_traj, time_algorithm_ms = self.run_solver_once(agent_position_qualisys, targets_position_qualisys, time_begin)
                    else:
                        position_traj = []
                        time_algorithm_ms = []
                        print("agent_position_qualisys is nan, check mocap system!")
                else:
                    print("Arrived home! No more planning.")

            # when infeasible, position_traj is [], and update_realtime_plot() input is [[]]
            # update the figure
            # t0 = time.time()
            # NOTE: this plotting takes over 0.2 sec
            self.MySimulator.update_realtime_plot([position_traj], [agent_position_qualisys], targets_position_qualisys, [], self.ax)
            # self.MySimulator.update_realtime_plot([np.array(path_qualisys)], [agent_position_qualisys], targets_position_qualisys, [], self.ax)
            # t1 = time.time()
            # print("Plotting time used [sec]:" + str(t1 - t0))

            # plot the algorithm time
            time_str = "Computation Time [ms]: " + str(time_algorithm_ms)
            plt.text(0.25, 0.9, time_str, fontsize=14, transform=plt.gcf().transFigure)

            plt.pause(1E-9)
            time_sleep = max(0, 1/frequency - time.time() + t_start)
            # time_used = time.time() - time_begin
            # print("Current Time [sec]: " + str(time_used))
            # print("time used [sec]: ")
            # print(time.time() - t_start)
            # NOTE: if time_sleep == 0, this thread will be blocked because no sleep
            await asyncio.sleep(time_sleep)

    async def update_states_mocap(self):
        """
        Update the positions from motion capture system.
        """
        msg = await self.protocol_states.recvfrom()
        position_now = np.frombuffer(msg, dtype=np.float64)
        # print("position_now")
        # print(position_now.tolist())
        return position_now.tolist()

    async def update_obs_mocap(self):
        """
        Update the obstacle positions from motion capture system.
        """
        msg = await self.protocol_obs.recvfrom()
        position_obs = np.frombuffer(msg, dtype=np.float64)
        # print("position_obs")
        # print(position_obs.tolist())
        return position_obs.tolist()

    def run_solver_once(self, agent_position_qualisys, targets_position_qualisys, time_begin):
        """
        Run the planning solver once.

        Input:
            agent_position_qualisys: [px, py, pz] in meter
            targets_position_qualisys: [[x0,y0,z0], [x1,y1,z1], ...]
            time_begin: the time stamp where the whole planner begins

        NOTE: need to think about when no feasible, how to handle it
        """
        # transform qualisys coordinates (meter) to map array (index)
        # t0 = time.time()
        agent_position_index = self.MySimulator.qualisys_to_map_index_all([agent_position_qualisys])
        targets_position_index = self.MySimulator.qualisys_to_map_index_all(targets_position_qualisys)
        # t1 = time.time()
        # print("Qualisys coordinates to map index. Time used [sec]: " + str(t1 - t0))
        
        # do the planning
        t0 = time.time()
        path_all_agents_index, task_allocation_result = DrMaMP.SolveOneAgent(
            agent_position_index, targets_position_index,
            self.MySimulator.map_array.flatten().tolist(),
            self.MySimulator.map_width, self.MySimulator.map_height)
        t1 = time.time()
        time_algorithm_ms = round((t1-t0)*1000, 2)  # milliseconds
        # print("Solver time used [sec]:" + str(t1 - t0))

        # transform map array (index) to qualisys coordinates (meter)
        # t0 = time.time()
        path_qualisys = self.MySimulator.path_index_to_qualisys(path_all_agents_index, self.height_fly)
        # t1 = time.time()
        # print("Map index to qualisys coordinate. Time used [sec]: " + str(t1 - t0))

        # generate position and velocity trajectories (as a motion planner)
        # t0 = time.time()
        dt = 0.1
        velocity_ave = 0.20
        # generate trajectories
        time_now = time.time()
        # for debugging, when infeasible, path_qualisys is []
        try:
            time_queue_vec, position_traj, velocity_traj = discrete_path_to_time_traj(
                path_qualisys, dt, velocity_ave, interp_kind='linear',
                velocity_flag=True, ini_velocity_zero_flag=False)

            # Mambo Tracking Controller uses the accumulated time, need to change the time trajectory here too
            time_queue_vec = time_queue_vec + max(time_now-time_begin-1.0, 0)
            # t1 = time.time()
            # print("Trajectory generator time used [sec]:" + str(t1 - t0))

            # output trajectories as a CSV file
            array_csv = np.vstack((time_queue_vec, np.array(position_traj).T, np.array(velocity_traj).T))
            time_name = time.strftime("%Y%m%d%H%M%S")
            filename_csv = os.path.expanduser("~") + "/Mambo-Tracking-Interface" + self.config_data["DIRECTORY_TRAJ"] + time_name + ".csv"
            np.savetxt(filename_csv, array_csv, delimiter=",")
        except:
            position_traj = list()

        return position_traj, time_algorithm_ms

    def update_target_list(self, agent_position: list, targets_position: list):
        """
        Check if an agent visits targets. If yes, remove this target.
        Only check x and y coordinates.

        Input:
            agent_position: [x, y, z] in meter
            targets_position: [[x0,y0,z0], [x1,y1,z1], ...] in meter

        Output:
            targets_position_new: [[x0,y0,z0], ...] in meter
        """
        if CASE_NUM == 4:
            if len(targets_position) == 3:
                if targets_position[-1][0] < 1.8:
                    targets_position[-1][0] += 0.25

        if not self.back_home_flag:
            # if agent is not going back home, check targets
            if targets_position:
                targets_position_new = targets_position.copy()
                for idx_target in range(len(targets_position)):
                    target_position_this = targets_position[idx_target]

                    if (agent_position[0]-target_position_this[0])**2 + \
                        (agent_position[1]-target_position_this[1])**2 <= DISTANCE_SQUARE_RESOLUTION:
                        del targets_position_new[idx_target]
                        print("Target visited!")

                        if CASE_NUM == 3:
                            num_obs = 2
                            self.MySimulator.gen_obs_ret(pt_lbc=[-1.24, 0.1], obs_size=[1.4, 0.3])
                            self.MySimulator.gen_obs_ret(pt_lbc=[-1.24, 0.71], obs_size=[0.3, 0.6])
                            # update self.map_array
                            for idx in range(-1, -1-num_obs, -1):
                                # buffer
                                pt_lbc_new, obs_size_new = self.MySimulator.buffer_obs(
                                    self.MySimulator.obs_lbc_list_manual[idx],
                                    self.MySimulator.obs_size_list_manual[idx])
                                pt_lbc_new.append(0)
                                self.MySimulator.update_obs_map_by_lbc(pt_lbc_new, obs_size_new)

                if not targets_position_new:
                    if CASE_NUM == 2:
                        self.back_home_flag = True
                        self.agent_home_qualisys = [-0.1, 1.37, self.height_fly]
                        targets_position_new = [self.agent_home_qualisys]
                    elif CASE_NUM == 3:
                        self.back_home_flag = True
                        self.agent_home_qualisys = [-2.05, -1.02, self.height_fly]
                        targets_position_new = [self.agent_home_qualisys]
                    elif CASE_NUM == 4:
                        self.back_home_flag = True
                        self.agent_home_qualisys = [-2.05, -1.52, self.height_fly]
                        targets_position_new = [self.agent_home_qualisys]
                    else:
                        # after delete finished targets, if targets_position_new is empty, go back home
                        targets_position_new = [self.agent_home_qualisys]
                    self.back_home_flag = True
                    print("No more targets, go back home!")
            else:
                # if no more targets, go back home
                targets_position_new = [self.agent_home_qualisys]
                self.back_home_flag = True
                print("No more targets, go back home!")
        else:
            # agent is going back home, don't change targets_position
            targets_position_new = targets_position
        return targets_position_new

    def generate_targets_manually(self, case_num: int):
        """
        Generate some targets in Qualisys coordinates (meter) manually.

        Output:
            targets_position: [[x0,y0,z0], [x1,y1,z1], ...]
        """
        if case_num == 1:
            targets_position = [[0.2, -0.4, self.height_fly], [1.8, 0.9, self.height_fly]]
        elif case_num == 2:
            targets_position = [[0.2, -0.4, self.height_fly], [1.8, 0.9, self.height_fly]]
        elif case_num == 3:
            targets_position = [[-0.18, 0.48, self.height_fly], [1.08, -0.57, self.height_fly],
                                [-1.6, -1.02, self.height_fly]]
        elif case_num == 4:
            targets_position = [[-1.21, -0.47, self.height_fly], [1.08, -0.57, self.height_fly],
                                [-1.85, -1.37, self.height_fly]]
        else:
            pass
        return targets_position
