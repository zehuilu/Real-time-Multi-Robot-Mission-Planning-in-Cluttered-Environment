#!/usr/bin/env python3
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from itertools import chain
import pathmagic
with pathmagic.context():
    from Simulator import Simulator


"""
This file contains two coordinate transformation functions with a configuration of AIMS LAB.
The height coordinate (Z) doesn't change and always points up in any coordinates.

The map array coordinate (meter) has an origin at the left bottom corner. Positive-X points right and positive-Y points forwards. (East-North-Up)
The qualisys coordinate's Positive-X points forwards and positive-Y points right. (North-West-Up)

The qualisys coordinate is shown below.
(+2.4, +1.6, 0)------------------(+2.4, -1.8, 0)
|                                              |
|                                              |
|                                              |
|                                              |
|                                              |
|                                              |
|                                              |
|                                              |
|                                              |
|                                              |
|                (0, 0, 0)                     |
|                                              |
|                                              |
|                                              |
|                                              |
|                                              |
|                                              |
|                                              |
|                                              |
|                                              |
|                                              |
(-2.4, +1.6, 0)------------------(-2.4, -1.8, 0)

"""


class SimulatorAimsLab(Simulator):
    def __init__(self, map_resolution: int, buffer_bdy=0.0):
        # configuration for the simulator of AIMS LAB

        self.max_x_qualisys = +3.0  # max x coordinates (meter) in qualisys
        self.min_x_qualisys = -3.0  # min x coordinates (meter) in qualisys
        self.max_y_qualisys = +2.0  # max y coordinates (meter) in qualisys
        self.min_y_qualisys = -2.6  # min y coordinates (meter) in qualisys

        # buffer boundary size for retangular obstacles
        if buffer_bdy < 0.0:
            raise Exception("buffer boundary must >= 0.0")
        self.buffer_bdy = buffer_bdy

        map_width_meter = abs(self.max_y_qualisys) + abs(self.min_y_qualisys)
        map_height_meter = abs(self.max_x_qualisys) + abs(self.min_x_qualisys)

        value_non_obs = 0  # the cell is empty
        value_obs = 255  # the cell is blocked
        Simulator.__init__(self, map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)

    def qualisys_to_map_meter(self, position_qualisys: list):
        """
        Translate a qualisys coordinate to a map array coordinate (meter).
        In the map array coordinate, the left bottom corner is the origin.
        """
        position_map_meter = [-position_qualisys[1] + self.max_y_qualisys,
                              position_qualisys[0] + abs(self.min_x_qualisys),
                              position_qualisys[2]]
        return position_map_meter

    def map_meter_to_qualisys(self, position_map_meter: list):
        """
        Translate a map array coordinate (meter) to a qualisys coordinate.
        In the map array coordinate, the left bottom corner is the origin.
        """
        position_qualisys = [position_map_meter[1] - abs(self.min_x_qualisys),
                             -position_map_meter[0] + self.max_y_qualisys,
                             position_map_meter[2]]
        return position_qualisys

    def qualisys_to_map_index(self, position_qualisys: list):
        """
        Transform a single qualisys coordinate (meter) to a map array (index) coordinate.
        NOTE: to speed up the transformation over a set of coordinates, a vectorization function can be defined.

        Input:
            position_qualisys: [px0,py0,pz0]

        Output:
            position_index: [width, height]
        """
        # qualisys coordinate to map array coordinate (meter)
        positions_meter = self.qualisys_to_map_meter(position_qualisys)
        # map array coordinate (meter) to map array coordinate (index)
        position_index = self.position_to_map_index(positions_meter[:-1])  # 2D, no height
        return position_index

    def qualisys_to_map_index_all(self, positions_qualisys: list):
        """
        Transform a list of qualisys coordinates (meter) to the map array (index) coordinates.
        NOTE: to speed up the transformation over a set of coordinates, a vectorization function can be defined.

        Input:
            positions_qualisys: [[px0,py0,pz0], [px1,py1,pz1], ...]

        Output:
            positions_index: [w0,h0, w1,h1, w2,h2, ...]
        """
        # # qualisys coordinate to map array coordinate (meter)
        # positions_meter = []
        # for idx in range(len(positions_qualisys)):
        #     positions_meter.append(coord_qualisys.qualisys_to_map_meter(positions_qualisys[idx]))

        # # map array coordinate (meter) to map array coordinate (index)
        # positions_index = []
        # for idx in range(len(positions_meter)):
        #     position_meter_now = positions_meter[idx]
        #     positions_index.extend(self.position_to_map_index(position_meter_now[:-1]))  # 2D, no height

        positions_index = list()
        for idx in range(len(positions_qualisys)):
            positions_index.extend(self.qualisys_to_map_index(positions_qualisys[idx]))
        return positions_index

    def path_index_to_qualisys(self, path_index: list, height: float):
        """
        Transform the path index coordinates of a single agent to the qualisys coordinates (meter).
        NOTE: to speed up the transformation over a set of coordinates, a vectorization function can be defined.

        Input:
            path_index: [[x0,y0, x1,y1, ..., x2,y2], [x2,y2, x3,y3, ..., x4,y4], [x4,y4, ..., x5,y5]]
                each sub-list is a part of path, where the last two elements (position indices) are the same with the first two elements of next sub-list.
            height: the height of the original coordinates

        Output:
            path_qualisys.tolist(): a 2D list [[x0,y0,z0], [x1,y1,z1], ..., [x2,y2,z2]] as a discrete path
                each sub-list is a qualisys coordinate as a single path point
        """
        # delete the duplicated ones.
        for idx in range(len(path_index)-1):
            if path_index[idx]:
                del path_index[idx][-1]
                del path_index[idx][-1]

        # flatten the path list, and reshape it to N by #dimension array
        path_index_1d = list(chain.from_iterable(path_index))
        path_index_2d = np.reshape(path_index_1d, (-1,2))

        # map array coordinate (index) to map array coordinate (meter)
        path_meter_2d = np.zeros(path_index_2d.shape)
        for idx in range(path_index_2d.shape[0]):
            path_meter_2d[idx] = self.map_index_to_position(path_index_2d[idx])

        # stack the constant height in meter
        height_array = height * np.ones((path_meter_2d.shape[0], 1))
        path_meter_3d = np.hstack((path_meter_2d, height_array))

        # map array coordinate (meter) to qualisys coordinate
        path_qualisys = np.zeros(path_meter_3d.shape)
        for idx in range(path_meter_3d.shape[0]):
            path_qualisys[idx] = self.map_meter_to_qualisys(path_meter_3d[idx])

        return path_qualisys.tolist()

    def path_index_all_to_qualisys(self, path_index_all: list, heights_all: list):
        """
        Transform the path index coordinates of multiple agents to the qualisys coordinates (meter).
        NOTE: to speed up the transformation over a set of coordinates, a vectorization function can be defined.

        Input:
            path_index_all: [[[x0,y0,...,x1,y1],[x1,y1,...,x2,y2],[x2,y2,...,x3,y3]], ..., [[x0,y0,...,x1,y1],[x1,y1,...,x2,y2],[x2,y2,...,x3,y3]], ...]
                each sub-list is a path of an agent, meaning path_index = path_index_all[0]
            heights_all: a list of heights of multiple agents

        Output:
            path_qualisys_all: a 3D list for the discrete paths of multiple agents, path_qualisys = path_qualisys_all[0]
        """
        path_qualisys_all = list()
        for idx in range(len(path_index_all)):
            path_qualisys_now = self.path_index_to_qualisys(path_index_all[idx], heights_all[idx])
            path_qualisys_all.append(path_qualisys_now)
        return path_qualisys_all

    def update_obs_map_by_center(self, position_obs_qualisys: list, size_obs_qualisys: list):
        """
        Update the map based on a single obstacle's center position and its size, assuming the obstacle is a rectangular.
        size_obs_qualisys is the size of the obstacle in qualisys coordinate [x, y] (meter) with infinite z dimension.
        
        Input:
            position_obs_qualisys: [px, py, pz] (meter)
            size_obs_qualisys: [x, y] (meter)
                size x is the height in the map, and size y is the width in the map.
        """
        # obs_center_index = [width, height]
        obs_center_index = self.qualisys_to_map_index(position_obs_qualisys)

        # the half number of index cells in each direction
        half_width_index = max(int(size_obs_qualisys[1]*self.resolution/2), 1)
        half_height_index = max(int(size_obs_qualisys[0]*self.resolution/2), 1)

        # the range of index in each direction
        min_height_index = max(obs_center_index[1]-half_height_index, 0)
        max_height_index = min(obs_center_index[1]+half_height_index, self.map_height-1)

        min_width_index = max(obs_center_index[0]-half_width_index, 0)
        max_width_index = min(obs_center_index[0]+half_width_index, self.map_width-1)

        # update the map
        self.map_array[min_height_index : max_height_index+1][:, min_width_index : max_width_index+1] = self.value_obs * \
            np.ones((max_height_index-min_height_index+1, max_width_index-min_width_index+1))

    def update_obs_map_by_lbc(self, position_obs_qualisys: list, size_obs_qualisys: list):
        """
        Update the map based on a single obstacle's left-bottom-corner position and its size, assuming the obstacle is a rectangular.
        size_obs_qualisys is the size of the obstacle in qualisys coordinate [x, y] (meter) with infinite z dimension.
        
        Input:
            position_obs_qualisys: [px, py, pz] (meter)
            size_obs_qualisys: [x, y] (meter)
                size x is the height in the map, and size y is the width in the map.
        """
        # obs_lbc_index = [width, height]
        obs_lbc_index = self.qualisys_to_map_index(position_obs_qualisys)

        # number of index cells in each direction
        width_range = max(int(size_obs_qualisys[1]*self.resolution), 1)
        height_range = max(int(size_obs_qualisys[0]*self.resolution), 1)

        # the range of index in each direction
        min_height_index = max(obs_lbc_index[1], 0)
        max_height_index = min(obs_lbc_index[1]+height_range, self.map_height-1)

        min_width_index = max(obs_lbc_index[0], 0)
        max_width_index = min(obs_lbc_index[0]+width_range, self.map_width-1)

        # update the map
        self.map_array[min_height_index : max_height_index+1][:, min_width_index : max_width_index+1] = self.value_obs * \
            np.ones((max_height_index-min_height_index+1, max_width_index-min_width_index+1))

    # the following is for real-time plotting
    def create_realtime_plot(self, realtime_flag=True, cluster_legend_flag=False, path_legend_flag=True):
        """
        Override method create_realtime_plot() of parent Object Simulator().
        Create a realtime plotting figure in metric.

        NOTE: x-axis in the plot is the negative y-axis in AIMS Lab,
        y-axis in the plot is the x-axis in AIMS Lab.
        I invert x-axis of the figure so that we only need to swap x and y coordinates.
        We don't need to invert coordinates.
        """
        _, ax = plt.subplots(1, 1)
        if realtime_flag:
            plt.ion()
            plt.show()
        # figure settings
        handles, labels = self.figure_settings(ax, cluster_legend_flag, path_legend_flag)
        legend = plt.legend(handles, labels, bbox_to_anchor=(1, 1), loc="upper left", framealpha=1)
        return ax

    def update_realtime_plot(self, position_traj_list: list, agents_position: list, targets_position: list,
                             task_allocation_result_many_agents: list, ax):
        """
        Override method update_realtime_plot() of parent Object Simulator().
        Update realtime plotting once in an existing figure in metric.

        Inputs:
            position_traj_list: a list of numpy 2d array, each element is a N-by-3 trajectory for an agent.
                The first and second columns are for x and y coordinates, and won't plot height (z coordinates) here.
            agents_position: [[x0,y0,z0], [x1,y1,z1], ...]
            targets_position: [[x0,y0,z0], [x1,y1,z1], ...]

        NOTE: x-axis in the plot is the negative y-axis in AIMS Lab,
        y-axis in the plot is the x-axis in AIMS Lab.
        I invert x-axis of the figure so that we only need to swap x and y coordinates.
        We don't need to invert coordinates.
        """
        ax.clear()  # clear the previous figure
        # offset to plot text in 2D space
        text_offset = 0.05
        cmap = matplotlib.colors.ListedColormap(['white','black'])
        # plot obstacles
        self.plot_obs(ax)
        # plot agents and targets
        self.plot_agents(agents_position, text_offset, ax)
        self.plot_targets(targets_position, text_offset, ax)
        # make sure position_traj_list is not [] or [[]]
        # if np.array(position_traj_list).size:
        if position_traj_list:
            # plot paths
            self.plot_path_multi_agent_figure(position_traj_list, agents_position, targets_position,
                                              task_allocation_result_many_agents, ax)
        # plot legends
        handles, labels = self.figure_settings(ax, cluster_legend_flag=False, path_legend_flag=True)
        legend = plt.legend(handles, labels, bbox_to_anchor=(1, 1), loc="upper left", framealpha=1)
        plt.draw()

    def plot_agents(self, agents_position: list, text_offset: float, ax):
        """
        Override method plot_agents() of parent Object Simulator().
        Plot agents in metric.

        Inputs:
            agents_position: [[x0,y0,z0], [x1,y1,z1], ...]

        NOTE: x-axis in the plot is the negative y-axis in AIMS Lab,
        y-axis in the plot is the x-axis in AIMS Lab.
        I invert x-axis of the figure so that we only need to swap x and y coordinates.
        We don't need to invert coordinates.
        """
        for idx_agent in range(len(agents_position)):
            ax.scatter(agents_position[idx_agent][1], agents_position[idx_agent][0], marker="o", color="blue")
            ax.text(agents_position[idx_agent][1]+text_offset, agents_position[idx_agent][0]+text_offset,
                    "A"+str(idx_agent), fontweight="bold", color="blue")

    def plot_targets(self, targets_position: list, text_offset: float, ax):
        """
        Override method plot_targets() of parent Object Simulator().
        Plot targets in metric.

        Inputs:
            targets_position: [[x0,y0,z0], [x1,y1,z1], ...]

        NOTE: x-axis in the plot is the negative y-axis in AIMS Lab,
        y-axis in the plot is the x-axis in AIMS Lab.
        I invert x-axis of the figure so that we only need to swap x and y coordinates.
        We don't need to invert coordinates.
        """
        for idx_target in range(len(targets_position)):
            ax.scatter(targets_position[idx_target][1], targets_position[idx_target][0], marker="x", color="red")
            ax.text(targets_position[idx_target][1]+text_offset, targets_position[idx_target][0]+text_offset,
                    "T"+str(idx_target), fontweight="bold", color="red")

    def plot_path_multi_agent_figure(self, position_traj_list: list, agents_position: list,
                                     targets_position: list, task_allocation_result_many_agents: list, ax):
        """
        Override method plot_path_multi_agent_figure() of parent Object Simulator().
        Plot path for multiple agents in an existing figure in metric.

        Inputs:
            position_traj_list: a list of numpy 2d array, each element is a N-by-3 trajectory for an agent.
                The first and second columns are for x and y coordinates, and won't plot height (z coordinates) here.

        NOTE: need to clean up task_allocation_result from DrMaMP

        NOTE: x-axis in the plot is the negative y-axis in AIMS Lab,
        y-axis in the plot is the x-axis in AIMS Lab.
        I invert x-axis of the figure so that we only need to swap x and y coordinates.
        We don't need to invert coordinates.
        """
        for idx_agent in range(len(position_traj_list)):
            position_traj_each_agent = position_traj_list[idx_agent]
            if np.array(position_traj_each_agent).size:
                ax.plot(position_traj_each_agent[:, 1], position_traj_each_agent[:, 0],
                        linewidth=2, color="green", linestyle="dashed")

    def plot_obs(self, ax):
        """
        Plot obstacles in Rectangle.

        NOTE: x-axis in the plot is the negative y-axis in AIMS Lab,
        y-axis in the plot is the x-axis in AIMS Lab.
        I invert x-axis of the figure so that we only need to swap x and y coordinates.
        We don't need to invert coordinates.
        """
        for idx in range(len(self.obs_lbc_list_manual)):
            pt_lbc = (self.obs_lbc_list_manual[idx][1], self.obs_lbc_list_manual[idx][0])
            rect = patches.Rectangle(pt_lbc,
                                     width=-self.obs_size_list_manual[idx][1],
                                     height=self.obs_size_list_manual[idx][0],
                                     color="black", alpha=0.5)
            ax.add_artist(rect)

    def figure_settings(self, ax, cluster_legend_flag: bool, path_legend_flag: bool):
        """
        Override method figure_settings() of parent Object Simulator().
        Settings for the figure in metric.

        cluster_legend_flag = True if plot cluster legends
        path_legend_flag = True if plot path legends

        NOTE: x-axis in the plot is the negative y-axis in AIMS Lab,
        y-axis in the plot is the x-axis in AIMS Lab.
        I invert x-axis of the figure so that we only need to swap x and y coordinates.
        We don't need to invert coordinates.
        """
        ax.set_xlabel("y")
        ax.set_ylabel("x")
        ax.set_aspect("equal")

        ax.set_ylim([self.min_x_qualisys, self.max_x_qualisys])
        ax.set_xlim([self.min_y_qualisys, self.max_y_qualisys])

        # set legends
        if cluster_legend_flag:
            colors = ["blue", "red", "purple"]
            marker_list = ["o", "x", "*"]
            labels = ["Agent", "Task", "Cluster Center"]
        else:
            colors = ["blue", "red"]
            marker_list = ["o", "x"]
            labels = ["Agent", "Task"]
        f = lambda m,c: plt.plot([],[],marker=m, color=c, ls="none")[0]
        handles = [f(marker_list[i], colors[i]) for i in range(len(labels))]

        if path_legend_flag:
            # add legend about path
            handles.append(plt.plot([],[], linestyle="dashed", color="green", linewidth=2)[0])
            handles.append(plt.plot([],[], linestyle="dashed", color="red", linewidth=2)[0])
            handles.append(patches.Patch(color="black", alpha=0.5))
            labels.extend(["Path", "No Path", "Obstacles"])
        plt.gca().invert_xaxis()
        return handles, labels

    def generate_obs_manually(self, case_num: int):
        """
        Generate some obstacles manually and update the map accordingly. Obstacles are rectangular.
        
        Each element of self.obs_lbc_list_manual is the left-bottom-corner position in qualisys coordinate.
        Each element of self.obs_size_list_manual is the size of the obstacle in qualisys coordinate [x, y] (meter) with infinite height.
        """
        # for occupancy grid map, obstacles are rectangular; for visualization here, obstacles are ellipses
        self.obs_lbc_list_manual = list()
        self.obs_size_list_manual = list()
        if case_num == 1:
            self.gen_obs_ret(pt_lbc=[-1.13, -0.06], obs_size=[0.3, 1.8])
            self.gen_obs_ret(pt_lbc=[1.05, 1.2], obs_size=[0.3, 0.9])
            self.gen_obs_ret(pt_lbc=[-1.15, -2.04], obs_size=[0.65, 0.65])
        elif case_num == 2:
            self.gen_obs_ret(pt_lbc=[-0.36, 1.31], obs_size=[0.3, 0.9])
            self.gen_obs_ret(pt_lbc=[-1.47, 0.77], obs_size=[1.7, 0.3])
            self.gen_obs_ret(pt_lbc=[-0.68, 0.50], obs_size=[0.3, 1.5])
            self.gen_obs_ret(pt_lbc=[-1.15, -2.04], obs_size=[0.65, 0.65])
        elif case_num == 3:
            self.gen_obs_ret(pt_lbc=[-0.22, -0.10], obs_size=[0.3, 1.0])
            self.gen_obs_ret(pt_lbc=[-1.15, -2.04], obs_size=[0.65, 0.65])
        elif case_num == 4:
            self.gen_obs_ret(pt_lbc=[-0.22, -0.10], obs_size=[0.3, 1.2])
            self.gen_obs_ret(pt_lbc=[-1.15, -2.04], obs_size=[0.65, 0.65])
        elif case_num == 5:
            self.gen_obs_ret(pt_lbc=[0.8, -0.63], obs_size=[0.63, 2.05])
            self.gen_obs_ret(pt_lbc=[-1.07, 1.90], obs_size=[0.63, 1.8])
            self.gen_obs_ret(pt_lbc=[-1.15, -2.04], obs_size=[0.65, 0.65])
        elif case_num == 6:
            self.gen_obs_ret(pt_lbc=[0.8, -0.63], obs_size=[0.63, 2.05])
            self.gen_obs_ret(pt_lbc=[-1.07, 1.90], obs_size=[0.63, 1.8])
            self.gen_obs_ret(pt_lbc=[-1.15, -2.04], obs_size=[0.65, 0.65])
        else:
            pass

        # update self.map_array
        for idx in range(len(self.obs_lbc_list_manual)):
            # buffer
            pt_lbc_new, obs_size_new = self.buffer_obs(self.obs_lbc_list_manual[idx], self.obs_size_list_manual[idx])
            pt_lbc_new.append(0)
            self.update_obs_map_by_lbc(pt_lbc_new, obs_size_new)

    def gen_obs_ret(self, pt_lbc: list, obs_size: list):
        """
        Generate a rectangular obstacle by the left-bottom-corner [px, py] and
        its size [height, wigth], i.e., [x, y] in Qualisys coordinates.

        Input:
            pt_lbc: [x0, y0]
            obs_size: [dx, dy]
        """
        self.obs_lbc_list_manual.append(pt_lbc)
        self.obs_size_list_manual.append(obs_size)

    def buffer_obs(self, pt_lbc: list, obs_size: list):
        """
        Generate the left-bottom-corner position and the obstacle size of a buffered obstacle.

        Input:
            pt_lbc: [x0, y0]
            obs_size: [dx, dy]
        """
        # left-bottom-corner for buffered obstacle
        x_obs_lbc = pt_lbc[0] - self.buffer_bdy
        y_obs_lbc = pt_lbc[1] + self.buffer_bdy

        pt_lbc_new = [x_obs_lbc, y_obs_lbc]
        obs_size_new = [obs_size[0] + 2 * self.buffer_bdy,
                        obs_size[1] + 2 * self.buffer_bdy]

        return pt_lbc_new, obs_size_new
