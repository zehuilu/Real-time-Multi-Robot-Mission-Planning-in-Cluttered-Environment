#!/usr/bin/env python3
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from random import randint


class Simulator:
    resolution: int
    map_width: int
    map_height: int
    value_non_obs: int
    value_obs: int
    size_obs_width: int
    size_obs_height: int

    def __init__(self, 
        map_width_meter: float,
        map_height_meter: float,
        resolution: int,
        value_non_obs: int,
        value_obs: int):
        """
        Constructor
        
        the cell is empty if value_non_obs, the cell is blocked if value_obs.
        """
        # map resolution, how many cells per meter
        self.resolution = resolution
        # how many cells for width and height
        map_width = map_width_meter * resolution + 1
        map_height = map_height_meter * resolution + 1

        self.map_width = int(map_width)
        self.map_height = int(map_height)

        self.value_non_obs = value_non_obs
        self.value_obs = value_obs

        # create an empty map
        self.map_array = np.array([self.value_non_obs]*(self.map_width*self.map_height)).reshape(-1, self.map_width)

    def position_to_map_index(self, position: list):
        """
        Given the position in meter [px, py], return the index of the map associated with this position.
        Positive-X points right, positive-Y points forwards.

        Input:
            position: a 1D list for position in meter [px, py]

        Output:
            idx_list: a 1D list [idx_column, idx_row] for index of width and height in self.map_array
        """
        idx_column = int( position[0] * self.resolution )
        idx_row = int( position[1] * self.resolution )
        if (idx_column > self.map_width-1) or (idx_column < 0) or (idx_row > self.map_height-1) or (idx_row < 0):
            raise Exception("Position in x-axis or y-axis exceed the range!")

        return [idx_column, idx_row]

    def map_index_to_position(self, map_index: list):
        """
        Given the map index list [idx_column, idx_row], return the position in meter [px, py] associated with this index list.
        Positive-X points right, positive-Y points forwards.

        Input:
            map_index: a 1D list [idx_column, idx_row] for index of width and height in self.map_array

        Output:
            position: a 1D list for position in meter [px, py]
        """
        return [map_index[0]/self.resolution, map_index[1]/self.resolution]

    def generate_random_obs(self, num_obs: int, size_obs_meter: list, empty_map_flag=False):
        """
        Generate random obstacles. size_obs_meter = [obs_width_meter, obs_height_meter]

            empty_map_flag: True to overwrite self.map_array as an empty array (meaning no obstacles).
        """
        if empty_map_flag:
            self.map_array = np.array([self.value_non_obs]*(self.map_width*self.map_height)).reshape(-1, self.map_width)

        self.size_obs_width = round(size_obs_meter[0] * self.resolution)
        self.size_obs_height = round(size_obs_meter[1] * self.resolution)

        self.obs_left_top_corner = list()

        if num_obs > 0:
            for idx in range(num_obs):
                # [width, height]
                self.obs_left_top_corner.append([randint(1, self.map_width-self.size_obs_width-1),
                                                 randint(1, self.map_height-self.size_obs_height-1)])

                obs_mat = self.map_array[self.obs_left_top_corner[idx][1]:\
                                         self.obs_left_top_corner[idx][1]+self.size_obs_height][\
                                         :, self.obs_left_top_corner[idx][0]:self.obs_left_top_corner[idx][0]+self.size_obs_width]

                self.map_array[self.obs_left_top_corner[idx][1]:\
                               self.obs_left_top_corner[idx][1]+self.size_obs_height][\
                               :, self.obs_left_top_corner[idx][0]:self.obs_left_top_corner[idx][0]+self.size_obs_width] \
                               = self.value_obs * np.ones(obs_mat.shape)

    def generate_agents_and_targets(self, num_agents: int, num_targets: int):
        """
        Randomly generate agents and targets which don't collide with obstacles.
        """
        agents = list()
        for _ in range(num_agents):
            start = [randint(1, self.map_width-1), randint(1, self.map_width-1)]
            while self.map_array[start[1]][start[0]] != self.value_non_obs:
                start = [randint(1, self.map_width-1), randint(1, self.map_width-1)]
            agents.extend(start)
        
        targets = list()
        for _ in range(num_targets):
            goal = [randint(1, self.map_width-1), randint(1, self.map_width-1)]
            while (self.map_array[goal[1]][goal[0]] != self.value_non_obs) or (self.check_target_collide_agents(goal, agents)):
                goal = [randint(1, self.map_width-1), randint(1, self.map_width-1)]
            targets.extend(goal)

        return agents, targets

    def check_target_collide_agents(self, target_position: list, agent_positions: list):
        """
        Check if a target collides with all the agents.

        Return True if a collision exists.
        """
        collision_flag = False
        for idx in range(0, len(agent_positions), 2):
            collision_flag = collision_flag or (target_position[0] == agent_positions[idx]) and (target_position[1] == agent_positions[idx+1])
        return collision_flag

    def generate_targets(self, num_targets: int):
        '''
        randomly generate targets.
        It's for testing KMeans and ClusterAssign.
        '''
        targets = list()
        for _ in range(0, num_targets):
            goal = [randint(0,self.map_width-1), randint(0,self.map_height-1)]
            targets.extend(goal)

        return targets

    def generate_targets_at_corner(self, num_targets: int):
        """
        randomly generate targets at some corners.
        It's for testing KMeans and ClusterAssign.
        """
        targets = list()
        for _ in range(0,int(num_targets/3)):
            goal = [randint(0,25), randint(0,25)]
            targets.extend(goal)
        
        for _ in range(0,int(num_targets/3)):
            goal = [randint(80,100), randint(80,100)]
            targets.extend(goal)

        for _ in range(0,int(num_targets/3)):
            goal = [randint(0,30), randint(80,100)]
            targets.extend(goal)

        return targets

    def plot_single_path(self, path_single: list):
        """
        Plot a single path.

        Input:
            path_single: a 1D list for the path [x0,y0, x1,y1, x2,y2, ...]
        """
        if path_single:
            ax_map = self.create_realtime_plot(realtime_flag=False, cluster_legend_flag=False, path_legend_flag=True)
            # plot the map
            cmap = matplotlib.colors.ListedColormap(['white','black'])
            ax_map.pcolormesh(self.map_array, cmap=cmap, edgecolors='none')
            ax_map.scatter(path_single[0]+0.5, path_single[1]+0.5, marker="o", color="blue")
            ax_map.scatter(path_single[-2]+0.5, path_single[-1]+0.5, marker="x", color="red")
            ax_map.plot(list(map(lambda x:x+0.5, path_single[0::2])),
                        list(map(lambda x:x+0.5, path_single[1::2])), color="green", linewidth=2)
            plt.show(block=False)
        else:
            print("Lazy Theta Star didn't find a path!")

    def plot_paths(self, path_many_agents: list, agents_position: list,
                       targets_position: list, task_order: list,
                       cluster_centers: list, points_idx_for_clusters: list):
        """
        Plot many paths for multiple agents.

        Input:
            path_many_agents: a 3D list for paths, each 2D list is the path for each agent,
                [[x0,y0, x1,y1, x2,y2, ...], [x0,y0, x1,y1, x2,y2, ...], ...]
            agents_position: a 1D list for all the agent positions, [x0,y0, x1,y1, x2,y2, ...]
            targets_position: a 1D list for all the target positions, [x0,y0, x1,y2, x2,y2, ...]
            task_order: a 2D list for the task allocation order of all the agents.
                If it is empty, there is no task allocation result.
                example: task_order[1] = [3, 0, 2, 3, 1, 4] means the task
                allocation order for Agent 1 is T3 -> T0 -> T2 -> T3 -> T1 -> T4, where Agent 1 is the
                second agent, and T0 is the first task.
        """
        # offset to plot text in 2D space
        text_offset = (self.map_width - 0) / 40

        # the first figure is without the solved path
        ax_before = self.create_realtime_plot(realtime_flag=False, cluster_legend_flag=False, path_legend_flag=False)
        # plot the map
        cmap = matplotlib.colors.ListedColormap(['white','black'])
        ax_before.pcolormesh(self.map_array, cmap=cmap, edgecolors='none')
        # plot agents and targets
        self.plot_agents(agents_position, text_offset, ax_before)
        self.plot_targets(targets_position, [], [], text_offset, ax_before)

        # the second figure is with the solved path
        ax = self.create_realtime_plot(realtime_flag=False, cluster_legend_flag=True, path_legend_flag=True)
        # plot the map
        cmap = matplotlib.colors.ListedColormap(['white','black'])
        ax.pcolormesh(self.map_array, cmap=cmap, edgecolors='none')
        # plot agents and targets
        self.plot_agents(agents_position, text_offset, ax)
        self.plot_targets(targets_position, cluster_centers, points_idx_for_clusters, text_offset, ax)
        # plot paths
        self.plot_paths_figure(path_many_agents, agents_position, targets_position, task_order, ax)
        plt.show(block=False)

    def plot_cluster_assign(self, agents_position: list, targets_position: list,
                            points_idx_for_clusters: list, cluster_centers: list, cluster_assigned_idx: list):
        """
        plot agents, targets, and cluster centers from ClusterAssign
        """
        # create a figure with legends
        ax = self.create_realtime_plot(realtime_flag=False, cluster_legend_flag=True, path_legend_flag=False)

        # a color list, each entry is for each cluster
        color_by_cluster = np.linspace(0, 1, num=int(len(points_idx_for_clusters)))
        
        # points_idx_for_clusters is a 2D list, stores the index of target for each cluster
        # plot clusters
        targets_plot_x = list()
        targets_plot_y = list()
        list_num_points_per_cluster = list()
        for i in range(0, int(len(points_idx_for_clusters))):
            list_num_points_per_cluster.append(len(points_idx_for_clusters[i]))
            for j in range(0, int(len(points_idx_for_clusters[i]))):
                idx_target = points_idx_for_clusters[i][j]
                targets_plot_x.append(targets_position[2*idx_target])
                targets_plot_y.append(targets_position[2*idx_target+1])
        # a color list, each entry is for each target
        color_all_targets = np.repeat(color_by_cluster, np.array(list_num_points_per_cluster))
        ax.scatter(list(map(lambda x:x+0.5, targets_plot_x)),
                   list(map(lambda x:x+0.5, targets_plot_y)), c=color_all_targets, cmap='viridis', marker='x', alpha=1)

        # plot cluster centers
        for i in range(0, int(len(cluster_centers)), 2):
            if cluster_centers[i] >= 0:
                ax.scatter(cluster_centers[i]+0.5, cluster_centers[i+1]+0.5, c=color_by_cluster[int(i/2)], marker='*', s=100)

        # plot agents and lines link to assigned clusters
        for idx_agent in range(int(len(agents_position)/2)):
            ax.scatter(agents_position[2*idx_agent]+0.5, agents_position[2*idx_agent+1]+0.5, c="blue", marker='o')
            # plot lines
            cluster_idx = cluster_assigned_idx[idx_agent]
            x_line = np.linspace(agents_position[2*idx_agent]+0.5, cluster_centers[2*cluster_idx]+0.5, num=100)
            y_line = np.linspace(agents_position[2*idx_agent+1]+0.5, cluster_centers[2*cluster_idx+1]+0.5, num=100)
            ax.plot(x_line, y_line, linewidth=2)

        plt.show(block=False)

    def create_realtime_plot(self, realtime_flag=True, cluster_legend_flag=False, path_legend_flag=True):
        """
        Create a realtime plotting figure.
        """
        _, ax = plt.subplots(1, 1)
        if realtime_flag:
            plt.ion()
            plt.show()
        # figure settings
        handles, labels = self.figure_settings(ax, cluster_legend_flag, path_legend_flag)
        legend = plt.legend(handles, labels, bbox_to_anchor=(1, 1), loc="upper left", framealpha=1)
        return ax

    def update_realtime_plot(self, path_many_agents: list, agents_position: list,
                             targets_position: list, task_order: list,
                             cluster_centers: list, points_idx_for_clusters: list, ax,
                             plot_cluster_flag=False):
        """
        Update realtime plotting once in an existing figure. See input details in self.plot_paths()
        """
        ax.clear()  # clear the previous figure

        # offset to plot text in 2D space
        text_offset = (self.map_width - 0) / 40
        cmap = matplotlib.colors.ListedColormap(['white','black'])
        # plot the map
        ax.pcolormesh(self.map_array, cmap=cmap, alpha=1.0, edgecolors='none')
        # plot agents and targets
        self.plot_agents(agents_position, text_offset, ax)
        self.plot_targets(targets_position, cluster_centers, points_idx_for_clusters,
                          text_offset, ax)
        if path_many_agents:
            # plot paths
            self.plot_paths_figure(path_many_agents, agents_position,
                                   targets_position, task_order, ax)
        # plot legends
        handles, labels = self.figure_settings(ax, cluster_legend_flag=True, path_legend_flag=True)
        legend = plt.legend(handles, labels, bbox_to_anchor=(1, 1), loc="upper left", framealpha=1)
        plt.draw()

    def plot_agents(self, agents_position: list, text_offset: float, ax):
        """
        Plot agents.
        """
        for idx_agent in range(int(len(agents_position)/2)):
            ax.scatter(agents_position[2*idx_agent]+0.5, agents_position[2*idx_agent+1]+0.5, marker="o", color="blue")
            ax.text(agents_position[2*idx_agent]+text_offset, agents_position[2*idx_agent+1]+text_offset,
                    "A"+str(idx_agent), fontweight="bold", color="blue")

    def plot_targets(self, targets_position: list, cluster_centers: list, points_idx_for_clusters: list,
                     text_offset: float, ax, target_color_flag=False):
        """
        Plot targets.

        plot_cluster_flag = True if plotting cluster centers
        target_color_flag = True if plotting targets by different colors, categorized by clusters
        """
        # if cluster_centers is not empty, plot targets by clusters
        if cluster_centers:
            # True to plot targets by different colors
            if target_color_flag:
                # a color list, each entry is for each cluster
                color_by_cluster = np.linspace(0, 1, num=int(len(cluster_centers)/2))
                # points_idx_for_clusters is a 2D list, stores the index of target for each cluster
                # plot clusters
                targets_plot_x = list()
                targets_plot_y = list()
                list_num_points_per_cluster = list()
                for i in range(int(len(points_idx_for_clusters))):
                    list_num_points_per_cluster.append(len(points_idx_for_clusters[i]))
                    for j in range(int(len(points_idx_for_clusters[i]))):
                        idx_target = points_idx_for_clusters[i][j]
                        targets_plot_x.append(targets_position[2*idx_target])
                        targets_plot_y.append(targets_position[2*idx_target+1])
                # a color list, each entry is for each target
                color_all_targets = np.repeat(color_by_cluster, np.array(list_num_points_per_cluster))
                ax.scatter(list(map(lambda x:x+0.5, targets_plot_x)),
                           list(map(lambda x:x+0.5, targets_plot_y)), c=color_all_targets, cmap='viridis', marker='x')
                # plot cluster centers
                for i in range(0, int(len(cluster_centers)), 2):
                    if cluster_centers[i] >= 0:
                        ax.scatter(cluster_centers[i]+0.5, cluster_centers[i+1]+0.5, c=color_by_cluster[int(i/2)], marker='*', s=100)
            else:
                # if cluster_centers is empty, plot targets
                for idx_target in range(int(len(targets_position)/2)):
                    ax.scatter(targets_position[2*idx_target]+0.5, targets_position[2*idx_target+1]+0.5, marker="x", color="red")
                    ax.text(targets_position[2*idx_target]+text_offset, targets_position[2*idx_target+1]+text_offset,
                            "T"+str(idx_target), fontweight="bold", color="red")
                # plot cluster centers
                for i in range(0, int(len(cluster_centers)), 2):
                    if cluster_centers[i] >= 0:
                        ax.scatter(cluster_centers[i]+0.5, cluster_centers[i+1]+0.5, color="purple", marker="*", s=100)
        else:
            # if cluster_centers is empty, plot targets
            for idx_target in range(int(len(targets_position)/2)):
                ax.scatter(targets_position[2*idx_target]+0.5, targets_position[2*idx_target+1]+0.5, marker="x", color="red")
                ax.text(targets_position[2*idx_target]+text_offset, targets_position[2*idx_target+1]+text_offset,
                        "T"+str(idx_target), fontweight="bold", color="red")

    def plot_paths_figure(self, path_many_agents: list, agents_position: list,
                                     targets_position: list, task_order: list, ax):
        """
        Plot path for multiple agents in an existing figure.
        """
        for idx_agent in range(len(path_many_agents)):
            path_many_each_agent = path_many_agents[idx_agent]
            if task_order:
                task_allocation_result_each_agent = task_order[idx_agent]
            else:
                task_allocation_result_each_agent = list()

            for idx_path in range(len(path_many_each_agent)):
                if path_many_each_agent[idx_path]:
                    ax.plot(list(map(lambda x:x+0.5, path_many_each_agent[idx_path][0::2])),
                            list(map(lambda x:x+0.5, path_many_each_agent[idx_path][1::2])),
                            linewidth=2, color="green", linestyle="dashed")
                else:
                    # plot infeasible path when task_allocation_result is not empty
                    if task_allocation_result_each_agent:
                        if idx_path == 0:
                            # the first path is from the agent to the first task
                            task_id = task_allocation_result_each_agent[0]
                            ax.plot([agents_position[2*idx_agent]+0.5,targets_position[2*task_id]+0.5],
                                    [agents_position[2*idx_agent+1]+0.5,targets_position[2*task_id+1]+0.5],
                                    linestyle='dashed', linewidth=2, color='red')
                        else:
                            idx_task_now = task_allocation_result_each_agent[idx_path-1]
                            idx_task_next = task_allocation_result_each_agent[idx_path]
                            # path from task idx_path-1 to task idx_path
                            ax.plot([targets_position[2*idx_task_now]+0.5, targets_position[2*idx_task_next]+0.5],
                                    [targets_position[2*idx_task_now+1]+0.5,targets_position[2*idx_task_next+1]+0.5],
                                    linestyle='dashed', linewidth=2, color='red')

    def figure_settings(self, ax, cluster_legend_flag: bool, path_legend_flag: bool):
        """
        Settings for the figure.

        cluster_legend_flag = True if plot cluster legends
        path_legend_flag = True if plot path legends
        """
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_aspect("equal")
        ax.set_xlim([0, self.map_width])
        ax.set_ylim([0, self.map_height])

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
        # a tuple includes the handles and labels of legend
        return handles, labels
