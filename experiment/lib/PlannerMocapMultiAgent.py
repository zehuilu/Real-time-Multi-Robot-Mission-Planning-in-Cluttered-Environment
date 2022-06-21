#!/usr/bin/env python3
import os
import asyncio
import math
import copy
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
DISTANCE_THRESHOLD = 0.25
BUFFER_BOUNDARY = 0.10
# 5 static, 6 dynamic obs, 7 dynamic task
CASE_NUM = 5
DYNAMIC_OBS_FLAG = False
DYNAMIC_TASK_FLAG = False
if CASE_NUM == 6:
    DYNAMIC_OBS_FLAG = True
if CASE_NUM == 7:
    DYNAMIC_TASK_FLAG = True


class PlannerMocapMultiAgent:
    numAgent: int  # number of agents
    planningFrequency: int  # planning and visualization frequency in Hz
    listAgentFSMExp: list  # a list of AgentFSMExp objects
    numCluster: int  # number of clusters for task decomposition, mission planning
    numIter: int  # number of iterations for task decomposition, mission planning
    configDataList: list  # a list of dictionaries for agents configurations
    heightFly: float

    def __init__(self):
        """
        Initialize a PlannerMocap Object. This is used with Motion Capture System.
        """
        self.numAgent = 2  # number of agents

        # create a simulator
        self.MySimulator = SimulatorAimsLab(map_resolution=20, buffer_bdy=BUFFER_BOUNDARY)

        # load multiple agents' configuration as a list of dictionaries
        self.configDataList = list()
        for idx in range(self.numAgent):
            fileName = os.getcwd() + "/experiment/config_aimslab_ex_" + str(idx+1) + ".json"
            self.configDataList.append(json.load(open(fileName)))

        self.mocapType = "QUALISYS"
        # the address for publishing agents positions from mocap
        self.stateAddressList = list()
        for idx in range(self.numAgent):
            self.stateAddressList.append(
                (self.configDataList[idx][self.mocapType]["IP_STATES_ESTIMATION"],
                int(self.configDataList[idx][self.mocapType]["PORT_POSITION_PLANNER"])))

        # the address for publishing obstalce positions from mocap
        self.serverAddressObs = (self.configDataList[0][self.mocapType]["IP_OBS_POSITION"],
                                 int(self.configDataList[0][self.mocapType]["PORT_OBS_POSITION"]))

        # the address for publishing tasks positions from mocap
        self.serverAddressTask = (self.configDataList[1][self.mocapType]["IP_OBS_POSITION"],
                                  int(self.configDataList[1][self.mocapType]["PORT_OBS_POSITION"]))

    async def runPlanner(self):
        """
        Run the planner online with Motion Capture System.
        """
        self.listAgentFSMExp = list()  # a list for Agent Finite State Machine
        self.planningFrequency = 5  # planner frequency in Hz
        # number of clusters for task decomposition
        self.numCluster = self.numAgent
        # number of iterations for task decomposition
        self.numIter = 500
        # each element is each agent's average velocity (m/s)
        velocityAveList = [0.20, 0.20]

        # remove all the existing files in the trajectory directory
        for idx in range(self.numAgent):
            # directoryDelete = os.path.expanduser("~") + "/Mambo-Tracking-Interface" + \
            #                   self.configDataList[idx]["DIRECTORY_TRAJ"] + "*"
            directoryDelete = os.path.expanduser("~") + "/Mambo-Tracking-Interface-DrMaMP" + \
                              self.configDataList[idx]["DIRECTORY_TRAJ"] + "*"
            csv_helper.remove_traj_ref_lib(directoryDelete)

        timeEscape = 100  # shut down the iteration after this time [sec]

        # create a customized UDP protocol for subscribing states from mocap
        loopState01 = asyncio.get_running_loop()
        _, protocolState01 = await loopState01.create_datagram_endpoint(
            UdpProtocol, local_addr=self.stateAddressList[0], remote_addr=None)

        loopState02 = asyncio.get_running_loop()
        _, protocolState02 = await loopState02.create_datagram_endpoint(
            UdpProtocol, local_addr=self.stateAddressList[1], remote_addr=None)

        self.protocolStateList = [protocolState01, protocolState02]

        # create a customized UDP protocol for subscribing obstacles positions from mocap
        loopObs = asyncio.get_running_loop()
        _, protocolObs01 = await loopObs.create_datagram_endpoint(
            UdpProtocol, local_addr=self.serverAddressObs, remote_addr=None)
        self.protocolObsList = [protocolObs01]
        obsSizeList = [[0.30, 0.30]]

        if not DYNAMIC_OBS_FLAG:
            obsPosiList = list()
            obsSizeList = list()

        # create a customized UDP protocol for subscribing a task position from mocap
        loopTask = asyncio.get_running_loop()
        _, self.protocolTask01 = await loopTask.create_datagram_endpoint(
            UdpProtocol, local_addr=self.serverAddressTask, remote_addr=None)

        # get the agent home position
        self.agentHomeList = await self.updateStateMocap()

        self.heightFly = 0.8  # a constant fly height in meter
        # generate a scenario
        # generate targets manually
        targetPositionQualisys = self.generateTargetManually(CASE_NUM)
        # generate obstacles manually
        self.MySimulator.generate_obs_manually(CASE_NUM)
        # initialize the map with static obstacles
        self.mapArrayInitial = self.MySimulator.map_array.copy()

        if DYNAMIC_TASK_FLAG:
            dynTaskPosi = await self.updateTaskMocap()
            targetPositionQualisys.append(dynTaskPosi)

        # transform qualisys coordinates (meter) to map array (index)
        agentPositionIndex = self.MySimulator.qualisys_to_map_index_all(self.agentHomeList)
        targetPositionIndex = self.MySimulator.qualisys_to_map_index_all(targetPositionQualisys)

        # initial mission planning
        _, taskOrder, clusterCenter, _, _ = DrMaMP.MissionPlanning(
            agentPositionIndex, targetPositionIndex,
            self.numCluster, self.numIter,
            self.MySimulator.map_array.flatten().tolist(),
            self.MySimulator.map_width, self.MySimulator.map_height)

        # rearrange the targets by task allocation result, 2D to 3D list
        targetPosiQualisys3d = self.target2dTo3dOrdered(targetPositionQualisys, taskOrder)

        # initialize Finite State Machine for each agent
        for idxAgent in range(self.numAgent):
            self.listAgentFSMExp.append(AgentFSMExp(agentIdx=idxAgent, distanceThreshold=DISTANCE_THRESHOLD))
            # if targetPosiQualisys3d[idxAgent] = [], targetPosiQualisys3d[idxAgent][0] won't work
            targetPosition = list(chain.from_iterable(targetPosiQualisys3d[idxAgent][0:1]))
            # initialize FSM with the first target
            self.listAgentFSMExp[idxAgent].initFSM(targetPosition=targetPosition,
                                                   homePosition=self.agentHomeList[idxAgent])

        self.ax = self.MySimulator.create_realtime_plot()  # create a realtime plotting figure
        timeBegin = time.time()
        timeUsed = 0  # initialize the global time as 0
        while(timeUsed < timeEscape):
            tStart = time.time()

            # update the agent position
            agentPositionList = await self.updateStateMocap()

            # update dynamic obstacles
            if DYNAMIC_OBS_FLAG:
                # update the obstacle position
                obsPosiList = await self.updateObsMocap()

                # initialize the map as the static one
                self.MySimulator.map_array = self.mapArrayInitial.copy()

                # update the map
                for idx in range (len(obsPosiList)):
                    self.MySimulator.update_obs_map_by_center(obsPosiList[idx], obsSizeList[idx])

            # update targets by Finite State Machine
            targetPosiQualisys3d, missionFinishFlagList, endFlagList = self.updateTargetSet(agentPositionList, targetPosiQualisys3d)

            # update dynamic tasks
            if DYNAMIC_TASK_FLAG:
                # update task position
                dynTaskPosiNow = await self.updateTaskMocap()
                breakFlag = False  # True to break for-loops

                for idxAgent in range(len(targetPosiQualisys3d)):
                    for idx in range(len(targetPosiQualisys3d[idxAgent])):
                        # check which one is the dynamic task
                        if math.sqrt((dynTaskPosi[0]-targetPosiQualisys3d[idxAgent][idx][0])**2 + \
                                     (dynTaskPosi[1]-targetPosiQualisys3d[idxAgent][idx][1])**2) <= 1E-2:
                            # update the dynamic task position
                            targetPosiQualisys3d[idxAgent][idx] = copy.deepcopy(dynTaskPosiNow)
                            dynTaskPosi = copy.deepcopy(dynTaskPosiNow)
                            breakFlag = True
                            break
                    if breakFlag:
                        break

            # do the planning
            pathAllIndex, targetPosiQualisys3d, clusterCenter, timeAlgo_ms = self.runSolverOnce(
                agentPositionList, targetPosiQualisys3d, clusterCenter, missionFinishFlagList)

            # save trajectories as csv files
            positionTrajList = self.saveTraj(pathAllIndex, velocityAveList, timeBegin, endFlagList)

            # for visualization only
            targetPlotList = self.targetQualisys3dTo2d(targetPosiQualisys3d)

            # update the figure
            self.MySimulator.update_realtime_plot(positionTrajList, agentPositionList,
                                                  targetPlotList, self.ax, obsPosiList, obsSizeList)

            # plot the algorithm time
            timeStr = "Computation Time [ms]: " + str(timeAlgo_ms)
            plt.text(0.25, 0.9, timeStr, fontsize=14, transform=plt.gcf().transFigure)

            plt.pause(1E-9)
            timeSleep = max(0, 1/self.planningFrequency - time.time() + tStart)
            timeUsed = time.time() - timeBegin
            # print("Current Time [sec]: " + str(timeUsed))
            await asyncio.sleep(timeSleep)

    def runSolverOnce(self, agentPosition2d, targetPosiQualisys3d, clusterCenter, missionFinishFlagList):
        """
        Run the planning solver once.

        Input:
            agentPosition2d: 2D list for agents positions (in Qualisys coordinates), [[x0,y0,z0], [x1,y1,z1], ...]
            targetPosiQualisys3d: 3D list, each sub-list is the targets positions (in Qualisys coordinates) in an execution
                sequence for each agent, [  [[x0,y0,z0], [x1,y1,z1], ...] , [[x2,y2,z2], [x3,y3,z3], ...] , ... ]
            clusterCenter: 1D list, clusters centroids positions in map index, [x0,y0, x1,y1, x2,y2, ...]
            missionFinishFlagList: a 1D list of boolean,
                missionFinishFlagList[0] = True means that the first agent is either "Homing" or "End".
        """
        # transform qualisys coordinates (meter) to map index
        agentPositionIndex = self.MySimulator.qualisys_to_map_index_all(agentPosition2d)

        # if none of agents is either "Homing" or "End", do mission planning
        if not any(missionFinishFlagList):
            # 3D targets (in Qualisys) to 2D targets (in map index) with order inherited
            targetPosiIndex1d = self.targetQualisys3dTo1dIndex(targetPosiQualisys3d)

            t0 = time.time()
            # do mission planning
            pathAllIndex, taskOrder, clusterCenter, _, _ = DrMaMP.MissionPlanningIteratively(
                agentPositionIndex, targetPosiIndex1d, clusterCenter,
                self.numCluster, self.numIter,
                self.MySimulator.map_array.flatten().tolist(),
                self.MySimulator.map_width, self.MySimulator.map_height)
            
            # rearrange tasks here, the beginning of next iteration needs the task set in ordered
            targetPosiQualisys3d = self.targetRearrange3d(targetPosiQualisys3d, taskOrder)
        else:
            # initialize the path for all agents (eventually it's a 3D list)
            pathAllIndex = list()
            # initialize the task order for all agents (eventually it's a 2D list)
            taskOrder = list()
            # in this case, we don't need clusterCenter
            clusterCenter = list()

            # convert home positions to 1d map index, [x0,y0, x1,y1, x2,y2, ...]
            homePosi1d = self.MySimulator.qualisys_to_map_index_all(self.agentHomeList)
            t0 = time.time()

            for idxAgent in range(self.numAgent):
                # if this agent is still executing tasks, do mission planning for this agent
                # there is a chance that targetPosiQualisys3d[idxAgent] is [], but the state is not updated yet
                # So do mission planning only when the target set is not empty and the state is not "Homing"
                if (not missionFinishFlagList[idxAgent]) and (targetPosiQualisys3d[idxAgent]):
                    # convert the targets of this agent into 1d map index positions
                    targetPosiIndex2d = self.MySimulator.qualisys_to_map_index_all(targetPosiQualisys3d[idxAgent])
                    # NOTE: taskOrderThis is the local index of its tasks
                    # BUT this section of codes ASSUMES that there are only two agents
                    # so the local index is the global index
                    path2dThis, taskOrderThis = DrMaMP.SolveOneAgent(\
                        agentPositionIndex[2*idxAgent:2*idxAgent+2],
                        targetPosiIndex2d,
                        self.MySimulator.map_array.flatten().tolist(),
                        self.MySimulator.map_width, self.MySimulator.map_height)

                    pathAllIndex.append(path2dThis)
                    taskOrder.append(taskOrderThis)
                # if this agent is "Homing" or the target set is empty, do find path
                else:
                    path1dThis, _ = DrMaMP.FindPath(
                        agentPositionIndex[2*idxAgent:2*idxAgent+2],
                        homePosi1d[2*idxAgent:2*idxAgent+2],
                        self.MySimulator.map_array.flatten().tolist(),
                        self.MySimulator.map_width, self.MySimulator.map_height)

                    # pathThis is a 1D list of path nodes
                    pathAllIndex.append([path1dThis])
                    # Homing, no tasks
                    taskOrder.append([])

            # rearrange tasks here, the beginning of next iteration needs the task set in ordered
            targetPosiQualisys3d = self.targetRearrange3d(targetPosiQualisys3d, taskOrder)

        t1 = time.time()
        timeAlgo_ms = round((t1-t0)*1000, 2)  # milliseconds
        # print("Solver time used [sec]:" + str(t1 - t0))
        return pathAllIndex, targetPosiQualisys3d, clusterCenter, timeAlgo_ms

    def saveTraj(self, pathAllIndex, velocityAveList, timeBegin, endFlagList):
        """
        Save the desired trajectory as csv files. These csv files are for quadrotor Mambo to track.

        Input:
            pathAllIndex:
            velocityAveList: 1D list, each element is each agent's average velocity (m/s), [0.25, 0.3, ...]
            timeBegin: the time stamp where the whole planner begins
            endFlagList: a 1D list of boolean, True if this agent is "End"

        """
        # transform map array (index) to qualisys coordinates (meter)
        pathQualisysList = list()
        for idxAgent in range(self.numAgent):
            pathQualisysList.append(self.MySimulator.path_index_to_qualisys(pathAllIndex[idxAgent], self.heightFly))

        # generate position and velocity trajectories (as a motion planner)
        # t0 = time.time()
        dt = 0.1
        # generate trajectories
        timeNow = time.time()

        positionTrajList = list()
        for idxAgent in range(self.numAgent):
            # if this agent is "End", stop sending trajectories and it will land
            if not endFlagList[idxAgent]:
                # pathAllIndex[idxAgent] could be [] or [[], [px,py, ...], [px,py, ...]]
                # to check if it's empty, you have to slice [0:1] and flatten as a 1d list
                path = list(chain.from_iterable(pathAllIndex[idxAgent][0:1]))
                if path:
                    try:
                        # if the path has more than two nodes, do quadratic interpolation
                        if len(pathQualisysList[idxAgent]) > 2:
                            timeQueueVec, positionTraj, velocityTraj = discrete_path_to_time_traj(
                                pathQualisysList[idxAgent], dt, velocityAveList[idxAgent],
                                interp_kind='linear', velocity_flag=True, ini_velocity_zero_flag=False)
                        # if the path has two nodes, do linear interpolation
                        else:
                            timeQueueVec, positionTraj, velocityTraj = discrete_path_to_time_traj(
                                pathQualisysList[idxAgent], dt, velocityAveList[idxAgent],
                                interp_kind='linear', velocity_flag=True, ini_velocity_zero_flag=False)

                        # Mambo Tracking Controller uses the accumulated time, need to change the time trajectory here too
                        timeQueueVec = timeQueueVec + max(timeNow-timeBegin-1.0, 0)

                        # output trajectories as a CSV file
                        array_csv = np.vstack((timeQueueVec, np.array(positionTraj).T, np.array(velocityTraj).T))
                        timeName = time.strftime("%Y%m%d%H%M%S")
                        # filename_csv = os.path.expanduser("~") + "/Mambo-Tracking-Interface" + \
                        #             self.configDataList[idxAgent]["DIRECTORY_TRAJ"] + timeName + ".csv"
                        filename_csv = os.path.expanduser("~") + "/Mambo-Tracking-Interface-DrMaMP" + \
                                    self.configDataList[idxAgent]["DIRECTORY_TRAJ"] + timeName + ".csv"
                        np.savetxt(filename_csv, array_csv, delimiter=",")
                        positionTrajList.append(positionTraj)
                    except:
                        positionTrajList.append(list())
                else:
                    positionTrajList.append(list())

                    # timeQueueVec, positionTraj, velocityTraj = discrete_path_to_time_traj(
                    #     agentPosition2d, dt, velocityAveList[idxAgent], interp_kind='linear',
                    #     velocity_flag=True, ini_velocity_zero_flag=False)

                    # # Mambo Tracking Controller uses the accumulated time, need to change the time trajectory here too
                    # timeQueueVec = timeQueueVec + max(timeNow-timeBegin-1.0, 0)

                    # # output trajectories as a CSV file
                    # array_csv = np.vstack((timeQueueVec, positionTraj.T, velocityTraj.T))
                    # timeName = time.strftime("%Y%m%d%H%M%S")
                    # filename_csv = os.path.expanduser("~") + "/Mambo-Tracking-Interface" + self.configDataList[idxAgent]["DIRECTORY_TRAJ"] + timeName + ".csv"
                    # np.savetxt(filename_csv, array_csv, delimiter=",")
                    # positionTrajList.append(positionTraj)

        return positionTrajList

    def updateTargetSet(self, agentPosition: list, targetPosiQualisys3d: list):
        """
        Update the whole target set by updating Finite State Machine.

        Inputs:
            agentPosition: a 2D list, each sub-list is each agent's positions (in Qualisys), [[x0,y0,z0], [x1,y1,z1], ...]
            targetPosiQualisys3d: a 3D list, each sub-list is a target set of an agent.
                For example, targetPosiQualisys3d[1] = [[x0,y0,z0], [x1,y1,z1]] is for the second agent (Agent-1).

        Outputs:
            targetPosiQualisys3d: a 3D list, as same as the input targetPosiQualisys3d
            missionFinishFlagList: a 1D list of boolean,
                missionFinishFlagList[0] = True means that the first agent is either "Homing" or "End".
            endFlagList: a 1D list of boolean, True if this agent is "End"
        """
        # initialize a list for Homing flags of all agent
        missionFinishFlagList = list()
        # initialize endFlagList as empty
        endFlagList = list()

        for idxAgent in range(self.numAgent):
            # if targetPosiQualisys3d[idxAgent] = [], targetPosiQualisys3d[idxAgent][0] won't work
            targetPosition = list(chain.from_iterable(targetPosiQualisys3d[idxAgent][0:1]))

            # transit states
            _, targetFinishFlagThis = self.listAgentFSMExp[idxAgent].transition(
                agentPositionNow=agentPosition[idxAgent],
                targetPosition=targetPosition)

            # update targets positions set
            if targetFinishFlagThis:
                # if there are still targets, delete the first one; else, do nothing
                # if targetPosiQualisys3d[idxAgent] = [], targetPosiQualisys3d[idxAgent][0] won't work
                if targetPosiQualisys3d[idxAgent][0:1]:
                    del targetPosiQualisys3d[idxAgent][0]

            missionFinishFlag = (self.listAgentFSMExp[idxAgent].StateNow.stateName == "Homing") or \
                                (self.listAgentFSMExp[idxAgent].StateNow.stateName == "End")
            missionFinishFlagList.append(missionFinishFlag)

            endFlagList.append(self.listAgentFSMExp[idxAgent].StateNow.stateName == "End")
        return targetPosiQualisys3d, missionFinishFlagList, endFlagList

    def targetRearrange3d(self, targetPosiQualisys3d: list, taskOrder: list):
        """
        Rearrange the 3D targets set to a 3D targets set given the task execution sequence.

        Input:
            targetPosiQualisys3d: 3D list, each sub-list is the targets positions (in Qualisys coordinates) in an execution
                sequence for each agent, [  [[x0,y0,z0], [x1,y1,z1], ...] , [[x2,y2,z2], [x3,y3,z3], ...] , ... ]
            taskOrder: 2D list, each sub-list is the targets indices in an execution sequence
        
        Output:
            targetPosiQualisys3dNew: 3D list, same as the input targetPosiQualisys3d

        Example:
            targetPosiQualisys3d = [  [[x0,y0,z0], [x1,y1,z1]] , [[x2,y2,z2], [x3,y3,z3]]  ]
            taskOrder = [[1,0], [3,2]]
            targetPosiQualisys3d = self.targetRearrange3d(targetPosiQualisys3d, taskOrder)
            ==> targetPosiQualisys3dNew = [ [[x1,y1,z1], [x0,y0,z0]], [[x3,y3,z3], [x2,y2,z2]] ]
        """
        # first, convert 3d list to 2d, [[x0,y0,z0], [x1,y1,z1], [x2,y2,z2], [x3,y3,z3], ...]
        targetPosiQualisys2d = [target for targetSet in targetPosiQualisys3d for target in targetSet]

        targetPosiQualisys3dNew = [[] for _ in range(self.numAgent)]
        for idxAgent in range(self.numAgent):
            for idxtask in range(len(taskOrder[idxAgent])):
                taskId = taskOrder[idxAgent][idxtask]
                targetPosiQualisys3dNew[idxAgent].append(targetPosiQualisys2d[taskId])
        return targetPosiQualisys3dNew

    def target2dTo3dOrdered(self, targetPositionInput: list, taskOrder: list):
        """
        Rearrange the 2D targets set (in Qualisys coordinates) to a 3D targets set (in Qualisys coordinates)
        given the task execution sequence.

        Input:
            targetPositionInput: 2D list, [[x0,y0,z0], [x1,y1,z1], [x2,y2,z2], ...]
            taskOrder: 2D list, each sub-list is the targets indices in an execution sequence
        
        Output:
            targetPositionOutput: 3D list, each sub-list is the targets positions in an execution sequence
                for each agent

        Example:
            targetPositionInput = [[x0,y0,z0], [x1,y1,z1], [x2,y2,z2], [x3,y3,z3]]
            taskOrder = [[2,0], [1,3]]
            targetPositionOutput = self.target2dTo3dOrdered(targetPositionInput, taskOrder)
            ==> targetPositionOutput = [  [[x2,y2,z2], [x0,y0,z0]] , [[x1,y1,z1], [x3,y3,z3]]  ]
        """
        targetPositionOutput = [[] for _ in range(self.numAgent)]
        for idxAgent in range(self.numAgent):
            for idxTask in range(len(taskOrder[idxAgent])):
                taskId = taskOrder[idxAgent][idxTask]
                targetPositionOutput[idxAgent].append(targetPositionInput[taskId])
        return targetPositionOutput

    def targetQualisys3dTo1dIndex(self, targetPosiQualisys3d: list):
        """
        Transform a 3D targets positions (in Qualisys coordinates) list to a 1D targets positions (in map index) list.

        Input:
            targetPosiQualisys3d: 3D list, each sub-list is the targets positions (in Qualisys coordinates) in an execution
                sequence for each agent, [  [[x0,y0,z0], [x1,y1,z1], ...] , [[x2,y2,z2], [x3,y3,z3], ...] , ... ]

        Output:
            targetPosiIndex1d: 1D list, all the targets positions (in map index), [x0,y0, x1,y1, x2,y2, x3,y3, ...]

        Example:
            targetPosiQualisys3d = [  [[x2,y2,z2], [x0,y0,z0]] , [[x1,y1,z1], [x3,y3,z3]]  ]
            targetPosiIndex1d = self.targetQualisys3dTo1dIndex(targetPosiQualisys3d)
            ==> targetPosiIndex1d =  [x2,y2, x0,y0, x1,y1, x3,y3, ...]
            NOTE: {xi,yi} in targetPosiIndex1d doesn't equal to {xi,yi} in targetPosiQualisys3d
        """
        targetPosiIndex1d = list()
        for idxAgent in range(self.numAgent):
            targetPosiIndex1d.extend(self.MySimulator.qualisys_to_map_index_all(targetPosiQualisys3d[idxAgent]))
        return targetPosiIndex1d

    def targetQualisys3dTo2d(self, targetPosition3d: list):
        """
        Transform a 3D targets positions (in Qualisys coordinates) list to a 2D targets positions (in Qualisys coordinates) list.
        Each sub-list is the position for each target. The order of output list does not matter. This is only for visualization.

        Input:
            targetPosition3d: 3D list, each sub-list is the targets positions (in Qualisys coordinates) in an execution
                sequence for each agent, [  [[x0,y0,z0], [x1,y1,z1], ...] , [[x2,y2,z2], [x3,y3,z3], ...] , ... ]

        Output:
            targetPosition2d: 2D list, each sub-list is the target position (in Qualisys coordinates) for each target,
                [[x0,y0,z0], [x1,y1,z1], [x2,y2,z2], [x3,y3,z3], ...]

        Example:
            targetPosition3d = [  [[x2,y2,z2], [x0,y0,z0], [x4,y4,z4]] , [[x1,y1,z1], [x3,y3,z3]]  ]
            targetPosition2d = self.targetQualisys3dTo2d(targetPosition3d)
            ==> targetPosition2d = [[x2,y2,z2], [x0,y0,z0], [x4,y4,z4], [x1,y1,z1], [x3,y3,z3]]
        """
        targetPosition2d = [target for targetSet in targetPosition3d for target in targetSet]
        return targetPosition2d

    async def updateStateMocap(self):
        """
        Update the positions from motion capture system.

        Output:
            positionList: 2D list for agents positions (in Qualisys coordinates),
                [[x0,y0,z0], [x1,y1,z1], ...]
        """
        positionList = list()
        for idx in range(self.numAgent):
            msg = await self.protocolStateList[idx].recvfrom()
            positionList.append(np.frombuffer(msg, dtype=np.float64).tolist())
        return positionList

    async def updateObsMocap(self):
        """
        Update the obstacle positions from motion capture system.

        Output:
            positionList: 2D list for obstacles positions (in Qualisys coordinates),
                [[x0,y0,z0], [x1,y1,z1], ...]
        """
        positionList = list()
        for idx in range(len(self.protocolObsList)):
            msg = await self.protocolObsList[idx].recvfrom()
            positionList.append(np.frombuffer(msg, dtype=np.float64).tolist())
        return positionList

    async def updateTaskMocap(self):
        """
        Update one task position from motion capture system.

        Output:
            position: 1D list for obstacles positions (in Qualisys coordinates), [x0, y0, self.heightFly]
        """
        msg = await self.protocolTask01.recvfrom()
        position = np.frombuffer(msg, dtype=np.float64).tolist()
        position[2] = self.heightFly
        return position

    def generateTargetManually(self, case_num: int):
        """
        Generate some targets in Qualisys coordinates (meter) manually.

        Output:
            targetPosition: [[x0,y0,z0], [x1,y1,z1], ...]
        """
        if case_num == 5:
            targetPosition = [[1.18, 0.0, self.heightFly],   [1.7, 0.5, self.heightFly],
                              [1.8, 0.9, self.heightFly],    [0.4, 1.1, self.heightFly],
                              [1.9, -0.9, self.heightFly],   [1.9, -1.4, self.heightFly],
                              [0.19, -0.46, self.heightFly], [0.23, 0.17, self.heightFly],
                              [1.76, -0.35, self.heightFly], [0.18, -1.52, self.heightFly]]
        elif case_num == 6:
            targetPosition = [[1.18, 0.0, self.heightFly],   [1.7, 0.5, self.heightFly],
                              [1.8, 0.9, self.heightFly],    [0.4, 1.1, self.heightFly],
                              [1.9, -0.9, self.heightFly],   [1.9, -1.4, self.heightFly],
                              [0.19, -0.46, self.heightFly], [0.23, 0.17, self.heightFly],
                              [1.76, -0.35, self.heightFly], [0.18, -1.52, self.heightFly]]
        elif case_num == 7:
            targetPosition = [[1.18, 0.0, self.heightFly],   [1.8, 0.9, self.heightFly],  
                              [0.4, 1.1, self.heightFly],    [1.9, -0.9, self.heightFly],
                              [1.9, -1.4, self.heightFly],   [0.19, -0.46, self.heightFly],
                              [0.23, 0.17, self.heightFly],  [1.76, -0.35, self.heightFly],
                              [0.18, -1.52, self.heightFly]]
        else:
            pass
        return targetPosition
