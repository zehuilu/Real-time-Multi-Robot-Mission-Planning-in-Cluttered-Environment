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
DISTANCE_THRESHOLD = 0.25
BUFFER_BOUNDARY = 0.15
CASE_NUM = 5


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
            directoryDelete = os.path.expanduser("~") + "/Mambo-Tracking-Interface" + \
                              self.configDataList[idx]["DIRECTORY_TRAJ"] + "*"
            csv_helper.remove_traj_ref_lib(directoryDelete)

        timeEscape = 100  # shut down the iteration after this time [sec]

        # create a customized UDP protocol for subscribing states from mocap
        loopState01 = asyncio.get_running_loop()
        transportState01, protocolState01 = await loopState01.create_datagram_endpoint(
            UdpProtocol, local_addr=self.stateAddressList[0], remote_addr=None)

        loopState02 = asyncio.get_running_loop()
        transportState02, protocolState02 = await loopState02.create_datagram_endpoint(
            UdpProtocol, local_addr=self.stateAddressList[1], remote_addr=None)

        self.transportStateList = [transportState01, transportState02]
        self.protocolStateList = [protocolState01, protocolState02]

        # create a customized UDP protocol for subscribing obstacles positions from mocap
        loopObs = asyncio.get_running_loop()
        self.transportObs, self.protocolObs = await loopObs.create_datagram_endpoint(
            UdpProtocol, local_addr=self.serverAddressObs, remote_addr=None)

        # get the agent home position
        self.agentHomeList = await self.updateStateMocap()

        self.heightFly = 0.8  # a constant fly height in meter
        # generate a scenario
        # generate targets manually
        targetPositionQualisys = self.generateTargetManually(CASE_NUM)
        # generate obstacles manually
        self.MySimulator.generate_obs_manually(CASE_NUM)

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

        # True if there is no target unfinished
        targetAllFinishFlag = False

        self.ax = self.MySimulator.create_realtime_plot()  # create a realtime plotting figure
        timeBegin = time.time()
        timeUsed = 0  # initialize the global time as 0
        while(timeUsed < timeEscape):
            tStart = time.time()

            # update the agent position
            agentPositionList = await self.updateStateMocap()

            # update targets by Finite State Machine
            targetPosiQualisys3d, targetAllFinishFlag, endFlag = self.updateTargetSet(
                agentPositionList, targetPosiQualisys3d, targetAllFinishFlag)





            print("Agent 1 target position: ", self.listAgentFSMExp[1].targetPosition)
            print("Agent 1 state: ", self.listAgentFSMExp[1].StateNow.stateName)





            # do the planning
            pathAllIndex, targetPosiQualisys3d, clusterCenter, timeAlgo_ms = self.runSolverOnce(
                agentPositionList, targetPosiQualisys3d, clusterCenter, targetAllFinishFlag)

            # save trajectories as csv files
            positionTrajList = self.saveTraj(pathAllIndex, velocityAveList, timeBegin)

            # for visualization only
            targetPlotList = self.targetQualisys3dTo2d(targetPosiQualisys3d)

            # update the figure
            self.MySimulator.update_realtime_plot(
                positionTrajList, agentPositionList, targetPlotList, self.ax)

            # plot the algorithm time
            timeStr = "Computation Time [ms]: " + str(timeAlgo_ms)
            plt.text(0.25, 0.9, timeStr, fontsize=14, transform=plt.gcf().transFigure)

            plt.pause(1E-9)
            timeSleep = max(0, 1/self.planningFrequency - time.time() + tStart)
            timeUsed = time.time() - timeBegin
            # print("Current Time [sec]: " + str(timeUsed))
            await asyncio.sleep(timeSleep)

    def runSolverOnce(self, agentPosition2d, targetPosiQualisys3d, clusterCenter, targetAllFinishFlag):
        """
        Run the planning solver once.

        Input:
            agentPosition2d: 2D list for agents positions (in Qualisys coordinates), [[x0,y0,z0], [x1,y1,z1], ...]
            targetPosiQualisys3d: 3D list, each sub-list is the targets positions (in Qualisys coordinates) in an execution
                sequence for each agent, [  [[x0,y0,z0], [x1,y1,z1], ...] , [[x2,y2,z2], [x3,y3,z3], ...] , ... ]
            clusterCenter: 1D list, clusters centroids positions in map index, [x0,y0, x1,y1, x2,y2, ...]
            targetAllFinishFlag: bool, True if there is no target unfinished

        """
        # transform qualisys coordinates (meter) to map index
        agentPositionIndex = self.MySimulator.qualisys_to_map_index_all(agentPosition2d)

        # if not every target is finished, do mission planning
        if not targetAllFinishFlag:
            # 3D targets (in Qualisys) to 2D targets (in map index) with order inherited
            targetPosiIndex1d = self.targetQualisys3dTo1dIndex(targetPosiQualisys3d)

            t0 = time.time()
            # do mission planning
            pathAllIndex, taskOrder, clusterCenter, _, _ = DrMaMP.MissionPlanningIteratively(
                agentPositionIndex, targetPosiIndex1d, clusterCenter,
                self.numCluster, self.numIter,
                self.MySimulator.map_array.flatten().tolist(),
                self.MySimulator.map_width, self.MySimulator.map_height)
            
            # rearrange tasks here!!!!!!!!!!!!!!!!!!!
            targetPosiQualisys3d = self.targetRearrange3d(targetPosiQualisys3d, taskOrder)
        # if every target is finished, do path planning to back home positions
        else:
            # convert home positions to 1d map index, [x0,y0, x1,y1, x2,y2, ...]
            homePosi1d = self.MySimulator.qualisys_to_map_index_all(self.agentHomeList)

            # convert to 2d map index, [[x0,y0], [x1,y1], [x2,y2], ...]
            homePosi2d = np.array(homePosi1d).reshape(-1, 2).tolist()

            t0 = time.time()
            # go back home, do path planning
            pathAllIndex = DrMaMP.PathPlanningMultiAgent(
                agentPositionIndex, homePosi2d,
                self.MySimulator.map_array.flatten().tolist(),
                self.MySimulator.map_width, self.MySimulator.map_height)

        t1 = time.time()
        timeAlgo_ms = round((t1-t0)*1000, 2)  # milliseconds
        # print("Solver time used [sec]:" + str(t1 - t0))

        return pathAllIndex, targetPosiQualisys3d, clusterCenter, timeAlgo_ms

    def saveTraj(self, pathAllIndex, velocityAveList, timeBegin):
        """
        Save the desired trajectory as csv files. These csv files are for quadrotor Mambo to track.

        Input:
            pathAllIndex:
            velocityAveList: 1D list, each element is each agent's average velocity (m/s), [0.25, 0.3, ...]
            timeBegin: the time stamp where the whole planner begins

        """
        # transform map array (index) to qualisys coordinates (meter)
        pathQualisysList = list()
        for idx in range(len(pathAllIndex)):
            pathQualisysList.append(self.MySimulator.path_index_to_qualisys(pathAllIndex[idx], self.heightFly))

        # generate position and velocity trajectories (as a motion planner)
        # t0 = time.time()
        dt = 0.1
        # generate trajectories
        timeNow = time.time()

        positionTrajList = list()
        for idx in range(len(pathQualisysList)):
            if pathQualisysList[idx]:
                timeQueueVec, positionTraj, velocityTraj = discrete_path_to_time_traj(
                    pathQualisysList[idx], dt, velocityAveList[idx], interp_kind='linear',
                    velocity_flag=True, ini_velocity_zero_flag=False)

                # Mambo Tracking Controller uses the accumulated time, need to change the time trajectory here too
                timeQueueVec = timeQueueVec + max(timeNow-timeBegin-1.0, 0)

                # output trajectories as a CSV file
                array_csv = np.vstack((timeQueueVec, np.array(positionTraj).T, np.array(velocityTraj).T))
                timeName = time.strftime("%Y%m%d%H%M%S")
                filename_csv = os.path.expanduser("~") + "/Mambo-Tracking-Interface" + self.configDataList[idx]["DIRECTORY_TRAJ"] + timeName + ".csv"
                np.savetxt(filename_csv, array_csv, delimiter=",")
                positionTrajList.append(positionTraj)
            else:
                positionTrajList.append(list())

                # timeQueueVec, positionTraj, velocityTraj = discrete_path_to_time_traj(
                #     agentPosition2d, dt, velocityAveList[idx], interp_kind='linear',
                #     velocity_flag=True, ini_velocity_zero_flag=False)

                # # Mambo Tracking Controller uses the accumulated time, need to change the time trajectory here too
                # timeQueueVec = timeQueueVec + max(timeNow-timeBegin-1.0, 0)

                # # output trajectories as a CSV file
                # array_csv = np.vstack((timeQueueVec, positionTraj.T, velocityTraj.T))
                # timeName = time.strftime("%Y%m%d%H%M%S")
                # filename_csv = os.path.expanduser("~") + "/Mambo-Tracking-Interface" + self.configDataList[idx]["DIRECTORY_TRAJ"] + timeName + ".csv"
                # np.savetxt(filename_csv, array_csv, delimiter=",")
                # positionTrajList.append(positionTraj)

        return positionTrajList

    def updateTargetSet(self, agentPosition: list, targetPosiQualisys3d: list, targetAllFinishFlag: bool):
        """
        Update the whole target set by updating Finite State Machine.

        Inputs:
            agentPosition: a 2D list, each sub-list is each agent's positions (in Qualisys), [[x0,y0,z0], [x1,y1,z1], ...]
            targetPosiQualisys3d: a 3D list, each sub-list is a target set of an agent.
                For example, targetPosiQualisys3d[1] = [[x0,y0,z0], [x1,y1,z1]] is for the second agent (Agent-1).
            targetAllFinishFlag: bool, True if there is no target unfinished

        Outputs:
            targetPosiQualisys3d: a 3D list, as same as the input targetPosiQualisys3d
            endFlag: boolean, True if all agents states are "End"
        """
        # initialize the output
        endFlagList = list()
        # initialize this flag
        targetAllFinishFlag = False

        for idxAgent in range(self.numAgent):
            # if targetPosiQualisys3d[idxAgent] = [], targetPosiQualisys3d[idxAgent][0] won't work
            targetPosition = list(chain.from_iterable(targetPosiQualisys3d[idxAgent][0:1]))

            # transit states
            _, targetFinishFlagThis = self.listAgentFSMExp[idxAgent].transition(
                agentPositionNow=agentPosition[idxAgent],
                targetPosition=targetPosition,
                targetAllFinishFlag=targetAllFinishFlag)

            # update targets positions set
            if targetFinishFlagThis:
                # if there are still targets, delete the first one; else, do nothing
                if targetPosiQualisys3d[idxAgent][0]:
                    del targetPosiQualisys3d[idxAgent][0]

            # if state is "End", this agent completed all assigned tasks
            if self.listAgentFSMExp[idxAgent].StateNow.stateName == "End":
                endFlagList.append(True)
            else:
                endFlagList.append(False)

        # test if updated target set is empty [[], [], ...]
        try:
            testMat = np.array(targetPosiQualisys3d, dtype=object)
            size = testMat.size
            # if size is zero, means targetPosiQualisys3d is empty, [[], [], ...]
            if size < 0.5:
                targetAllFinishFlag = True
        except:
            pass

        # True if all agents states are "End"
        endFlag = all(endFlagList)
        return targetPosiQualisys3d, targetAllFinishFlag, endFlag





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
        """
        msg = await self.protocolObs.recvfrom()
        positionObs = np.frombuffer(msg, dtype=np.float64)
        # print("positionObs")
        # print(positionObs.tolist())
        return positionObs.tolist()

    def generateTargetManually(self, case_num: int):
        """
        Generate some targets in Qualisys coordinates (meter) manually.

        Output:
            targetPosition: [[x0,y0,z0], [x1,y1,z1], ...]
        """
        if case_num == 5:
            targetPosition = [[1.18, 0.0, self.heightFly], [1.7, 0.5, self.heightFly],
                                [1.8, 0.9, self.heightFly], [0.4, 0.9, self.heightFly],
                                [1.8, -0.9, self.heightFly], [1.86, -1.8, self.heightFly],
                                [1.9, -1.4, self.heightFly]]
        elif case_num == 6:
            targetPosition = [[1.0, 0.0, self.heightFly], [1.7, 0.5, self.heightFly],
                                [1.5, 0.9, self.heightFly], [0.4, 0.9, self.heightFly],
                                [1.8, -0.5, self.heightFly], [1.86, -1.8, self.heightFly],
                                [1.9, -1.4, self.heightFly]]
        else:
            pass
        return targetPosition
