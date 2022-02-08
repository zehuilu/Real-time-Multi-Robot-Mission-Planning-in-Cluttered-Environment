#!/usr/bin/env python3
import math
import copy
import pathmagic
with pathmagic.context():
    import AgentFSM


class Homing(AgentFSM.State):
    def __init__(self, stateName="Homing"):
        """
        Initialize a Homing State object.
        """
        super(Homing, self).__init__(stateName=stateName)

class End(AgentFSM.State):
    def __init__(self, stateName="End"):
        """
        Initialize an End State object.
        """
        super(End, self).__init__(stateName=stateName)


class AgentFSMExp(AgentFSM.AgentFSM):
    agentIdx: int  # the index for this agent
    distanceThreshold: float  # when distance between A and B < this number, we say A and B have same position
    StatesPool: dict  # a dictionary for all included State objects
    targetPosition: list  # 1D list for current target position (in Qualisys coordinates), [px, py, z0]
    homePosition: list  # 1D list, home position (in Qualisys coordinates), [x0,y0,z0]

    def __init__(self, agentIdx=0, distanceThreshold=0.1):
        """
        Initialize a Agent Finite State Machine object.

        Input:
            agentIdx: int, an integer for the agent index
            distanceThreshold: float, when distance between A and B < distanceThreshold, we say A and B have same position
        """
        super(AgentFSMExp, self).__init__(agentIdx, distanceThreshold)
        self.StatesPool["Homing"] = Homing()
        self.StatesPool["End"] = End()

    def initFSM(self, targetPosition: list, homePosition: list):
        """
        Initialize the Agent Finite State Machine by a target position.

        Input:
            targetPosition: 1D list, a target position (in Qualisys coordinates) that need to be visited, [x0, y0, z0]. If no target, empty list.
            homePosition: 1D list, home position (in Qualisys coordinates), [x0,y0,z0]
        """
        self.StateNow = self.StatesPool["Unassigned"]
        self.targetPosition = copy.deepcopy(targetPosition)
        self.homePosition = copy.deepcopy(homePosition)

    def transition(self, agentPositionNow: list, targetPosition: list, targetAllFinishFlag: bool):
        """
        Make state transition based on the current agent position and target position.
        NOTE: this function ignores the distance in height (z-axis).

        Input:
            agentPositionNow: 1D list, current agent position (in Qualisys coordinates), [x0,y0,z0]
            targetPosition: 1D list, current target position (in Qualisys coordinates), [px, py, z0]
            targetAllFinishFlag: bool, True if all agents finish all the targets

        Output:
            stateName: str, the state name after the transition
            targetFinishFlag: True if self.targetPosition is finished
        """
        if self.StateNow.stateName == "Completed":
            self.targetPosition = copy.deepcopy(targetPosition)
            stateName = "Unassigned"
            targetFinishFlag = False
        elif self.StateNow.stateName == "Unassigned":
            # if there exists a new target, assign it
            if targetPosition:
                self.targetPosition = copy.deepcopy(targetPosition)
                stateName = "Assigned"
            # if not, and if not all targets are visited, "Unassigned"
            elif not targetAllFinishFlag:
                self.targetPosition = list()
                stateName = "Unassigned"
            # if no new target for this agent, and all targets are visited, "Homing"
            else:
                stateName = "Homing"
            targetFinishFlag = False
        elif self.StateNow.stateName == "Assigned":
            self.targetPosition = copy.deepcopy(targetPosition)
            targetFinishFlag = False
            if self.targetPosition:
                # compute the distance between the current agent position and current target position
                distance = math.sqrt(pow(agentPositionNow[0]-self.targetPosition[0], 2) + pow(agentPositionNow[1]-self.targetPosition[1], 2))
                # no finish current target, don't change it
                if distance > self.distanceThreshold:
                    stateName = self.StateNow.stateName
                else:
                    stateName = "Completed"
                    targetFinishFlag = True
            else:
                stateName = "Unassigned"
        elif self.StateNow.stateName == "Homing":
            # compute the distance between the current agent position and its home position
            distance = math.sqrt(pow(agentPositionNow[0]-self.homePosition[0], 2) + pow(agentPositionNow[1]-self.homePosition[1], 2))
            if distance > self.distanceThreshold:
                stateName = self.StateNow.stateName
            else:
                stateName = "End"
            targetFinishFlag = True
        elif self.StateNow.stateName == "End":
            # if State == End, keep it
            stateName = self.StateNow.stateName
        else:
            Exception("AgentFSM only supports 5 states: Unassigned, Assigned, Completed, Homing, End!")

        # update the state
        self.StateNow = self.StatesPool[stateName]
        return stateName, targetFinishFlag
