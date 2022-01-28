#!/usr/bin/env python3
import math
import copy

class State:
    stateName: str  # the string for this state name

    def __init__(self, stateName="State"):
        """
        Initialize a State object.
        """
        self.stateName = stateName

    def printState(self):
        """
        Print the state name.
        """
        print("State: " + self.stateName)

class Unassigned(State):
    def __init__(self, stateName="Unassigned"):
        """
        Initialize an Unassigned State object.
        """
        State.__init__(self, stateName=stateName)

class Assigned(State):
    def __init__(self, stateName="Assigned"):
        """
        Initialize an Assigned State object.
        """
        State.__init__(self, stateName=stateName)

class Completed(State):
    def __init__(self, stateName="Completed"):
        """
        Initialize a Completed State object.
        """
        State.__init__(self, stateName=stateName)

class End(State):
    def __init__(self, stateName="End"):
        """
        Initialize an End State object.
        """
        State.__init__(self, stateName=stateName)

class AgentFSM:
    agentIdx: int  # the index for this agent
    distanceThreshold: float  # when distance between A and B < this number, we say A and B have same position
    StatesPool: dict  # a dictionary for all included State objects
    targetPosition: list  # a 1d list for current target position, [px, py]


    def __init__(self, agentIdx=0, distanceThreshold=1.414):
        """
        Initialize a Agent Finite State Machine object.

        Input:
            agentIdx: int, an integer for the agent index
            distanceThreshold: float, when distance between A and B < distanceThreshold, we say A and B have same position
        """
        self.agentIdx = agentIdx
        self.distanceThreshold = distanceThreshold
        self.StatesPool = {"Unassigned": Unassigned(),
                           "Assigned": Assigned(),
                           "Completed": Completed(),
                           "End": End()}
        self.StateNow = self.StatesPool["Unassigned"]
    
    def initFSM(self, targetPosition: list):
        """
        Initialize the Agent Finite State Machine by a target position.

        Input:
            targetPosition: 1D list, a target position that need to be visited, [x0, y0]. If no target, empty list.
        """
        self.StateNow = self.StatesPool["Unassigned"]
        self.targetPosition = copy.deepcopy(targetPosition)

    def transition(self, agentPositionNow: list, targetPosition: list):
        """
        Make state transition based on the current agent position and target position.

        Input:
            agentPositionNow: 1D list, current agent position, [x0, y0]
            targetPosition: 1D list, current target position, [px, py]

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
            # if not, leave it
            else:
                self.targetPosition = list()
                stateName = "Unassigned"
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
        else:
            Exception("AgentFSM only supports 3 states: Unassigned, Assigned, Completed!")

        # update the state
        self.StateNow = self.StatesPool[stateName]
        return stateName, targetFinishFlag
