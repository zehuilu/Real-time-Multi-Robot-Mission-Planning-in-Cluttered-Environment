#!/usr/bin/env python3
import pathmagic
with pathmagic.context():
    from AgentFSM import AgentFSM


if __name__ == "__main__":
    # initialize the Agent Finite State Machine
    MyAgentFSM = AgentFSM(agentIdx=0, distanceThreshold=1.414)

    # set up the targets
    # targetSetTotal = [5,0]
    targetSetTotal = [3,0, 7,0, 10,0]

    # initialize the FSM with a set of targets
    MyAgentFSM.initFSM(targetSetTotal=targetSetTotal)

    # initialize the agent position
    agentPositionNow = [0,0]

    horizon = 20
    for idx in range(horizon):
        # print information
        print("Step: " + str(idx))
        print("Current State: " + MyAgentFSM.StateNow.stateName)
        print("Current Position: [" + str(agentPositionNow[0]) + ", " + str(agentPositionNow[1]) + "]")
        print("Current target index: " + str(MyAgentFSM.targetIdxNow))

        # reset the target set for testing
        if idx == 8:
            targetSetTotal = [13,0]

        # transit states
        MyAgentFSM.transition(agentPositionNow=agentPositionNow, targetSetTotal=targetSetTotal)

        # move the agent when Assigned or Completed
        if MyAgentFSM.StateNow.stateName != "End" and MyAgentFSM.StateNow.stateName != "Unassigned":
            agentPositionNow = [agentPositionNow[0]+1, 0]
