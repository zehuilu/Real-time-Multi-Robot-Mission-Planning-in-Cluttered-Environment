#!/usr/bin/env python3
import asyncio
import pathmagic
with pathmagic.context():
    from PlannerMocapMultiAgent import PlannerMocapMultiAgent


if __name__ == "__main__":
    # initialize a planner with Motion Capture System for multiple agents
    MyPlannerMocap = PlannerMocapMultiAgent()

    # run the planner online
    asyncio.run(MyPlannerMocap.runPlanner())
    asyncio.ensure_future(MyPlannerMocap.runPlanner())
