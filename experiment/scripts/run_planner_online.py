#!/usr/bin/env python3
import sys
import asyncio
import pathmagic
with pathmagic.context():
    from PlannerMocap import PlannerMocap


if __name__ == "__main__":
    # load mambo index from command line arguments
    if len(sys.argv) == 2:
        mambo_idx = sys.argv[1]
    else:
        mambo_idx = 1

    # initialize a planner with Motion Capture System
    MyPlannerMocap = PlannerMocap(mambo_idx)

    # run the planner online
    asyncio.run(MyPlannerMocap.run_planner())
    asyncio.ensure_future(MyPlannerMocap.run_planner())
