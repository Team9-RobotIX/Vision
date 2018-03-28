from planning import Planner as PL
from planning import Tracker as TR
from planning import Controller as CT

def Planner(robot, cam):
    return PL.Planner(robot, cam)

def Tracker(rd, robot):
    return TR.Tracker(rd, robot)

def Controller(robots):
    return CT.Controller(robots)
