from planning import Planner as PL
from planning import Tracker as TR
from planning import Controller as CT
from planning import Dispatch as D
from planning import Manager as M

def Planner(robot, cam):
    return PL.Planner(robot, cam)

def Tracker(rd, robot, disp):
    return TR.Tracker(rd, robot, disp)

def Controller(robots):
    return CT.Controller(robots)

def Dispatch():
    return D.Dispatch()

def Manager(robots, camera):
    return M.Manager(robots, camera)
