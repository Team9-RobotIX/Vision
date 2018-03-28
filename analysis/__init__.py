from analysis import WallDetector as WD
from analysis import PathDetector as PD
from analysis import TargetDetector as TD
from analysis import RobotDetector as RD
from analysis import Camera as C
from analysis import Robot as R

def WallDetector(camera):
    return WD.WallDetector(camera)

def PathDetector(robot, camera):
    return PD.PathDetector(robot, camera)

def TargetDetector(camera):
    return TD.TargetDetector(camera)

def RobotDetector(camera):
    return RD.RobotDetector(camera)

def Camera():
    return C.Camera()

def Robot(fl,fu,bl,bu,l,w):
    return R.Robot(fl,fu,bl,bu,l,w)
