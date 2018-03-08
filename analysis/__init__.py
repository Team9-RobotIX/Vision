from analysis import WallDetector as WD
from analysis import PathDetector as PD
from analysis import TargetDetector as TD
from analysis import RobotDetector as RD
from analysis import Camera as C

def WallDetector(camera):
    return WD.WallDetector(camera)

def PathDetector():
    return PD.PathDetector()

def TargetDetector(camera):
    return TD.TargetDetector(camera)

def RobotDetector(camera):
    return RD.RobotDetector(camera)

def Camera():
    return C.Camera()
