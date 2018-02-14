from analysis import WallDetector as WD
from analysis import PathDetector as PD
from analysis import TargetDetector as TD

def WallDetector(vf):
    return WD.WallDetector(vf)

def PathDetector():
    return PD.PathDetector()

def TargetDetector(vf):
    return TD.TargetDetector(vf)
