from analysis import WallDetector
from analysis import PathDetector
import cv2
vid = cv2.VideoCapture(0)
walls = WallDetector.WallDetector(vid)
wall = walls.getWalls()
frame = walls.getFrame()
paths = PathDetector.PathDetector(vid,frame,wall,40,40)
#while(vid.isOpened()):
#    ret, frame = vid.read()
#
#    cv2.imshow('frame',frame)
#    if cv2.waitKey(1) & 0xFF == ord('q'):
#        break
