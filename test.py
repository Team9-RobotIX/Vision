import video
import cv2
vid = cv2.VideoCapture(0)
walls = video.WallDetector(vid)
paths = video.PathDetector(walls.getWalls(),20,20)
