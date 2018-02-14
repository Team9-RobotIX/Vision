import analysis
import cv2
vid = cv2.VideoCapture(0)
#walls = analysis.WallDetector(vid)
targets = analysis.TargetDetector(vid)
#paths = analysis.PathDetector()
#while(vid.isOpened()):
#    ret, frame = vid.read()
#
#    cv2.imshow('frame',frame)
#    if cv2.waitKey(1) & 0xFF == ord('q'):
#        break
