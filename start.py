import analysis
import planning
import cv2

#planner = planning.Planner()
controller = planning.Controller()
controller.run()
#controller.sendInstructions()
#vid = cv2.VideoCapture(0)
#cam = analysis.Camera()
#walls = analysis.WallDetector(cam)
#targets = analysis.TargetDetector(vid,[])
#paths = analysis.PathDetector()
#robot = analysis.RobotDetector(vid,[])
#while(vid.isOpened()):
#    ret, frame = vid.read()
#
#    cv2.imshow('frame',frame)
#    if cv2.waitKey(1) & 0xFF == ord('q'):
#        break
