import numpy as np
import cv2

class RobotDetector:

    def __init__(self,camera):
        self.camera = camera
        self.enableSettings()

    def getSize(self,robot):
        self.enableSettings()
        image = self.camera.getFrame()
        mask = cv2.inRange(image, robot.frontLower, robot.frontUpper)
        cv2.imshow('m',mask)
        cv2.imshow('i',image)
        cv2.waitKey(1000)
        cv2.destroyAllWindows()
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel,iterations=3)
        img, blueContour, hier = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        x,y,w,h = cv2.boundingRect(blueContour[0])
        return (h*robot.length, w*robot.width)

    def findFront(self,robot,image):
        mask = cv2.inRange(image, robot.frontLower,robot.frontUpper)
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel,iterations=3)
        M = cv2.moments(mask)
        if M['m00'] == 0:
            M['m00'] = 0.0000000001
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return (cx,cy)

    def findBack(self,robot,image):
        mask = cv2.inRange(image, robot.backLower,robot.backUpper)
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel,iterations=3)

        M = cv2.moments(mask)
        if M['m00'] == 0:
            M['m00'] = 0.0000000001
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        return (cx,cy)


    def center(self,image,robot,newFrame=False):
        if newFrame:
            self.enableSettings()
            image = self.camera.getFrame()
        return np.mean(np.array([self.findFront(robot,image), self.findBack(robot,image)]), axis=0, dtype=int)

    def orientation(self,image,robot,newFrame=False):
        if newFrame:
            self.enableSettings()
            image = self.camera.getFrame()
        (fx, fy) = self.findFront(robot,image)
        (bx, by) = self.findBack(robot,image)
        vector = (fx - bx, fy - by)
        angle = np.arctan2(-vector[1], vector[0])
        return angle

    def enableSettings(self):
        settings = [(3,800.0),
                    (4,800.0),
                    (5,30.0),
                    (9,0.0),
                    (10,0.48627451062202454),
                    (11,0.125490203499794),
                    (12,0.15294118225574493),
                    (14,0.05098039284348488)]
        self.camera.enableSettings(settings)
