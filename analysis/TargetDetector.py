import numpy as np
import cv2

class Target:

    name = None

    color = None
    tolerance = None
    contour = None
    center = None

    def __init__(self, n, col, tol):
        self.name = n
        self.color = col
        self.tolerance = tol

class TargetDetector:

    videoFeed = None
    targets = None

    mask = None

    def __init__(self, vf):
        self.videoFeed = vf
        self.targets = self.createTargets()
        self.findTargets()
        self.mask = self.createMask()
        cv2.imshow('MASK', self.mask)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()

    def createTargets(self):
        targets = [
            ('RED',(10,20,240),30),
            ('GREEN',(90,170,10),40),
            ('BLUE',(245,190,10),30)
        ]
        targetObjects = []
        for target in targets:
            targetObjects.append(Target(target[0],np.array(target[1]),target[2]))
        return targetObjects

    def createMask(self):
        ret, frame = self.videoFeed.read()
        shape = frame.shape
        mask = np.zeros((shape[0],shape[1]), np.uint8)
        for target in self.targets:
            cv2.fillPoly(mask, target.contour,255)
        return mask

    def findTargets(self):
        for target in self.targets:
            self.findTarget(target)


    def findTarget(self, target):
        samples = 5
        threshold = 2.1
        runningSum = self.findTargetSingle(target)
        for i in range(samples - 1):
            runningSum = runningSum + self.findTargetSingle(target)
        ret, detected = cv2.threshold(runningSum, threshold, 255, cv2.THRESH_BINARY)

        M = cv2.moments(detected)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        target.center = (cx, cy)
        im2, contours, hierarchy = cv2.findContours(detected, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        target.contour = contours

    def findTargetSingle(self, target):

        ret, frame = self.videoFeed.read()
        thresh = cv2.inRange(frame, target.color - target.tolerance,
                                    target.color + target.tolerance)
        eroded = cv2.erode(thresh, np.ones((3,3))) #Used to delete the stray pixels that appear in our image
        dilated = cv2.dilate(eroded, np.ones((5,5))) #Used to add in the pixels we want but were removed by erosion
        dilated[dilated == 255] = 1
        return dilated

    def getTarget(self, name):
        for target in self.targets:
            if target.name == name:
                return target
