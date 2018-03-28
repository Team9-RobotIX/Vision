import numpy as np
import cv2

class Target:

    name = None

    lower = None
    upper = None
    contour = None
    center = None

    def __init__(self, n, low, high):
        self.name = n
        self.lower = low
        self.upper = high

class TargetDetector:

    targets = None

    mask = None

    def __init__(self, camera):
        self.camera = camera
        self.enableSettings()
        self.targets = self.createTargets()
        self.mask = self.createMask()


    def enableSettings(self):
        settings = [
    		(3,800.0),
    		(4,800.0),
    		(5,30.0),
    		(9,0.0),
    		(10,0.35686275362968445),
    		(11,0.13333334028720856),
    		(12,0.7372549176216125),
    		(14,0.05098039284348488)
        ]
        self.camera.enableSettings(settings)


    def createTargets(self):
        targets = [
            ('reception', (12,  30, 235), (52,  70, 275)),
            ('desk',(0,0,0),(30,30,30)),
            ('office',(200,200,200),(255,255,255)),
            ('er', (100,100,100),(150,150,150))
        ]
        targetObjects = []
        for target in targets:
            targetObjects.append(Target(target[0],np.array(target[1]),np.array(target[2])))
        targetObjects[0].center = (123,241)
        targetObjects[1].center = (454,113)
        targetObjects[2].center = (448,330)
        targetObjects[3].center = (100,100)
        return targetObjects

    def createMask(self):
        frame = self.camera.getFrame()
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
        threshold = 1.1
        runningSum = self.findTargetSingle(target)
        for i in range(samples - 1):
            runningSum = runningSum + self.findTargetSingle(target)
        ret, detected = cv2.threshold(runningSum, threshold, 255, cv2.THRESH_BINARY)

        M = cv2.moments(detected)

        if M['m00'] == 0:
            print('Issue detecting target:', target.name)

        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        target.center = (cx, cy)
        im2, contours, hierarchy = cv2.findContours(detected, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        target.contour = contours

    def findTargetSingle(self, target):
        frame = self.camera.getFrame()
        thresh = cv2.inRange(frame, target.lower, target.upper)

        eroded = cv2.erode(thresh, np.ones((3,3))) #Used to delete the stray pixels that appear in our image
        dilated = cv2.dilate(eroded, np.ones((5,5))) #Used to add in the pixels we want but were removed by erosion
        dilated[dilated == 255] = 1

        return dilated

    def getTarget(self, name):
        for target in self.targets:
            if target.name == name:
                return target
