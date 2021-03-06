import numpy as np
import cv2

class WallDetector:

    camera = None
    walls = None

    def __init__(self, camera):
        print('Beginning Wall Detection')
        self.camera = camera
        self.enableSettings()
        self.walls = self.findWalls()

        self.camera.setCrop(self.getCrop())
        print('Wall Detection Complete')

    def getWalls(self):
        wallCopy = self.walls.copy()
        im2, contours, hierarchy = cv2.findContours(wallCopy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        maxArea = -1
        maxCnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > maxArea:
                maxArea = area
                maxCnt = cnt
        x,y,w,h = cv2.boundingRect(maxCnt)
        return self.walls[y:y+h, x:x+w]

    def getCrop(self):
        wallCopy = self.walls.copy()
        im2, contours, hierarchy = cv2.findContours(wallCopy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        maxArea = -1
        maxCnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > maxArea:
                maxArea = area
                maxCnt = cnt
        x,y,w,h = cv2.boundingRect(maxCnt)
        return {'x':x,'y':y,'w':w,'h':h}

    def enableSettings(self):
        settings = [
        	(3,800.0),
        	(4,800.0),
        	(5,30.0),
        	(9,0.0),
        	(10,0.5058823823928833),
        	(11,0.03921568766236305),
        	(12,0.250980406999588),
        	(14,0.6117647290229797)
        ]
        self.camera.enableSettings(settings)

    def findWalls(self):
        samples = 5
        threshold = 0.1
        print('  Acquiring Wall Sample 0')
        runningSum = self.findWallsSingle()
        for i in range(samples - 1):
            print('  Acquiring Wall Sample '+str(i+1))
            runningSum = runningSum + self.findWallsSingle()
        print('  Creating Composite Image')
        ret, walls = cv2.threshold(runningSum, threshold, 255, cv2.THRESH_BINARY)
        return walls.astype(np.uint8)

    def findWallsSingle(self):
        #----------------------------------------------
        img, base  = self.getImage()
        #--------- use these for remote testing -------
        #img = cv2.imread('snap2.jpeg')
        #base = cv2.imread('snap2.jpeg')
        #----------------------------------------------q
        #Picks walls by color
        print('    Applying Color Filter')
        thresh = cv2.inRange(img,(50,70,150),(110,160,180))
        #Applies morphological transforms to smooth stuff out
        print('    Applying Morphological Transforms')
        dilated = cv2.dilate(thresh,np.ones((3,3)), iterations=3)
        eroded = cv2.erode(dilated,np.ones((3,3)))
        print('    Detecting contours')
        #Detects the contours, which is for us the walls
        im2, contours, hierarchy = cv2.findContours(eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #Filters out little blips that occur from time to time but are not wall
        print('    Filtering Contours')
        trueContours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                trueContours.append(cnt)
        #overlays the contours
        print('    Filling Contours')
        walls = np.zeros((base.shape[0],base.shape[1]), dtype='float32')
        cv2.fillPoly(walls,trueContours,1)
        return walls

    def getImage(self):
        print('    Acquiring Image Samples')
        samples = 1
        if samples == 1:
            frame = self.camera.getFrame(cropped=False)
            return (frame, frame)
        frames = []
        for i in range(samples):
            frames.append(self.camera.getFrame(cropped=False))
        npFrames = np.asarray(frames)
        npMed = np.median(npFrames, axis = 0)
        print('    Computing Median Image')
        return (npMed, frames[0])
