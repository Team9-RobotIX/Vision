import numpy as np
import cv2

class WallDetector:

    videoFeed = None
    walls = None

    def __init__(self, vf):
        print('Beginning Wall Detection')
        self.videoFeed = vf
        self.enableSettings()
        self.walls = self.findWalls()
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

    def getFrame(self):
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
        return self.videoFeed.read()[1][y:y+h, x:x+w,:]

    def enableSettings(self):
        settings = [(3,800), #640
                    (4,800), #480
                    #(5,25),
                    (9,0),
                    (10,0.46666666865348816),
                    (11,0.21259842813014984),
                    (12,0.9055117964744568)]#,
                    #(13,0.4901960790157318)]
        for setting in settings:
            self.videoFeed.set(setting[0],setting[1])

    def findWalls(self):
        samples = 5
        threshold = 3.1
        print('  Acquiring Wall Sample 0')
        runningSum = self.findWallsSingle()
        for i in range(samples - 1):
            print('  Acquiring Wall Sample '+str(i+1))
            runningSum = runningSum + self.findWallsSingle()
        print('  Creating Composite Image')
        ret, walls = cv2.threshold(runningSum, threshold, 255, cv2.THRESH_BINARY)
        return walls.astype(np.uint8)

    def findWallsSingle(self):
        #----- Comment this out for remote testing ----
        if not self.videoFeed.isOpened():
            raise VideoError('Video feed is not opened')
        #----------------------------------------------
        img, base  = self.getImage()
        #--------- use these for remote testing -------
        #img = cv2.imread('snap2.jpeg')
        #base = cv2.imread('snap.jpeg')
        #----------------------------------------------
        #Picks walls by color
        print('    Applying Color Filter')
        thresh = cv2.inRange(img,(0,40,200),(20,190,255))
        #Applies morphological transforms to smooth stuff out
        print('    Applying Morphological Transforms')
        dilated = cv2.dilate(thresh,np.ones((7,7)))
        eroded = cv2.erode(dilated,np.ones((3,3)))
        print('    Detecting contours')
        #Detects the contours, which is for us the walls
        im2, contours, hierarchy = cv2.findContours(eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #Filters out little blips that occur from time to time but are not wall
        print('    Filtering Contours')
        trueContours = []
        for cnt in contours:
            if cv2.contourArea(cnt) > 100:
                trueContours.append(cnt)
        #overlays the contours
        print('    Filling Contours')
        walls = np.zeros((base.shape[0],base.shape[1]), dtype='float32')
        cv2.fillPoly(walls,trueContours,255)
        cv2.imwrite('wallsample.png',base)
        return walls

    def getImage(self):
        print('    Acquiring Image Samples')
        samples = 1
        if samples == 1:
            frame = self.videoFeed.read()[1]
            return (frame, frame)
        frames = []
        for i in range(samples):
            frames.append(self.videoFeed.read()[1])
        npFrames = np.asarray(frames)
        npMed = np.median(npFrames, axis = 0)
        print('    Computing Median Image')
        return (npMed, frames[0])
