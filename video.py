import cv2
import numpy as np

#Used in the cases of issues with the video feed
class VideoError(Exception):

    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)

class WallDetection:

    videoFeed = None
    walls = None

    def __init__(self, vf):
        #----- Comment this out for remote testing ----
        self.videoFeed = vf
        
        self.walls = self.findWalls()

    def findWalls(self):
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
        thresh = cv2.inRange(img,(0,0,150),(142,120,255))
        #Applies morphological transforms to smooth stuff out
        dilated = cv2.dilate(thresh,np.ones((7,7)))
        eroded = cv2.erode(dilated,np.ones((5,5)))
        #Detects the contours, which is for us the walls
        im2, contours, hierarchy = cv2.findContours(eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #Filters out little blips that occur from time to time but are not wall
        trueContours = []
        for cnt in contours:
            if cv2.contourArea(cnt) > 100:
                trueContours.append(cnt)
        #overlays the contours
        cv2.drawContours(base,trueContours,-1,(0,255,0),1)
        cv2.imshow('Image',base)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()
        #cv2.imwrite('wallsample.png',base)
        return None

    def getImage(self):
        samples = 5
        frames = []
        for i in range(samples):
            frames.append(self.videoFeed.read()[1])
        npFrames = np.asarray(frames)
        npMed = np.median(npFrames, axis = 0)
        return (npMed, frames[0])
