import cv2
import numpy as np

#Used in the cases of issues with the video feed
class VideoError(Exception):

    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)

class PathDetector:

    walls = None
    deadSpace = None
    spacing = None
    maxima = None
    length = 0
    width = 0

    def __init__(self, ws, length, width):
        print('Beginning Path Detection')
        self.walls = ws
        self.length = length
        self.width = width
        self.deadSpace = self.createDeadSpace()
        self.spacing = self.createSpacing()
        self.maxima = self.createMaxima()
        self.graph = self.createGraph()
        self.check = self.createCheck()
        cv2.imshow('Image', self.walls)
        cv2.waitKey(15000)
        cv2.destroyAllWindows()
        print('Path Detection Complete')

    def createCheck(self):
        return cv2.bitwise_or(self.graph, self.deadSpace)

    def createGraph(self):
        print('  Generating Graph')
        maxima = self.maxima
        dead = self.deadSpace
        im2, contours, hier = cv2.findContours((maxima * 255).astype(np.uint8),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        pointCounts = []
        points = []
        for cnt in contours:
            box = cv2.boxPoints(cv2.minAreaRect(cnt))
            box = np.int0(box)
            rects.append(box)
            uniquePoints = []
            for p1 in box:
                keep = True
                for p2 in uniquePoints:
                    if np.linalg.norm(p1 - p2) < 50:
                        keep = False
                        break
                if keep:
                    uniquePoints.append(p1)
            count = 0
            for p in uniquePoints:
                points.append(p)
                count = count + 1
            pointCounts.append(count)

        #this matrix will represent all the pairs of points that can reach eachother
        #without travelling through a dead space
        connectMatrix = np.zeros((len(points),len(points))) + 10
        #fills in points that are part of the same line with a -1
        partialSum = 0
        for i in range(len(rects)):
            count = pointCounts[i]
            for j in range(count):
                for k in range(count):
                    connectMatrix[partialSum + j, partialSum + k] = -1
            partialSum = partialSum + count
        #iterates over all points checking which can be joined
        for i in range(len(points)):
            for j in range(len(points)):
                if connectMatrix[i,j] != 10: #means points lie on the same line
                    continue
                pointA = tuple(points[i])
                pointB = tuple(points[j])
                mask = np.zeros(dead.shape, np.uint8)
                cv2.line(mask,pointA,pointB,1,4)
                valid = np.sum(cv2.bitwise_and(mask,dead)) == 0
                if valid:
                    connectMatrix[i,j] = 1
                    connectMatrix[j,i] = 1
                else:
                    connectMatrix[i,j] = 0
                    connectMatrix[j,i] = 0
        graph = np.zeros(maxima.shape, np.uint8)
        for i in range(len(points)):
            for j in range(len(points)):
                if not connectMatrix[i,j] == 0:
                    pointA = tuple(points[i])
                    pointB = tuple(points[j])
                    cv2.line(graph,pointA,pointB,255,1)
        return graph

    def createMaxima(self):
        print('  Finding Space Maxima')
        space = self.spacing
        dead = self.deadSpace
        buff = 10
        kernel = np.zeros((buff,buff), np.uint8)
        center = buff / 2
        for i in range(buff):
            for j in range(buff):
                if np.linalg.norm((i-center,j-center)) > center:
                    kernel[i,j] = 0
                else:
                    kernel[i,j] = 1
        dilated = cv2.dilate(space, kernel)
        adjusted = (1 - (dilated - space)) - dead
        maxima = cv2.threshold(adjusted,.99,2,cv2.THRESH_BINARY)[1]
        return cv2.dilate(maxima, np.ones((5,5)))

    def createSpacing(self):
        print('  Computing Spacing')
        dead = self.deadSpace
        #creates an image of all 0s
        border = np.zeros(dead.shape, np.uint8)
        #fills in the border with 255's
        for i  in range(dead.shape[0]):
            border[i,0] = 255
            border[i,dead.shape[1] - 1] = 255
        for i in range(dead.shape[1]):
            border[0,i] = 255
            border[dead.shape[0] - 1,i] = 255
        #the template has white on borders and deadSpace and black elsewhere
        template = cv2.bitwise_or(dead, border)
        #flip the black and white for the distance transform
        template = cv2.bitwise_not(template)
        space = cv2.distanceTransform(template, cv2.DIST_LABEL_PIXEL, cv2.DIST_MASK_PRECISE)
        return space / np.max(space)

    def createDeadSpace(self):
        print(' Creating Dead Space')
        buff = self.length
        if self.length > self.width:
            buff = self.width #use the lesser of the two
        buff = int((buff + 1))
        kernel = np.zeros((buff,buff), np.uint8)
        center = buff / 2
        for i in range(buff):
            for j in range(buff):
                if np.linalg.norm((i-center,j-center)) > center:
                    kernel[i,j] = 0
                else:
                    kernel[i,j] = 1
        return cv2.dilate(self.walls,kernel)



class WallDetector:

    videoFeed = None
    walls = None

    def __init__(self, vf):
        #----- Comment this out for remote testing ----
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

    def enableSettings(self):
        settings = [(3,800), #640
                    (4,800), #480
                    #(5,25),
                    (9,0),
                    (10,0.46666666865348816),
                    (11,0.21259842813014984),
                    (12,0.9055117964744568),
                    (13,0.4901960790157318)]
        for setting in settings:
            self.videoFeed.set(setting[0],setting[1])

    def findWalls(self):
        samples = 5
        threshold = 2.1
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
        thresh = cv2.inRange(img,(0,0,150),(50,120,255))
        #Applies morphological transforms to smooth stuff out
        print('    Applying Morphological Transforms')
        dilated = cv2.dilate(thresh,np.ones((7,7)))
        eroded = cv2.erode(dilated,np.ones((5,5)))
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
        #cv2.imwrite('wallsample.png',base)
        return walls

    def getImage(self):
        print('    Acquiring Image Samples')
        samples = 5
        frames = []
        for i in range(samples):
            frames.append(self.videoFeed.read()[1])
        npFrames = np.asarray(frames)
        npMed = np.median(npFrames, axis = 0)
        print('    Computing Median Image')
        return (npMed, frames[0])
