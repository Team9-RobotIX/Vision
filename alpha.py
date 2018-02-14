#THIS PROJECT RUNS ON PYTHON 3 AND REQUIRES openCV

import cv2
import numpy as np

#Useful for extracting where the different objects in a scene are
class Detector:

    filename = None #The name of the file holding the image
    image    = None #The loaded in image variable
    walls    = None #Processed image, containing just the walls
    targets  = None #Processed image, containing just the targets
    robot    = None #Processed image, containing just the robot

    #Takes the filename as an argument
    def __init__(self, fn):
        self.filename = fn
        self.image   = cv2.imread(fn, cv2.IMREAD_COLOR)
        self.walls   = self.createWalls(self.image)
        self.targets = self.createTargets(self.image)
        self.robot   = self.createRobot(self.image)

    def createWalls(self, img):
        return cv2.inRange(img, (0,0,0),(10,10,10))

    def createTargets(self, img):
        #Blue, green, red, yellow
        #OpenCV also uses BGR instead of RGB
        targetColors = [np.array((255,5,1)),np.array((0,255,128)),
                        np.array((0,0,255)),np.array((0,230,255))]

        #creates a filter for only where the targets are
        filtered = cv2.inRange(img, targetColors[0]-5, targetColors[0]+5)
        for color in targetColors:
            filtered = cv2.bitwise_or(filtered, cv2.inRange(img, color-5, color+5))

        #fill in the locations of the targets with their colors
        return cv2.bitwise_and(img,img,mask=filtered)

    def createRobot(self, img):
        #The colors used on just the robot
        baseColor = np.array((255,0,187))
        markerColor = np.array((0,140,255))

        #does a logical or for on what is the markers and the base
        filtered = cv2.bitwise_or(cv2.inRange(img,markerColor-5,markerColor+5),
                        cv2.inRange(img, baseColor-5, baseColor+5))

        #uses the filter to extract just the robot
        return cv2.bitwise_and(img, img, mask=filtered)

    # 1:original image, 2:just walls, 3:just targets, 4:just robot
    def displayImage(self, img=1):
        options = [None,self.image,self.walls,self.targets,self.robot]
        dispImg = dispImg = options[img]
        cv2.imshow('Image', dispImg)
        cv2.waitKey(1000)
        cv2.destroyAllWindows()

    def destroyImages(self):
        cv2.destroyAllWindows()
        cv2.waitKey(1)

class PathDetector(Detector):

    length    = 0 #denotes the length of the robot in pixels
    width     = 0 #denotes the width of the robot in pixels
    deadSpace = None #denots the space the center of the robot cannot reach

    #takes the filename and the length and width of the robot
    def __init__(self,fn,length,width):
        Detector.__init__(self, fn)
        self.length = length
        self.width  = width
        self.deadSpace = self.createDeadSpace(self.walls)
        self.spacing = self.createSpacing(self.deadSpace)
        self.maxima = self.createMaxima(self.spacing, self.deadSpace)
        self.graph = self.createGraph(self.maxima, self.deadSpace)
        self.check = self.createCheck(self.graph, self.deadSpace)
        self.overlay = self.createOverlay()

    def createOverlay(self):
        graph = 255 - self.graph
        mask = cv2.bitwise_and(graph, cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY))
        return cv2.bitwise_or(self.image, self.image,mask=mask)

    def createCheck(self, graph, dead):
        return cv2.bitwise_or(graph, dead)

    def createGraph(self, maxima, dead):
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

    def createMaxima(self, space, dead):
        buff = 10 #divide by to only use the center
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
        print(np.max(adjusted))
        maxima = cv2.threshold(adjusted,.99,2,cv2.THRESH_BINARY)[1]
        return cv2.dilate(maxima, np.ones((5,5)))

    def createSpacing(self, dead):
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
        #print(np.max(space))
        return space / np.max(space)

    def createDeadSpace(self, walls):
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
        return cv2.dilate(walls,kernel)


    def displayImage(self, img=1):
        options = [None,self.image,self.walls,self.targets,
                        self.robot,self.deadSpace,self.spacing, self.maxima,
                        self.graph,self.check, self.overlay]
        dispImg = dispImg = options[img]
        cv2.imshow('Image', dispImg)
        cv2.waitKey(3000)
        cv2.destroyAllWindows()
