import numpy as np
import cv2
import analysis

class PathDetector:

    camera = None

    wallDetector = None
    targetDetector = None
    robotDetector = None

    frame = None

    walls = None
    deadSpace = None
    spacing = None
    maxima = None
    length = 0
    width = 0

    def __init__(self):
        print('Beginning Path Detection')
        self.camera = analysis.Camera()
        self.wallDetector = analysis.WallDetector(self.camera)
        self.frame = self.camera.getFrame()
        self.walls = self.wallDetector.getWalls()
        self.targetDetector = analysis.TargetDetector(self.camera)
        self.targets = self.targetDetector.targets
        self.robotDetector = analysis.RobotDetector(self.camera)
        self.length = self.robotDetector.length
        self.width = self.robotDetector.width
        self.deadSpace = self.createDeadSpace()
        self.spacing = self.createSpacing()
        self.maxima = self.createMaxima()
        (self.vertices, self.matrix) = self.createGraph() #probably not needed
        self.check = self.createCheck()
        flip = cv2.bitwise_not(self.graph)
        overlay = cv2.bitwise_and(self.frame, self.frame, mask=flip)
        print('Path Detection Complete')

    def createCheck(self):
        return cv2.bitwise_or(self.graph, self.deadSpace)

    def getFrame(self):
        return self.wallDetector.getFrame()

    def recreateGraph(self):
        (self.vertices, self.matrix) = self.createGraph()

    def createGraph(self):
        print('  Generating Graph')
        maxima = self.maxima
        dead = self.deadSpace
        im2, contours, hier = cv2.findContours((maxima * 255).astype(np.uint8),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        pointCounts = []
        points = []
        robot = self.robotDetector.center(self.frame, newFrame=True)
        targets = self.targetDetector.targets
        points.append(robot)
        pointCounts.append(1)
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
        for target in targets:
            points.append(target.center)
            pointCounts.append(1)
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
        graphMask = np.zeros(maxima.shape, np.uint8)
        for i in range(len(points)):
            for j in range(len(points)):
                if not connectMatrix[i,j] == 0:
                    pointA = tuple(points[i])
                    pointB = tuple(points[j])
                    cv2.line(graphMask,pointA,pointB,255,1)
        binaryMatrix = np.zeros(connectMatrix.shape)
        for i in range(len(points)):
            for j in range(len(points)):
                if connectMatrix[i,j] == 0:
                    binaryMatrix[i,j] = 0
                else:
                    binaryMatrix[i,j] = 1
        self.graph = graphMask
        return (points,binaryMatrix)

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
