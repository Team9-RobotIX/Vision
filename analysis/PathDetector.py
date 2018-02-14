import numpy as np
import cv2
import analysis

class PathDetector:

    videoFeed = None

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
        self.videoFeed = cv2.VideoCapture(0)
        self.wallDetector = analysis.WallDetector(self.videoFeed)
        self.targetDetector = None
        self.robotDetector = None
        self.frame = self.wallDetector.getFrame()
        self.walls = self.wallDetector.getWalls()
        self.length = 20 #TODO Detect the robot
        self.width = 20  #TODO Detect the robot
        self.deadSpace = self.createDeadSpace()
        self.spacing = self.createSpacing()
        self.maxima = self.createMaxima()
        self.graph = self.createGraph()
        self.check = self.createCheck()

        flip = cv2.bitwise_not(self.graph)
        overlay = cv2.bitwise_and(self.frame, self.frame, mask=flip)

        cv2.imshow('Image', overlay)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()
        
        print('Path Detection Complete')

    def blend_transparent(self, capture, overlay):
        # Split out the transparency mask from the colour info

        overlay_img = overlay[:,:,:3]
        overlay_mask = overlay[:,:,3:]
        background_mask = 255 - overlay_mask

        overlay_mask = cv2.cvtColor(overlay_mask, cv2.COLOR_GRAY2BGR)
        background_mask = cv2.cvtColor(background_mask, cv2.COLOR_GRAY2BGR)

        # Create a masked out face image, and masked out overlay
        capture_part = (capture * (1 / 255.0)) * (background_mask * (1 / 255.0))
        overlay_part = (overlay_img * (1 / 255.0)) * (overlay_mask * (1 / 255.0))

        return np.uint8(cv2.addWeighted(capture_part, 255.0, overlay_part, 255.0, 0.0))

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
