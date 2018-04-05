import cv2
import numpy as np
import math

class Manager:

    def __init__(self, robots, camera, dispatch):
        self.robots = robots
        self.camera = camera
        self.intersection = None
        self.createRivers()
        self.dispatch = dispatch

    def createRivers(self):
        self.layers = []
        for robo in self.robots:
            plan = robo.getPlan()
            if plan == None or robo.moveNoDelivery:
                self.layers.append(None)
                continue
            points = []
            contours = []
            for p in range(len(plan) - 1):
                p1 = plan[p]
                p2 = plan[p+1]
                angle = math.atan2(-p2[2]+p1[2],p2[1]-p1[1])
                for i in range(5):
                    contours.append(
                        robo.getBox(
                            (p1[1]*i/4 + p2[1]*(4-i)/4,p1[2]*i/4 + p2[2]*(4-i)/4),
                            -angle
                        )
                    )
            layer = np.zeros(self.camera.size, np.uint8)
            for cnt in contours:
                cv2.fillPoly(layer, pts=[cnt], color=255)
            self.layers.append(layer)
        inter = np.ones(self.camera.size,np.uint8) * 255
        count = 0
        for i in range(len(self.layers)):
            if type(self.layers[i]) is np.ndarray:
                count += 1
                inter = cv2.bitwise_and(inter, self.layers[i])
        dilated = cv2.dilate(inter, np.ones((50,50), np.uint8))
        if count < 2:
            self.intersection = np.zeros(self.camera.size,np.uint8)
        else:
            self.intersection = dilated

    def inOverlap(self,overlap, box):
        boxDraw = np.zeros(self.camera.size,np.uint8)
        cv2.fillPoly(boxDraw,pts=[box],color=255)
        intersect = cv2.bitwise_and(boxDraw, overlap)
        area = np.sum(intersect) / 255
        return not area == 0

    def manage(self):
        #Don't manage a single robot, no need
        if len(self.robots) < 2:
            self.intersection = None
            return
        allStopped = True
        for robo in self.robots:
            if robo.isMoving():
                allStopped = False
        #Don't manage a bunch of stationary robots
        if allStopped:
            self.intersection = None
            return
        self.createRivers()
        overlapArea = np.sum(self.intersection) / 255
        #case where they won't both move into eachother
        if overlapArea == 0:
            allMoving = True
            for robo in self.robots:
                if not robo.isMoving():
                    allMoving = False
            #everything moving but not towards eachother
            if allMoving:
                #is k, bc overlap is 0 we chill.
                return
            else:
                if type(self.layers[0]) is np.ndarray:
                    dil = cv2.dilate(self.layers[0], np.ones((50,50), np.uint8))
                    cv2.imshow('avoid',dil)
                    if self.inOverlap(dil, self.robots[1].box):
                        print('in overlap')
                        self.stopBot(0)
                        self.moveBot(1)
                    else:
                        return
                else:
                    if self.inOverlap(self.layers[1], self.robots[0].box):
                        self.stopBot(1)
                        self.moveBot(0)
                    else:
                        return
        else:
            over0 = self.inOverlap(self.intersection, self.robots[0].box)
            over1 = self.inOverlap(self.intersection, self.robots[1].box)
            print(98,over0,over1)
            if (not over0) and (not over1):
                return
            if over0:
                #pass
                if self.inOverlap(cv2.dilate(self.intersection, np.ones((50,50), np.uint8)),
                    self.robots[1].box):
                    self.stopBot(1)
            else:
                #pass
                if self.inOverlap(cv2.dilate(self.intersection, np.ones((50,50), np.uint8)),
                    self.robots[0].box):
                    self.stopBot(0)

    def moveBot(self, roboId):
        if self.robots[roboId].moveNoDelivery:
            return
        y, x = self.camera.size
        q1 = (3*x/4,y/4)
        q2 = (x/4,y/4)
        q3 = (x/4,3*y/4)
        q4 = (3*x/4,3*y/4)
        candidates = [q1,q2,q3,q4]
        #4 potential move spots, the center of the quadrants
        minDist = self.distance(self.robots[roboId].center, q1)
        minIdx = 0
        for i in range(4):
            dist = self.distance(self.robots[roboId].center, candidates[i])
            if dist < minDist:
                minDist = dist
                minIdx = i
        #removes the closest point to the robots current place
        del candidates[minIdx]
        minDist = self.distance(self.robots[1-roboId].center, candidates[0])
        minIdx = 0
        for i in range(3):
            dist = self.distance(self.robots[1-roboId].center, candidates[i])
            if dist < minDist:
                minDist = dist
                minIdx = i
        #removes the closest point to the other robot
        del candidates[minIdx]
        #takes the farther of the remaining two
        targ = self.robots[1 - roboId].plan[-1]
        targ = (targ[1],targ[2])
        points = [candidates[1], candidates[0]]
        if self.distance(candidates[1], targ) > self.distance(candidates[0], targ):
            points = [candidates[0], candidates[1]]
        if (q1 in points and q3 in points) or (q2 in points and q4 in points):
            points = [points[1]]
        self.robots[roboId].moveNoDelivery = True
        self.robots[roboId].points = points

    def distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])*(p1[0] - p2[0]) + (p1[1] - p2[1])*(p1[1] - p2[1]))

    def stopBot(self, roboId):
        self.dispatch.overwrite({
            'angle':0.0,
            'correction':0.0,
            'motor':False,
            'distance':0.0,
            'robot':roboId
        })
