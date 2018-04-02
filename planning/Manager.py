import cv2
import numpy as np
import math

class Manager:

    def __init__(self, robots, camera):
        self.robots = robots
        self.camera = camera
        self.createRivers()

    def createRivers(self):
        layers = []
        for robo in self.robots:
            plan = robo.getPlan()
            if plan == None:
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
            layers.append(layer)
        if len(layers) == 0:
            return
        inter = layers[0]
        for i in range(len(layers)):
            inter = cv2.bitwise_and(inter, layers[i])
        dilated = cv2.dilate(inter, np.ones((50,50), np.uint8))
        cv2.imshow('intersection',dilated)
        cv2.waitKey(1)

    def manage(self):
        self.createRivers()
