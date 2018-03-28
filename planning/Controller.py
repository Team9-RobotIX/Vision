import cv2
import numpy as np
import planning
import analysis
import requests
import json
import math
import time
from time import sleep
from math import sqrt

base = 'http://18.219.63.23/edge'

post = 'http://18.219.63.23/post'

class Controller:

    def __init__(self, robots):
        self.camera = analysis.Camera()
        self.robots = self.createRobots(robots, self.camera)
        #self.plan = []
        #self.tracker = planning.Tracker(self.planner.robotDetector)
        #self.robotDetector = self.planner.robotDetector
        #self.targets = self.planner.targets
        #self.camera = self.planner.pathDetector.camera

    def createRobots(self, robots, camera):
        r = []
        for rob in robots:
            robo = analysis.Robot(rob[0],rob[1],rob[2],rob[3],rob[4],rob[5])
            robo.setPlanner(planning.Planner(robo, camera))
            robo.setTracker(planning.Tracker(analysis.RobotDetector(camera), robo))
            robo.setRobotDetector(analysis.RobotDetector(camera))
            r.append(robo)
        return r

    def run(self):

        #self.tracker.setPlan(insts)
        t = time.time()
        f = 0
        try:
            while self.camera.isOpened():
                f += 1
                if time.time() - t > 1:
                    print(f)
                    f = 0
                    t = time.time()
                if(self.hasFreeRobot()):
                    delivery = self.getDelivery()
                    if delivery != None:
                        freeBot = self.getFreeRobot()
                        freeBot.setDelivery(delivery)
                self.deliver()
        except KeyboardInterrupt:
            cv2.destroyAllWindows()
            self.camera.release()
            self.stop()

    def hasFreeRobot(self):
        for r in self.robots:
            if r.isFree():
                return True
        return False

    def getFreeRobot(self):
        for r in self.robots:
            if r.isFree():
                return r
        return None

    def stop(self):
        for r in self.robots:
            r.stop()

    def deliver(self):
        frame = self.camera.getFrame()
        #cv2.destroyAllWindows()
        for r in self.robots:
            r.deliver(frame)
        cv2.imshow('Frame', self.drawInfo(frame))
        cv2.waitKey(1)

    def drawInfo(self,frame):
        targets = self.robots[0].planner.targets
        if targets != None:
            for target in targets:
                tc = (target.center[0], target.center[1]+10)
                cv2.putText(frame, target.name, tc, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
                cv2.circle(frame, target.center, 5, thickness=3, color=(0,0,255))
        return frame

    def getDelivery(self):
        url = "http://18.219.63.23/development/deliveries"
        r = requests.get(url)
        queue = json.loads(r.text)
        if len(queue) == 0:
            return None
        return queue[0]
        #return
        #num = np.random.randint(low=0, high = 2)
        #if num == 0:
        #    return {'id':0,'from':{'name':'reception'},'to':{'name':'office'}}
        #else:
        #    return {'id':0,'from':{'name':'ER'},'to':{'name':'desk'}}
        #return {'id':0,'from':{'name':'reception'},'to':{'name':'office'}}
