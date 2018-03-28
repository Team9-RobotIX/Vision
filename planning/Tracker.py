import cv2
import numpy as np
import requests
import math
import json
from time import sleep

class Tracker:
    def __init__(self, rd, robot):
        self.robotDetector = rd
        self.plan = []
        self.instruction = None
        self.robot = robot

    #Keep track of the robot
    #If centre of robot is more than (threshold) away from line, turn.
    #Takes a current position
    #Threshold in terms of pixels.

    def setPlan(self, plan):
        self.plan = plan

    def start(self):
        if(len(self.plan) == 0):
            return
        else:
            self.instruction = self.plan[0]

    def update(self,frame):
        if self.instruction == None:
            self.stop()
        elif self.instruction['type'] == 'MOVE':
            #self.checkOrientation(frame)
            self.checkPointDistance(frame)
        elif self.instruction['type'] == 'TURN':
            target = self.instruction['angle']
            fix = self.checkOrientation(frame)
            #TODO fix turning and delete this boi
            if fix:
                sleep(0.5)
                orientation = self.robotDetector.orientation(None, self.robot, newFrame=True)
                diff = orientation - target
                if diff > np.pi:
                    diff -= 2*np.pi
                else:
                    diff += 2*np.pi
                self.turn(-diff)
                sleep(0.25)
                self.stop()
        return len(self.plan) == 0



    def checkPointDistance(self,frame):
        center = np.array(self.robotDetector.center(frame,self.robot))
        orientation = self.robotDetector.orientation(frame,self.robot)
        start = self.instruction['start']
        end = self.instruction['end']
        correctOrientation = np.arctan2(center[1]-end[1], end[0]-center[0])
        correction = correctOrientation - orientation
        if correction > np.pi:
            correction -= 2 * np.pi
        elif correction < -np.pi:
            correction += 2 * np.pi
        point = np.array(self.instruction['end'])
        distance = np.linalg.norm(center - point)
        if distance < 30:
            self.stop()
            del self.plan[0]
            if len(self.plan) > 0:
                self.instruction = self.plan[0]
            else:
                self.instruction = None
        else:
            self.go(correction, distance)
            #self.go(correction)






    def checkOrientation(self,frame):
        orientation = self.robotDetector.orientation(frame, self.robot)
        absolute = self.instruction['angle']
        dif = orientation - absolute
        turn = absolute - orientation
        if turn > np.pi:
            turn -= 2 * np.pi
        elif turn < -np.pi:
            turn += 2 * np.pi
        if np.absolute(orientation - absolute) < 25 / 180 * math.pi:
            self.stop()
            del self.plan[0]
            if len(self.plan) > 0:
                self.instruction = self.plan[0]
            else:
                self.instruction = None
            return True
        else:
            self.turn(turn)
            return False


    def go(self,correction,dist):
        post = 'http://18.219.63.23/development/robot/'+str(self.robot.id)+'/batch'
        r = requests.post(post, data=json.dumps({
            'angle':0.0,
            'correction':correction,
            'motor':True,
            'distance':dist
        }))

    def stop(self):
        post = 'http://18.219.63.23/development/robot/'+str(self.robot.id)+'/batch'
        r = requests.post(post, data=json.dumps({
            'angle':0,
            'correction':0,
            'motor':False,
            'distance':0
        }))

    def turn(self, ang):
        post = 'http://18.219.63.23/development/robot/'+str(self.robot.id)+'/batch'
        r = requests.post(post, data=json.dumps({
            'angle':ang,
            'correction':0,
            'motor':True,
            'distance':0
        }))
