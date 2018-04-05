import cv2
import numpy as np
import requests
import math
import json
from time import sleep

class Tracker:
    def __init__(self, rd, robot, disp):
        self.robotDetector = rd
        self.plan = []
        self.instruction = None
        self.robot = robot
        self.dispatch = disp

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
                '''sleep(0.5)
                orientation = self.robotDetector.orientation(None, self.robot, newFrame=True)
                diff = orientation - target
                if diff > np.pi:
                    diff -= 2*np.pi
                else:
                    diff += 2*np.pi
                self.turn(-diff)
                sleep(0.25)'''
                self.stop()
        return len(self.plan) == 0



    def checkPointDistance(self,frame):
        center = np.array(self.robotDetector.center(frame,self.robot))
        self.robot.center = center
        orientation = self.robotDetector.orientation(frame,self.robot)
        self.robot.orientation = orientation
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
        if distance < 35:
            self.stop()
            del self.plan[0]
            del self.robot.getPlan()[0]
            if len(self.plan) > 0:
                self.instruction = self.plan[0]
            else:
                self.instruction = None
        else:
            self.go(correction, distance)
            #self.go(correction)

    def checkOrientation(self,frame):
        orientation = self.robotDetector.orientation(frame, self.robot)
        self.robot.orientation = orientation
        absolute = self.instruction['angle']
        dif = orientation - absolute
        turn = absolute - orientation
        if turn > np.pi:
            turn -= 2 * np.pi
        elif turn < -np.pi:
            turn += 2 * np.pi
        if np.absolute(orientation - absolute) < 7.5 / 180 * math.pi:
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
        #pass
        #post = 'http://35.177.199.115/flaskapp/post?onOff=1&turnAngle=0&correction=0'
        #r = requests.get(post)
        #post = 'http://35.177.199.115/development/robot/'+str(self.robot.id)+'/batch'
        #r = requests.post(post), data=json.dumps({
        self.dispatch.add({
            'angle':0.0,
            'correction':1.5*correction,
            'motor':True,
            'distance':dist,
            'robot':self.robot.id
        })

    def stop(self):
        #pass
        #post = 'http://35.177.199.115/development/robot/'+str(self.robot.id)+'/batch'
        #r = requests.post(post, data=json.dumps({
        self.dispatch.add({
            'angle':0.0,
            'correction':0.0,
            'motor':False,
            'distance':0.0,
            'robot':self.robot.id
        })

    def turn(self, ang):
        #pass
        #post = 'http://35.177.199.115/development/robot/'+str(self.robot.id)+'/batch'
        #r = requests.post(post, data=json.dumps(
        #print(ang)
        self.dispatch.add({
            'angle':ang*.4 ,
            'correction':0.0,
            'motor':True,
            'distance':0.0,
            'robot':self.robot.id
        })
