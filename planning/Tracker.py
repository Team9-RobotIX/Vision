import cv2
import numpy as np
import requests
import math
from Config import Config

conf = Config
class Tracker:
    def __init__(self, rd):
        self.robotDetector = rd
        self.plan = []
        self.instruction = None

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
        print('update')
        #print("inst",self.instruction)
        if self.instruction == None:
            print('none')
            self.stop()
        elif self.instruction['type'] == 'MOVE':
            #self.checkOrientation(frame)
            self.checkPointDistance(frame)
        elif self.instruction['type'] == 'TURN':
            self.checkOrientation(frame)
        return len(self.plan) == 0



    def checkPointDistance(self,frame):
        center = np.array(self.robotDetector.center(frame))
        orientation = self.robotDetector.orientation(frame)
        start = self.instruction['start']
        end = self.instruction['end']
        correctOrientation = np.arctan2(center[1]-end[1], end[0]-center[0])
        print("ROBOT ORIENTATION:", orientation)
        print("DESIRED ORIENTATION:", correctOrientation)
        correction = correctOrientation - orientation
        point = np.array(self.instruction['end'])
        distance = np.linalg.norm(center - point)
        if distance < 40:
            self.stop()
            del self.plan[0]
            if len(self.plan) > 0:
                self.instruction = self.plan[0]
            else:
                self.instruction = None
        else:
            self.go(correction)






    def checkOrientation(self,frame):
        orientation = self.robotDetector.orientation(frame)
        absolute = self.instruction['angle']
        print('orientation:',orientation,' angle: ',absolute)
        if np.absolute(orientation - absolute) < 25 / 180 * math.pi:
            self.stop()
            del self.plan[0]
            if len(self.plan) > 0:
                self.instruction = self.plan[0]
            else:
                self.instruction = None
        turn = absolute - orientation
        if turn > np.pi:
            turn -= 2 * np.pi
        elif turn < -np.pi:
            turn += 2 * np.pi
        self.turn(turn)


    def go(self,correction):
        print("correction",correction)
        post = conf.BASE_URL + '/flaskapp/post?onOff=1&turnAngle=0.0&correction='+str(correction)
        r = requests.get(post)
        #print("go",r.text)

    def stop(self):
        post = conf.BASE_URL + '/flaskapp/post?onOff=0&turnAngle=0.0&correction=0'
        r = requests.get(post)
        #print("stop",r.text)

    def turn(self, d):
        post = conf.BASE_URL + '/flaskapp/post?onOff=1&turnAngle='+str(d)+'&correction=0'
        r = requests.get(post)
        #print("turn",r.text)

    def checkDistance(self):
        start = np.array(self.instruction.start)
        end = np.array(self.instruction.end)
        pos = np.array(self.robotDetector.center())

        endAdj = end - start
        posAdj = pos - start

        projected = (np.dot(posAdj,endAdj) / np.dot(endAdj,endAdj)) * endAdj
        distance = np.linalg.norm(projected, endAdj)
        return distance
