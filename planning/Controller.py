import cv2
import numpy as np
import planning
import requests
import json
import math
from time import sleep
from math import sqrt

base = 'http://18.219.63.23/edge'

post = 'http://18.219.63.23/post'

class Controller:

    def __init__(self):
        self.planner = planning.Planner()
        self.plan = []
        self.tracker = planning.Tracker(self.planner.robotDetector)
        self.robotDetector = self.planner.robotDetector
        #self.targets = self.planner.targets
        self.camera = self.planner.pathDetector.camera

    def run(self):

        #self.tracker.setPlan(insts)
        try:
            while self.camera.isOpened():
                delivery = self.getDelivery()
                if delivery == None:
                    sleep(1.0)
                    continue
                self.deliver(delivery)
        except KeyboardInterrupt:
            cv2.destroyAllWindows()
            self.camera.release()
            self.tracker.stop()

    def deliver(self, delivery):
        id = delivery['id']
        data = {'state':   'MOVING_TO_SOURCE'}
        r = requests.patch("http://18.219.63.23/development/delivery/" + str(id), json=data)
        self.toTarget(delivery['from']['name'].lower())
        self.tracker.stop()
        data = {'state':   'AWAITING_AUTHENTICATION_SENDER'}
        r = requests.patch("http://18.219.63.23/development/delivery/" + str(id), json=data)

        # WAIT FOR PACKAGE_LOAD_COMPLETE
        while not self.checkState(id) == 'PACKAGE_LOAD_COMPLETE':
            sleep(1.0)

        data = {'state':   'MOVING_TO_DESTINATION'}
        r = requests.patch("http://18.219.63.23/development/delivery/" + str(id), json=data)
        self.toTarget(delivery['to']['name'].lower())
        self.tracker.stop()
        data = {'state':   'AWAITING_AUTHENTICATION_RECEIVER'}
        r = requests.patch("http://18.219.63.23/development/delivery/" + str(id), json=data)

        # WAIT FOR PACKAGE_RETRIEVAL_COMPLETE
        while not self.checkState(id) == 'PACKAGE_RETRIEVAL_COMPLETE':
            sleep(1.0)

        r = requests.delete('http://18.219.63.23/development/delivery/' + str(id))


    def checkState(self, id):
        r = requests.get("http://18.219.63.23/development/delivery/" + str(id))
        return json.loads(r.text)['state']

    def toTarget(self, name):
        self.plan = self.planner.plan(name)
        insts = self.pointConversion(self.plan)
        frame = self.camera.getFrame()
        absolute = self.robotDetector.orientation(frame)
        for i in range(len(self.plan) - 1):
            insts[i*2+1]['start'] = (self.plan[i][1],self.plan[i][2])
            insts[i*2+1]['end'] = (self.plan[i+1][1],self.plan[i+1][2])
        for i in range(len(self.plan) - 1):
            insts[i*2]['angle'] = np.arctan2(-self.plan[i+1][2] + self.plan[i][2],
                                            self.plan[i+1][1] - self.plan[i][1])
        self.tracker.setPlan(insts)
        print("PLAN:", self.plan)
        self.tracker.start()
        while True:
            frame = self.camera.getFrame()
            complete = self.tracker.update(frame)
            if complete:
                break
            cv2.imshow('frame',self.drawPath(frame))
            #cv2.cvtColor(o, cv2.COLOR_BGR2RGB)
            cv2.waitKey(1)
            #if cv2.waitKey(1): & 0xFF == ord('q'):
            #    break
        self.tracker.stop()



    def pointConversion(self, points):
        frame = self.camera.getFrame()
        distances = []
        angles = []
        for i in range(len(points) - 1):
            distances.append(self.euclidean(points[i], points[i+1]))
        for i in range(len(points) - 1):
            angles.append(np.arctan2(-points[i+1][2] + points[i][2],
                                    points[i+1][1] - points[i][1]))
        relativeAngles = [0]
        for i in range(len(angles) - 1):
            relativeAngles.append(angles[i+1] - angles[i])
        relativeAngles[0] = angles[0] - self.robotDetector.orientation(frame)
        instructions = []
        print(distances)
        print(angles)
        print(relativeAngles)
        for i in range(len(relativeAngles)):
            instructions.append({'type':'TURN', 'value':relativeAngles[i]})
            instructions.append({'type':'MOVE', 'value':distances[i]})
        return instructions

    def euclidean(self, a, b):
        D = 1
        dx = abs(a[1] - b[1])
        dy = abs(a[2] - b[2])
        return D * sqrt(dx * dx + dy * dy)

    def drawPath(self,frame):
        graph = self.plan
        red = self.robotDetector.findRed(frame)
        blue = self.robotDetector.findBlue(frame)
        c = self.robotDetector.center(frame) #(int((red[0]+blue[0])/2),int((red[1]+blue[1])/2))
        center = (c[0],c[1])
        #print('orientation: ',np.arctan2(-blue[1]+red[1],blue[0]-red[0])*180/np.pi)
        cv2.circle(frame, red, 5, thickness=3, color=(0,255,255))
        cv2.circle(frame, blue, 5, thickness=3, color=(255,0,255))
        cv2.circle(frame, center, 5, thickness=3, color=(255,255,0))
        for i in range(len(graph)-1):
            pt1 = (graph[i][1],graph[i][2])
            pt2 = (graph[i+1][1],graph[i+1][2])
            cv2.line(frame, pt1, pt2, thickness = 2, color = (0,255,0))
            cv2.line(frame, center, pt2, thickness=2, color = (255,0,0))
        for target in self.planner.targets:
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

    def sendInstructions(self):
        # Set state of delivery to "in progress"

        self.plan = self.toTarget(fro)
        #self.plan = self.planner.plan('YELLOW')
        instructions = self.pointConversion(self.plan)
        url = "http://ec2-18-219-63-23.us-east-2.compute.amazonaws.com/edge/instructions"
        r = requests.post(url, json = instructions)
        print(r.text)

    def sendAdjustment(self):
        pass
