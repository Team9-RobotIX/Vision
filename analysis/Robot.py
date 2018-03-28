import numpy as np
import time
import requests
import json

class Robot:

    MAX_ID = 0

    def __init__(self, fl, fu, bl, bu, l, w):
        self.frontLower = fl
        self.frontUpper = fu
        self.backLower = bl
        self.backUpper = bu
        self.length = l
        self.width = w
        self.delivery = None
        self.state = 'FREE'
        self.id = Robot.MAX_ID
        Robot.MAX_ID += 1


    def setPlanner(self,planner):
        self.planner = planner

    def setPlan(self,plan):
        self.plan = plan

    def setTracker(self,tracker):
        self.tracker = tracker

    def setRobotDetector(self, rd):
        self.robotDetector = rd

    def setDelivery(self,delivery):
        self.delivery = delivery
        id = delivery['id']
        data = {'state':   'MOVING_TO_SOURCE'}
        r = requests.patch("http://18.219.63.23/development/delivery/" + str(id), json=data)
        self.state = 'MOVING_TO_SOURCE'
        self.tracker.plan = None

    def isFree(self):
        return self.delivery == None

    def deliver(self,frame):
        if self.delivery == None:
            return
        if self.state == 'MOVING_TO_SOURCE':
            if self.tracker.plan == None:
                self.toTarget(self.delivery['from']['name'].lower(), frame)
                self.tracker.start()
            if len(self.tracker.plan) != 0:
                self.tracker.update(frame)
            else:
                self.tracker.stop()
                data = {'state':   'AWAITING_AUTHENTICATION_SENDER'}
                r = requests.patch("http://18.219.63.23/development/delivery/" + str(id), json=data)
                self.state = 'AWAITING_AUTHENTICATION_SENDER'
                self.tracker.plan = None
                self.waitingSince = time.time()
        if self.state == 'AWAITING_AUTHENTICATION_SENDER':
            if time.time() - self.waitingSince > 1:
                self.waitingSince = time.time()
                # WAIT FOR PACKAGE_LOAD_COMPLETE
                state = self.checkState(self.delivery['id'])
                if state == 'PACKAGE_LOAD_COMPLETE':
                    data = {'state':   'MOVING_TO_DESTINATION'}
                    r = requests.patch("http://18.219.63.23/development/delivery/" + str(self.delivery.id), json=data)
                    self.state = 'MOVING_TO_DESTINATION'
        if self.state == 'MOVING_TO_DESTINATION':
            if self.tracker.plan == None:
                self.toTarget(self.delivery['to']['name'].lower(), frame)
                self.tracker.start()
            if len(self.plan) != 0:
                self.tracker.update(frame)
            else:
                self.tracker.stop()
                data = {'state':   'AWAITING_AUTHENTICATION_RECEIVER'}
                r = requests.patch("http://18.219.63.23/development/delivery/" + str(id), json=data)
                self.state = 'AWAITING_AUTHENTICATION_RECEIVER'
                self.plan = None
                self.waitingSince = time.time()
        if self.state == 'AWAITING_AUTHENTICATION_RECEIVER':
            if time.time() - self.waitingSince > 1:
                self.waitingSince = time.time()
                # WAIT FOR PACKAGE_RETRIEVAL_COMPLETE
                state = self.checkState(self.delivery.id)
                if state == 'PACKAGE_RETRIEVAL_COMPLETE':
                    r = requests.delete('http://18.219.63.23/development/delivery/' + str(self.delivery.id))
                    self.state = 'FREE'
                    self.delivery = None

    def toTarget(self, name, frame):
        self.plan = self.planner.plan(name)
        insts = self.pointConversion(self.plan, frame)
        #frame = self.camera.getFrame()
        absolute = self.robotDetector.orientation(frame, self)
        for i in range(len(self.plan) - 1):
            insts[i*2+1]['start'] = (self.plan[i][1],self.plan[i][2])
            insts[i*2+1]['end'] = (self.plan[i+1][1],self.plan[i+1][2])
        for i in range(len(self.plan) - 1):
            insts[i*2]['angle'] = np.arctan2(-self.plan[i+1][2] + self.plan[i][2],
                                            self.plan[i+1][1] - self.plan[i][1])
        self.tracker.setPlan(insts)

    def pointConversion(self, points, frame):
        #frame = self.camera.getFrame()
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
        relativeAngles[0] = angles[0] - self.robotDetector.orientation(frame, self)
        instructions = []
        for i in range(len(relativeAngles)):
            instructions.append({'type':'TURN', 'value':relativeAngles[i]})
            instructions.append({'type':'MOVE', 'value':distances[i]})
        return instructions

    def euclidean(self, a, b):
        D = 1
        dx = abs(a[1] - b[1])
        dy = abs(a[2] - b[2])
        return D * np.sqrt(dx * dx + dy * dy)

    def checkState(self, id):
        r = requests.get("http://18.219.63.23/development/delivery/" + str(id))
        return json.loads(r.text)['state']

    def stop(self):
        self.tracker.stop()
