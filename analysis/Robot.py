import numpy as np
import time
import requests
import json
import math

class Robot:

    '''http://35.177.199.115/development'''

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
        self.plan = None
        self.box = None
        self.center = None
        self.orientation = None
        self.point = None
        self.moveNoDelivery = False
        Robot.MAX_ID += 1


    def isMoving(self):
        return self.state in ['MOVING_TO_SOURCE','MOVING_TO_DESTINATION']

    def setPlanner(self,planner):
        self.planner = planner

    def setPlan(self,plan):
        self.plan = plan

    def getPlan(self):
        return self.plan

    def setTracker(self,tracker):
        self.tracker = tracker

    def setRobotDetector(self, rd):
        self.robotDetector = rd

    def setDelivery(self,delivery):
        self.delivery = delivery
        id = delivery['id']
        data = {'state':   'MOVING_TO_SOURCE','robot':self.id}
        r = requests.patch("http://35.177.199.115/development/delivery/" + str(id), json=data)
        self.state = 'MOVING_TO_SOURCE'
        self.tracker.plan = None

    def contourIntersect(self):
        return 0

    def getBox(self, center, direction, save=False):
        length = self.planner.pathDetector.length
        width = self.planner.pathDetector.width
        p1 = [length/2,width/2]
        p2 = [length/2,-width/2]
        p3 = [-length/2,-width/2]
        p4 = [-length/2,width/2]
        rotP1 = [p1[0] * math.cos(direction) - p1[1] * math.sin(direction),
            p1[0] * math.sin(direction) + p1[1] * math.cos(direction)]
        rotP2 = [p2[0] * math.cos(direction) - p2[1] * math.sin(direction),
            p2[0] * math.sin(direction) + p2[1] * math.cos(direction)]
        rotP3 = [p3[0] * math.cos(direction) - p3[1] * math.sin(direction),
            p3[0] * math.sin(direction) + p3[1] * math.cos(direction)]
        rotP4 = [p4[0] * math.cos(direction) - p4[1] * math.sin(direction),
            p4[0] * math.sin(direction) + p4[1] * math.cos(direction)]
        finP1 = np.array([int(rotP1[0] + center[0]), int(rotP1[1] + center[1])])
        finP2 = np.array([int(rotP2[0] + center[0]), int(rotP2[1] + center[1])])
        finP3 = np.array([int(rotP3[0] + center[0]), int(rotP3[1] + center[1])])
        finP4 = np.array([int(rotP4[0] + center[0]), int(rotP4[1] + center[1])])
        box = np.array([finP1,finP2,finP3,finP4])
        if save:
            self.box = box
        return box

    def isFree(self):
        return self.delivery == None

    def deliver(self,frame):
        print(self.state)
        if self.moveNoDelivery:
            self.moveToPoint(frame)
            return
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
                r = requests.patch("http://35.177.199.115/development/delivery/" + str(self.delivery['id']), json=data)
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
                    r = requests.patch("http://35.177.199.115/development/delivery/" + str(self.delivery['id']), json=data)
                    self.state = 'MOVING_TO_DESTINATION'
        if self.state == 'MOVING_TO_DESTINATION':
            if self.tracker.plan == None:
                print('suh')
                self.toTarget(self.delivery['to']['name'].lower(), frame)
                self.tracker.start()
            if len(self.tracker.plan) != 0:
                print('hey')
                self.tracker.update(frame)
            else:
                print('yoooo')
                self.tracker.stop()
                data = {'state':   'AWAITING_AUTHENTICATION_RECEIVER'}
                r = requests.patch("http://35.177.199.115/development/delivery/" + str(self.delivery['id']), json=data)
                self.state = 'AWAITING_AUTHENTICATION_RECEIVER'
                self.tracker.plan = None
                self.waitingSince = time.time()
        if self.state == 'AWAITING_AUTHENTICATION_RECEIVER':
            if time.time() - self.waitingSince > 1:
                self.waitingSince = time.time()
                # WAIT FOR PACKAGE_RETRIEVAL_COMPLETE
                state = self.checkState(self.delivery['id'])
                if state == 'PACKAGE_RETRIEVAL_COMPLETE':
                    r = requests.delete('http://35.177.199.115/development/delivery/' + str(self.delivery['id']))
                    self.state = 'FREE'
                    self.delivery = None

    def moveToPoint(self, frame):
        if self.tracker.plan == None:
            self.toPoint(self.points, frame)
            self.tracker.start()
        if len(self.tracker.plan) != 0:
            self.tracker.update(frame)
        else:
            self.tracker.stop()
            self.tracker.plan = None
            self.point = None
            self.moveNoDelivery = False

    def toPoint(self, points, frame):
        current = self.robotDetector.center(frame, self)
        pl = [(1,int(current[0]),int(current[1]))]
        for p in range(len(points)):
            pl.append((p+2,int(points[p][0]),int(points[p][1])))
        self.plan = pl
        insts = self.pointConversion(self.plan, frame)
        absolute = self.robotDetector.orientation(frame, self)
        for i in range(len(self.plan) - 1):
            insts[i*2+1]['start'] = (self.plan[i][1],self.plan[i][2])
            insts[i*2+1]['end'] = (self.plan[i+1][1],self.plan[i+1][2])
        for i in range(len(self.plan) - 1):
            insts[i*2]['angle'] = np.arctan2(-self.plan[i+1][2] + self.plan[i][2],
                                            self.plan[i+1][1] - self.plan[i][1])
        self.tracker.setPlan(insts)

    def toTarget(self, name, frame):
        self.plan = self.planner.plan(name)
        insts = self.pointConversion(self.plan, frame)
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
        r = requests.get("http://35.177.199.115/development/delivery/" + str(id))
        return json.loads(r.text)['state']

    def stop(self):
        self.tracker.stop()
