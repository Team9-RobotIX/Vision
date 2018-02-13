#!/usr/bin/env python2.7
import cv2
import numpy as np

#vid = cv2.VideoCapture(0)



class path:
    startPoint = (0,0)
    endPoint = (0,0)
    length = 0.0
    angle = 0

def findT():
    #_, frame = vid.read()
    frame = cv2.imread("T.jpg")

    lower_red = np.array([173, 3, 6])
    upper_red = np.array([255, 104, 107])
    mask = cv2.inRange(frame, upper_red, lower_red)
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)
    M = cv2.moments(mask)
    # cx = int(M['m10']/M['m00'])
    # cy = int(M['m01']/M['m00'])
    while(True):
        cv2.imshow("image", mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    return


def rotateT (image):
    #This function calculates the angle between the thresholded T and the control T.
    return

def findAngle(robPos, targetPos):
    rx = robPos[0]
    ry = robPos[1]
    tx = targetPos[0]
    ty = targetPos[1]
    return math.degrees(math.atan2(tx-rx,ty-ry))

def angle_normalize(self,x):
    return



def findDistance(robPos, targetPos):
    #This function calculates the shortest distance between the target and the robot.
    #In pixel terms for now. (will be converted to x,y for easier understanding)
    rx = robPos[0]
    ry = robPos[1]
    tx = targetPos[0]
    ty = targetPos[1]
    return np.sqrt(np.square(tx-rx)+np.square(ty-ry))

def convertToInstruction(distance, angle):
    data = {'instruction': 'MOVE', 'value': '500'}
    turn = {'instruction': 'TURN', 'value': angle}
    ###wait for callback to say turning is complete
    move = {'instruction': 'MOVE', 'value': distance}
    return newData

def main():
    print("hello")

    if __name__ == "__main__":
        main()
