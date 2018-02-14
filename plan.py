#!/usr/bin/env python2.7
import cv2
import numpy as np

vid = cv2.VideoCapture(0)



class path:
    startPoint = (0,0)
    endPoint = (0,0)
    length = 0.0
    angle = 0

def findRed():
    _, frame = vid.read()
    #frame = cv2.imread("T.png")
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    # lower mask (0-10)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(frame_hsv, lower_red, upper_red)

    # upper mask (170-180)
    lower_red = np.array([170,50,200])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(frame_hsv, lower_red, upper_red)
    mask = mask0+mask1

    output_img = frame.copy()
    output_img[np.where(mask==0)] = 0

    output_hsv = frame_hsv.copy()
    output_hsv[np.where(mask==0)] = 0

    M = cv2.moments(mask)
    while(True):
        cv2.imshow("image", output_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    print(cx,cy)
    return(cx,cy)

def findBlue():
    #_, frame = vid.read()
    frame = cv2.imread("blue.png")
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(frame_hsv, lower_blue, upper_blue)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)

    M = cv2.moments(mask)
    while(True):
        cv2.imshow("image", res)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    print(cx,cy)
    return(cx,cy)


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

findRed()

# def main():
#     print("hello")
#     findT()
#
#     if __name__ == "__main__":
#         main()
