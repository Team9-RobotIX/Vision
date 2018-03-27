import numpy as np
import cv2

class RobotDetector:

    def __init__(self,camera):
        self.camera = camera
        self.enableSettings()
        (self.length, self.width) = self.getSize()

    def getSize(self):
        print('get size')
        self.enableSettings()
        image = self.camera.getFrame()
        #image = image[self.crop['y']:self.crop['y'] + self.crop['h'],
        #              self.crop['x']:self.crop['x'] + self.crop['w']]
        #maskRed = cv2.inRange(image, (0,0,180),(50,200,295))
        maskBlue = cv2.inRange(image, (200,0,0),(255,150,150))
        kernel = np.ones((5,5),np.uint8)
        #maskRed = cv2.dilate(maskRed,kernel,iterations=3)
        maskBlue = cv2.dilate(maskBlue,kernel,iterations=3)

        img, blueContour, hier = cv2.findContours(maskBlue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        x,y,w,h = cv2.boundingRect(blueContour[0])
        print(w,h)
        return (h*2.5, w*2.5)

        '''maskBlue = cv2.dilate(maskBlue,kernel,iterations=3)
        MR = cv2.moments(maskRed)
        MB = cv2.moments(maskBlue)

        rx = int(MR['m10']/MR['m00'])
        ry = int(MR['m01']/MR['m00'])
        bx = int(MB['m10']/MB['m00'])
        by = int(MB['m01']/MB['m00'])

        mask = cv2.bitwise_or(maskRed, maskBlue)
        cv2.line(mask,(rx,ry),(bx,by),thickness=5,color=255)

        img2, contour, hier =  cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        box = cv2.boxPoints(cv2.minAreaRect(contour[0]))
        c1 = box[0]
        c2 = box[1]
        c3 = box[3]
        length = int(np.linalg.norm(c1 - c2))
        width = int(np.linalg.norm(c1 - c3))
        if length < width:
            temp = length
            length = width
            width = temp
        print(length * 2, width * 3)
        return (length * 2, width * 3)'''

    def findRed(self,image):
        mask = cv2.inRange(image, (0,0,225),(50,200,295))
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel,iterations=3)
        M = cv2.moments(mask)
        if M['m00'] == 0:
            M['m00'] = 0.0000000001
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return (cx,cy)

    def findBlue(self,image):
        mask = cv2.inRange(image, (200,0,0),(255,150,150))
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel,iterations=3)

        M = cv2.moments(mask)
        if M['m00'] == 0:
            M['m00'] = 0.0000000001
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        return (cx,cy)


    def center(self,image,newFrame=False):
        if newFrame:
            self.enableSettings()
            image = self.camera.getFrame()
        return np.mean(np.array([self.findRed(image), self.findBlue(image)]), axis=0, dtype=int)

    def orientation(self,image, newFrame=False):
        if newFrame:
            self.enableSettings()
            image = self.camera.getFrame()
        #define rx and ry as the origin
        (bx, by) = self.findBlue(image)
        (rx, ry) = self.findRed(image)
        vector = (bx - rx, by - ry)
        angle = np.arctan2(-vector[1], vector[0])
        return angle

    def enableSettings(self):
        settings = [(3,800.0),
                    (4,800.0),
                    (5,30.0),
                    (9,0.0),
                    (10,0.48627451062202454),
                    (11,0.125490203499794),
                    (12,0.15294118225574493),
                    (14,0.05098039284348488)]
        self.camera.enableSettings(settings)
