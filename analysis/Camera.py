import cv2
import numpy as np
from time import sleep

class Camera:

    def __init__(self):
        self.settings = []
        for i in range(16):
            self.settings.append(0)
        self.crop = None
        self.videoFeed = cv2.VideoCapture(-1)

    def enableSettings(self,settings):
        updated = False
        for setting in settings:
            current = self.settings[setting[0]]
            if current != setting[1]:
                updated = True
                self.settings[setting[0]] = setting[1]
                self.videoFeed.set(setting[0],setting[1])
        if updated:
            self.getFrame()
            self.getFrame()
            self.getFrame()
            self.getFrame()
            self.getFrame()


    def getFrame(self,cropped=True):
        ret, frame = self.videoFeed.read()
        if cropped and not self.crop == None:
            return frame[self.crop['y']:self.crop['y'] + self.crop['h'],
                          self.crop['x']:self.crop['x'] + self.crop['w']]
        else:
            return frame

    def setCrop(self,crop):
        self.crop = crop
        self.size = (self.crop['h'],self.crop['w'])

    def isOpened(self):
        return self.videoFeed.isOpened()

    def release(self):
        self.videoFeed.release()
