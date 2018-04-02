import cv2
import numpy as np
image_src = None
image_hsv = None   # global ;(
pixel = (20,60,80) # some stupid default

# mouse callback function
def pick_color(event,x,y,flags,param):
    print(x,y)
    if event == cv2.EVENT_LBUTTONDOWN:
        pixel = image_src[y,x]

        #you might want to adjust the ranges(+-10, etc):
        upper =  np.array([pixel[0] + 20, pixel[1] + 20, pixel[2] + 20])
        lower =  np.array([pixel[0] - 20, pixel[1] - 20, pixel[2] - 20])
        print(pixel, lower, upper)

        image_mask = cv2.inRange(image_src,lower,upper)
        cv2.imshow("mask",image_mask)

def main():
    import sys
    global image_hsv, pixel, image_src # so we can use it in mouse callback
    cap = cv2.VideoCapture(-1)
    image_src = cap.read()[1]  # pick.py my.png
    if image_src is None:
        print ("the image read is None............")
        return
    cv2.imshow("bgr",image_src)

    ## NEW ##
    cv2.namedWindow('hsv')
    cv2.setMouseCallback('hsv', pick_color)

    # now click into the hsv img , and look at values:
    image_hsv = cv2.cvtColor(image_src,cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv",image_hsv)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()
