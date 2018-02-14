#!/usr/bin/env python2.7


import numpy as np
import cv2

def blend_transparent(capture, overlay):
    # Split out the transparency mask from the colour info
    overlay_img = overlay[:,:,:3]
    overlay_mask = overlay[:,:,3:]
    background_mask = 255 - overlay_mask

    overlay_mask = cv2.cvtColor(overlay_mask, cv2.COLOR_GRAY2BGR)
    background_mask = cv2.cvtColor(background_mask, cv2.COLOR_GRAY2BGR)

    # Create a masked out face image, and masked out overlay
    capture_part = (capture * (1 / 255.0)) * (background_mask * (1 / 255.0))
    overlay_part = (overlay_img * (1 / 255.0)) * (overlay_mask * (1 / 255.0))

    return np.uint8(cv2.addWeighted(capture_part, 255.0, overlay_part, 255.0, 0.0))



cap = cv2.VideoCapture(1)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
#print("is it opened?", cap.isOpened())
overlay = cv2.imread("Yellow.png", -1)



while(cap.isOpened()):
    ret, frame = cap.read()
    o = blend_transparent(frame, overlay)
    cv2.imshow('frame',frame)
    #cv2.cvtColor(o, cv2.COLOR_BGR2RGB)
    cv2.imshow("r", o[:,:,0])
    cv2.imshow("g", o[:,:,1])
    cv2.imshow("b", o[:,:,2])
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()




#capture = cv2.imread("test.jpg")
 # Load with transparency

#cv2.imwrite("merged.png", blend_transparent(capture, overlay))
