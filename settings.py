import cv2
vid = cv2.VideoCapture(0)
for i in range(16):
    print(str(i)+':'+str(vid.get(i)))
