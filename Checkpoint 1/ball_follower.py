#!/usr/bin/env python3

import cv2
import numpy as np
#import imutils

cap = cv2.VideoCapture(0)
prevcircle = None
dist = lambda x1,y1,x2,y2: (x1-x2)**2+(y1-y2)**2

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret: break

    #frame=imutils.resize(frame,width=500)
    # Our operations on the frame come here
    grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    Gaussframe = cv2.GaussianBlur(grayframe, (15,15),1.5,1.5)
    blurframe = cv2.medianBlur(Gaussframe,7)
    cv2.imshow('Median Filter',blurframe)
    colorframe = cv2.cvtColor(grayframe, cv2.COLOR_GRAY2BGR)

    circles = cv2.HoughCircles(blurframe, cv2.HOUGH_GRADIENT, 0.6, 150, param1 = 100, param2 = 30, minRadius=50, maxRadius=0)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        chosen  = None
        for i in circles[0,:]:
            if chosen is None: chosen = i
            if prevcircle is not None:
                if dist(chosen[0],chosen[1],prevcircle[0],prevcircle[1]) == dist(i[0],i[1],prevcircle[0],prevcircle[1]):
                    chosen = i

        cv2.circle(frame, (chosen[0],chosen[1]),1,(0,100,100),3)    
        cv2.circle(frame, (chosen[0],chosen[1]),chosen[2],(255,0,255),3)

        #print(chosen[0],chosen[1])

        height, width = frame.shape[:2]
        if width//2<chosen[0]:
            print("Right!")
        else:
            print("Left")
        prevcircle = chosen
        print(chosen.shape)
        

    #Display the resulting frame
    #cv2.imshow('Color frame',frame)
    cv2.imshow('Color frame',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break 

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()