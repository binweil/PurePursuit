import numpy as np
import cv2
import math

cap = cv2.VideoCapture(0)
theta = 0
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    frame = frame[300:400, 200:800]
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)
    canny = cv2.Canny(blur,50,50,apertureSize=3)

    lines = cv2.HoughLinesP(canny,1,np.pi/180,10,10,2)
    if lines is not None:
        for x in range(0,len(lines)):
            for x1,y1,x2,y2 in lines[x]:
                cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),2)
                theta = theta + math.atan2((y2-y1),(x2-x1))
    # Display the resulting frame
    cv2.imshow('frame',frame)
    print(theta)
    theta = 0
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()