## OBJECT DETECTION USING RASPBERRY PI 3

#!/usr/bin/env python3
 
#Import only if not previously imported
 
import cv2
 
import RPi.GPIO as GPIO
 
from time import sleep
 
from ultrasonic import *
 
 
 
myPin=7
 
myPin2=8
 
GPIO.setmode(GPIO.BCM)
 
GPIO.setwarnings(False)
 
GPIO.setup(myPin,GPIO.OUT)
 
GPIO.setup(myPin2,GPIO.OUT)
 
# In VideoCapture object either Pass address of your Video file
 
# Or If the input is the camera, pass 0 instead of the video file
 
cap = cv2.VideoCapture(-1)
 
cap.set(3,480)
 
cap.set(4,320)
 
if cap.isOpened() == False:
 
    print("Error in opening video stream or file")
 
while(cap.isOpened()):
 
    ret, frame = cap.read()
 
    #dist = distance()
 
    #print(dist)
 
    #if (dist < 13):
 
        #GPIO.output(myPin2, GPIO.HIGH)
 
   
 
    if ret:
 
 
 
           
 
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
 
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
 
        parameters = cv2.aruco.DetectorParameters_create()
 
        corners, ids, rejected = cv2.aruco.detectMarkers(
 
            frame, aruco_dict, parameters=parameters)
 
 
 
        if ids in range(0,10) and corners:
 
            print("Friend")
 
            #cv2.aruco.drawDetectedMarkers(frame,corners, ids)
 
        # Display the resulting frame
 
        elif ids in range(10,20) and corners:
 
            print("Enemy")
 
            GPIO.output(myPin, GPIO.HIGH)
 
            state = GPIO.input(myPin)
 
            print(state)
 
            print("HIGH")
 
 
 
            cv2.aruco.drawDetectedMarkers(frame,corners,ids)
 
        else:
 
            print("No TAG")
 
        cv2.imshow('Frame',frame)
 
        # Press esc to exit
 
        if cv2.waitKey(20) & 0xFF == 27:
 
            break
 
        GPIO.output(myPin, GPIO.LOW)
 
        state = GPIO.input(myPin)
 
        print(state,"low")
 
        print("LOW")
 
    else:
 
        break
 
cap.release()
 
cv2.destroyAllWindows()
