from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
from picamera import PiCamera
import time
import cv2
import numpy as np
import math

in1 = 17
in2 = 27
en_a = 13


GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en_a,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.setup(22,GPIO.OUT)
servo1 = GPIO.PWM(22,50)
servo1.start(0)
q=GPIO.PWM(en_a,100)
q.start(30)
#7 = 22
#8= 13

theta=0

camera = PiCamera()
camera.resolution = (240, 120)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(240,120))
time.sleep(0.1)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
   image = frame.array
 hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  lower_blue = np.array([0, 0, 212])
  upper_blue = np.array([131, 255, 255])

  mask = cv2.inRange(hsv, lower_blue, upper_blue)

   #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
   #blurred = cv2.GaussianBlur(gray, (5, 5), 0)
   #edged = cv2.Canny(blurred, 85, 85)
   lines = cv2.HoughLinesP(mask,1,np.pi/180,10,minLineLength,maxLineGap)
   if(lines.any()):
       for x in range(0, len(lines)):
           for x1,y1,x2,y2 in lines[x]:
               cv2.line(image,(x1,y1),(x2,y2),(0,255,0),2)
               theta=theta+math.atan2((y2-y1),(x2-x1))
   threshold=6
   if(theta>threshold):
       angle = 180#float(input('Enter angle between 0 & 180: '))
       servo1.ChangeDutyCycle(2+(angle/18))
       time.sleep(0.25)
       servo1.ChangeDutyCycle(0)
       print(angle)
       GPIO.output(in1,GPIO.LOW)
       GPIO.output(in2,GPIO.HIGH) 
       print("right")
   elif(theta<-threshold):
       angle = 25#float(input('Enter angle between 0 & 180: '))
       servo1.ChangeDutyCycle(2+(angle/18))
       time.sleep(0.25)
       servo1.ChangeDutyCycle(0)
       print(angle)
       GPIO.output(in1,GPIO.LOW)
       GPIO.output(in2,GPIO.HIGH)
       print("left")
   elif(abs(theta)<threshold):
       angle = 90#float(input('Enter angle between 0 & 180: '))
       servo1.ChangeDutyCycle(2+(angle/18))
       time.sleep(0.25)
       servo1.ChangeDutyCycle(0)
       print(angle)
       GPIO.output(in1,GPIO.LOW)
       GPIO.output(in2,GPIO.HIGH)
       print("straight")
   theta=0
   cv2.imshow("Frame",image)
   key = cv2.waitKey(1) & 0xFF
   rawCapture.truncate(0)
   if key == ord("q"):
       break
