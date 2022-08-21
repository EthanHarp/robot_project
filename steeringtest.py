import cv2
import picamera
import io
import numpy as np
import time
from adafruit_motorkit import MotorKit

kit = MotorKit()

camera = picamera.PiCamera()
camera.resolution = (648, 486)

i = 0
j = 0
while i == 0:
    stream = io.BytesIO()
    camera.capture(stream, format='jpeg')

    buff = np.frombuffer(stream.getvalue(), dtype=np.uint8)

    #Now creates an OpenCV image
    image = cv2.imdecode(buff, 1)
    crop_img = image[50:486, 0:648]
    #crop_img = image
    blurImg = cv2.GaussianBlur(crop_img, (3,3), 0)

    imgGray = cv2.cvtColor(blurImg, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(imgGray, 50, 255, cv2.THRESH_BINARY)
    imgNot = cv2.bitwise_not(thresh)

    contours, hierarchy = cv2.findContours(image=imgNot, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
    imgCon = image.copy()

    avgX = 0
    avgY = 0
    total = 0
    for c in contours:
        area = cv2.contourArea(c)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        # convert all coordinates floating point values to int
        box = np.int0(box)
        # draw a green 'nghien' rectangle
        if area>1000:
            cv2.drawContours(imgCon, [box], 0, (0, 255, 0),1)
            avgX += (box[0][0] + box[1][0] + box[2][0] + box[3][0]) / 4
            avgY += (box[0][1] + box[1][1] + box[2][1] + box[3][1]) / 4
            total += 1
            #print(box[0][0])
            #print(box[1][0])
            #print('\n')
            #print([box])
    if total > 0:
        avgX = int(avgX)
        avgX = int(avgX / total)
        avgY = int(avgY)
        avgY = int(avgY / total)
        print(avgX)
        #print(avgY)
        print(avgX - 324)
        
        imgCon = cv2.circle(imgCon, (avgX, avgY), radius = 10, color=(0, 0, 255), thickness=-1)
        imgCon = cv2.circle(imgCon, (324, 243), radius = 10, color=(0, 255, 255), thickness=-1)
        #cv2.namedWindow('contours', cv2.WINDOW_NORMAL)
        #cv2.resizeWindow('contours', 600,600)
        #cv2.imshow('contours', imgCon)
    
        #NOW FOR MOVEMENT
        if avgX - 324 < -50:
            leftSpeed = .2
            rightSpeed = -.3
            sleep = .2
            print('LEFT')
        elif avgX - 324 > 50:
            leftSpeed = .3
            rightSpeed = -.2
            sleep = .2
            print('RIGHT')
        else:
            leftSpeed = .3
            rightSpeed = -.3
            sleep = .3
            print('STRAIGHT')
   
    else:
        #cv2.destroyAllWindows()
        print('nothing found')
        #i += 1
        leftSpeed = 0
        rightSpeed = 0
    kit.motor1.throttle = leftSpeed
    kit.motor2.throttle = rightSpeed
    #time.sleep(sleep)
    print('\n')
kit.motor1.throttle = 0
kit.motor2.throttle = 0
camera.close()