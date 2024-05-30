'''Haar Cascade detection through OpenCV using the OpenCV documentation. This version includes the blue circles for the landing pads.
By Branden Pinney, Quintin Jepsen and Shayne Duncan 2022.'''

import cv2 as cv
import os
import numpy as np
from djitellopy import Tello

CWD = os.getcwd()

def findTurbine(img, cascade=0):
    '''Take an input image and searches for the target object using an xml file. 
    Returns the inupt image with boundaries drawn around the detected object and the x and y values of the center of the target in the image
    as well as the area of the detection boundary.'''
    # Use Haar Cascades to detect objects using the built-in classifier tool
    if cascade == 0:
        cascade = cv.CascadeClassifier(CWD + "\outdoor_cascade.xml")
    else:
        cascade = cv.CascadeClassifier(CWD + "\haarcascade_eye.xml")

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    turbines = cascade.detectMultiScale(gray, 1.2, 8)

    turbineListC = []
    turbineListArea = []
    # turbineListW = []

    for (x,y,w,h) in turbines:
        # draw a rectangle around the detected object
        # code for creating a rectangle to see dectection boundaries --
        cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # determine the center of the detection boundaries and the area
        centerX = x + w // 2
        centerY = y + h // 2
        area = w * h
        turbineListC.append([centerX, centerY])
        turbineListArea.append(area)
    if len(turbineListArea) != 0:
        # if there is items in the area list, find the maximum value and return
        i = turbineListArea.index(max(turbineListArea))
        return img, [turbineListC[i], turbineListArea[i], w]
    else:
        return img, [[0, 0], 0, 0]

def find_circles(img, down=True, green=False):
    radius = 0
    x_center = 0
    if down == True:
        img = cv.medianBlur(img,5)   

        cimg = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # cimg = cv.cvtColor(img,cv.COLOR_GRAY2BGR)
        circles = cv.HoughCircles(cimg, cv.HOUGH_GRADIENT, 1, 120,
                            param1=200, param2=40,
                            minRadius = 85, maxRadius = 125)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                # draw the outer circle
                cv.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv.circle(img,(i[0],i[1]),2,(0,0,255),3)
        if green == True:
            return img, circles
        else:
            return img, circles, radius, x_center
    else:
        if green == True:
            greenLower = (29, 86, 6)
            greenUpper = (64, 255, 255)
        else:
            # blueLower = (40, 95, 95)
            #orig LOW (40, 115, 115), 50, 75
            # Angel
            blueLower = (10, 60, 87)
            # blueUpper = (115, 255, 255)
            #orig HIGH (102, 255, 255)
            # Angel
            blueUpper = (43, 106, 141)
        # greenLower = (40, 97, 20)
        # greenUpper = (64, 255, 255)
        blurred = cv.GaussianBlur(img, (11, 11), 0)
        hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
        if green == True:
            mask = cv.inRange(hsv, greenLower, greenUpper)
        else:
            mask = cv.inRange(hsv, blueLower, blueUpper)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)
        output = cv.bitwise_and(img, img, mask = mask)
        cimg = cv.cvtColor(output, cv.COLOR_BGR2GRAY)
        if green == False:
            circles = cv.HoughCircles(cimg, cv.HOUGH_GRADIENT, 1, 120,
                            param1=80, param2=20,
                            minRadius = 5, maxRadius = 250)
        else:
            circles = cv.HoughCircles(cimg, cv.HOUGH_GRADIENT, 1, 120,
                            param1=80, param2=35, 
                            minRadius = 15, maxRadius = 200) # original: 80


        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                print(i[0])
                # draw the outer circle
                cv.circle(output,(i[0],i[1]),i[2],(0,255,0),2)
                radius = 2 * i[2]
                x_center = i[0]
                # draw the center of the circle
                cv.circle(output,(i[0],i[1]),2,(0,0,255),3)
        if green == True:
            return output, circles
        else:
            return output, circles, radius, x_center

if __name__ == "__main__":
    # cap = cv.VideoCapture(0)
    # while True:
    #     _, img = cap.read()
    #     img, circles = find_circles(img, False, False)
    #     #img = cv.resize(img, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
    #     cv.imshow("Output", img)
    #     cv.waitKey(1)

    drone = Tello()
    drone.connect()

    drone.streamon()
    while True:
        frame = drone.get_frame_read()
        img = frame.frame

        # Below line is used for green circles since green circles returns only 2 values
        # img, info = find_circles(img, down=False, green=True) 
        
        # Below line is used for blue circles since blue circles returns 4 values
        img, info, radius, center = find_circles(img, down=False, green=False)
        cv.imshow("Output", img)
        cv.waitKey(1)