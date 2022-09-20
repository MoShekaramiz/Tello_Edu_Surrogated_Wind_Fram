'''Haar Cascade detection through OpenCV using the OpenCV documentation. By Branden Pinney and Shayne Duncan 2022.'''

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

def find_circles(img, down=True):
    if down == True:
        img = cv.medianBlur(img,5)   
        # clahe = cv.createCLAHE(clipLimit=3., tileGridSize=(8,8))

        # lab = cv.cvtColor(img, cv.COLOR_BGR2LAB)  # convert from BGR to LAB color space
        # l, a, b = cv.split(lab)  # split on 3 different channels

        # l2 = clahe.apply(l)  # apply CLAHE to the L-channel

        # lab = cv.merge((l2,a,b))  # merge channels
        # img = cv.cvtColor(lab, cv.COLOR_LAB2BGR)  # convert from LAB to BGR
        # img = cv.normalize(img, None, alpha=-1.5*255, beta=1.5*255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U)
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
        return img, circles
    else:
        greenLower = (29, 86, 6)
        greenUpper = (64, 255, 255)
        # greenLower = (40, 97, 20)
        # greenUpper = (64, 255, 255)
        blurred = cv.GaussianBlur(img, (11, 11), 0)
        hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, greenLower, greenUpper)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)
        output = cv.bitwise_and(img, img, mask = mask)
        cimg = cv.cvtColor(output, cv.COLOR_BGR2GRAY)
        circles = cv.HoughCircles(cimg, cv.HOUGH_GRADIENT, 1, 120,
                            param1=80, param2=25,
                            minRadius = 15, maxRadius = 80)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                print(i[0])
                # draw the outer circle
                cv.circle(output,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv.circle(output,(i[0],i[1]),2,(0,0,255),3)
        return output, circles

if __name__ == "__main__":
    cap = cv.VideoCapture(0)
    while True:
        _, img = cap.read()
        img, circles = find_circles(img, False)
        #img = cv.resize(img, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
        cv.imshow("Output", img)
        cv.waitKey(1)

    # drone = Tello()
    # drone.connect()

    # drone.streamon()
    # while True:
    #     frame = drone.get_frame_read()
    #     img = frame.frame
    #     img, info = find_circles(img, down=False)
    #     cv.imshow("Output", img)
    #     cv.waitKey(1)
