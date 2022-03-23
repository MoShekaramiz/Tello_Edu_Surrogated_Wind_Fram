#### This code was written following the tutorial located at https://www.youtube.com/watch?v=LmEcyQnfpDA&t=1890s ####

import numpy as np
import cv2 as cv

def findTurbine(img):
    '''Take an input image and searches for the target object using an xml file. 
    Returns the inupt image with boundaries drawn around the detected object and the x and y values of the center of the target in the image
    as well as the area of the detection boundary.'''
    # Use Haar Cascades to detect objects using the built-in classifier tool
    cascade = cv.CascadeClassifier("fan_cascade_1.xml")
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    turbines = cascade.detectMultiScale(gray, 1.2, 8)

    turbineListC = []
    turbineListArea = []

    for (x,y,w,h) in turbines:
        # draw a rectangle around the detected face
        # code for creating a rectangle to see dectection boundaries -- 
        cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # determine the center of the detection boundaries and the area
        centerX = x + w // 2
        centerY = y + h // 2
        area = w * h
        # draw a circle in the center
        # cv.circle(img, (centerX, centerY), 5, (0, 255, 0), cv.FILLED)
        turbineListC.append([centerX, centerY])
        turbineListArea.append(area)
    if len(turbineListArea) != 0: 
        # if there is items in the area list, find the maximum value and return
        i = turbineListArea.index(max(turbineListArea))
        return img, [turbineListC[i], turbineListArea[i], w]
    else:
        return img, [[0, 0], 0, 0]

if __name__ == "__main__":
    cap = cv.VideoCapture(0)
    while True:
        _, img = cap.read()
        findTurbine(img)
        img = cv.resize(img, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
        cv.imshow("Output", img)
        cv.waitKey(1)