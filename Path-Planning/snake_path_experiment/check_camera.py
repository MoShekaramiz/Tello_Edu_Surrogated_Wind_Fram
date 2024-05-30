'''Written by Branden Pinney 2022'''
import haar_cascade as hc
import cv2 as cv
from time import sleep

w, h = 720, 480 # display size of the screen

def check_camera(drone, circles=False):
    '''Takes the lower drone class frome drone.get_drone(), a boolean parameter for circle detection, and returns
    the information from the object detection. circles=False returns the information of the haar-cascade detection.
    circles=True returns the image, the list of circles, the width, and center pixel of the detected circle.'''
    frame = drone.get_frame_read()
    sleep(0.2)
    img = frame.frame
    img = cv.resize(img, (w, h))
    if circles == False:
        img, info = hc.findTurbine(img)
    else:
        img, circle, width, center = hc.find_circles(img)
        return img, circle, width, center
    return info