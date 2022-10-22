import haar_cascade as hc
import cv2 as cv
from time import sleep

w, h = 720, 480 # display size of the screen

def check_camera(drone, circles=False):
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