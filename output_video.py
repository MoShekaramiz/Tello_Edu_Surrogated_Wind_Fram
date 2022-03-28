import cv2 as cv
import haar_cascade as hc
import threading
from time import sleep

w, h = 720, 480

class LiveFeed(threading.Thread):
    def __init__(self, drone):
        threading.Thread.__init__(self)
        self.haar = threading.Event()
        self.stop = threading.Event()
        self.drone = drone

    def run(self):
        while not self.haar.is_set():
            frame = self.drone.get_frame_read()
            sleep(0.2)
            img = frame.frame
            img, info = hc.findTurbine(img)
            cv.imshow("Output", img)
            cv.waitKey(1)

        while not self.stop.is_set():
            frame = self.drone.get_frame_read()
            sleep(0.2)
            img = frame.frame
            cv.imshow("Output", img)
            cv.waitKey(1)

    def stop_haar(self):
        self.haar.set()

    def stop_image(self):
        self.stop.set()
