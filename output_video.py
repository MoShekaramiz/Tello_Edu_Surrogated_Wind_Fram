'''Multithreaded video output for the user of the drone. By Branden Pinney and Shayne Duncan 2022.'''

import cv2 as cv
import haar_cascade as hc
from qr_reader import droneReadQR
import threading

class LiveFeed(threading.Thread):
    '''Multithreaded class for providing live video to the user.
    This uses 3 while loops to output either regular video, haar cascade detection,
    or QR code detection.'''
    def __init__(self, drone):
        self.thread = threading.Thread.__init__(self)
        self.haar = threading.Event()
        self.qr = threading.Event()
        self.stop = threading.Event()
        self.drone = drone

    def run(self):
        while not self.stop.is_set():
            while not self.haar.is_set():
                frame = self.drone.get_frame_read()
                img = frame.frame
                img, info = hc.findTurbine(img)
                cv.imshow("Output", img)
                cv.waitKey(1)

            while not self.qr.is_set():
                QR, img, info = droneReadQR(self.drone)
                if len(QR) > 0:
                    # font
                    font = cv.FONT_HERSHEY_PLAIN
                    try:
                        org = (info[0][0], info[3])
                    except:
                        pass
                    
                    fontScale = 1# fontScale
                    color = (0, 255, 0) # Green color in BGR
                    thickness = 2 # Line thickness of 2 px
                    img = cv.putText(img, QR, org, font, fontScale, color, thickness, cv.LINE_AA) # Using cv2.putText() method

                cv.imshow("Output", img)
                cv.waitKey(1)
            frame = self.drone.get_frame_read()
            img = frame.frame
            cv.imshow("Output", img)
            cv.waitKey(1)

    def stop_haar(self):
        '''Turn off the haar cascade'''
        self.haar.set()
    
    def start_haar(self):
        '''Restart the haar cascade'''
        self.haar.clear()
    
    def stop_qr(self):
        '''Stop the qr code detection'''
        self.qr.set()
    
    def start_qr(self):
        '''Restart the qr code detection'''
        self.qr.clear()

    def stop_image(self):
        '''Stop the video feed'''
        self.qr.set()
        self.haar.set()
        self.stop.set()
        self.thread.join()