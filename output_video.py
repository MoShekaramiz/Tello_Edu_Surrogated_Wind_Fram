import cv2 as cv
import haar_cascade as hc
from qr_reader import droneReadQR
import threading
from time import sleep

w, h = 720, 480

class LiveFeed(threading.Thread):
    def __init__(self, drone):
        threading.Thread.__init__(self)
        self.haar = threading.Event()
        self.qr = threading.Event()
        self.stop = threading.Event()
        self.drone = drone

    def run(self):
        while not self.stop.is_set():
            while not self.haar.is_set():
                frame = self.drone.get_frame_read()
                sleep(0.2)
                img = frame.frame
                img, info = hc.findTurbine(img)
                cv.imshow("Output", img)
                cv.waitKey(1)
            while not self.qr.is_set():
                QR, img, info = droneReadQR(self.drone)
                if len(QR) > 0:
                    # font
                    font = cv.FONT_HERSHEY_PLAIN
                    
                    # org
                    try:
                        org = (info[0][0], info[3])
                    except:
                        pass
                    # fontScale
                    fontScale = 1
                    
                    # Blue color in BGR
                    color = (0, 255, 0)
                    
                    # Line thickness of 2 px
                    thickness = 2
                    
                    # Using cv2.putText() method
                    img = cv.putText(img, QR, org, font, fontScale, color, thickness, cv.LINE_AA)
                cv.imshow("Output", img)
                cv.waitKey(1)
            frame = self.drone.get_frame_read()
            sleep(0.2)
            img = frame.frame
            cv.imshow("Output", img)
            cv.waitKey(1)

    def stop_haar(self):
        self.haar.set()
    
    def start_haar(self):
        self.haar.clear()
    
    def stop_qr(self):
        self.qr.set()
    
    def start_qr(self):
        self.qr.clear()

    def stop_image(self):
        self.stop.set()
