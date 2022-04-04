import cv2 as cv
from time import sleep

def boundingBox(img, bbox):
    if bbox is not None:
        bbox = [bbox[0].astype(int)]
        n = len(bbox[0])
        for i in range(n):
            cv.line(img, tuple(bbox[0][i]), tuple(bbox[0][(i+1) % n]), (0,255,0), 3)
        width = int(bbox[0][1][0] - bbox[0][3][0])
        center = int((bbox[0][1][0] - bbox[0][3][0]) / 2) + int(bbox[0][3][0])
        area = width ** 2
        height = bbox[0][0][0]
        info = [[center, 0], area, width, height]
        return img, info

def droneReadQR(drone):
    '''Takes the drone variable as a parameter and returns the value of any detected qr code data and an output array.'''
    frame = drone.get_frame_read()
    img = frame.frame
    qr = cv.QRCodeDetector()
    QR, bbox, s = qr.detectAndDecode(img)
    sleep(0.5)
    info = [[0, 0], 0, 0]
    if len(QR) > 0:
        img, info = boundingBox(img, bbox)
    return QR, img, info