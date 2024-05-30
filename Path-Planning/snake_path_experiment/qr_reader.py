'''Module to assist with detecting QR codes in OpenCV. By Branden Pinney and Shayne Duncan 2022.'''
import pyqrcode
import cv2 as cv

  
def make_QR(names):
    for name in names:
        code = pyqrcode.create(name)
        # code.png(name + '.png', scale=12)

def boundingBox(img, bbox):
    '''Creates a bounding box around the detected QR code'''
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
    info = [[0, 0], 0, 0]
    if len(QR) > 0:
        img, info = boundingBox(img, bbox)

    return QR, img, info

if __name__ == "__main__":
    make_QR(["WindTurbine_1", "WindTurbine_2", "WindTurbine_3", "WindTurbine_4", "WindTurbine_5",
             "WindTurbine_6", "WindTurbine_7", "WindTurbine_8", "WindTurbine_9", "WindTurbine_10"])