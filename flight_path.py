from djitellopy import Tello
import cv2 as cv
from time import sleep
import movement as mv
import haar_cascade as hc
import mission
import math

fbRange = [32000, 48000] # preset parameter for detected image boundary size
w, h = 720, 480 # display size of the screen
location = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
turbine_locations = [] # List containing the locations of found turbines
detected_object = 0 # A flag to determine if the camera detected an object in the previous 5 frames

def trackObject(drone, info, location, mission_list, turbine_list, detected_object):
    '''Take the variable for the drone, the output array from calling findTurbine in haar_cascade.py, and the current drone x,y location and relative angle (initialized as [0, 0, 0]).
    It scans for the target object of findTurbine and approaches the target. Once it is at a pre-determined distance fbRange, it will scan and return the value of the QR code
    and return the drone to the starting point.'''
    # Looks for the detected object through Haar Cascades and slowly approaches 
    area = info[1]
    x, y = info[0]
    width = info[2]
    # No object detected
    if(x == 0):
        if detected_object == 0:
            location = mv.move(location, drone, ccw=30)
            sleep(0.5)
        elif detected_object < 5: # An object was detected in on of the previous 5 frames but is no longer present
            detected_object += 1
            if detected_object % 2 == 0:
                #mv.move(location, drone, back=20)
                sleep(0.2)
            else:
                sleep(0.2)
        else:
            detected_object = 0 # 5 frames have passed without detecting an object and the drone will now move on
            sleep(0.1)
    else:
        detected_object = 1
        distance = int((690 * 16) / width) - 50
        if distance < 20:
            distance = 20
        targetx = location[0] + distance * math.cos(math.radians(location[2]))
        targety = location[1] + distance * math.sin(math.radians(location[2]))
        for i in turbine_locations:
            if(i[0] < targetx < i[1]) and (i[2] < targety < i[3]):
                print(">>>>>>>>>>>>>>>>>>>>>>>>>Turbine already located (For debugging)")
                location = mv.move(location, drone, ccw=45)
                sleep(0.5)
                return location, detected_object
        if(0 < x < 300):
            # The drone needs to angle to the left to center the target.
            new_angle = int((x / 360) * 41.3)
            location = mv.move(location, drone, ccw=new_angle)        
            return location, detected_object
        elif(x >= 420):
            # The drone needs to angle to the right to center the target.
            new_angle = int(((x - 360) / 360) * 41.3)
            location = mv.move(location, drone, cw=new_angle)        
            return location, detected_object
        if area > fbRange[0] and area < fbRange[1]:
            # The drone has approached the target and will scan for a QR code
            QR = None 
            while True:
                QR, img, info = droneReadQR(drone)
                if len(QR) > 0:
                    print(">>>>>>>>>>>>>>>>QR CODE FOUND: ", QR)
                    turbine_locations.append([location[0] + 70, location[0] - 70, location[1] + 70, location[1] - 70, QR])
                    turbine_found = 0 # Flag to determine if the correct turbine was found
                    for i in turbine_list:
                        if i == QR:
                            turbine_found = 1
                            #mission.mission0(location, drone, mission_list, QR)
                            mv.return_path(location, drone)
                            drone.streamoff()
                            quit()
                    if turbine_found == 0:
                        mv.move(location, drone, ccw=45)
                        sleep(0.5)
                    break
        elif area > fbRange[1]:
            # The drone is too close to the target
            location = mv.move(location, drone, back=20)
            return location, detected_object
        elif area < fbRange[0] and area != 0:
            # The drone is too far from the target
            location = mv.move(location, drone, distance)
            return location, detected_object
    return location, detected_object

def boundingBox(img, bbox):
    if bbox is not None:
        bbox = [bbox[0].astype(int)]
        n = len(bbox[0])
        for i in range(n):
            cv.line(img, tuple(bbox[0][i]), tuple(bbox[0][(i+1) % n]), (0,255,0), 3)
        width = int(bbox[0][1][0] - bbox[0][3][0])
        center = int((bbox[0][1][0] - bbox[0][3][0]) / 2) + int(bbox[0][3][0])
        area = width ** 2
        info = [[center, 0], area, width]
        return img, info

def droneReadQR(drone):
    '''Takes the drone variable as a parameter and returns the value of any detected qr code data and an output array.'''
    frame = drone.get_frame_read()
    img = frame.frame
    img = cv.resize(img, (w, h))
    qr = cv.QRCodeDetector()
    QR, bbox, s = qr.detectAndDecode(img)
    sleep(0.5)
    info = [[0, 0], 0, 0]
    if len(QR) > 0:
        img, info = boundingBox(img, bbox)
    return QR, img, info

if __name__ == "__main__":
    mission_list = [1, 1, 1, 1]
    turbine_list = ["WindTurbine_2"]
    drone = Tello()
    drone.connect()
    sleep(0.5)
    print("Current battery remaining: ", drone.get_battery())
    sleep(0.3)
    drone.streamon()
    sleep(0.5)
    drone.takeoff()
    sleep(0.5)
    while True:
        frame = drone.get_frame_read()
        sleep(0.2)
        img = frame.frame
        img = cv.resize(img, (w, h))
        #img, info = hc.findTurbine(img)
        QR, img, info = droneReadQR(drone)
        location, detected_object = trackObject(drone, info, location, mission_list, turbine_list, detected_object)
        img = cv.resize(img, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
        cv.imshow("Output", img)
        cv.waitKey(1)