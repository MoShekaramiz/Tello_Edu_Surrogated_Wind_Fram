'''The main object detection and drone flight module. By Branden Pinney and Shayne Duncan 2022'''

from time import sleep
import math
import cv2 as cv
from djitellopy import Tello
import mission
from qr_reader import droneReadQR
import movement_test as mv
from output_video import LiveFeed
import haar_cascade as hc

FB_RANGE = [62000, 82000]#[32000, 52000] # preset parameter for detected image boundary size
W, H = 720, 480 # display size of the screen
LOCATION = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
TURBINE_LOCATIONS = [] # List containing the LOCATIONs of found turbines
DETECTED_OBJECT = 0 # A flag to determine if the camera detected an object in the previous 5 frames

def track_object(drone, info, LOCATION, turbines, DETECTED_OBJECT):
    '''Take the variable for the drone, the output array from calling findTurbine in haar_cascade.py,
    and the current drone x,y LOCATION and relative angle (initialized as [0, 0, 0]).
    It scans for the target object of findTurbine and approaches the target.
    Once it is at a pre-determined distance FB_RANGE, it will scan and return the value of the QR code
    and return the drone to the starting point.'''
    # Looks for the detected object through Haar Cascades and slowly approaches
    area = info[1]
    x, y = info[0]
    width = info[2]
    # No object detected
    if(x == 0):
        if DETECTED_OBJECT == 0:
            LOCATION = mv.move(LOCATION, drone, ccw=25)
            sleep(1)

        elif DETECTED_OBJECT < 5: # An object was detected in on of the previous 5 frames but is no longer present
            DETECTED_OBJECT += 1
            if DETECTED_OBJECT % 2 == 0:
                #mv.move(LOCATION, drone, back=20)
                sleep(0.2)
            else:
                sleep(0.2)

        else:
            DETECTED_OBJECT = 0 # 5 frames have passed without detecting an object and the drone will now move on
            sleep(0.1)

    else:
        DETECTED_OBJECT = 1
        distance = int((650 * 40.64) / width) - 40
        if distance < 20:
            distance = 20
        
        targetx = LOCATION[0] + distance * math.cos(math.radians(LOCATION[2]))
        targety = LOCATION[1] + distance * math.sin(math.radians(LOCATION[2]))

        for i in TURBINE_LOCATIONS:
            if(i[0] < targetx < i[1]) and (i[2] < targety < i[3]):
                LOCATION = mv.move(LOCATION, drone, ccw=45)
                sleep(0.5)
                return LOCATION, DETECTED_OBJECT
    
        if 0 < x <= 340:
            # The drone needs to angle to the left to center the target.
            new_angle = int(round(((360 - x) / 360) * 41.3))
            LOCATION = mv.move(LOCATION, drone, ccw=new_angle)        
            return LOCATION, DETECTED_OBJECT

        elif x >= 380:
            # The drone needs to angle to the right to center the target.
            new_angle = int(round(((x - 360) / 360) * 41.3))
            LOCATION = mv.move(LOCATION, drone, cw=new_angle)        
            return LOCATION, DETECTED_OBJECT
    
        if area > FB_RANGE[0] and area < FB_RANGE[1]:
            # The drone has approached the target and will scan for a QR code
            qr_detection(drone, LOCATION, turbines)

        elif area > FB_RANGE[1]:
            # The drone is too close to the target
            LOCATION = mv.move(LOCATION, drone, back=20)
            return LOCATION, DETECTED_OBJECT

        elif area < FB_RANGE[0] and area != 0:
            # The drone is too far from the target
            LOCATION = mv.move(LOCATION, drone, fwd=distance)
            qr_detection(drone, LOCATION, turbines)
            #return LOCATION, DETECTED_OBJECT

    return LOCATION, DETECTED_OBJECT

def qr_detection(drone, LOCATION, turbines):
    '''Begins searching for a QR code at the current location of the drone'''
    drone.move_down(80)
    QR = None 
    video.stop_haar()
    img_counter = 0
    while True:
        QR, img, info = droneReadQR(drone)
        img = frame.frame
        img = cv.resize(img, (W, H))

        if len(QR) > 0:
            print(">>>>>>>>>>>>>>>>QR CODE FOUND: ", QR)
            print(LOCATION[0], LOCATION[1])
            TURBINE_LOCATIONS.append([LOCATION[0] - 60, LOCATION[0] + 60, LOCATION[1] - 60, LOCATION[1] + 60, QR, LOCATION[0], LOCATION[1]])
            turbine_found = 0 # Flag to determine if the correct turbine was found
            video.stop_qr()
            sleep(1)

            for i in turbines:
                if i == QR:
                    turbine_found = 1
                    LOCATION = mv.move(LOCATION, drone, up=40)
                    mission.mission0(LOCATION, drone, turbines[i], QR)
                    LOCATION = mv.move(LOCATION, drone, up=40)
                    turbines.pop(i)

                    if len(turbines) != 0:
                        video.start_haar()
                        sleep(0.5)
                        video.start_qr()
                        sleep(0.5)
                        mv.move(LOCATION, drone, ccw=45)
                        break

                    else:
                        video.stop_qr()
                        mv.return_path(LOCATION, drone, TURBINE_LOCATIONS)
                        video.stop_image()
                        quit()

            if turbine_found == 0:
                mv.move(LOCATION, drone, up=70)
                video.start_haar()
                sleep(0.5)
                video.stop_qr()
                sleep(1)
                video.start_qr()
                sleep(0.5)
                mv.move(LOCATION, drone, ccw=45)
            break

        else:
            img_counter += 1
            if img_counter == 10:
                drone.move_left(20)

            elif img_counter == 60:
                drone.move_right(40)

            elif img_counter == 90:
                drone.move_left(20)
                img_counter = 0

def gui_interface(turbines):
    '''Function called by the GUI. Takes a mission list of selected angles and the name
    of the turbine being selected that matches the QR code format.
    Examples: [0, 0, 0, 0], WindTurbine_1.'''
    drone = Tello()
    drone.connect()
    sleep(0.5)
    print("Current battery remaining: ", drone.get_battery())
    sleep(0.3)
    drone.streamon()
    sleep(0.5)
    video = LiveFeed(drone)
    video.start()
    drone.takeoff()
    sleep(1.5)
    mv.move(LOCATION, drone, up=40)
    while True:
        frame = drone.get_frame_read()
        sleep(0.2)
        img = frame.frame
        img = cv.resize(img, (W, H))
        img, info = hc.findTurbine(img)
        LOCATION, DETECTED_OBJECT = track_object(drone, info, LOCATION, turbines, DETECTED_OBJECT)
        img = cv.resize(img, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
        

if __name__ == "__main__":
    turbines = {"WindTurbine_1": [0, 0, 0, 0]} # Target: [front, right, back, left]
    drone = Tello()
    drone.connect()
    sleep(0.5)
    print("Current battery remaining: ", drone.get_battery())
    sleep(0.3)
    drone.streamon()
    sleep(0.5)
    video = LiveFeed(drone)
    video.start()
    drone.takeoff()
    sleep(1.5)
    mv.move(LOCATION, drone, up=40)
    while True:
        frame = drone.get_frame_read()
        sleep(0.2)
        img = frame.frame
        img = cv.resize(img, (W, H))
        img, info = hc.findTurbine(img)
        LOCATION, DETECTED_OBJECT = track_object(drone, info, LOCATION, turbines, DETECTED_OBJECT)
        img = cv.resize(img, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
        