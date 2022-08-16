'''The main object detection and drone flight module. By Branden Pinney and Shayne Duncan 2022'''

from time import sleep
import math
import cv2 as cv
from djitellopy import Tello
from qr_reader import droneReadQR
from check_camera import check_camera
import haar_cascade as hc
import mission
import movement as mov

fbRange = [62000,82000] # [32000, 52000] # preset parameter for detected image boundary size
w, h = 720, 480         # display size of the screen
location = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
turbine_locations = []  # List containing the locations of found turbines
DETECTED_OBJECT = 0 # A flag to determine if the camera detected an object in the previous 5 frames

def track_object(drone, info, turbines, DETECTED_OBJECT):
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
            drone.move(ccw=25)
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

        targetx = drone.get_x_location() + distance * math.cos(math.radians(drone.get_angle()))
        targety = drone.get_y_location() + distance * math.sin(math.radians(drone.get_angle()))

        turbine_locations = drone.get_turbine_locations()
        for i in turbine_locations:
            if(i[0] < targetx < i[1]) and (i[2] < targety < i[3]):
                drone.move(ccw=45)
                return DETECTED_OBJECT
    
        if 0 < x <= 340:
            # The drone needs to angle to the left to center the target.
            new_angle = int(round(((360 - x) / 360) * 41.3))
            drone.move(ccw=new_angle)         
            return DETECTED_OBJECT

        elif x >= 380:
            # The drone needs to angle to the right to center the target.
            new_angle = int(round(((x - 360) / 360) * 41.3))
            drone.move(cw=new_angle)         
            return DETECTED_OBJECT
    
        if area > fbRange[0] and area < fbRange[1]:
            # The drone has approached the target and will scan for a QR code
            qr_detection(drone, turbines)

        elif area > fbRange[1]:
            # The drone is too close to the target
            drone.move(back=20)
            return DETECTED_OBJECT

        elif area < fbRange[0] and area != 0:
            # The drone is too far from the target
            drone.move(fwd=distance)
            qr_detection(drone, turbines)
            #return LOCATION, DETECTED_OBJECT

    return DETECTED_OBJECT

def qr_detection(drone, turbines):
    '''Begins searching for a QR code at the current location of the drone'''
    sleep(0.1)
    drone.move(down=80)
    QR = None 
    video = drone.get_video()
    img_counter = 0
    while True:
        QR, img, info = droneReadQR(drone.get_drone())
        img = cv.resize(img, (w, h))

        if len(QR) > 0:
            drone_var = drone.get_drone()
            print(">>>>>>>>>>>>>>>>CURRENT FLIGHT TIME: ", drone_var.get_flight_time())
            print(">>>>>>>>>>>>>>>>QR CODE FOUND: ", QR)
            drone.append_turbine_locations(QR)
            turbine_found = 0 # Flag to determine if the correct turbine was found
            video.stop_qr()

            for i in turbines:
                if i == QR:
                    turbine_found = 1
                    drone.move(up=60)
                    mission.mission0(drone, turbines[i], QR)
                    turbines.pop(i)

                    if len(turbines) != 0:
                        video.start_haar()
                        sleep(0.5)
                        video.start_qr()
                        sleep(0.5)
                        drone.move(ccw=45)
                        break

                    else:
                        video.stop_qr()
                        drone.go_to(0, 0, 0)
                        drone.land()
                        video.stop_image()
                        quit()

            if turbine_found == 0:
                drone.move(up=70)
                video.start_haar()
                sleep(0.5)
                video.stop_qr()
                sleep(1)
                video.start_qr()
                sleep(0.5)
                drone.move(ccw=45)
            break

        else:
            img_counter += 1
            if img_counter == 10:
                drone.move(left=20)

            elif img_counter == 60:
                drone.move(right=40)

            elif img_counter == 90:
                drone.move(left=20)
                img_counter = 0

def gui_interface(turbines):
    '''Function called by the GUI. Takes a mission list of selected angles and the name
    of the turbine being selected that matches the QR code format.
    Examples: [0, 0, 0, 0], WindTurbine_1.'''
    drone = mov.movement()
    while True:
        frame = drone.get_frame_read()
        sleep(0.2)
        img = frame.frame
        img = cv.resize(img, (w, h))
        img, info = hc.findTurbine(img)
        DETECTED_OBJECT = track_object(drone, info, turbines, DETECTED_OBJECT)
        img = cv.resize(img, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
        

if __name__ == "__main__":
    turbines = {"WindTurbine_3": [1, 0, 0, 0]} # Target: [front, right, back, left]
    drone = mov.movement()
    frm = drone.get_drone()
    while True:
        frame = frm.get_frame_read()
        sleep(0.2)
        img = frame.frame
        img = cv.resize(img, (w, h))
        img, info = hc.findTurbine(img)
        DETECTED_OBJECT = track_object(drone, info, turbines, DETECTED_OBJECT)
        img = cv.resize(img, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
        