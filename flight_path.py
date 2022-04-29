from djitellopy import Tello
import cv2 as cv
from time import sleep
from qr_reader import droneReadQR
import movement_test as mv
from output_video import LiveFeed
import haar_cascade as hc
import mission
import math

fbRange = [62000,82000]#[32000, 52000] # preset parameter for detected image boundary size
w, h = 720, 480 # display size of the screen
location = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
turbine_locations = [] # List containing the locations of found turbines
detected_object = 0 # A flag to determine if the camera detected an object in the previous 5 frames

def trackObject(drone, info, location, turbines, detected_object):
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
            location = mv.move(location, drone, ccw=25)
            sleep(1)
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
        distance = int((650 * 40.64) / width) - 40
        if distance < 20:
            distance = 20
        targetx = location[0] + distance * math.cos(math.radians(location[2]))
        targety = location[1] + distance * math.sin(math.radians(location[2]))
        for i in turbine_locations:
            if(i[0] < targetx < i[1]) and (i[2] < targety < i[3]):
                location = mv.move(location, drone, ccw=45)
                sleep(0.5)
                return location, detected_object
        if(0 < x <= 340):
            # The drone needs to angle to the left to center the target.
            new_angle = int(round(((360 - x) / 360) * 41.3))
            location = mv.move(location, drone, ccw=new_angle)        
            return location, detected_object
        elif(x >= 380):
            # The drone needs to angle to the right to center the target.
            new_angle = int(round(((x - 360) / 360) * 41.3))
            location = mv.move(location, drone, cw=new_angle)        
            return location, detected_object
        if area > fbRange[0] and area < fbRange[1]:
            # The drone has approached the target and will scan for a QR code
            qrDetection(drone, location, turbines)
        elif area > fbRange[1]:
            # The drone is too close to the target
            location = mv.move(location, drone, back=20)
            return location, detected_object
        elif area < fbRange[0] and area != 0:
            # The drone is too far from the target
            location = mv.move(location, drone, fwd=distance)
            qrDetection(drone, location, turbines)
            #return location, detected_object
    return location, detected_object

def qrDetection(drone, location, turbines):
    drone.move_down(80)
    QR = None 
    video.stop_haar()
    img_counter = 0
    while True:
        QR, img, info = droneReadQR(drone)
        img = frame.frame
        img = cv.resize(img, (w, h))
        if len(QR) > 0:
            print(">>>>>>>>>>>>>>>>QR CODE FOUND: ", QR)
            print(location[0], location[1])
            turbine_locations.append([location[0] - 60, location[0] + 60, location[1] - 60, location[1] + 60, QR, location[0], location[1]])
            turbine_found = 0 # Flag to determine if the correct turbine was found
            video.stop_qr()
            sleep(1)
            for i in turbines:
                if i == QR:
                    turbine_found = 1
                    location = mv.move(location, drone, up=40)
                    mission.mission0(location, drone, turbines[i], QR)
                    location = mv.move(location, drone, up=40)
                    turbines.pop(i) 
                    if len(turbines) != 0:
                        video.start_haar()
                        sleep(0.5)
                        video.start_qr()
                        sleep(0.5)
                        mv.move(location, drone, ccw=45)
                        break
                    else:
                        video.stop_qr()
                        mv.return_path(location, drone, turbine_locations)
                        video.stop_image()
                        quit()
            if turbine_found == 0:
                mv.move(location, drone, up=70)
                video.start_haar()
                sleep(0.5)
                video.stop_qr()
                sleep(1)
                video.start_qr()
                sleep(0.5)
                mv.move(location, drone, ccw=45)
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

def test(mission_list, turbine_list):
    '''Function called by the GUI. Takes a mission list of selected angles and the name
    of the turbine being selected that matches the QR code format. Examples: [0, 0, 0, 0], WindTurbine_1.'''
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
        img = frame.frame
        img = cv.resize(img, (w, h))
        img, info = hc.findTurbine(img)
        QR, img, info = droneReadQR(drone)
        location, detected_object = trackObject(drone, info, location, mission_list, turbine_list, detected_object)
        img = cv.resize(img, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
        cv.imshow("Output", img)
        cv.waitKey(1)

if __name__ == "__main__":
    turbines = {"WindTurbine_1": [0,0,0,0]}
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
    mv.move(location, drone, up=40)
    while True:
        frame = drone.get_frame_read()
        sleep(0.2)
        img = frame.frame
        img = cv.resize(img, (w, h))
        img, info = hc.findTurbine(img)
        #QR, img, info = droneReadQR(drone)
        location, detected_object = trackObject(drone, info, location, turbines, detected_object)
        img = cv.resize(img, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
        