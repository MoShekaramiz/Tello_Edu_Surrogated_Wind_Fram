'''The main object detection and drone flight module. By Branden Pinney 2022'''

from re import search
from time import sleep, time
import math
from turtle import left, right
import cv2 as cv
from djitellopy import Tello
from qr_reader import droneReadQR
from check_camera import check_camera
import haar_cascade as hc
import mission
import math
import movement as mov

fbRange = [62000,82000] # [32000, 52000] # preset parameter for detected image boundary size
w, h = 720, 480         # display size of the screen
location = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
turbine_locations = []  # List containing the locations of found turbines


def trackObject(drone, info, turbines, starting_location):
    '''Take the variable for the drone, the output array from calling findTurbine in haar_cascade.py, and the current drone x,y location and relative angle (initialized as [0, 0, 0]).
    It scans for the target object of findTurbine and approaches the target. Once it is at a pre-determined distance fbRange, it will scan and return the value of the QR code
    and return the drone to the starting point.'''
    # if the object is no longer detected, attempt to find it with 10 more frames
    camera = drone.get_drone()
    if info[0][0] == 0:
        for i in range(10):
            if info[0][0] == 0:
                frame = camera.get_frame_read()
                img = frame.frame
                img = cv.resize(img, (w, h))
                img, info = hc.findTurbine(img)
            else:
                break
        if info[0][0] == 0:
            return False
    
    area = info[1]  # The area of the bounding box
    x, y = info[0]  # The x and y location of the center of the bounding box in the frame
    width = info[2] # The width of the bounding box
    img_pass = 0    # Flag to determine if the drone is returning from a target to skip point distance calculations

    # object detected
    if(x != 0):
        distance = int((650 * 40.64) / width) - 30 # (Focal length of camera lense * Real-world width of object)/Width of object in pixels  -  40 centimeters to stop short
        if distance < 20:
            distance = 20

        targetx = drone.get_x_location() + distance * math.cos(math.radians(drone.get_angle()))
        targety = drone.get_y_location() + distance * math.sin(math.radians(drone.get_angle()))

        turbine_locations = drone.get_turbine_locations()
        for i in turbine_locations:
            if(i[0] < targetx < i[1]) and (i[2] < targety < i[3]):
                drone.go_to(starting_location[0], starting_location[1], starting_location[2])
                return

        if(0 < x <= 340):
            # The drone needs to angle to the left to center the target.
            new_angle = int(round(((360 - x) / 360) * 41.3))
            targetx = drone.get_x_location() + distance * math.cos(math.radians((drone.get_angle()+new_angle)%360))
            targety = drone.get_y_location() + distance * math.sin(math.radians((drone.get_angle()+new_angle)%360))

            turbine_locations = drone.get_turbine_locations()
            for i in turbine_locations:
                if(i[0] < targetx < i[1]) and (i[2] < targety < i[3]):
                    drone.go_to(starting_location[0], starting_location[1], starting_location[2])
                    return False
            drone.move(ccw=new_angle)  
            info = check_camera(camera)      
            trackObject(drone, info, turbines, starting_location)
            img_pass = 1

        elif(x >= 380):
            # The drone needs to angle to the right to center the target.
            new_angle = int(round(((x - 360) / 360) * 41.3))
            target_angle = drone.get_angle()-new_angle
            if target_angle < 0: target_angle += 360

            targetx = drone.get_x_location() + distance * math.cos(math.radians(target_angle))
            targety = drone.get_y_location() + distance * math.sin(math.radians(target_angle))

            turbine_locations = drone.get_turbine_locations()
            for i in turbine_locations:
                if(i[0] < targetx < i[1]) and (i[2] < targety < i[3]):
                    drone.go_to(starting_location[0], starting_location[1], starting_location[2])
                    return False
            drone.move(cw=new_angle) 
            info = check_camera(camera)       
            trackObject(drone, info, turbines, starting_location)
            img_pass = 1

        if area > fbRange[0] and area < fbRange[1] and img_pass == 0:
            # The drone has approached the target and will scan for a QR code 
            qr_detection(drone, turbines, starting_location)
            return True

        elif area > fbRange[1] and img_pass == 0:
            # The drone is too close to the target
            drone.move(back=20)
            info = check_camera(camera)
            trackObject(drone, info, turbines, starting_location)

        elif area < fbRange[0] and area != 0 and img_pass == 0:
            # The drone is too far from the target
            if distance <= 500:
                drone.move(fwd=distance)
            else:
                while distance != 0:
                    if distance > 500:
                        drone.move(fwd=500)
                        distance -= 500
                    else:
                        drone.move(fwd=distance)
                        distance -= distance
            qr_detection(drone, turbines, starting_location)
            return True
            #return location
    # else:
    #     if location[0:2] != starting_location: # If the drone has moved, return to the starting position
    #         drone.go_to(starting_location[0], starting_location[1], starting_location[2])

def qr_detection(drone, turbines, starting_location):
    '''Begins searching for a QR code at the current location of the drone'''
    # with open('OutputLog.csv', 'r') as outFile:
    #             start = int(outFile.readline())
    drone_lower = drone.get_drone()
    drone.move(down=110)
    if drone_lower.get_height() > 50:
        drone.move(down=110)
    QR = None 
    video = drone.get_video()
    video.stop_haar()
    img_counter = 0

    # Counter to determine which search loop the drone is in
    # It is no good to have drone continually search until the battery dies
    search_loop_counter = 1
    # Remember this spot to fall back on for the different search loops below
    search_orign_location = [drone.get_x_location(), drone.get_y_location()]
    # Search algorithm uses an octagon with each turn being 45 degrees
    # The length of the octagon sides can be edited below, use smaller length for small rooms
    octagon_side_length = 60
    # Calculate the diameter of the octagon
    octagon_diameter = 2.41*octagon_side_length
    octagon_radius = octagon_diameter / 2 

    while True:
        QR, img, info = droneReadQR(drone.get_drone())
        img = cv.resize(img, (w, h))

        if len(QR) > 0:
            drone_var = drone.get_drone()
            # print(">>>>>>>>>>>>>>>>CURRENT FLIGHT TIME: ", drone_var.get_flight_time())
            print(">>>>>>>>>>>>>>>>QR CODE FOUND: ", QR)
            # with open('OutputLog.csv', 'a') as outFile:
            #     outFile.write(f"Found QR code:{QR} at {round(time()-start)}\n")
            drone.append_turbine_locations(QR)
            turbine_found = 0 # Flag to determine if the correct turbine was found
            video.stop_qr()

            try: 
                for i in turbines:
                    if i == QR:
                        turbine_found = 1
                        drone.move(up=90)
                        mission.mission0(drone, turbines[i][0], QR)
                        turbines.pop(i) 

                        if len(turbines) != 0:
                            video.start_haar()
                            sleep(0.5)
                            video.start_qr()
                            sleep(0.5)
                            #drone.go_to(starting_location[0], starting_location[1], starting_location[2])
                            break

                        else:
                            video.stop_qr()
                            drone.go_to(0, 0, 0)
                            drone.land()

                if turbine_found == 0:
                    drone.move(up=110)
                    video.start_haar()
                    sleep(0.5)
                    video.stop_qr()
                    sleep(1)
                    video.start_qr()
                    sleep(0.5)
                    #drone.go_to(starting_location[0], starting_location[1], starting_location[2])
                break

            except:
                break

        # Orbiting algorithm to find qr code to scan
        # By default, the forward search loop comes first
        else:
            img_counter += 1
            # This if statement is checking if the drone has made a full loop searching
            # Changing from 195 to 255 to make the drone go in a full loop before changing search
            if img_counter == 270:
                # Check which search pattern the drone has already attempted.
                if search_loop_counter == 1:
                    # Back side search loop
                    print("The drone has made a full forward loop, time to try backwards search loop")
                    drone.move(ccw=180)
                    # This will start the seach again
                    img_counter = 0
                    # Keep track of which search loop we are in
                    search_loop_counter += 1
                elif search_loop_counter == 2:
                    # Left side search loop
                    print("The drone has made a full backwards loop, time to try left side search loop")
                    drone.move(ccw=270)
                    # This will start the seach again
                    img_counter = 0
                    # Keep track of which search loop we are in
                    search_loop_counter += 1
                elif search_loop_counter == 3:
                    # right side search loop
                    print("The drone has made a full left side loop, time to try right side search loop")
                    drone.move(ccw=180)
                    # This will start the seach again
                    img_counter = 0
                    # Keep track of which search loop we are in
                    search_loop_counter += 1
                elif search_loop_counter == 4:
                    # Tell drone to go home and land
                    print("Telling drone to go back to helipad and land")
                    drone.go_to(0, 0, 0)
                    # calibrate(drone_class, land=False, x_coordinate=0, y_coordinate=0):
                    drone.land()
                else:
                    # search_loop_counter should not ever be this value
                    print("ERROR, search_loop_counter is an unexpected value of : " + str(search_loop_counter))
            elif (img_counter%30) == 0:
                # Rotating counter clockwise 45 degrees
                drone.move(ccw=45)
                drone.move(right=(octagon_side_length))
                print("Drone moving: " + str(octagon_side_length) + " cm to the right\n")
            elif (img_counter%15) == 0:
                drone.move(right=(octagon_side_length))
                print("Drone moving: " + str(octagon_side_length) + " cm to the right\n")

# def test(mission_list, turbine_list):
#     '''Function called by the GUI. Takes a mission list of selected angles and the name
#     of the turbine being selected that matches the QR code format. Examples: [0, 0, 0, 0], WindTurbine_1.'''
#     drone = Tello()
#     drone.connect()
#     sleep(0.5)
#     print("Current battery remaining: ", drone.get_battery())
#     sleep(0.3)
#     drone.streamon()
#     sleep(0.5)
#     drone.takeoff()
#     sleep(0.5)
#     while True:
#         frame = drone.get_frame_read()
#         img = frame.frame
#         img = cv.resize(img, (w, h))
#         img, info = hc.findTurbine(img)
#         QR, img, info = droneReadQR(drone)
#         location = trackObject(drone, info, location, mission_list, turbine_list)
#         img = cv.resize(img, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
#         cv.imshow("Output", img)
#         cv.waitKey(1)

# if __name__ == "__main__":
#     turbines = {"WindTurbine_1": [0, 0, 0, 0]}
#     drone = mov.movement()
    
#     while True:
#         frame = drone.get_frame_read()
#         sleep(0.2)
#         img = frame.frame
#         img = cv.resize(img, (w, h))
#         img, info = hc.findTurbine(img)
#         location = trackObject(drone, info, location, turbines)
#         img = cv.resize(img, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
        