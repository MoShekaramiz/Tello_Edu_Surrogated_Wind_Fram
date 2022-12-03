'''The main object detection and drone flight module. By Branden Pinney 2022'''
# Angel's comment
from downvision_calibration import calibrate
from time import sleep
import math
import cv2 as cv
from qr_reader import droneReadQR
from check_camera import check_camera
import haar_cascade as hc
import mission
import math
import movement as mov
import numpy
import csv
import time

fbRange = [62000,82000] # [32000, 52000] # preset parameter for detected image boundary size
w, h = 720, 480         # display size of the screen
location = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
turbine_locations = []  # List containing the locations of found turbines


def trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target=None, flag_rotate=0, flag_shift=0, flag_shift_direction = "none"):
    '''Take the variable for the drone, the output array from calling findTurbine in haar_cascade.py, and the current drone x,y location and relative angle (initialized as [0, 0, 0]).
    It scans for the target object of findTurbine and approaches the target. Once it is at a pre-determined distance fbRange, it will scan and return the value of the QR code
    and return the drone to the starting point.'''
    # if the object is no longer detected, attempt to find it with 10 more frames
    print("fileName: ", fileName)
    camera = drone.get_drone()
    if info[0][0] == 0:
        for i in range(100): #originally 10
            if info[0][0] == 0:
                frame = camera.get_frame_read()
                img = frame.frame
                img = cv.resize(img, (w, h))
                img, info = hc.findTurbine(img)
            else:
                break

    # if we did not find the center after finding it previously and our last command was a rotate. Rotate the same degree in the opposite direction
    if info[0][0] == 0 and flag_rotate != 0:
        drone.move(ccw=flag_rotate)
        trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target, flag_rotate)
    # if we did not find the center after finding it previously and our last command was a shift left. Move right the same distance.
    elif info[0][0] == 0 and flag_shift != 0 and flag_shift_direction == "left":
        drone.move(right=flag_shift)
        trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target,flag_shift)
    # if we did not find the center after finding it previously and our last command was a shift right. Move left the same distance.
    elif info[0][0] == 0 and flag_shift != 0 and flag_shift_direction == "right":
        drone.move(left=flag_shift)
        trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target, flag_shift)
    elif info[0][0] == 0:
        return False

    area = info[1]  # The area of the bounding box
    x, y = info[0]  # The x and y location of the center of the bounding box in the frame
    width = info[2] # The width of the bounding box
    img_pass = 0    # Flag to determine if the drone is returning from a target to skip point distance calculations

    # Angel - declare variable to be the distance we want to stop short in the x-axis
    x_distance_cutoff = 30

    # object detected
    if(x != 0):
        distance = int((650 * 40.64) / width) - 40 # (Focal length of camera lense * Real-world width of object)/Width of object in pixels  -  40 centimeters to stop short
        if distance < 20:
            distance = 20

        targetx = drone.get_x_location() + distance * math.cos(math.radians(drone.get_angle()))
        targety = drone.get_y_location() + distance * math.sin(math.radians(drone.get_angle()))

        turbine_locations = drone.get_turbine_locations()
        for i in turbine_locations:
            if(i[0] < targetx < i[1]) and (i[2] < targety < i[3]):
                # Angel - Make the drone stop short in the x direction and face the fan
                # Third parameter should be 0 for the angle to be facing the turbines
                drone.go_to(starting_location[0], starting_location[1], 0)
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
            print("new_angle: " , new_angle)
            shift = numpy.abs(distance * numpy.sin(new_angle * numpy.pi/180))
            print("Shift Left: " , shift)
            # If our opposite O found from distance * sin(theta) < 20 rotate the drone counter-clockwise to center on the blue circle.
            if shift < 20:
                drone.move(ccw=new_angle)  
                flag_rotate = new_angle * -1
                flag_shift = 0
            else:
            # If our opposite O found from distance * sin(theta) > 20 shift the drone left by O to center on the blue circle.
                drone.move(left=shift)
                flag_shift = shift
                flag_rotate = 0
                flag_shift_direction = "left" 
            info = check_camera(camera)      
            trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target, flag_rotate,  flag_shift, flag_shift_direction)
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
            print("new_angle: " , new_angle)
            shift = numpy.abs(distance * numpy.sin(new_angle * numpy.pi/180))
            print("Shift Right: " , shift)
            # If our opposite O found from distance * sin(theta) < 20 rotate the drone clockwise to center on the blue circle.
            if shift < 20:
                drone.move(cw=new_angle)
                flag_rotate = new_angle
                flag_shift = 0
            # If our opposite O found from distance * sin(theta) > 20 shift the drone right by O to center on the blue circle.
            else:
                drone.move(right=shift)
                flag_shift = shift
                flag_rotate = 0
                flag_shift_direction = "right"  
            info = check_camera(camera)       
            trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target, flag_rotate,  flag_shift, flag_shift_direction)
            img_pass = 1

        if area > fbRange[0] and area < fbRange[1] and img_pass == 0:
            # The drone has approached the target and will scan for a QR code 
            qr_detection(drone, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target)
            return True

        elif area > fbRange[1] and img_pass == 0:
            # The drone is too close to the target
            drone.move(back=20)
            info = check_camera(camera)
            trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag,target)

        elif area < fbRange[0] and area != 0 and img_pass == 0:
            # The drone is too far from the target
            if distance <= 500:
                # Angel's edit of - x_distance_cutoff
                drone.move(fwd=distance - x_distance_cutoff)
            else:
                while distance != 0:
                    if distance > 500:
                        drone.move(fwd=500)
                        distance -= 500
                    else:
                        drone.move(fwd=distance)
                        distance -= distance
            qr_detection(drone, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target)
            return True
            #return location
    # else:
    #     if location[0:2] != starting_location: # If the drone has moved, return to the starting position
    #         drone.go_to(starting_location[0], starting_location[1], starting_location[2])

def qr_detection(drone, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target=None):
    '''Begins searching for a QR code at the current location of the drone'''
    # with open('OutputLog.csv', 'r') as outFile:
    #             start = int(outFile.readline())
    print("flag_time:", flag_time)
    print("fileName:", fileName)
    if(flag_time == 1):
        previous_time = start
    drone_lower = drone.get_drone()
    drone.move(down=110)
    if drone_lower.get_height() > 50:
        drone.move(down=110)
    QR = None 
    video = drone.get_video()
    video.stop_haar()
    img_counter = 0

    # Have the drone face angle 0 degrees since that would make the drone face the front of the fans and qr codes
    drone.go_to(drone.get_x_location(), drone.get_y_location(), 0)
    # Counter to determine which search loop the drone is in
    # It is no good to have drone continually search until the battery dies
    search_loop_counter = 1
    snake_path_length = 30
    # Distance to move foward for snake path
    forward_distance = 30

    while True:
        QR, img, info = droneReadQR(drone.get_drone())
        img = cv.resize(img, (w, h))

        if len(QR) > 0:
            drone_var = drone.get_drone()
            # print(">>>>>>>>>>>>>>>>CURRENT FLIGHT TIME: ", drone_var.get_flight_time())
            print(">>>>>>>>>>>>>>>>QR CODE FOUND: ", QR)
            current = time.time()
            csvFile = open(fileName, "a")
            csvwriter = csv.writer(csvFile, lineterminator='\n')
            turb = QR
            turb = turb.replace('d', 'd ')
            turb = turb.replace('_', ' ')
            if((current-previous_time)%60 < 10 and (current-start)% 60 < 10):
                csvwriter.writerow([turb, current-previous_time, str(int((current-previous_time)//60)) + '.0' + str((current-previous_time)%60), current-start, str(int((current-start)//60)) + '.0' + str((current-start)%60), str(drone_var.get_battery()) + ' %'])
            elif((current-previous_time)%60 < 10):
                csvwriter.writerow([turb, current-previous_time, str(int((current-previous_time)//60)) + '.0' + str((current-previous_time)%60), current-start, str(int((current-start)//60)) + '.' + str((current-start)%60), str(drone_var.get_battery()) + ' %'])
            elif((current-start)% 60 < 10):
                csvwriter.writerow([turb, current-previous_time, str(int((current-previous_time)//60)) + '.' + str((current-previous_time)%60), current-start, str(int((current-start)//60)) + '.0' + str((current-start)%60), str(drone_var.get_battery()) + ' %'])
            else:
                csvwriter.writerow([turb, current-previous_time, str(int((current-previous_time)//60)) + '.' + str((current-previous_time)%60), current-start, str(int((current-start)//60)) + '.' + str((current-start)%60), str(drone_var.get_battery()) + ' %'])
            csvFile.close()
            previous_time = current
            
            # Angel - Comment this line below when not testing individual fans
            # drone.land()
            # with open('OutputLog.csv', 'a') as outFile:
            #     outFile.write(f"Found QR code:{QR} at {round(time()-start)}\n")
            # 
            if QR == 'FanTurbine_3': #target
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
            
            else: # This section controls what the drone does if the QR code doesn't match the target
                print(f">>>>>>>>>>>>>>>>QR CODE NOT MATCHING: {QR} != {target}")
                video.stop_qr()
                drone.move(up=70)
                drone.go_to(0, 0, 0)
                calibrate(drone, fileName, start, st, fileFlag, False)
                drone.land()
        # Snake path algorithm to find qr code to scan
        # By default, the forward search loop comes first
        else:
            img_counter += 1
            # This if statement is checking if the drone has made a full loop searching
            if img_counter == 120:
                # Check which search pattern the drone has already attempted.
                if search_loop_counter == 1:
                    # Snake path search
                    print("Drone completed snake search loop, repeating forward")
                    # This will start the search again
                    img_counter = 0
                    # Keep track of which search increment we are in
                    search_loop_counter += 1
                elif search_loop_counter == 2:
                    # Snake path search
                    print("Drone completed snake search loop, repeating forward")
                    # This will start the search again
                    img_counter = 0
                    # Keep track of which search increment we are in
                    search_loop_counter += 1
                elif search_loop_counter == 3:
                    # Snake path search
                    print("Drone completed snake search loop, repeating forward")
                    # This will start the search again
                    img_counter = 0
                    # Keep track of which search increment we are in
                    search_loop_counter += 1
                elif search_loop_counter == 4:
                    # Tell drone to go home and land
                    print("Qr code not found. Telling drone to go back to helipad and land")
                    drone.move(up=70)
                    drone.go_to(0, 0, 0)
                    # calibrate(drone_class, land=False, x_coordinate=0, y_coordinate=0):
                    drone.land()
                else:
                    # search_loop_counter should not ever be this value
                    print("ERROR, search_loop_counter is an unexpected value of : " + str(search_loop_counter))
            elif (img_counter%105) == 0:
                drone.move(left=(snake_path_length))
            elif (img_counter%90) == 0:
                drone.move(fwd=(2 * forward_distance))
            elif (img_counter%75) == 0:
                drone.move(right=(snake_path_length))
            elif (img_counter%60) == 0:
                drone.move(right=(snake_path_length))
            elif (img_counter%45) == 0:
                drone.move(fwd=(2 * forward_distance))
            elif (img_counter%30) == 0:
                drone.move(left=(2*snake_path_length))
            elif (img_counter%15) == 0:
                drone.move(right=(snake_path_length))

# if __name__ == "__main__":
#     turbines = {"WindTurbine_1": [0, 0, 0, 0]}
#     drone = mov.movement()
    
#     while True:
#         frame = drone.get_frame_read()
#         sleep(0.2)
#         img = frame.frame
#         img = cv.resize(img, (w, h))
#         img, info = hc.findTurbine(img)
#         location = trackObject(drone, info, location, turbines, , fileName, start, flag_time)
#         img = cv.resize(img, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
        