'''The main object detection and drone flight module. By Branden Pinney 2022'''
from downvision_calibration import calibrate
from time import sleep
import math
import cv2 as cv
from qr_reader import droneReadQR
from check_camera import check_camera
import haar_cascade as hc
import mission
import math
import numpy
import csv
import time
import vars
from vars import shift_cutoff

fbRange = [62000,82000] # [32000, 52000] # preset parameter for detected image boundary size
w, h = 720, 480         # display size of the screen
location = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
turbine_locations = []  # List containing the locations of found turbines
# Used to update the drone's coordinates in qr_detection
fans_x_coordinates = [360, 832, 550, 613, 161, 832, 188]
fans_y_coordinates = [48, 409, 224, 460, 227, 150, 457]
global TRACK_OBJECT_SHIFT
TRACK_OBJECT_SHIFT = 0

def trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target=None, flag_rotate=0, flag_shift=0, flag_shift_direction = "none", update_coordinates=True):
    '''Take the variable for the drone, the output array from calling findTurbine in haar_cascade.py, and the current drone x,y location and relative angle (initialized as [0, 0, 0]).
    It scans for the target object of findTurbine and approaches the target. Once it is at a pre-determined distance fbRange, it will scan and return the value of the QR code
    and return the drone to the starting point.'''
    # if the object is no longer detected, attempt to find it with 10 more frames
    # print("fileName: ", fileName)
    
    camera = drone.get_drone()
    if info[0][0] == 0:
        for i in range(200): #originally 10
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
        # Angel - it could be that the drone put a bounding box over something that was not a fan (stanchion)
        return
        # trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target, flag_rotate, update_coordinates)
    # if we did not find the center after finding it previously and our last command was a shift left. Move right the same distance.
    elif info[0][0] == 0 and flag_shift != 0 and flag_shift_direction == "left":
        drone.move(right=flag_shift)
        # Angel - it could be that the drone put a bounding box over something that was not a fan (stanchion)
        return
        # trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target,flag_shift, update_coordinates)
    # if we did not find the center after finding it previously and our last command was a shift right. Move left the same distance.
    elif info[0][0] == 0 and flag_shift != 0 and flag_shift_direction == "right":
        drone.move(left=flag_shift)
        # Angel - it could be that the drone put a bounding box over something that was not a fan (stanchion)
        return
        # trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target, flag_shift, update_coordinates)
    elif info[0][0] == 0:
        # Instead of returning False since we did not find the fan, we will only return False when the same qr code was scanned
        # return False
        return
        
    area = info[1]  # The area of the bounding box
    x, y = info[0]  # The x and y location of the center of the bounding box in the frame
    width = info[2] # The width of the bounding box
    img_pass = 0    # Flag to determine if the drone is returning from a target to skip point distance calculations

    # Angel - declare variable to be the distance we want to stop short in the x-axis
    x_distance_cutoff = 40

    # object detected
    if(x != 0):
        # (Focal length of camera lense * Real-world width of object)/Width of object in pixels - minus 40 cm otherwise we fly behind fan
        distance = int((650 * 40.64) / width) - 40  
        print(f"\n>>>>>>>>>>>>>>>> distance: {distance}\n")
        if distance < 20:
            distance = 20

        targetx = drone.get_x_location() + distance * math.cos(math.radians(drone.get_angle()))
        targety = drone.get_y_location() + distance * math.sin(math.radians(drone.get_angle()))

        turbine_locations = drone.get_turbine_locations()
        for i in turbine_locations:
            if(i[0] < targetx < i[1]) and (i[2] < targety < i[3]):
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
            # print("new_angle: " , new_angle)
            shift = numpy.abs(distance * numpy.sin(new_angle * numpy.pi/180))
            if shift > shift_cutoff:
                TRACK_OBJECT_SHIFT = shift
            print("Shift Left: " , shift)
            # If our opposite O found from distance * sin(theta) < 20 rotate the drone counter-clockwise to center on the blue circle.
            if shift < 20:
                drone.move(ccw=new_angle)  
                flag_rotate = new_angle * -1
                flag_shift = 0
            else:
            # If our opposite O found from distance * sin(theta) > 20 shift the drone left by O to center on the blue circle.
                if distance > 400: # Angel - lets just move a tad closer before centering the fan in view
                    drone.move(fwd=100)
                else:
                    drone.move(left=shift)
                    flag_shift = shift
                    flag_rotate = 0
                    flag_shift_direction = "left" 
            info = check_camera(camera)      
            target_qr_found = trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target, flag_rotate,  flag_shift, flag_shift_direction, update_coordinates)
            print("Leaving trackObject function on line 109 in image_interface")
            img_pass = 1
            # Maybe this will work, we need to continue trying to find the turbine we missed by returning False
            if target_qr_found == False:
                return False
            return True

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
            if shift > shift_cutoff:
                TRACK_OBJECT_SHIFT = shift
            print("Shift Right: " , shift)
            # If our opposite O found from distance * sin(theta) < 20 rotate the drone clockwise to center on the blue circle.
            if shift < 20:
                drone.move(cw=new_angle)
                flag_rotate = new_angle
                flag_shift = 0
            # If our opposite O found from distance * sin(theta) > 20 shift the drone right by O to center on the blue circle.
            else:
                # Angel - lets just move a tad closer before centering the fan in view
                if distance > 400:
                    drone.move(fwd=100)
                else:
                    drone.move(right=shift)
                    flag_shift = shift
                    flag_rotate = 0
                    flag_shift_direction = "right"  
            info = check_camera(camera)       
            target_qr_found = trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target, flag_rotate,  flag_shift, flag_shift_direction, update_coordinates)
            print("Leaving trackObject function on line 144 in image_interface")
            img_pass = 1
            # Maybe this will work, we need to continue trying to find the turbine we missed by returning False
            if target_qr_found == False:
                return False
            return True

        if area > fbRange[0] and area < fbRange[1] and img_pass == 0:
            # The drone has approached the target and will scan for a QR code 
            target_qr_found = qr_detection(drone, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target, update_coordinates)
            print("Leaving qr_detection function on line 148 in image_interface")
            if target_qr_found == False: # Incorrect QR code was scanned
                return False
            return True

        elif area > fbRange[1] and img_pass == 0:
            # The drone is too close to the target
            drone.move(back=20)
            info = check_camera(camera)
            trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag,target, update_coordinates)
            print("Leaving trackObject function on line 156 in image_interface")


        elif area < fbRange[0] and area != 0 and img_pass == 0:
            # The drone is too far from the target
            if distance <= 500:
                # Angel's edit of - x_distance_cutoff
                if target == "WindTurbine_1": # Drone keeps overshooting the first fan
                    drone.move(fwd=distance - (2*x_distance_cutoff) - 20)
                else:
                    drone.move(fwd=distance - x_distance_cutoff)
            else:
                while distance != 0:
                    if distance > 500:
                        drone.move(fwd=500)
                        distance -= 500
                    else:
                        drone.move(fwd=distance)
                        distance -= distance
            # qr_detection returns True if the correct QR code was scanned and returns False if incorrect QR code was scanned
            target_qr_found = qr_detection(drone, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target, update_coordinates)
            print("Leaving qr_detection function on line 185 in image_interface")
            print(f"target_qr_found: {target_qr_found}")
            
            try:
                flight_time = drone.get_flight_time()
                print(f"Flight time: {flight_time}")
            except:
                pass
            if target_qr_found == False: # Incorrect QR code was scanned
                return False
            return True
            #return location
    # else:
    #     if location[0:2] != starting_location: # If the drone has moved, return to the starting position
    #         drone.go_to(starting_location[0], starting_location[1], starting_location[2])

def qr_detection(drone, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target=None, update_coordinates=True):
    '''Begins searching for a QR code at the current location of the drone'''
        
    # with open('OutputLog.csv', 'r') as outFile:
    #             start = int(outFile.readline())
    print("flag_time:", flag_time)
    print("fileName:", fileName)
    if(flag_time == 1):
        previous_time = start
    else:
        try:
            CsvFile = open(fileName, 'r') 
            csvreader = csv.reader(CsvFile, delimiter=',')
            for row in csvreader:
                previous_time = row[3]
            previous_time = float(previous_time)
            CsvFile.close()
            print('previous_time:', previous_time)

        except:
            print("ERROR WITHIN TRY BLOCK OF QR_DETECTION FUNCTION")
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
    snake_path_length = 40
    # Distance to move foward for snake path
    forward_distance = 60

    while True:
        QR, img, info = droneReadQR(drone.get_drone())
        img = cv.resize(img, (w, h))

        if len(QR) > 0:
            print(f"\n>>>>>>>>>>>>>>>>QR CODE FOUND: {QR}\n")
            drone_var = drone.get_drone()
            current = time.time()
            try:
                with open(fileName, "a") as csvFile:
                    csvwriter = csv.writer(csvFile, lineterminator='\n')
                    turb = QR  
                    turb = turb.replace('d', 'd ')
                    turb = turb.replace('_', ' ')
                    if(flag_time == 1):
                        if((current-previous_time)%60 < 10 and (current-start)% 60 < 10):
                            csvwriter.writerow([turb, current-previous_time, str(int((current-previous_time)//60)) + '.0' + str((current-previous_time)%60), current-start, str(int((current-start)//60)) + '.0' + str((current-start)%60), str(drone_var.get_battery()) + ' %'])
                        elif((current-previous_time)%60 < 10):
                            csvwriter.writerow([turb, current-previous_time, str(int((current-previous_time)//60)) + '.0' + str((current-previous_time)%60), current-start, str(int((current-start)//60)) + '.' + str((current-start)%60), str(drone_var.get_battery()) + ' %'])
                        elif((current-start)% 60 < 10):
                            csvwriter.writerow([turb, current-previous_time, str(int((current-previous_time)//60)) + '.' + str((current-previous_time)%60), current-start, str(int((current-start)//60)) + '.0' + str((current-start)%60), str(drone_var.get_battery()) + ' %'])
                        else:
                            csvwriter.writerow([turb, current-previous_time, str(int((current-previous_time)//60)) + '.' + str((current-previous_time)%60), current-start, str(int((current-start)//60)) + '.' + str((current-start)%60), str(drone_var.get_battery()) + ' %'])
                    else:
                        if((current-previous_time-start)%60 < 10 and (current-start)%60 < 10):
                            csvwriter.writerow([turb, current-previous_time-start, str(int((current-previous_time-start)//60)) + '.0' + str((current-previous_time-start)%60), current-start, str(int((current-start)//60)) + '.0' + str((current-start)%60), str(drone_var.get_battery()) + ' %'])
                        elif((current-previous_time-start)%60 < 10):
                            csvwriter.writerow([turb, current-previous_time-start, str(int((current-previous_time-start)//60)) + '.0' + str((current-previous_time-start)%60), current-start, str(int((current-start)//60)) + '.' + str((current-start)%60), str(drone_var.get_battery()) + ' %'])
                        elif((current-start)%60 < 10):
                            csvwriter.writerow([turb, current-previous_time-start, str(int((current-previous_time-start)//60)) + '.' + str((current-previous_time-start)%60), current-start, str(int((current-start)//60)) + '.0' + str((current-start)%60), str(drone_var.get_battery()) + ' %'])
                        else:
                            csvwriter.writerow([turb, current-previous_time-start, str(int((current-previous_time-start)//60)) + '.' + str((current-previous_time-start)%60), current-start, str(int((current-start)//60)) + '.' + str((current-start)%60), str(drone_var.get_battery()) + ' %'])
            except FileNotFoundError:
                # print("File not appeneded") 
                pass
            # previous_time = current
            # with open('OutputLog.csv', 'a') as outFile:
            #     outFile.write(f"Found QR code:{QR} at {round(time()-start)}\n")
            target = QR
            if QR == target:
                drone.append_turbine_locations(QR)
                turbine_found = 0 # Flag to determine if the correct turbine was found
                video.stop_qr()
                # print("in qr_detection function on line 335")
                try: 
                    for i in turbines: 
                        if i == QR:
                            # print("in qr_detection function on line 348")
                            turbine_found = 1
                            drone.move(up=90)
                            mission.mission0(drone, turbines[i][0], QR)
                            turbines.pop(i) 
                            
                            if len(turbines) != 0:
                                # print("in qr_detection function on line 346")
                                video.start_haar()
                                sleep(0.5)
                                video.start_qr()
                                sleep(0.5)
                                #drone.go_to(starting_location[0], starting_location[1], starting_location[2])
                                break
                            else:
                                video.stop_qr()
                                print(f">>>>>>>>>>>>>>>> ALL FANS FOUND, MISSION COMPLETE, PREPARING TO LAND ON HELIPAD")
                                print(f">>>>>>>>>>>>>>>> CALIBRATE IS CALLED WITHIN THE QR_DETECTION FUNCTION")
                                calibrate(drone, fileName, start, st, fileFlag, False)

                    if turbine_found == 0:
                        print(f"\n>>>>>>>>>>>>>>>> QR CODE FOR {QR} WAS SCANNED AGAIN\n")
                        drone.move(up=110)
                        video.start_haar()
                        sleep(0.5)
                        video.stop_qr()
                        sleep(1)
                        video.start_qr()
                        sleep(0.5)
                        return False # This is supposed to mean that the same qr code was scanned
                        #drone.go_to(starting_location[0], starting_location[1], starting_location[2])
                    break

                except:
                    break
            
            else: # This section controls what the drone does if the QR code doesn't match the target
                print(f">>>>>>>>>>>>>>>>QR CODE NOT MATCHING: QR {QR} != TARGET {target}")
                print(f">>>>>>>>>>>>>>>>QR = {QR}")
                print(f">>>>>>>>>>>>>>>>Target = {target}")
                video.stop_qr()
                drone.move(up=110)
                video.start_haar()
                sleep(0.5)
                video.stop_qr()
                sleep(1)
                video.start_qr()
                sleep(0.5)
                return False # This will let us know that we found the incorrect QR code
        
        else: # Do snake path algorithm to find qr code to scan
            img_counter += 1
            # This if statement is checking if the drone has made a full loop searching
            if img_counter == 120:
                # Check which search pattern the drone has already attempted.
                if search_loop_counter < 3:
                    print("Drone completed snake search loop, repeating again")
                    # This will start the search again
                    img_counter = 0
                    # Keep track of which search increment we are in
                    search_loop_counter += 1
                elif search_loop_counter == 3:
                    # Tell drone to go home and land
                    print("QR code not found. Telling drone to go back to helipad and land")
                    drone.move(up=110)
                    # Tell drone to land on helipadd
                    calibrate(drone, fileName, start, st, fileFlag, False)
                    return
                else:
                    # search_loop_counter should not ever be this value
                    print("ERROR, search_loop_counter is an unexpected value of : " + str(search_loop_counter))
            # Snake Path search for QR code
            elif (img_counter%105) == 0:
                drone.move(left=(snake_path_length))
            elif (img_counter%90) == 0:
                drone.move(fwd=(forward_distance))
            elif (img_counter%75) == 0:
                drone.move(right=(snake_path_length))
            elif (img_counter%60) == 0:
                drone.move(right=(snake_path_length))
            elif (img_counter%45) == 0:
                drone.move(fwd=(forward_distance))
            elif (img_counter%30) == 0:
                drone.move(left=(2*snake_path_length))
            elif (img_counter%15) == 0:
                drone.move(right=(snake_path_length))

    return True # We should be here if we found the correct qr code

if __name__ == "__main__":
    print("ANGEL, WRONG FILE, RUN THE OTHER ONE.")
    pass

   
        