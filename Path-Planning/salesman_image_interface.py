'''The main object detection and drone flight module. 
By Branden Pinney 2022 and Angel Rodriguez 2022'''
from downvision_calibration import calibrate
from time import sleep
import math
import cv2 as cv
from qr_reader import droneReadQR
from check_camera import check_camera
import haar_cascade as hc
import mission
import numpy
import csv
import time
from vars import *
import movement as mov

fbRange = [62000,82000] # [32000, 52000] # preset parameter for detected image boundary size
w, h = 720, 480         # display size of the screen
location = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
turbine_locations = []  # List containing the locations of found turbines

# Lower this value if the drone is over shifting when centering the fan in place
# Supposed to be width of real world object, assuming the bounding box width is exactly the width of the object in frame
# real_width = 41 # fans without the protective cages
real_width = 53 # fans with the protective cages


def trackObject(drone, info, turbines, starting_location, fileName=None, start=None, flag_time=None, st=None, fileFlag=None, target=None, flag_rotate=0, flag_shift=0, flag_shift_direction = "none"):
    '''Take the variable for the drone, the output array from calling findTurbine in haar_cascade.py, and the current drone x,y location and relative angle (initialized as [0, 0, 0]).
    It scans for the target object of findTurbine and approaches the target. Once it is at a pre-determined distance fbRange, it will scan and return the value of the QR code
    and return the drone to the starting point.'''
    # if the object is no longer detected, attempt to find it with 10 more frames
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
        info = check_camera(camera)
        trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target, flag_rotate)
    # if we did not find the center after finding it previously and our last command was a shift left. Move right the same distance.
    elif info[0][0] == 0 and flag_shift != 0 and flag_shift_direction == "left":
        drone.move(right=flag_shift)
        info = check_camera(camera)
        trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target,flag_shift)
    # if we did not find the center after finding it previously and our last command was a shift right. Move left the same distance.
    elif info[0][0] == 0 and flag_shift != 0 and flag_shift_direction == "right":
        drone.move(left=flag_shift)
        info = check_camera(camera)
        trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target, flag_shift)
    elif info[0][0] == 0:
        return False

    area = info[1]  # The area of the bounding box
    x, y = info[0]  # The x and y location of the center of the bounding box in the frame
    width = info[2] # The width of the bounding box
    img_pass = 0    # Flag to determine if the drone is returning from a target to skip point distance calculations
    
    # object detected
    if(x != 0):
        distance = int(650 * real_width / width) # (Focal length of camera lense * Real-world width of object)/Width of object in pixels
        print(f'>>>>>>>>>>>>>>>> CALCULATED DISTANCE: {distance}')
        if distance > 400: # Sometimes the width of the bounding box is too small so we overcalculate the distance by alot
            print(f'>>>>>>>>>>>>>>>> CALCULATED DISTANCE TOO GREAT, DECREASING CALCULATED DISTANCE VALUE')
            distance *= 0.5
            print(f'>>>>>>>>>>>>>>>> NEW DISTANCE VALUE CALCULATED: {distance}')

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
            # FOV (field of view) for the drone is 82.6 degrees
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
            target_qr_found = trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target, flag_rotate,  flag_shift, flag_shift_direction)
            print("Leaving trackObject function on line 109 in image_interface")
            img_pass = 1
            # Maybe this will work, we need to continue trying to find the turbine we missed by returning False
            print(f'target_qr_found: {target_qr_found}')
            return target_qr_found

        elif(x >= 380):
            # The drone needs to angle to the right to center the target.
            # FOV (field of view) for the drone is 82.6 degrees
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
            target_qr_found = trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target, flag_rotate,  flag_shift, flag_shift_direction)
            print("Leaving trackObject function on line 144 in image_interface")
            img_pass = 1
            # Maybe this will work, we need to continue trying to find the turbine we missed by returning False
            print(f'target_qr_found: {target_qr_found}')
            return target_qr_found
            
        if area > fbRange[0] and area < fbRange[1] and img_pass == 0:
            # The drone has approached the target and will scan for a QR code 
            target_qr_found = qr_detection(drone, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target)
            print("Leaving qr_detection function on line 148 in image_interface")
            print(f'target_qr_found: {target_qr_found}')
            return target_qr_found

        elif area > fbRange[1] and img_pass == 0:
            # The drone is too close to the target
            drone.move(back=20)
            info = check_camera(camera)
            trackObject(drone, info, turbines, starting_location, fileName, start, flag_time, st, fileFlag,target)
            print("Leaving trackObject function on line 156 in image_interface")

        elif area < fbRange[0] and area != 0 and img_pass == 0:
            # The drone is too far from the target
            if distance <= 500:
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
            target_qr_found = qr_detection(drone, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target)
            print("Leaving qr_detection function on line 185 in image_interface")
            print(f"target_qr_found: {target_qr_found}")
            return target_qr_found
    # else:
    #     if location[0:2] != starting_location: # If the drone has moved, return to the starting position
    #         drone.go_to(starting_location[0], starting_location[1], starting_location[2])

def qr_detection(drone, turbines, starting_location, fileName, start, flag_time, st, fileFlag, target=None):
    '''Begins searching for a QR code at the current location of the drone'''
    # distance_to_scan_qr = 30 # This is guessed distance of how far drone was in the x-direction in front of the fan when qr code was scanned
    distance_to_scan_qr = 100 # The drone seems to scan the qr code approximately 100 cm away 

    # Get x and y coordinates of the target turbine
    target_coordinates = turbines_copy[target][1]
    print(f'coordinates of {target}: {target_coordinates}')
    # Update drone location so it's y-coordinate matches that of the target turbine
    drone.set_coordinates(y_location=target_coordinates[1])
    print(f"\n>>>>>>>>>>>>>>>> UPDATED DRONE'S Y-COORDINATE TO MATCH THAT OF {target}\n")
    if target is None:
        print("\n>>>>>>>>>>>>>>>> ERROR, THERE IS NO TARGET WHEN THERE SHOULD BE IN QR_DETECTION\n")
            
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
    print(f'>>>>>>>>>>>>>>>> Z_LOCATION: {drone.get_z_location()}')
    traversal_altitude = drone.get_traversal_altitude()
    # Move drone down so that it is only 20 cm off the ground to scan QR code
    drone.set_coordinates(drone.get_x_location(), drone.get_y_location(), z_location=drone.get_height())
    drone.go_to(drone.get_x_location(), drone.get_y_location(), targetz=20)
    # drone.move(down=110)
    drone.get_height()
    print(f'>>>>>>>>>>>>>>>> Z_LOCATION: {drone.get_z_location()}')



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
        try:
            QR, img, info = droneReadQR(drone.get_drone())
        except:
            print(f'\n>>>>>>>>>> ERROR FROM droneReadQR')
            if img is None:
                print(f'\n>>>>>>>>>> IMAGE IS NONE')  
        img = cv.resize(img, (w, h))

        if len(QR) > 0:
            drone_var = drone.get_drone()
            drone.get_flight_time()
            print("\n>>>>>>>>>>>>>>>>QR CODE FOUND: ", QR)

            # Get x and y coordinates of the turbine whose QR code was scanned (may not be the target turbine)
            QR_coordinates = turbines_copy[QR][1]
            print(f'coordinates of {QR}: {QR_coordinates}')
            # Update drone location so it's x-coordinate matches that of the target turbine minus the distance to read the QR code
            drone.set_coordinates(x_location=QR_coordinates[0])
            print(f">>>>>>>>>>>>>>>> UPDATED DRONE'S X-COORDINATE TO MATCH THAT OF {QR} - {distance_to_scan_qr} cm\n")
            if QR not in turbines_copy:
                print("\n>>>>>>>>>>>>>>>> ERROR, QR CODE IS NOT FOR FANS 1-7\n")
            current = time.time()
            try:
                with open(fileName, "a") as csvFile:
                    csvwriter = csv.writer(csvFile, lineterminator='\n')
                    turb = QR  
                    turb = turb.replace('d', 'd ')
                    turb = turb.replace('_', ' ')
                    if(flag_time == 1):
                        if((current-previous_time)%60 < 10 and (current-start)% 60 < 10):
                            csvwriter.writerow([turb, current-previous_time, str(int((current-previous_time)//60)) + '.0' + str((current-previous_time)%60), current-start, str(int((current-start)//60)) + '.0' + str((current-start)%60), str(drone.get_battery()) + ' %'])
                        elif((current-previous_time)%60 < 10):
                            csvwriter.writerow([turb, current-previous_time, str(int((current-previous_time)//60)) + '.0' + str((current-previous_time)%60), current-start, str(int((current-start)//60)) + '.' + str((current-start)%60), str(drone.get_battery()) + ' %'])
                        elif((current-start)% 60 < 10):
                            csvwriter.writerow([turb, current-previous_time, str(int((current-previous_time)//60)) + '.' + str((current-previous_time)%60), current-start, str(int((current-start)//60)) + '.0' + str((current-start)%60), str(drone.get_battery()) + ' %'])
                        else:
                            csvwriter.writerow([turb, current-previous_time, str(int((current-previous_time)//60)) + '.' + str((current-previous_time)%60), current-start, str(int((current-start)//60)) + '.' + str((current-start)%60), str(drone.get_battery()) + ' %'])
                    else:
                        if((current-previous_time-start)%60 < 10 and (current-start)%60 < 10):
                            csvwriter.writerow([turb, current-previous_time-start, str(int((current-previous_time-start)//60)) + '.0' + str((current-previous_time-start)%60), current-start, str(int((current-start)//60)) + '.0' + str((current-start)%60), str(drone.get_battery()) + ' %'])
                        elif((current-previous_time-start)%60 < 10):
                            csvwriter.writerow([turb, current-previous_time-start, str(int((current-previous_time-start)//60)) + '.0' + str((current-previous_time-start)%60), current-start, str(int((current-start)//60)) + '.' + str((current-start)%60), str(drone.get_battery()) + ' %'])
                        elif((current-start)%60 < 10):
                            csvwriter.writerow([turb, current-previous_time-start, str(int((current-previous_time-start)//60)) + '.' + str((current-previous_time-start)%60), current-start, str(int((current-start)//60)) + '.0' + str((current-start)%60), str(drone.get_battery()) + ' %'])
                        else:
                            csvwriter.writerow([turb, current-previous_time-start, str(int((current-previous_time-start)//60)) + '.' + str((current-previous_time-start)%60), current-start, str(int((current-start)//60)) + '.' + str((current-start)%60), str(drone.get_battery()) + ' %'])
            except FileNotFoundError:
                print("File not appeneded") 
            # previous_time = current
            # with open('OutputLog.csv', 'a') as outFile:
            #     outFile.write(f"Found QR code:{QR} at {round(time()-start)}\n")

            if QR == target:
                drone.append_turbine_locations(QR)
                video.stop_qr()
                print("in qr_detection function on line 343")
                # video.start_haar() # This will enable bounding
                # sleep(0.5)
                drone.move(up=70) # Raise drone to be in front of the blades
                mission.mission0(drone, turbines[QR][0], QR) # Take pictures of the blades
                turbines.pop(QR) # Pop from the list since we won't need to visit it again
                    
                if len(turbines) != 0:
                    print("in qr_detection function on line 353")
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
            
            else: # This section controls what the drone does if the QR code doesn't match the target
                print(f">>>>>>>>>>>>>>>>QR CODE NOT MATCHING: QR {QR} != TARGET {target}")
                print(f">>>>>>>>>>>>>>>>QR = {QR}")
                print(f">>>>>>>>>>>>>>>>Target = {target}")
                video.stop_qr()

                # Get x and y coordinates of the turbine whose QR code was scanned (not the target turbine)
                # QR_coordinates = turbines_copy[QR][1]
                # We should already have coordinates of the QR code from code block where the drone's x-coordinate was updated
                print(f'coordinates of {QR}: {QR_coordinates}')
                # Update drone location so it's y-coordinate matches that of the target turbine
                drone.set_coordinates(y_location=QR_coordinates[1])
                print(f"\n>>>>>>>>>>>>>>>> UPDATED DRONE'S Y-COORDINATE TO MATCH THAT OF {QR}\n")
                if QR not in turbines_copy:
                    print("\n>>>>>>>>>>>>>>>> ERROR, QR CODE IS NOT FOR FANS 1-7\n")

                drone.move(up=110) # Move the drone up before continuing the mission
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
                if search_loop_counter < 2:
                    print("Drone completed snake search loop, repeating again")
                    # This will start the search again
                    img_counter = 0
                    # Keep track of which search increment we are in
                    search_loop_counter += 1
                elif search_loop_counter == 2:
                    # Tell drone to go home and land
                    print("QR code not found")
                    video.stop_qr()
                    sleep(1)
                    video.start_haar()
                    sleep(0.5)
                    drone.move(up=110)
                    # calibrate(drone, fileName, start, st, fileFlag, False) # Tell drone to land on helipad
                    return False # Return False so that we know a QR code was not found
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

def calculate_distance(drone, info):
    '''Used for testing the accuracy of used distance calculation'''
    # if the object is no longer detected, attempt to find it with 10 more frames
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

    area = info[1]  # The area of the bounding box
    x, y = info[0]  # The x and y location of the center of the bounding box in the frame
    width = info[2] # The width of the bounding box
    
    # object detected
    if(x != 0):
        distance = int(650 * real_width / width) # (Focal length of camera lense * Real-world width of object)/Width of object in pixels
        return distance # Used for testing the accuracy of distance
    else:
        print(f'>>>>>>>>>> OBJECT NOT DETECTED')

def write_to_csv(arr, header, filename):
    # Open the CSV file in append mode
    with open(filename, 'r', newline='') as file:
        reader = csv.reader(file)
        rows = []
        for row in reader:
            rows.append(row)
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        # Write the header to the first row of the new column
        if len(rows) > 0:
            writer.writerow(rows[0] + [header])
        else:
            writer.writerow([header])
        # Write the array to the new column
        for i in range(len(arr)):
            if i+1 < len(rows):
                writer.writerow(rows[i+1] + [arr[i]])
            else:
                writer.writerow([arr[i]])

if __name__ == "__main__":

    testing = input('Would you like to test distance calculation? (y/n): ')
    if testing == 'y' or 'Y':
        real_distance = int(input('Enter the real distance of the object in cm: '))

        # These are garbage fill in variables to do downvision function
        st = mission_st
        fileName = "CSV Files/Data Log " + st + ".csv"
        start_time = time.time()
        fileFlag = 0
        
        drone = mov.movement()
        camera = drone.get_drone()
        info = check_camera(camera)
        distances = []
        filename = 'DistanceLogCageFans.csv'
        
        calibrate(drone, fileName, start_time, st, fileFlag)
        drone.move(up = drone.traversal_altitude - drone.drone.get_height())
        video = drone.get_video()
        sleep(1)
        video.start_haar()
        sleep(0.5)

        for i in range(48):    
            drone.go_to()
            distance = calculate_distance(drone, info)
            print(f'>>>>>>>>>> distance = {distance}')
            distances.append(distance)
            info = check_camera(camera)

        print('>>>>>>>>>> WRITING TO CSV FILE')
        write_to_csv(distances, real_distance, filename)
        print('>>>>>>>>>> LANDING DRONE')
        drone.land()


#     while True:
#         frame = drone.get_frame_read()
#         sleep(0.2)
#         img = frame.frame
#         img = cv.resize(img, (w, h))
#         img, info = hc.findTurbine(img)
#         location = trackObject(drone, info, location, turbines, , fileName, start, flag_time)
#         img = cv.resize(img, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
    # drone = Tello()
    # drone.connect()
    # drone.streamon()


   
        