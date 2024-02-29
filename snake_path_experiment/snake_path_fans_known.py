'''The snake path search algorithm experiment in which the drone knows now many turbines are in the search and scan mission, but the drone does not know their coordinates. 
By Angel Rodriguez 2023'''

from check_camera import check_camera
from salesman_image_interface import trackObject
from downvision_calibration import calibrate
import time
import csv
import movement as mov
from datetime import datetime
 
# Fan coordinate stuff we will not use but have since alot of functions created expect these parameters
fans_x_coordinates = [360, 832, 550, 613, 161, 832, 188]
fans_y_coordinates = [48, 409, 224, 460, 227, 150, 457]

# Keeping this code for now to avoid errors
turbines = {"WindTurbine_1": [[0, 0, 0, 0], [fans_x_coordinates[0], fans_y_coordinates[0]]], "WindTurbine_2": [[0, 0, 0, 0], [fans_x_coordinates[1], fans_y_coordinates[1]]], "WindTurbine_3": [[0, 0, 0, 0], [fans_x_coordinates[2], fans_y_coordinates[2]]],
                "WindTurbine_4": [[0, 0, 0, 0], [fans_x_coordinates[3], fans_y_coordinates[3]]], "WindTurbine_5": [[0, 0, 0, 0], [fans_x_coordinates[4], fans_y_coordinates[4]]],
                "WindTurbine_6": [[0, 0, 0, 0], [fans_x_coordinates[5], fans_y_coordinates[5]]], "WindTurbine_7": [[0, 0, 0, 0], [fans_x_coordinates[6], fans_y_coordinates[6]]]}

turbines_length = len(turbines)
turbine_quantity = 5 # Quantity of fans in the mission

# CSV file stuff
st = datetime.now().strftime('%B %d,%Y %H.%M.%S')
fileName = "CSV Files/Data Log " + st + ".csv"
fileFlag = 0
flag_time = 1

# Initialize drone object and take off
drone = mov.movement()
# Get the start time of flight
start_time = time.time()

def check_for_turbines():
    '''Return True if a new QR code was found
       Return False if a previously scanned QR code was scanned again
       Return None if a fan was not found at all'''
    # Take 10 images to find the location of the target and do the mission if it is found
    info = check_camera(camera)      
    # Update_coordinates is False since we are not supposed to know the location of the fans
    found = trackObject(drone, info, turbines, [drone.get_x_location(), drone.get_y_location(), drone.get_angle()], fileName, start_time, flag_time, st, fileFlag, update_coordinates=False)
    print(">>>>>>>>>>>>>>>> TOTAL FLIGHT TIME: ", time.time() - start_time)
    # print("Left trackObject function on line 47 in snake_path_fans_known")
    print(f">>>>>>>>>>>>>>>> found: {found}")
    if (turbines_length - len(turbines) == turbine_quantity): # This means mission is completed since all fans were detected
        print(">>>>>>>>>>>>>>>> MISSION SUCCESS, ALL FANS FOUND, PREPARING TO LAND")
        calibrate(drone, fileName, start_time, st, fileFlag = 0)
        print(">>>>>>>>>>>>>>>> MISSION SUCCESS, ALL FANS FOUND")
        print(">>>>>>>>>>>>>>>> TOTAL FLIGHT TIME: ", time.time() - start_time)
    return found


if __name__ == "__main__":
    # Writing flight time and other info to CSV file    
    with open('OutputLog.csv', 'w') as outFile:
        outFile.write(f"{round(start_time)}\n")
    camera = drone.get_drone()
    Headings  = ['Location', 'Time to find Location (s)', 'Time to find Location (m.s)', 'Total time up to this point (s)', 'Total time up to this point (m.s)', 'Current Battery Life (%)']
    csvFile = open(fileName, "w")
    csvwriter = csv.writer(csvFile, lineterminator='\n')
    csvwriter.writerow(Headings)
    csvwriter.writerow(['Starting Helipad', 0, 0, 0, 0, str(camera.get_battery()) + ' %'])
    csvFile.close()

    if camera.get_battery() < 20: # if battery is under 20%
        print("\n" + ">>>>>>>>>>>>>>>> DRONE BATTERY LOW. CHANGE BATTERY!")

    x_step = 120 # Steps the drone takes fowards and backwards
    y_step = 90*2.54 # Steps the drone takes left (90" in cm to match fan spacing)
    x_boundary = 1000 # x-axis boundary of experiment
    y_boundary = 600 # y-axis boundary of experiment

    while drone.get_y_location() <= y_boundary: # The snake path progressively moves towards the y_boundary which should be the end of its mission
        while drone.get_x_location() + x_step <= x_boundary: # Keep moving forward until the x_boundary is reached
             # Record the drone's coordinates for future use in case we scan a qr code again
            temp_y_coordinate = drone.get_y_location()   
            # Check for fans
            found_new_qr_code = check_for_turbines()
            if found_new_qr_code == True: # Return back to search path
                drone.y_go_to(temp_y_coordinate)
            drone.move(fwd=x_step)
        if drone.get_y_location() + y_step <= y_boundary: # We should pass into here every time except when we are on the y_boundary
            # Continue to the next row of the snake path search algorithm
            drone.move(left=y_step)
        else: # When here, we should be done with the mission, so exit the while loop
            break
        while drone.get_x_location() > 0: # Move all the way back to x=0
            # Record the drone's coordinates for future use in case we scan a qr code again
            temp_x_coordinate = drone.get_x_location()
            temp_y_coordinate = drone.get_y_location()
            # Check for fans
            found_new_qr_code = check_for_turbines()
            # Want to move back extra as to avoid scanning the same fan again
            drone.x_go_to(temp_x_coordinate) 
            # Move to the previous y-coordinate in case we stepped out of the snake search path
            drone.y_go_to(temp_y_coordinate) 
            if found_new_qr_code == False: 
                pass
            drone.move(back=x_step)

        # The drone is now behind x = 0 (helipad location), Before we move left to continue forward, check for fans
        temp_x_coordinate = drone.get_x_location()
        temp_y_coordinate = drone.get_y_location()
        check_for_turbines()
        # Want to move back extra as to avoid scanning the same fan again
        drone.x_go_to(temp_x_coordinate) 
        # Move to the previous y-coordinate in case we stepped out of the snake search path
        drone.y_go_to(temp_y_coordinate) 

        if drone.get_y_location() + y_step <= y_boundary: # We should pass into here every time except when we are on the y_boundary
            # Continue to the next row of the snake path search algorithm
            drone.move(left=y_step)
        else: # When here, we should be done with the mission, so exit the while loop
            break

    print("OUTSIDE OF THE WHILE LOOP, NOT ALL FANS WERE FOUND, PREPARING TO GO TO HELIPAD AND LAND")
    drone.move(cw=180)
    calibrate(drone, fileName, start_time, st, fileFlag = 0)
    print(">>>>>>>>>>>>>>>> MISSION FAILED, NOT ALL FANS WERE FOUND")
    print(">>>>>>>>>>>>>>>> TOTAL FLIGHT TIME: ", time.time() - start_time)
    
      
        
        