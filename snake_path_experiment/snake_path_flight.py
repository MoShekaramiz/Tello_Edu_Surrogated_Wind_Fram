'''The snake path search algorithm experiment in which the coordinates of fans is unknown. 
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
target_turbine = None

# Keeping this code for now to avoid errors
turbines = {"WindTurbine_1": [[0, 0, 0, 0], [fans_x_coordinates[0], fans_y_coordinates[0]]], "WindTurbine_2": [[0, 0, 0, 0], [fans_x_coordinates[1], fans_y_coordinates[1]]], "WindTurbine_3": [[0, 0, 0, 0], [fans_x_coordinates[2], fans_y_coordinates[2]]],
                "WindTurbine_4": [[0, 0, 0, 0], [fans_x_coordinates[3], fans_y_coordinates[3]]], "WindTurbine_5": [[0, 0, 0, 0], [fans_x_coordinates[4], fans_y_coordinates[4]]],
                "WindTurbine_6": [[0, 0, 0, 0], [fans_x_coordinates[5], fans_y_coordinates[5]]], "WindTurbine_7": [[0, 0, 0, 0], [fans_x_coordinates[6], fans_y_coordinates[6]]]}

# lab_fans_x_coordinates = [282, 282, 282]
# lab_fans_y_coordinates = [119, -117, 0] 
# Lab Test Turbines   
# Lab Fan Coordinates and test points
# turbines = {"WindTurbine_8": [[0, 0, 0, 0], [lab_fans_x_coordinates[0], lab_fans_y_coordinates[0]]], 
#             "WindTurbine_9": [[0, 0, 0, 0], [lab_fans_x_coordinates[1], lab_fans_y_coordinates[1]]],
#             "WindTurbine_10": [[0, 0, 0, 0], [lab_fans_x_coordinates[2], lab_fans_y_coordinates[2]]]}

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
    # Take 10 images to find the location of the target and do the mission if it is found
    info = check_camera(camera)      
    found = trackObject(drone, info, turbines, [drone.get_x_location(), drone.get_y_location(), drone.get_angle()], fileName, start_time, flag_time, st, fileFlag)
    print("Left trackObject function on line 52 in snake_path_experiment")
    print(f">>>>>>>>>>>>>>>> found: {found}")

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

    x_step = 120 # Steps the drone takes forwards and backwards
    y_step = 90*2.54 # Steps the drone takes left (90" in cm to match fan spacing)
    x_boundary = 1000 # x-axis boundary of experiment
    y_boundary = 500 # y-axis boundary of experiment

    while drone.get_y_location() <= y_boundary: # The snake path progressively moves towards the y_boundary which should be the end of its mission
        print("----------------------------------------")
        print(f"{drone.get_location()}") 
        while drone.get_x_location() + x_step <= x_boundary: # Keep moving forward until the x_boundary is reached
            drone.move(fwd=x_step)
            # Check for fans
            # check_for_turbines()
        if drone.get_y_location() + y_step <= y_boundary: # We should pass into here every time except when we are on the y_boundary
            # Continue to the next row of the snake path search algorithm
            drone.move(left=y_step)
        else: # When here, we should be done with the mission, so exit the while loop
            break
        while drone.get_x_location() > 0: # Move all the way back to x=0
            drone.move(back=x_step)
            # check_for_turbines()
        if drone.get_y_location() + y_step <= y_boundary: # We should pass into here every time except when we are on the y_boundary
            # Continue to the next row of the snake path search algorithm
            drone.move(left=y_step)
        else: # When here, we should be done with the mission, so exit the while loop
            break
        print("----------------------------------------")
        print(f"{drone.get_location()}")

    print("OUTSIDE OF THE WHILE LOOP, NOT ALL FANS WERE FOUND, PREPARING TO GO TO HELIPAD AND LAND")
    # print("----------------------------------------")
    # print(f"{drone.get_location()}")
    # Land drone on helipad
    drone.move(cw=180)
    calibrate(drone, fileName, start_time, st, fileFlag = 0)
    print(">>>>>>>>>>>>>>>> MISSION FAILED, NOT ALL FANS WERE FOUND")
    print(">>>>>>>>>>>>>>>> TOTAL FLIGHT TIME: ", time.time() - start_time)
      
        
        