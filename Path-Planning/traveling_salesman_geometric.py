import math
import numpy as np
import matplotlib.pyplot as plt
import random
from check_camera import check_camera
from salesman_image_interface import trackObject, qr_detection
from downvision_calibration import calibrate
import time
import csv
import movement as mov
import random
from datetime import datetime
from vars import *


# Set boundaries of plots
boundaries = [0, 1000, 0, 600]

# Testing 1-7 random fans
number_of_fans = 7
# Set number outside of 0-7 if you want to target different quantities every run without changing code since you will be prompted to select a quantity
while number_of_fans < 1 or number_of_fans > 7: 
    number_of_fans = int(input('Enter the quantity of fans being used for the experiment (1-7): '))
    if number_of_fans < 1 or number_of_fans > 7:
        print('Invalid quantity of fans entered, choose a integer between 1-7.')
list1 = [1, 2, 3, 4, 5, 6, 7] # List of fans 1-7 identified by their QR code number
fans_list = [4] # Randomly selected order of fans will be inputted here, 4 is damaged fan we always want to include  
list1.remove(4)
for x in range(number_of_fans - 1): # Put fan numbers into list in which the order is random, minus 1 since we always include 4
    random_fan = random.choice(list1)
    fans_list.append(random_fan)
    list1.remove(random_fan)
# Change these numbers below to determine which fans to test, if you want random fans, comment out the fans_list below, extra fan quantity in list will be ignored
# fans_list = [4, 3, 4, 3, 6, 1, 7]

# Get rid of the turbines in the dictionary that are not part of the mission
turbines_to_pop = []
for turbine in turbines:
    if int(turbine[-1]) not in fans_list:
        turbines_to_pop.append(turbine)
for turbine in turbines_to_pop:
    turbines.pop(turbine)

if number_of_fans == 1: # The program requires 3 points, so testing just 1 fan will need the (0,0) coordinate twice
    path_x_coordinates = [0, fans_x_coordinates[fans_list[0] - 1], 0] # Minus 1 since lists are 0-based and there is no 0th fan
    path_y_coordinates = [0, fans_y_coordinates[fans_list[0] - 1], 0] # Minus 1 since lists are 0-based and there is no 0th fan
    print("\n>>>>>>>>>>>>>>>>>>>>> CHOOSING FAN " + str(fans_list[0]))
else:  # When we are using more than 1 fan
    path_x_coordinates = [0] # This will have starting point 0 and randomly selected x coordinates of fans
    path_y_coordinates = [0] # This will have starting point 0 and randomly selected y coordinates of fans
    print("\n>>>>>>>>>>>>>>>>>>>>> CHOOSING FANS ", end ="")
    for index in range(number_of_fans): # Set up x and y coordinates of randomly selected fans and print which fans are part of the path
        path_x_coordinates.append(fans_x_coordinates[fans_list[index] - 1]) # Minus 1 since lists are 0-based and there is no 0th fan
        path_y_coordinates.append(fans_y_coordinates[fans_list[index] - 1]) # Minus 1 since lists are 0-based and there is no 0th fan
        if index != number_of_fans-1: # Not the last fan
            print(str(fans_list[index]) + ", ", end="")
        else: # The last fan
            print("and " + str(fans_list[index]) + "\n")

# This will be used to know which identication goes with each coordinate
fans_coordinates = []
for x, y in zip( fans_x_coordinates, fans_y_coordinates ):
    fans_coordinates.append( [ x, y ] )

# Set coordinates for the drone's path planning
xpos = np.array(path_x_coordinates)
xpos = np.append(xpos, xpos[0])
ypos = np.array(path_y_coordinates)
ypos = np.append(ypos, ypos[0])
data = np.array([xpos, ypos], np.int32) 

class TravelingSalesman():
    def __init__(self):
        path1 = np.copy(data)
        
        e1 = self.energy_calc(data)
        e_min = e1
        self.path_min = data
        print(data)
        print(f">>>>>>>>>>>>>>>>>>>>>> STARTING PATH ENERGY: {e1}\n")

        array_size = xpos.size
        T = 150 * array_size
        while T > 1:
            for i in range(30 * array_size):
                if e1 < e_min:
                    e_min = e1
                    self.path_min = np.copy(path1)
                rand_index = random.randint(1, xpos.size-3)
                path2 = np.copy(path1)# line 34
                path2[0, rand_index] = path1[0, rand_index+1]
                path2[1, rand_index] = path1[1, rand_index+1]
                path2[0, rand_index+1] = path1[0, rand_index]
                path2[1, rand_index+1] = path1[1, rand_index]
                e2 = self.energy_calc(path2)

                if e2 < e1:
                    e1 = e2
                    path1 = np.copy(path2)
                else:
                    c1 = math.exp(-(e2 - e1)/T)
                    # random.seed(rand_seed)
                    c2 = random.random()
                    # random.seed(a=None)

                    if c1 > c2:
                        path1 = np.copy(path2)
                        e1 = e2
                    
            T = 0.99 * T
        # print(path1) # The last generated path of the simulated annealing algorithm, may or may not be optimized path
        # self.path_min = np.array([np.flip(self.path_min[0]), np.flip(self.path_min[1])]) # Reverse the order of the traveling salesman, does not change energy
        print(self.path_min) # The optimized path 
        print(f">>>>>>>>>>>>>>>>>>>>> OPTIMIZED PATH ENERGY: {self.energy_calc(self.path_min)}\n")
        
    def plot(self):
        plt.rcParams["font.family"] = "Times New Roman"
        figure, axis = plt.subplots(1, 2)
        axis[0].axis(boundaries)
        axis[0].set_title(f'Path Before Optimization: {number_of_fans} Turbine(s) {fans_list}', fontsize=20)
        axis[0].set_xlabel('x (cm)', fontsize=16)
        axis[0].set_ylabel('y (cm)', fontsize=16)
        for i, j, k in zip(fans_x_coordinates, fans_y_coordinates, turbines_copy):  # We will plot and label all the turbines, even if they are not targeted
            # Writes the (x,y) coordinates and turbine number above the coordinate location
            axis[0].text(i-80, j+20, 'Turbine #{} ({}, {})'.format(k[-1], i, j), weight="bold", size=13)
        axis[0].text(boundaries[0]+20, boundaries[-1]-40, f'Total Energy: {int(self.energy_calc(data))}', size=15, weight="bold")
        # Plot all the turbines, even if they are not targeted
        axis[0].plot(fans_x_coordinates, fans_y_coordinates, 'o', markersize=10) 
        # Draw the arrows for the original unoptomized path (some turbines may not be a part of the path)
        axis[0].quiver(data[0][:-1], data[1][:-1], data[0][1:]-data[0][:-1], 
                   data[1][1:]-data[1][:-1],scale_units='xy', angles='xy', scale=1, color='teal', width=0.010)
        axis[0].set_aspect('equal', adjustable='box') # Set the scale of the x and y axes to be equivalent
        axis[0].grid(color = 'green', linestyle = '--', linewidth = 0.5) # Plot grid lines

        axis[1].axis(boundaries)
        axis[1].set_title(f'Path After Optimization: {number_of_fans} Turbine(s) {fans_list}', fontsize=20)
        axis[1].set_xlabel('x (cm)', fontsize=16)
        axis[1].set_ylabel('y (cm)', fontsize=16)
        for i, j, k in zip(fans_x_coordinates, fans_y_coordinates, turbines_copy): # Label all the turbines, even if they are not targeted
            # Writes the (x,y) coordinates and turbine number above the coordinate location
            axis[1].text(i-80, j+20, 'Turbine #{} ({}, {})'.format(k[-1], i, j), weight="bold", size=13)
        axis[1].text(boundaries[0]+20, boundaries[-1]-40, f'Total Energy: {int(self.energy_calc(self.path_min))}', size=15, weight="bold")
        # Plot all the turbines, even if they are not targeted
        axis[1].plot(fans_x_coordinates, fans_y_coordinates, 'o', markersize=10)
        # Draw the arrows for the new optimized path (some turbines may not be a part of the path)
        axis[1].quiver(self.path_min[0][:-1], self.path_min[1][:-1], self.path_min[0][1:]-self.path_min[0][:-1], 
                   self.path_min[1][1:]-self.path_min[1][:-1],scale_units='xy', angles='xy', scale=1, color='teal', width=0.010)
        # Set the scale of the x and y axes to be equivalent
        axis[1].set_aspect('equal', adjustable='box') # This will make the scale of both axes equal
        axis[1].grid(color = 'green', linestyle = '--', linewidth = 0.5) # Plot grid lines

        # You can play with the configure subplots button on bottom of graph for desired look, but changes are not remembered
        # Below, change the values to values desired on the subplots and they come like this every time
        # plt.subplots_adjust(left=0.07, bottom=0.26, right=0.7, top=0.93, wspace=0.2, hspace=0.4)
        # Make full screen for better view, Alt-F4 to exit full screen
        # manager = plt.get_current_fig_manager()
        # manager.full_screen_toggle() 
        plt.show()

    def get_path(self):
        return np.copy(self.path_min)

    def energy_calc(self, data, num = 0):
        energy = 0
        for i in range(int((data.size)/2)-1):
            calc = math.sqrt((data[0][i] - data[0][i+1])**2 + (data[1][i] - data[1][i+1])**2)
            if num != 0 and calc < 20: print(f"Path too short at index: {i} to {i+1}")
            energy += calc
        return energy

if __name__ == "__main__":
    
    st = mission_st
    fileName = "CSV Files/Data Log " + st + ".csv"
    # Uncomment to get positions of each target in inches
    # with open('Positions.csv', 'w') as outFile: 
    #     for item in turbines:
    #         positionx = turbines[item][1][0]/2.54
    #         positiony = turbines[item][1][1]/2.54
    #         outFile.write(f"{item}: ({positionx}, {positiony})\n")
    path = TravelingSalesman() 
    # with open('OutputLog.csv', 'a') as outFile:
    #     outFile.write(f"Annealing finished at {round(time.time()-start_time)}\n")
    fileFlag = 0
    coordinates = path.get_path()
    coordinates = np.delete(coordinates, 0, axis=1)
    coordinates = np.delete(coordinates, -1, axis=1)
    print(f'>>>>>>>>>> COORDINATES: {coordinates}\n')

    # Code to make sure that the turbines dictionary is in the order that they would be visited
    turbines_path = {}
    for x,y in zip(coordinates[0], coordinates[1]):
        for turbine in turbines:
            if [x,y] == turbines[turbine][1]:
                turbines_path.update({turbine: turbines[turbine]})
    for turbine in turbines_path:
        print(turbine)

    path.plot()
    finished = False
    drone = mov.movement()
    print(f'>>>>>>>>>> Z_LOCATION: {drone.get_z_location()}')
    start_time = time.time()
    camera = drone.get_drone()
    Headings  = ['Mission Date and Time', 'Starting Battery (s)', 'Turbine Quantity']
    with open(mission_filename, "a") as csvFile:
        csvwriter = csv.writer(csvFile, lineterminator='\n')
        csvwriter.writerow([])
        csvwriter.writerow(Headings)
        csvwriter.writerow([mission_st, drone.get_battery(), number_of_fans])
        csvwriter.writerow(['Turbine ID', 'Time Spent for Mission Around Turbine (s)', 'Battery Spent for Mission around Turbine (%)', 'Battery Remaining (%)', 'Current Flight Time (s)'])
    flag_time = 1
    
    image_processing_distance = 200 # Value we want to be in front of the fan for image processing

    while finished == False:
        index = -1
        test = 0
        for location in range(int(coordinates.size/2)):
            index += 1
            if drone.get_battery() < 20:
                print("\n" + ">>>>>>>>>>>>>>>> DRONE BATTERY LOW. CHANGE BATTERY!")
                calibrate(drone, fileName, start_time, st, fileFlag = 0, land=True)
                try:
                    input("DRONE BATTERY LOW. CHANGE BATTERY, RECONNECT, THEN PRESS ENTER.")
                except:
                    input("DRONE BATTERY LOW. CHANGE BATTERY, RECONNECT, THEN PRESS ENTER.")
                drone = mov.movement()
                path = TravelingSalesman() 
                path.plot()
                coordinates = path.get_path()
                camera = drone.get_drone()
                break
            test += 1
            if coordinates[0][location] == 0 or coordinates[0][location] == 1000: # second number to be changed to whatever the boundary size is
                fileFlag = 1
                calibrate(drone, fileName, start_time, st, fileFlag, False, coordinates[0][location], coordinates[1][location])
                print("Left calibrate function in line 252 of traveling_salesman")
                # Print total time of flying
                print(">>>>>>>>>>>>>>>> TOTAL FLIGHT TIME: ", time.time() - start_time)
                
            else: # We get here after detecting incorrect fan and landing
                print("not Calibrate")
                print(">>>>>>>>>>>>>>>> TOTAL FLIGHT TIME: ", time.time() - start_time)
                target_turbine = None
                for name in turbines:
                    if turbines[name][1][0] == coordinates[0][location] and turbines[name][1][1] == coordinates[1][location]:
                        target_turbine = name
                # List the target turbine number
                print(">>>>>>>>>>>>>>>> SELECTING TARGET TURBINE: " + str(target_turbine))

                # Something went wrong if we do not have a target turbine
                if target_turbine == None:
                    print(f">>>>>>>>>>>>>>>> ERROR THERE IS NO TARGET TURBINE")
                    print(f">>>>>>>>>>>>>>>> turbines: {turbines}")
      
                # Rotate the drone to face the next fan
                # drone.go_to(coordinates[0][location], coordinates[1][location], rotate_only=True) 
                
                # Angel's edit - Move the drone towards the fan, stop short on x-axis and face forward to avoid detecting other fans
                drone.go_to(coordinates[0][location] - image_processing_distance, coordinates[1][location], 0)#, half_travel=True)

                # Take 10 images to find the location of the target and do the mission if it is found
                info = check_camera(camera)      
                found = trackObject(drone, info, turbines, [drone.get_x_location(), drone.get_y_location(), drone.get_angle()], fileName, start_time, flag_time, st, fileFlag, target_turbine)
                print("Left trackObject function on line 276 in traveling_salesman")
                print(f">>>>>>>>>>>>>>>> found: {found}")

                if len(turbines) == 0: # Sometimes we end up here after landing on helipad
                    print(">>>>>>>>>>>>>>>> TOTAL FLIGHT TIME: ", time.time() - start_time)
                    print("I am in line 279 of traveling_salesman")
                    finished = True
                if found == False: # We should end up here when we detected the incorrect QR code
                    print(">>>>>>>>>>>>>>>> REATTEMPTING TARGET TURBINE: " + str(target_turbine))
                    # Get x and y coordinates of the target turbine
                    target_coordinates = turbines[target_turbine][1]
                    print(f'coordinates of {target_turbine}: {target_coordinates}')
                    # Move the drone to the target fan, stop short on x-direction by image_processing_distance and face forward (angle=0) to avoid detecting other fans
                    drone.go_to(target_coordinates[0] - image_processing_distance, target_coordinates[1], 0)
                    if target_turbine is None:
                        print(f"\n>>>>>>>>>>>>>>>> ERROR, THERE IS NO TARGET_TURBINE WHEN THERE SHOULD BE IN TRAVELING_SALESMAN")
                        print(f">>>>>>>>>>>>>>>> turbines: {turbines}\n")
                    info = check_camera(camera)
                    found = trackObject(drone, info, turbines, [drone.get_x_location(), drone.get_y_location(), drone.get_angle()], fileName, start_time, flag_time, st, fileFlag, target_turbine)
                    print("Left trackObject function on line 300 in traveling_salesman")
                
                img_counter = 0 
                while found == False: # Turbine was not seen at image_processing_distance in x-direction
                    # y-coordinate of the drone should be the same as the y-coordinate of the turbine and drone should be facing forward
                    # Calculate total distance from drone to target turbine using Pythagorean theorem: c = sqrt(a^2 + b^2) 
                    dist = math.sqrt((drone.get_x_location() - coordinates[0][location])**2 + (drone.get_y_location() - coordinates[1][location])**2)
                    lost_flag = 0
                    if lost_flag: # Should be true since we are expected to be image_processing_distance in front of fan
                        print('In line 275 of the traveling salesman')
                        # drone.go_to(coordinates[0][location] - x_distance_cutoff, coordinates[1][location], half_travel=True)
                        # Let's back up, face forward and check our camera 
                        drone.move(back=image_processing_distance/2)
                        info = check_camera(camera)
                        # Check if we can track the fan position
                        found = trackObject(drone, info, turbines, [drone.get_x_location(), drone.get_y_location(), drone.get_angle()], fileName, start_time, flag_time, st, fileFlag, target_turbine)
                        print("Left trackObject function on line 311 in traveling_salesman")
                        flag_time = 0
                        lost_flag = 1
                    else:
                        print(f'>>>>>>>>>>>>>>>> I THINK WE ARE LOST')
                        # Go to expected fan location minus the x_cutoff_distance in front of the fan and search for qr code
                        drone.go_to(coordinates[0][location] - x_distance_cutoff, coordinates[1][location], 0)
                        found = qr_detection(drone, turbines, [drone.get_x_location(), drone.get_y_location(), drone.get_z_location()], fileName, start_time, flag_time, st, fileFlag, target_turbine)
                        print("Left qr_detection function on line 315 of traveling salesman")
                        flag_time = 0
                        print(f'>>>>>>>>>>>>>>> found: {found}') # found should be False if we did not scan a QR code or did not scan any QR code
                        # found = True
                if coordinates[0][location] == coordinates[0][-1] and coordinates[1][location] == coordinates[1][-1]:
                    fileFlag = 1
                    calibrate(drone, fileName, start_time, st, fileFlag, False, coordinates[0][location], coordinates[1][location])
                    print("Left calibrate function in line 321 of traveling_salseman")
                    # Print total time of flying
                    print(">>>>>>>>>>>>>>>> TOTAL FLIGHT TIME: ", time.time() - start_time)
                    finished = True

    # drone.go_to(ending_angle=0)
    # print(">>>>>>>>>>>>>>>> TOTAL FLIGHT TIME: ", time.time() - start_time)
    # # with open('OutputLog.csv', 'a') as outFile:
    # #             outFile.write(f"Ended at: {round(time.time()-start_time)}\n")
    # #             outFile.write(f"Ending battery: {camera.get_battery()}\n")
    # calibrate(drone, fileName, start_time, st, fileFlag = 1, land=True)
    # drone.land(turn_off = True)


    