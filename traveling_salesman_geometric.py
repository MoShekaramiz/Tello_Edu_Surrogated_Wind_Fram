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
start = time.time()

# Testing 1-7 random fans
number_of_fans = 7 # Change this number to the number of fans you would like to test 
fans_x_coordinates = [360, 832, 550, 613, 61, 832, 188]
fans_y_coordinates = [48, 409, 224, 460, 127, 150, 457]
list1 = [1, 2, 3, 4, 5, 6, 7] # List of fans 1-7 identified by their QR code number
fans_list = [] # Randomly selected order of fans will be inputted here  
for x in range(number_of_fans): # Put fan numbers into list in which the order is random
    random_fan = random.choice(list1)
    fans_list.append(random_fan)
    list1.remove(random_fan)
# Change these numbers below to determine which fans to test, if you want random fan, comment out the fans_list below
# fans_list = [3, 2, 3, 4, 5, 6, 7]
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
        print(path1)
        print(f">>>>>>>>>>>>>>>>>>>>> OPTIMIZED PATH ENERGY: {self.energy_calc(self.path_min)}\n")
        print("Run time: ", time.time() - start)
        
    def plot(self):
        plt.rcParams["font.family"] = "Times New Roman"
        figure, axis = plt.subplots(2, 1)
        axis[0].axis([0, 1000, 0, 550])
        axis[0].set_title('Path Before Optimization', fontsize=20)
        axis[0].set_xlabel('x (cm)', fontsize=16)
        axis[0].set_ylabel('y (cm)', fontsize=16)
        for i, j in zip(data[0], data[1]): # Writes the (x,y) coordinates above the coordinate location
            axis[0].text(i, j+20, '({}, {})'.format(i, j), fontsize='small')
        axis[0].text(10, 500, f'Total Energy: {int(self.energy_calc(data))}', fontsize='medium', weight="bold")
        axis[0].plot(data[0], data[1], '-o', markersize=4) 
        axis[0].quiver(data[0][:-1], data[1][:-1], data[0][1:]-data[0][:-1], 
                   data[1][1:]-data[1][:-1],scale_units='xy', angles='xy', scale=1, color='teal', width=0.005)
        # Labeling Turbine numbers - beginning
        plot_index = 0
        for x,y in zip(data[0],data[1]):
            if plot_index == 0: # Skip first loop since there is no turbine at origin (0,0)
                plot_index += 1
                continue
            if plot_index > number_of_fans:
                break
            turbine_number = fans_y_coordinates.index(y) + 1 # Plus 1 since lists are 0-based, using y since y-coordinates never repeated in fans_y_coordinates
            label = "Turbine #" + str(turbine_number)
            axis[0].annotate(label, # this is the text which we want to use as a label
                        (x,y), # x and y is the points location where we want to label
                        textcoords="offset points",
                        weight="bold",
                        xytext=(-26,7), # this for the distance between the points and the turbine text label
                        ha='center')
            plot_index += 1
        # Labeling Turbine numbers - end

        axis[1].axis([0, 1000, 0, 550])
        axis[1].set_title('Path After Optimization', fontsize=20)
        axis[1].set_xlabel('x (cm)', fontsize=16)
        axis[1].set_ylabel('y (cm)', fontsize=16)
        for i, j in zip(self.path_min[0], self.path_min[1]): # Writes the (x,y) coordinates above the coordinate location
            axis[1].text(i, j+20, '({}, {})'.format(i, j), fontsize='small')
        axis[1].text(10, 500, f'Total Energy: {int(self.energy_calc(self.path_min))}', fontsize='medium', weight="bold")
        axis[1].plot(self.path_min[0], self.path_min[1], '-o', markersize=4)
        axis[1].quiver(self.path_min[0][:-1], self.path_min[1][:-1], self.path_min[0][1:]-self.path_min[0][:-1], 
                   self.path_min[1][1:]-self.path_min[1][:-1],scale_units='xy', angles='xy', scale=1, color='teal', width=0.005)
        # Labeling Turbine numbers - beginning
        plot_index = 0
        for x,y in zip(self.path_min[0],self.path_min[1]):
            if plot_index == 0: # Skip first loop since there is no turbine at origin (0,0)
                plot_index += 1
                continue
            if plot_index > number_of_fans:
                break
            # turbine_number = fans_list[plot_index-1]
            turbine_number = fans_y_coordinates.index(y) + 1 # Plus 1 since lists are 0-based, using y since y-coordinates never repeated in fans_y_coordinates
            label = "Turbine #" + str(turbine_number)
            axis[1].annotate(label, # this is the text which we want to use as a label
                        (x,y), # x and y is the points location where we want to label
                        textcoords="offset points",
                        weight="bold",
                        xytext=(-26,7), # this for the distance between the points and the turbine text label
                        ha='center')
            plot_index += 1
        # Labeling Turbine numbers - end

        # You can play with the configure subplots button on bottom of graph for desired look, but changes are not remembered
        # Below, change the values of None to values desired 
        plt.subplots_adjust(left=0.07, bottom=0.26, right=0.7, top=0.93, wspace=0.2, hspace=0.4)
        manager = plt.get_current_fig_manager()
        manager.full_screen_toggle() # Make full screen for better view, Alt-F4 to exit full screen
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
    with open('OutputLog.csv', 'w') as outFile:
        outFile.write(f"{round(start)}\n")
    turbines = {"WindTurbine_1": [[0, 0, 0, 0], [fans_x_coordinates[0], fans_y_coordinates[0]]], "WindTurbine_2": [[0, 0, 0, 0], [fans_x_coordinates[1], fans_y_coordinates[1]]], "WindTurbine_3": [[0, 0, 0, 0], [fans_x_coordinates[2], fans_y_coordinates[2]]],
                "WindTurbine_4": [[0, 0, 0, 0], [fans_x_coordinates[3], fans_y_coordinates[3]]], "WindTurbine_5": [[0, 0, 0, 0], [fans_x_coordinates[4], fans_y_coordinates[4]]],
                "WindTurbine_6": [[0, 0, 0, 0], [fans_x_coordinates[5], fans_y_coordinates[5]]], "WindTurbine_7": [[0, 0, 0, 0], [fans_x_coordinates[6], fans_y_coordinates[6]]]}
    st = datetime.now().strftime('%B %d,%Y %H.%M.%S')
    fileName = "CSV Files/Data Log " + st + ".csv"
    # Uncomment to get positions of each target in inches
    # with open('Positions.csv', 'w') as outFile: 
    #     for item in turbines:
    #         positionx = turbines[item][1][0]/2.54
    #         positiony = turbines[item][1][1]/2.54
    #         outFile.write(f"{item}: ({positionx}, {positiony})\n")
    path = TravelingSalesman() 
    # with open('OutputLog.csv', 'a') as outFile:
    #     outFile.write(f"Annealing finished at {round(time.time()-start)}\n")
    path.plot()
    fileFlag = 0
    drone = mov.movement()
    start_time = time.time()
    coordinates = path.get_path()
    # drone.append_current_path(coordinates)
    camera = drone.get_drone()
    Headings  = ['Location', 'Time to find Location (s)', 'Time to find Location (m.s)', 'Total time up to this point (s)', 'Total time up to this point (m.s)', 'Current Battery Life (%)']
    csvFile = open(fileName, "w")
    csvwriter = csv.writer(csvFile, lineterminator='\n')
    csvwriter.writerow(Headings)
    csvwriter.writerow(['Starting Helipad', 0, 0, 0, 0, str(camera.get_battery()) + ' %'])
    csvFile.close()
    flag_time = 1
    # with open('OutputLog.csv', 'a') as outFile:
    #     outFile.write(f"Starting battery: {camera.get_battery()}\n")
    coordinates = np.delete(coordinates, 0, axis=1)
    coordinates = np.delete(coordinates, -1, axis=1)
    finished = False
    
    # Angel - declare variable to be the distance we want to stop short in the x-axis
    x_distance_cutoff = 50

    while finished == False:
        index = -1
        test = 0
        for location in range(int(coordinates.size/2)):
            index += 1
            if camera.get_battery() < 20:
                print("\n" + ">>>>>>>>>>>>>>>> DRONE BATTERY LOW. CHANGE BATTERY!")
                xpos = coordinates[0][index:]
                ypos = coordinates[1][index:]
                current_x = drone.get_x_location()
                current_y = drone.get_y_location()
                quadrant = 0
                if 1000 - current_x > 500:
                    if 550 - current_y > 325:
                        drone.go_to(0, 0, 0)
                    else:
                        drone.go_to(0, 550, 0)
                        quadrant = 2
                else:
                    if 550 - current_y > 325:
                        drone.go_to(1000, 0, 0)
                        quadrant = 4
                    else:
                        drone.go_to(1000, 550, 0)
                        quadrant = 1
                calibrate(drone, fileName, start, st, fileFlag = 0, land=False)
                drone.land(True)
                try:
                    input("DRONE BATTERY LOW. CHANGE BATTERY, RECONNECT, THEN PRESS ENTER.")
                except:
                    input("DRONE BATTERY LOW. CHANGE BATTERY, RECONNECT, THEN PRESS ENTER.")
                drone = mov.movement()
                if quadrant == 1:
                    drone.set_coordinates(1000, 550)
                elif quadrant == 2:
                    drone.set_coordinates(0, 550)
                elif quadrant == 4:
                    drone.set_coordinates(1000, 550)
                path = TravelingSalesman() 
                path.plot()
                coordinates = path.get_path()
                camera = drone.get_drone()
                break
            test += 1
            if coordinates[0][location] == 0 or coordinates[0][location] == 1000: # second number to be changed to whatever the boundary size is
                fileFlag = 1
                calibrate(drone, fileName, start, st, fileFlag, False, coordinates[0][location], coordinates[1][location])
            else:
                print("not Calibrate")
                target_turbine = None
                for name in turbines:
                    if turbines[name][1][0] == coordinates[0][location] and turbines[name][1][1] == coordinates[1][location]:
                        target_turbine = name
                # List the target turbine number
                print(">>>>>>>>>>>>>>>> TARGET TURBINE: " + str(target_turbine) + "\n")
      
                # Rotate the drone to face the next location
                drone.go_to(coordinates[0][location], coordinates[1][location], rotate_only=True) 
                
                # Angel's edit - Move the drone towards the fan, stop short on x-axis and face forward to avoid detecting other fans
                drone.go_to(coordinates[0][location] - 300, coordinates[1][location], 0)#, half_travel=True)

                # Take 10 images to find the location of the target and do the mission if it is found
                info = check_camera(camera)      
                found = trackObject(drone, info, turbines, [drone.get_x_location(), drone.get_y_location(), drone.get_angle()], fileName, start, flag_time, st, fileFlag, target_turbine)
                # If it is not seen move towards the target
                img_counter = 0
                while found == False:
                    dist = math.sqrt((drone.get_x_location() - coordinates[0][location])**2 + (drone.get_y_location() - coordinates[1][location])**2)
                    # Angel's edit of + x_distance_cutoff
                    if dist > 35 + x_distance_cutoff:
                        # Angel's edit of - x_distance_cutoff
                        drone.go_to(coordinates[0][location] - x_distance_cutoff, coordinates[1][location], half_travel=True)
                        found = trackObject(drone, info, turbines, [drone.get_x_location(), drone.get_y_location(), drone.get_angle()], fileName, start, flag_time, st, fileFlag, target_turbine)
                        flag_time = 0
                    else:
                        qr_detection(drone, turbines, [drone.get_x_location(), drone.get_y_location(), drone.get_z_location()], fileName, start, flag_time, st, fileFlag, target_turbine)
                        flag_time = 0
                        found = True
                if coordinates[0][location] == coordinates[0][-1] and coordinates[1][location] == coordinates[1][-1]:
                    fileFlag = 1
                    calibrate(drone, fileName, start, st, fileFlag, False, coordinates[0][location], coordinates[1][location])
                    finished = True



    # drone.go_to(ending_angle=0)
    # print(">>>>>>>>>>>>>>>> TOTAL FLIGHT TIME: ", time.time() - start_time)
    # # with open('OutputLog.csv', 'a') as outFile:
    # #             outFile.write(f"Ended at: {round(time.time()-start)}\n")
    # #             outFile.write(f"Ending battery: {camera.get_battery()}\n")
    # calibrate(drone, fileName, start, st, fileFlag = 1, land=True)
    # drone.land(turn_off = True)


    