import math
import numpy as np
import matplotlib.pyplot as plt
import random
from check_camera import check_camera
from salesman_image_interface import trackObject, qr_detection
from downvision_calibration import calibrate
import time
import sys
import csv
import movement as mov
# Angel's edit - import random
import random
from datetime import datetime
start = time.time()




# xfans = [360, 832, 217, 613, 58, 531, 188]
# yfans = [61, 409, 224, 460, 125, 150, 457]

# Array of 8 test points
# xpos = np.array([0, 360, 832, 217, 613, 58, 531, 188])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, 61, 409, 224, 460, 125, 150, 457])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32)

# Array of 7 test points
# xpos = np.array([0, 1000, 0, 1000, 360, 832, 217, 613, 801, 58, 531])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, 0, 550, 550, 61, 409, 224, 460, 99, 125, 150])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32)

# Array of 6 test points
# xpos = np.array([0, 1000, 0, 1000, 360, 832, 217, 613, 801, 58])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, 0, 550, 550, 61, 409, 224, 460, 99, 125])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32)

# Array of 5 test points
# xpos = np.array([0, 1000, 0, 1000, 360, 832, 217, 613, 801])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, 0, 550, 550, 61, 409, 224, 460, 99])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32)

# Array of 4 test points
# xpos = np.array([0, 360, 832, 217, 613])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, 61, 409, 224, 460])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32) 

# Lab Test Points
# distance check points
# xpos = np.array([0, 300, 0])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, -200, 0])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32)

# Testing landing pad Lab
# xpos = np.array([0, 320, 260, 150])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, 0, 220, 400])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32)

# Testing landing pad Atrium
# xpos = np.array([0, 447, 1000])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, 0, 0])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32)

# Testing landing pad Atrium Fan 3 
xpos = np.array([0, 217, 1000])
xpos = np.append(xpos, xpos[0])
ypos = np.array([0, -426, 0])
ypos = np.append(ypos, ypos[0])
data = np.array([xpos, ypos], np.int32)

# Testing three random fans
# xfans = [360, 832, 217, 613, 58, 531, 188]
# yfans = [61, 409, 224, 460, 125, 150, 457]
# list of fan numbers, we will choose a random value from the list
# list1 = [1, 2, 3, 4, 5, 6, 7]
# random_choice1 = random.choice(list1)
# list1.remove(random_choice1)
# random_choice2 = random.choice(list1)
# list1.remove(random_choice2)
# random_choice3 = random.choice(list1)
# Change this number below to determine which fan to test, if you want random fan, comment out the whole line below
# random_choice1 = 4
# random_choice2 = 5
# random_choice2 = 5
# print(">>>>>>>>>>>>>>>> CHOOSING FANS " + str(random_choice1) + ", " + str(random_choice2) + ", " + str(random_choice3))
# xpos = np.array([0, xfans[random_choice1-1], xfans[random_choice2-1], xfans[random_choice3-1],])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, yfans[random_choice1-1], yfans[random_choice2-1], yfans[random_choice3-1],])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32) 

# Testing two random fans
# xfans = [360, 832, 217, 613, 58, 531, 188]
# yfans = [61, 409, 224, 460, 125, 150, 457]
# # list of fan numbers, we will choose a random value from the list
# list1 = [1, 2, 3, 4, 5, 6, 7]
# random_choice1 = random.choice(list1)
# random_choice2 = random.choice(list1)
# while random_choice1 == random_choice2:
#     random_choice2 = random.choice(list1)
# # Change this number below to determine which fan to test, if you want random fan, comment out the whole line below
# random_choice1 = 4
# random_choice2 = 5
# print("Choosing fans " + str(random_choice1) + ", " + str(random_choice2))
# xpos = np.array([0, xfans[random_choice1-1], xfans[random_choice2-1]])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, yfans[random_choice1-1], yfans[random_choice2-1]])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32) 

# Testing single random fans
# xfans = [360, 832, 217, 613, 58, 531, 200]
# yfans = [61, 409, 224, 460, 125, 150, 440]
# # list of fan numbers, we will choose a random value from the list
# list1 = [1, 2, 3, 4, 5, 6, 7, 8]
# random_choice = random.choice(list1)
# # Change this number below to determine which fan to test, if you want random fan, comment out the whole line below
# random_choice = 5
# print("Choosing fan " + str(random_choice))
# xpos = np.array([0, xfans[random_choice-1], 0])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, yfans[random_choice-1], 0])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32) 


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
        # Angel
        # print(f">>>>>>>>>>>>>>>>>>>>> BATTERY: ",{camera.get_battery()})
        
    def plot(self):
        plt.rcParams["font.family"] = "Times New Roman"
        figure, axis = plt.subplots(2, 1)
        axis[0].axis([0, 1000, 0, 550])
        axis[0].title.set_text("Path Before Optimization")
        axis[0].set_xlabel('x')
        axis[0].set_ylabel('y')
        # plt.axis([0, 285, 0, 285])
        for i, j in zip(data[0], data[1]):
            axis[0].text(i, j+20, '({}, {})'.format(i, j), fontsize='small')
        axis[0].text(10, 500, f'Total Energy: {int(self.energy_calc(data))}', fontsize='small')
        axis[0].plot(data[0], data[1], '-o', markersize=4)
        axis[0].quiver(data[0][:-1], data[1][:-1], data[0][1:]-data[0][:-1], 
                   data[1][1:]-data[1][:-1],scale_units='xy', angles='xy', scale=1, color='teal', width=0.005)

        axis[1].axis([0, 1000, 0, 550])
        axis[1].title.set_text("Path After Optimization")
        axis[1].set_xlabel('x')
        axis[1].set_ylabel('y')
        # plt.axis([0, 285, 0, 285])
        for i, j in zip(self.path_min[0], self.path_min[1]):
            axis[1].text(i, j+20, '({}, {})'.format(i, j), fontsize='small')
        axis[1].text(10, 500, f'Total Energy: {int(self.energy_calc(self.path_min))}', fontsize='small')
        axis[1].plot(self.path_min[0], self.path_min[1], '-o', markersize=4)
        axis[1].quiver(self.path_min[0][:-1], self.path_min[1][:-1], self.path_min[0][1:]-self.path_min[0][:-1], 
                   self.path_min[1][1:]-self.path_min[1][:-1],scale_units='xy', angles='xy', scale=1, color='teal', width=0.005)
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
    turbines = {"WindTurbine_1": [[0, 0, 0, 0], [360, 61]], "WindTurbine_2": [[0, 0, 0, 0], [832, 409]], "WindTurbine_3": [[0, 0, 0, 0], [217, 224]],
                "WindTurbine_4": [[0, 0, 0, 0], [613, 460]], "WindTurbine_5": [[0, 0, 0, 0], [58, 125]],
                "WindTurbine_6": [[0, 0, 0, 0], [531, 150]], "WindTurbine_7": [[0, 0, 0, 0], [188, 457]]}
    st = datetime.now().strftime('%B %d,%Y %H.%M.%S')
    fileName = "CSV Files\Data Log " + st + ".csv"
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
                target_turbine = None
                for name in turbines:
                    if turbines[name][1][0] == coordinates[0][location] and turbines[name][1][1] == coordinates[1][location]:
                        target_turbine = name
                # List the target turbine number
                print(">>>>>>>>>>>>>>>> TARGET TURBINE " + str(target_turbine) + "\n")
      
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
                    finished = True

    # drone.go_to(ending_angle=0)
    # print(">>>>>>>>>>>>>>>> TOTAL FLIGHT TIME: ", time.time() - start_time)
    # # with open('OutputLog.csv', 'a') as outFile:
    # #             outFile.write(f"Ended at: {round(time.time()-start)}\n")
    # #             outFile.write(f"Ending battery: {camera.get_battery()}\n")
    # calibrate(drone, fileName, start, st, fileFlag = 1, land=True)
    # drone.land(turn_off = True)


    