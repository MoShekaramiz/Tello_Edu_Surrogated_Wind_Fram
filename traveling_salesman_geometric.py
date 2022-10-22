import math
from turtle import goto
import numpy as np
import matplotlib.pyplot as plt
import random
from check_camera import check_camera
from salesman_image_interface import trackObject, qr_detection
from downvision_calibration import calibrate
import time
import sys
import movement as mov

start = time.time()

# Array of 8 test points
# xpos = np.array([0, 1000, 0, 1000, 360, 832, 217, 613, 801, 58, 531, 200])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, 0, 650, 650, 52, 409, 224, 550, 82, 125, 150, 560])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32)

# Array of 7 test points
# xpos = np.array([0, 1000, 0, 1000, 360, 832, 217, 613, 801, 58, 531])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, 0, 650, 650, 52, 409, 224, 550, 82, 125, 150])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32)

# Array of 6 test points
# xpos = np.array([0, 1000, 0, 1000, 360, 832, 217, 613, 801, 58])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, 0, 650, 650, 52, 409, 224, 550, 82, 125])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32)

# Array of 5 test points
# xpos = np.array([0, 1000, 0, 1000, 360, 832, 217, 613, 801])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, 0, 650, 650, 52, 409, 224, 550, 82])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32)

# Array of 4 test points
# xpos = np.array([0, 1000, 0, 1000, 360, 832, 217, 613])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, 0, 650, 650, 52, 409, 224, 550])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32)

# Array of 3 test points
# xpos = np.array([0, 1000, 0, 1000, 360, 832, 217])
# xpos = np.append(xpos, xpos[0])
# ypos = np.array([0, 0, 650, 650, 52, 409, 224])
# ypos = np.append(ypos, ypos[0])
# data = np.array([xpos, ypos], np.int32)

# Lab Test Points
xpos = np.array([0, 120, 50, 165, 100])
xpos = np.append(xpos, xpos[0])
ypos = np.array([0, 100, 28, 20, 100])
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

        T = 2000

        while T > 1:
            for i in range(400):
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
        axis[0].axis([0, 1000, 0, 650])
        axis[0].title.set_text("Path Before Optimization")
        axis[0].set_xlabel('x')
        axis[0].set_ylabel('y')
        # plt.axis([0, 285, 0, 285])
        for i, j in zip(data[0], data[1]):
            axis[0].text(i, j+20, '({}, {})'.format(i, j), fontsize='small')
        axis[0].text(10, 900, f'Total Energy: {int(self.energy_calc(data))}', fontsize='small')
        axis[0].plot(data[0], data[1], '-o', markersize=4)
        axis[0].quiver(data[0][:-1], data[1][:-1], data[0][1:]-data[0][:-1], 
                   data[1][1:]-data[1][:-1],scale_units='xy', angles='xy', scale=1, color='teal', width=0.005)

        axis[1].axis([0, 1000, 0, 650])
        axis[1].title.set_text("Path After Optimization")
        axis[1].set_xlabel('x')
        axis[1].set_ylabel('y')
        # plt.axis([0, 285, 0, 285])
        for i, j in zip(self.path_min[0], self.path_min[1]):
            axis[1].text(i, j+20, '({}, {})'.format(i, j), fontsize='small')
        axis[1].text(10, 900, f'Total Energy: {int(self.energy_calc(self.path_min))}', fontsize='small')
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
    turbines = {"WindTurbine_1": [[0, 0, 0, 0], [360, 52]], "WindTurbine_2": [[0, 0, 0, 0], [832, 409]], "WindTurbine_3": [[0, 0, 0, 0], [217, 224]],
                "WindTurbine_4": [[0, 0, 0, 0], [613, 550]], "WindTurbine_5": [[0, 0, 0, 0], [801, 82]], "WindTurbine_6": [[0, 0, 0, 0], [58, 125]],
                "WindTurbine_7": [[0, 0, 0, 0], [531, 150]], "WindTurbine_8": [[0, 0, 0, 0], [200, 560]]}
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
    drone = mov.movement()
    start_time = time.time()
    coordinates = path.get_path()
    camera = drone.get_drone()
    # with open('OutputLog.csv', 'a') as outFile:
    #     outFile.write(f"Starting battery: {camera.get_battery()}\n")
    coordinates = np.delete(coordinates, 0, axis=1)
    coordinates = np.delete(coordinates, -1, axis=1)
    finished = False
    while finished == False:
        index = -1
        test = 0
        for location in range(int(coordinates.size/2)):
            index += 1
            if camera.get_battery() < 20:
                xpos = coordinates[0][index:]
                ypos = coordinates[1][index:]
                current_x = drone.get_x_location()
                current_y = drone.get_y_location()
                quadrant = 0
                if 1000 - current_x > 500:
                    if 650 - current_y > 325:
                        drone.go_to(0, 0, 0)
                    else:
                        drone.go_to(0, 650, 0)
                        quadrant = 2
                else:
                    if 650 - current_y > 325:
                        drone.go_to(1000, 0, 0)
                        quadrant = 4
                    else:
                        drone.go_to(1000, 650, 0)
                        quadrant = 1
                calibrate(drone, land=False)
                drone.land(True)
                try:
                    input("DRONE BATTERY LOW. CHANGE BATTERY, RECONNECT, THEN PRESS ENTER.")
                except:
                    input("DRONE BATTERY LOW. CHANGE BATTERY, RECONNECT, THEN PRESS ENTER.")
                drone = mov.movement()
                if quadrant == 1:
                    drone.set_coordinates(1000, 650)
                elif quadrant == 2:
                    drone.set_coordinates(0, 650)
                elif quadrant == 4:
                    drone.set_coordinates(1000, 650)
                path = TravelingSalesman() 
                path.plot()
                coordinates = path.get_path()
                camera = drone.get_drone()
                break
            test += 1
            if coordinates[0][location] == 0 or coordinates[0][location] == 1000: # second number to be changed to whatever the boundary size is
                calibrate(drone, False, coordinates[0][location], coordinates[1][location])
            else:
                # Rotate the drone to face the next location
                drone.go_to(coordinates[0][location], coordinates[1][location], rotate_only=True) 
                drone.go_to(coordinates[0][location], coordinates[1][location])

                # Take 10 images to find the location of the target and do the mission if it is found
                info = check_camera(camera)      
                found = trackObject(drone, info, turbines, [drone.get_x_location(), drone.get_y_location(), drone.get_angle()])
                
                # If it is not seen move towards the target
                img_counter = 0
                while found == False:
                    dist = math.sqrt((drone.get_x_location() - coordinates[0][location])**2 + (drone.get_y_location() - coordinates[1][location])**2)
                    if dist > 35:
                        drone.go_to(coordinates[0][location], coordinates[1][location], half_travel=True)
                        found = trackObject(drone, info, turbines, [drone.get_x_location(), drone.get_y_location(), drone.get_angle()])
                    else:
                        qr_detection(drone, turbines, [drone.get_x_location(), drone.get_y_location(), drone.get_z_location()])
                        found = True
                if coordinates[0][location] == coordinates[0][-1] and coordinates[1][location] == coordinates[1][-1]:
                    finished = True

    drone.go_to(ending_angle=0)
    print(">>>>>>>>>>>>>>>> TOTAL FLIGHT TIME: ", time.time() - start_time)
    # with open('OutputLog.csv', 'a') as outFile:
    #             outFile.write(f"Ended at: {round(time.time()-start)}\n")
    #             outFile.write(f"Ending battery: {camera.get_battery()}\n")
    calibrate(drone, land=True)
    drone.land(turn_off = True)


    