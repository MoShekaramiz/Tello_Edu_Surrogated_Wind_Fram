'''Test the TravelingSalesman class by Angel Rodriguez 2023'''

import math
import numpy as np
import matplotlib.pyplot as plt
import random

''' {number_of_fans} Random points within a {boundary_length} x {boundary_length} area'''
number_of_fans = 20 # Override number of fans
boundary_length = 1000
fans_list = [1,2,3,4,5,6,7,8,9,10] # Override fans_list
# Override path coordinates
path_x_coordinates = random.sample(list(range(int(boundary_length))), number_of_fans) # Random list of integers up to 1000
path_y_coordinates = random.sample(list(range(int(boundary_length))), number_of_fans) # Random list of integers up to 1000
# Override coordinates of fans
fans_x_coordinates = path_x_coordinates.copy()
fans_y_coordinates = path_y_coordinates.copy()

path_x_coordinates.insert(0,0)
path_y_coordinates.insert(0,0)
boundaries = [-100, 1100, -100, 1100] # Override plot boundaries

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
        energy_list = []
        path1 = np.copy(data)
        e1 = self.energy_calc(data)
        e_min = e1
        self.path_min = data
        print(data)
        print(f">>>>>>>>>>>>>>>>>>>>>> STARTING PATH ENERGY: {e1}\n")

        array_size = xpos.size
        T = 150 * array_size
        counter = 0
        while T > 1:

            for i in range(30 * array_size):
                if e1 < e_min:
                    e_min = e1
                    self.path_min = np.copy(path1)
                rand_index = random.randint(1, xpos.size-3)
                path2 = np.copy(path1)
                path2[0, rand_index] = path1[0, rand_index+1]
                path2[1, rand_index] = path1[1, rand_index+1]
                path2[0, rand_index+1] = path1[0, rand_index]
                path2[1, rand_index+1] = path1[1, rand_index]
                e2 = self.energy_calc(path2)

                if e2 < e1:
                    e1 = e2
                    path1 = np.copy(path2)
                    energy_list.append(e2)
                else:
                    c1 = math.exp(-(e2 - e1)/T)
                    c2 = random.random()

                    if c1 > c2:
                        path1 = np.copy(path2)
                        e1 = e2

            T = 0.99 * T
            counter += 1
            if counter % 1000 == 0:
                print(f'Temperature: {T}, Energy: {self.energy_calc(self.path_min)}, Iteration: {counter}')
            if counter >= 4001:
              break
        print(path1)
        print(f">>>>>>>>>>>>>>>>>>>>> OPTIMIZED PATH ENERGY: {self.energy_calc(self.path_min)}\n")
        print(f">>>>>>>>>>>>>>>>>>>>> MINIMUM PATH ENERGY: {min(energy_list)}\n")

    def plot(self):
        plt.rcParams["font.family"] = "Times New Roman"
        figure, axis = plt.subplots(1, 2)
        axis[0].axis(boundaries)
        axis[0].set_title(f'Path Before Optimization: {number_of_fans} Targets', fontsize=30)
        axis[0].set_xlabel('x (cm)', fontsize=16)
        axis[0].set_ylabel('y (cm)', fontsize=16)
        # for i, j in zip(fans_x_coordinates, fans_y_coordinates):  # We will plot and label all the turbines, even if they are not targeted
            # Writes the (x,y) coordinates and turbine number above the coordinate location
            # axis[0].text(i-80, j+20, '({}, {})'.format(i, j), weight="bold", size=13)
        axis[0].text(boundaries[0]+20, boundaries[-1]-60, f'Total Energy: {int(self.energy_calc(data))}', size=20, weight="bold")
        # Plot all the turbines, even if they are not targeted
        axis[0].plot(fans_x_coordinates, fans_y_coordinates, 'o', markersize=10)
        # Draw the arrows for the original unoptomized path (some turbines may not be a part of the path)
        axis[0].quiver(data[0][:-1], data[1][:-1], data[0][1:]-data[0][:-1],
                   data[1][1:]-data[1][:-1],scale_units='xy', angles='xy', scale=1, color='teal', width=0.010)
        axis[0].set_aspect('equal', adjustable='box') # Set the scale of the x and y axes to be equivalent
        axis[0].grid(color = 'green', linestyle = '--', linewidth = 0.5) # Plot grid lines

        axis[1].axis(boundaries)
        axis[1].set_title(f'Path After Optimization: {number_of_fans} Targets', fontsize=30)
        axis[1].set_xlabel('x (cm)', fontsize=16)
        axis[1].set_ylabel('y (cm)', fontsize=16)
        # for i, j in zip(fans_x_coordinates, fans_y_coordinates): # Label all the turbines, even if they are not targeted
        #     # Writes the (x,y) coordinates and turbine number above the coordinate location
        #     axis[1].text(i-80, j+20, '({}, {})'.format(i, j), weight="bold", size=13)
        axis[1].text(boundaries[0]+20, boundaries[-1]-60, f'Total Energy: {int(self.energy_calc(self.path_min))}', size=20, weight="bold")
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

    path = TravelingSalesman()
    coordinates = path.get_path()
    coordinates = np.delete(coordinates, 0, axis=1)
    coordinates = np.delete(coordinates, -1, axis=1)
    path.plot()



