'''A flight simulator using a psuedo drone class. This can be used to test certain flight algorithms so that a drone does not need to be flown for testing
By Angel Rodriguez 2023'''

import matplotlib.pyplot as plt
import numpy as np

class pseudo_drone:
    def __init__(self):
        self.x_locations = [0]
        self.y_locations = [0]

    def move(self, fwd=0, back=0, left=0, right=0):
        if fwd:
            self.x_locations.append(self.get_x_location() + fwd)
            self.y_locations.append(self.get_y_location())
        elif back:
            self.x_locations.append(self.get_x_location() - back)
            self.y_locations.append(self.get_y_location())
        elif left:
            self.y_locations.append(self.get_y_location() + left)
            self.x_locations.append(self.get_x_location())
        elif right:
            self.y_locations.append(self.get_y_location() - right)
            self.x_locations.append(self.get_x_location())


    def get_x_location(self):
        return self.x_locations[-1]
    
    def get_y_location(self):
        return self.y_locations[-1]
    
    def get_x_path(self):
        return self.x_locations
    
    def get_y_path(self):
        return self.y_locations
    
    def go_to(self, x, y):
        self.x_locations.append(x)
        self.y_locations.append(y)

def convert_to_cm(array):
    '''Convert from in to cm'''
    new_array = []
    for element in array:
        new_array.append(2.54 * float(element))
    return new_array

x_turbine_coordinates = [90, 180, 270, 360, 360, 270, 180, 90, 90, 180, 270, 360]
y_turbine_coordinates = [0, 0, 0, 0, 90, 90, 90, 90, 180, 180, 180, 180]
x_turbine_coordinates = convert_to_cm(x_turbine_coordinates)
# I think the helipad was pushed back 100 cm or so
x_turbine_coordinates = [x + 100 for x in x_turbine_coordinates]
y_turbine_coordinates = convert_to_cm(y_turbine_coordinates)

turbine_locations = []
for x,y in zip(x_turbine_coordinates, y_turbine_coordinates):
    turbine_locations.append([x, y])

x_step = 120 # Steps the drone takes forwards and backwards
y_step = 90*2.54 # Steps the drone takes left (90" in cm to match fan spacing)
x_boundary = 1000 # x-axis boundary of experiment
y_boundary = 500 # y-axis boundary of experiment
turbine_quantity = 1 # Quantity of fans in the mission

drone = pseudo_drone()

while drone.get_y_location() <= y_boundary: # The snake path progressively moves towards the y_boundary which should be the end of its mission
    while drone.get_x_location() + x_step <= x_boundary: # Keep moving forward until the x_boundary is reached
        drone.move(fwd=x_step)
    if drone.get_y_location() + y_step <= y_boundary: # We should pass into here every time except when we are on the y_boundary
        # Continue to the next row of the snake path search algorithm
        drone.move(left=y_step)
    else: # When here, we should be done with the mission, so exit the while loop
        break
    while drone.get_x_location() -x_step >= 0: # Move all the way back to x=0
        drone.move(back=x_step)
    if drone.get_y_location() + y_step <= y_boundary: # We should pass into here every time except when we are on the y_boundary
        # Continue to the next row of the snake path search algorithm
        drone.move(left=y_step)
    else: # When here, we should be done with the mission, so exit the while loop
        break

# Go to helipad
drone.go_to(0, 0)
# Get all drone traveling coordinates into a single array as [[], []]
data = np.array([drone.get_x_path(), drone.get_y_path()])

plt.rcParams["font.family"] = "Times New Roman"
# Boundaries for the plot
boundaries = [-100, max(data[0])+100, -100, max(data[1])+100]
plt.axis(boundaries)
plt.title(f'Snake Path Flight with Possible Fan Locations', fontsize=30)
plt.xlabel('x (cm)', fontsize=30)
plt.ylabel('y (cm)', fontsize=30)

# Add text for simulation parameters
# plt.text(0, boundaries[-1]-50, f'x boundary: {x_boundary}', fontsize='medium', weight='bold')
# plt.text(0, boundaries[-1]-60, f'y boundary: {y_boundary}', fontsize='medium', weight='bold')
# plt.text(0, boundaries[-1]-70, f'x step: {x_step}', fontsize='medium', weight='bold')
# plt.text(0, boundaries[-1]-80, f'y step: {y_step}', fontsize='medium', weight='bold')

# Label possible location numbers for the turbines
counter = 1
for i, j in zip(x_turbine_coordinates, y_turbine_coordinates): # Writes the (x,y) coordinates above the coordinate location
    plt.text(i-50, j+20, f'Location #{counter}', size=20, weight='bold')
    counter += 1

# Plot Helipad
# plt.text(-40, -30, 'Helipad Location', size=15, weight='bold')

# Plot arrows showing the direction of travel
plt.quiver(data[0][:-1], data[1][:-1], data[0][1:]-data[0][:-1], 
            data[1][1:]-data[1][:-1],scale_units='xy', angles='xy', scale=1, color='teal', width=0.005)
# Plot all possible turbine locations even if they are not included in the experiment
plt.plot(x_turbine_coordinates, y_turbine_coordinates, 'o', markersize=20) 

plt.axis('equal') # Set the scale of the x and y axes to be equivalent
plt.grid(color = 'green', linestyle = '--', linewidth = 0.5) # Plot grid lines
# You can play with the configure subplots button on bottom of graph for desired look, but changes are not remembered
# Below, change the values to values desired on the subplots 
# Angel's values that work with his computer screen
# plt.subplots_adjust(left=0.07, bottom=0.26, right=0.7, top=0.93, wspace=0.2, hspace=0.4)

# manager = plt.get_current_fig_manager()
# manager.full_screen_toggle() # Make full screen for better view, Alt-F4 to exit full screen
plt.show()
