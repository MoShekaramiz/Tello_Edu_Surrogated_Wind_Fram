'''The movment module that facilitates drone coordinate tracking and return path algorithms. By Branden Pinney and Shayne Duncan 2022'''

from ast import operator
from cmath import atan
from djitellopy import Tello
import operator
import math
from time import sleep

def takeoff(drone):
    '''Launches the tello drone and returns a variable pointing to the drone'''
    drone = Tello()
    drone.connect()
    sleep(0.5)
    drone.takeoff()
    sleep(0.5)
    return drone

def move(new_location, drone, fwd = 0, back = 0, ccw = 0, cw = 0, up = 0, down = 0):
    '''Takes a list holding the x, y cartesian coordinates of the drone and the angle relative to takeoff [x, y, angle] (initialized as [0, 0, 0]).
    The variable for the drone being used is also required.
    Then commands for fwd and back will be taken that can range from 20-500, and ccw and cw from 0-359.
    The function will then rotate the drone first with counter-clockwise (ccw) being the priority, then move with forward (fwd) being the priority.
    The function will return a list with length 3 of updated coordinates and the drone angle as [x, y, angle]'''
    sleep(0.3)
    if up != 0:
        drone.move_up(up)
        new_location[3] += up

    elif up == 0 and down != 0:
        if new_location[3] > down:
            drone.move_down(down)
            new_location[3] -= down

    if ccw != 0:
        drone.rotate_counter_clockwise(ccw)
        sleep(0.5)
        # Returned angle if ccw
        new_location[2] = (new_location[2] + ccw) % 360 

    elif ccw == 0 and cw != 0:
        drone.rotate_clockwise(cw)
        sleep(0.5)

        if cw > new_location[2]:
            # Returned angle if cw angle is greater than the current angle
            new_location[2] = 360 - abs((new_location[2] - cw)) 

        else:
            # Returned angle if cw angle is less than the current angle
            new_location[2] = abs((new_location[2] - cw)) 

    if fwd != 0:
        # forward takes priority -- returns x and y coordinates after movement
        drone.move_forward(int(fwd))
        new_location[0] += round(fwd * math.cos(math.radians(new_location[2])))
        new_location[1] += round(fwd * math.sin(math.radians(new_location[2])))
        sleep(0.5)

    elif fwd == 0 and back != 0:
        # returns x and y coordinates after backwards movement
        drone.move_back(back)
        new_location[0] += round(-(back) * math.cos(math.radians(new_location[2])))
        new_location[1] += round(-(back) * math.sin(math.radians(new_location[2])))
        sleep(0.5)

    return new_location

def return_path(new_location, drone, turbine_locations):
    '''Takes a list with length 3 of the current x, y coordinates and angle relative to takeoff of the drone [x, y, angle].
    The variable for the drone being used is also required.
    This function will first angle the drone back to zero degrees, rotate towards the origin, then move forward the calculated distance'''
    x = new_location[0]
    y = new_location[1]
    angle = new_location[2]
    if(0 < x) and (0 < y): # drone in quadrant 1
        quadrant = 1

    elif (x < 0) and (0 < y): # drone in quadrant 2
        quadrant = 2

    elif (x < 0) and (y < 0): # drone in quadrant 3
        quadrant = 3

    elif (x > 0) and (y < 0): # drone in quadrant 4
        quadrant = 4

    else: # drone on a cartesian axis point
        quadrant = 0

    # calculate return vector distance and break it up into pieces if over 500 centimeters or land if under 20 centimeters.
    return_distance = int(round(math.sqrt(x**2 + y**2)))
    vector_angle = int(round(math.degrees(math.atan(y/x))))

    ##################### collision avoidance section #####################
    possible_collisions = []

    for i in turbine_locations:
        centerx = i[5]
        centery = i[6]
        center_distance = int(math.sqrt((x-centerx)**2 + (y-centery)**2))

        if quadrant == 1: # drone in quadrant 1
            if (0 < centerx < x) and (0 < centery < y):
                possible_collisions.append([i[0], i[1], i[2], i[3], center_distance, centerx, centery])

        elif quadrant == 2: # drone in quadrant 2
            if (x < centerx < 0) and (0 < centery < y):
                possible_collisions.append([i[0], i[1], i[2], i[3], center_distance, centerx, centery])

        elif quadrant == 3: # drone in quadrant 3
            if (0 < centerx < x) and (y < centery < 0):
                possible_collisions.append([i[0], i[1], i[2], i[3], center_distance, centerx, centery])

        else: # drone in quadrant 4
            if (x < centerx < 0) and (y < centery < 0):
                possible_collisions.append([i[0], i[1], i[2], i[3], center_distance, centerx, centery])
    
    if len(possible_collisions) != 0:
        possible_collisions = sorted(possible_collisions, key=operator.itemgetter(4)) # sort the possible collision list by distance from drone
        starting_point = new_location
        for i in range(return_distance): 
                targetx = i * math.cos(vector_angle)
                targety = i * math.sin(vector_angle)

                for j in possible_collisions:
                    centerx = j[5]
                    centery = j[6] 
                    if(j[0] < targetx < j[1]) and (j[2] < targety < j[3]): # if the drone will pass near the turbine
                        if(quadrant == 1) or (quadrant == 3): # if in quadrant 1 or 3
                            right_corner = [j[1], j[2]] # bottom right corner of the no-go zone
                            left_corner = [j[0], j[3]] # top left corner of the no-go zone

                        else: # if in quadrant 2 or 4
                            right_corner = [j[1], j[3]] # top right corner of the no-go zone
                            left_corner =[j[0], j[2]] # bottom left corner of the no-go zone

                        right_distance = math.sqrt((right_corner[0] - new_location[0])**2 + (right_corner[1] - new_location[1])**2) # point-distance to the bottom right corner
                        left_distance = math.sqrt((left_corner[0] - new_location[0])**2 + (left_corner[1] - new_location[1])**2) # point-distance to the bottom left corner

                        if right_distance < left_distance:
                            try:
                                return_angle = abs(math.degrees(math.atan((x-right_corner[0])/(y-right_corner[1]))))
                            except ZeroDivisionError:
                                pass
                            new_location = target_angle(new_location, drone, return_angle, right_corner[0], right_corner[1], quadrant)
                            new_location = move(new_location, drone, fwd=right_distance)

                        else:
                            try:
                                return_angle = abs(math.degrees(math.atan((x-left_corner[0])/(y-left_corner[1]))))
                            except ZeroDivisionError:
                                pass
                            new_location = target_angle(new_location, drone, return_angle, left_corner[0], left_corner[1], quadrant)
                            new_location = move(new_location, drone, fwd=left_distance)

                        return_path(new_location, drone, turbine_locations) # call return path again until there are no possible collisions remaining
                        ######### Work on if the no-go zone overlaps an axis
        if new_location == starting_point:
            straight_path(new_location, drone)
    else:
        straight_path(new_location, drone)

def straight_path(new_location, drone):# No possible collisions were detected
    '''Takes the location list and drone and flies directly back to the origin making the smallest rotation possible'''
    # calculate the return angle
    x = new_location[0]
    y = new_location[1]
    return_distance = int(math.sqrt(x**2 + y**2))
    try:
        return_angle = abs(math.degrees(math.atan(x/y)))
    except ZeroDivisionError:
        return_angle = 180

    # cases for rotation based on current cartesian quadrant of the drone
    new_location = target_angle(new_location, drone, return_angle, 0, 0)
    # Travel the full return distance    

    while (return_distance != 0):
        if (return_distance >= 500):
            drone.move_forward(500)
            sleep(0.5)
            return_distance -= 500

        elif (return_distance < 500) and (return_distance < 20):
            return_distance = 0

        else:
            drone.move_forward(return_distance)
            sleep(0.5)
            return_distance = 0

    drone.land()
    drone.streamoff()
    quit()

def target_angle(new_location, drone, return_angle, x, y, quadrant=0):
    '''This function helps the drone move the shortest distance to the correct angle
    by taking the intended angle and the x and y coordinates'''
    dronex = new_location[0]
    droney = new_location[1]
    angle = new_location[2]

    #          /|relative angle
    #         / |
    #        /  |
    #       /   |
    #      /    |
    #     /     |
    #    /      |
    #   /       |
    #  /        |
    # /)return angle
    #___________
    # cases for rotation based on current cartesian quadrant of the drone 
    
    if quadrant == 1:# quadrant 1
        relative_angle = 90 - return_angle
        if angle < return_angle:
            new_location = move(new_location, drone, cw=int(angle + relative_angle + 90))
        elif return_angle <= angle < 180 + return_angle:
            new_location = move(new_location, drone, ccw=int(180 + return_angle - angle))
        elif 180 + return_angle <= angle:
            new_location = move(new_location, drone, cw=int(angle - 180 + return_angle))

    elif quadrant == 2:# quadrant 2
        relative_angle = 90 + return_angle
        if angle < 90 + relative_angle:
            new_location = move(new_location, drone, cw=int(angle + abs(return_angle)))
        elif 90 + relative_angle <= angle <= 270 + relative_angle:
            new_location = move(new_location, drone, ccw=int(270 - angle + relative_angle))
        elif 270 + relative_angle < angle:
            new_location = move(new_location, drone, cw=int(angle - 270 - relative_angle))

    elif quadrant == 3:# quadrant 3
        relative_angle = 90 - return_angle
        if angle < return_angle:
            new_location = move(new_location, drone, ccw=int(return_angle - angle))
        elif relative_angle <= angle < return_angle + 180:
            new_location = move(new_location, drone, cw=int(angle - return_angle))
        elif 180 + return_angle <= angle:
            new_location = move(new_location, drone, ccw=int(360 - angle + return_angle))

    elif quadrant == 4:# quadrant 4
        relative_angle = 90 + return_angle
        if angle < 90 + relative_angle:
            new_location = move(new_location, drone, ccw=int(90 - angle + relative_angle))
        elif 90 + relative_angle < angle < 270 + relative_angle:
            new_location = move(new_location, drone, cw=int(angle - relative_angle - 90))
        elif 270 + relative_angle < angle:
            new_location = move(new_location, drone, ccw=int(450 - angle + relative_angle))

    elif x == dronex and y > droney:# positive Y axis
        if 0 <= angle < 90:
            new_location = move(new_location, drone, ccw=int(90 - angle))
        elif 90 <= angle < 180:
            new_location = move(new_location, drone, cw=int(angle - 90))
        elif 180 <= angle < 270:
            new_location = move(new_location, drone, cw=int(90 + angle))
        elif 270 <= angle < 360:
            new_location = move(new_location, drone, cw=int(90 + (360 - angle)))

    elif x == dronex and y < droney:# negative Y axis
        if 90 <= angle < 270:
            new_location = move(new_location, drone, ccw=int(270 - angle))
        elif 0 <= angle < 90 or 270 <= angle < 360:
            new_location = move(new_location, drone, cw=int(270 - angle))

    elif x < dronex and y == droney:# positive X axis
        if 0 <= angle <= 180:
            new_location = move(new_location, drone, ccw=int(180 - angle))
        elif 180 < angle < 360:
            new_location = move(new_location, drone, cw=int(360 - angle))

    elif x > dronex and y == droney:# negative X axis
        if 0 <= angle <= 180:
            new_location = move(new_location, drone, cw=int(angle))
        elif 180 < angle <= 360:
            new_location = move(new_location, drone, ccw=int(360 - angle))

    return new_location
########################################################################

def go_to(new_location, drone, turbine_locations, target_x=0, target_y=0, ending_angle=None):
    '''Used to tell the drone to go to a specific cartesian coordinate with a target X and Y value,
    along with the desired ending angle.'''
    x = new_location[0]
    y = new_location[1]
    angle = new_location[2]

    if(x > target_x) and (y > target_y): # drone in quadrant 1
        quadrant = 1

    elif (x < target_x) and (y > target_y): # drone in quadrant 2
        quadrant = 2

    elif (x < target_x) and (y < target_y): # drone in quadrant 3
        quadrant = 3

    elif (x > target_x) and (y < target_y): # drone in quadrant 4
        quadrant = 4

    elif (x == target_x) or (y == target_y): # drone on a cartesian axis
        quadrant = 0

    point_distance = int(round(math.sqrt((x-target_x)**2 + (y-target_y)**2)))
    try:
        vector_angle = int(round(math.degrees(math.atan((y-target_y)/(x-target_x)))))
    except ZeroDivisionError:
        vector_angle = 0
    possible_collisions = []

    for i in turbine_locations:
        centerx = i[5]
        centery = i[6]
        center_distance = int(math.sqrt((x-centerx)**2 + (y-centery)**2))

        if quadrant == 1: # drone in quadrant 1
            if (target_x < centerx < x) and (target_y < centery < y):
                possible_collisions.append([i[0], i[1], i[2], i[3], center_distance, centerx, centery])

        elif quadrant == 2: # drone in quadrant 2
            if (x < centerx < target_x) and (target_y < centery < y):
                possible_collisions.append([i[0], i[1], i[2], i[3], center_distance, centerx, centery])

        elif quadrant == 3: # drone in quadrant 3
            if (target_x < centerx < x) and (y < centery < target_y):
                possible_collisions.append([i[0], i[1], i[2], i[3], center_distance, centerx, centery])

        else: # drone in quadrant 4
            if (x < centerx < target_x) and (y < centery < target_y):
                possible_collisions.append([i[0], i[1], i[2], i[3], center_distance, centerx, centery])
    
    if len(possible_collisions) != 0:
        possible_collisions = sorted(possible_collisions, key=operator.itemgetter(4)) # sort the possible collision list by distance from drone
        starting_point = new_location
        for i in range(point_distance): 
                targetx = i * math.cos(vector_angle)
                targety = i * math.sin(vector_angle)

                for j in possible_collisions:
                    centerx = j[5]
                    centery = j[6] 
                    if(j[0] < targetx < j[1]) and (j[2] < targety < j[3]): # if the drone will pass near the turbine
                        if(quadrant == 1) or (quadrant == 3): # if in quadrant 1 or 3
                            right_corner = [j[1], j[2]] # bottom right corner of the no-go zone
                            left_corner = [j[0], j[3]] # top left corner of the no-go zone

                        else: # if in quadrant 2 or 4
                            right_corner = [j[1], j[3]] # top right corner of the no-go zone
                            left_corner =[j[0], j[2]] # bottom left corner of the no-go zone

                        right_distance = math.sqrt((right_corner[0] - new_location[0])**2 + (right_corner[1] - new_location[1])**2) # point-distance to the bottom right corner
                        left_distance = math.sqrt((left_corner[0] - new_location[0])**2 + (left_corner[1] - new_location[1])**2) # point-distance to the bottom left corner

                        if right_distance < left_distance:
                            try:
                                return_angle = abs(math.degrees(math.atan((x-right_corner[0])/(y-right_corner[1]))))
                            except ZeroDivisionError:
                                pass
                            new_location = target_angle(new_location, drone, return_angle, right_corner[0], right_corner[1], quadrant)
                            new_location = move(new_location, drone, fwd=right_distance)

                        else:
                            try:
                                return_angle = abs(math.degrees(math.atan((x-left_corner[0])/(y-left_corner[1]))))
                            except ZeroDivisionError:
                                pass
                            new_location = target_angle(new_location, drone, return_angle, left_corner[0], left_corner[1], quadrant)
                            new_location = move(new_location, drone, fwd=left_distance)

                        new_location = go_to(new_location, drone, turbine_locations, target_x, target_y, ending_angle) # call return path again until there are no possible collisions remaining
                        ######### Work on if the no-go zone overlaps an axis
        if new_location == starting_point:
            new_location = target_angle(new_location, drone, vector_angle, target_x, target_y, quadrant)
            while (point_distance != 0):
                if (point_distance >= 500):
                    new_location = move(new_location, drone, fwd=500)
                    point_distance -= 500

                elif (point_distance < 500) and (point_distance < 20):
                    point_distance = 0

                else:
                    new_location = move(new_location, drone, fwd=point_distance)
                    sleep(0.5)
                    point_distance = 0
            if ending_angle is not None: # rotate the shortest distance to the ending angle
                angle = new_location[2] # current angle has to be updated
                if ending_angle < 0:
                    ending_angle = abs(360 + angle) % 360
                else:
                    ending_angle = ending_angle % 360
                alpha = ending_angle - angle
                if alpha < 0:
                    alpha = round(abs(alpha))
                    if alpha < 180:
                        new_location = move(new_location, drone, cw=(alpha))
                    else:
                        new_location = move(new_location, drone, ccw=(360 - alpha))
                elif alpha < 180:
                    new_location = move(new_location, drone, cw=(round(alpha)))
                else:
                    new_location = move(new_location, drone, ccw=(round(360 - alpha)))
    else:
        new_location = target_angle(new_location, drone, vector_angle, target_x, target_y, quadrant)
        while (point_distance != 0):
            if (point_distance >= 500):
                new_location = move(new_location, drone, fwd=500)
                point_distance -= 500

            elif (point_distance < 500) and (point_distance < 20):
                point_distance = 0

            else:
                new_location = move(new_location, drone, fwd=point_distance)
                sleep(0.5)
                point_distance = 0
        if ending_angle is not None: # rotate the shortest distance to the ending angle
            angle = new_location[2] # current angle has to be updated
            if ending_angle < 0:
                ending_angle = abs(360 + angle) % 360
            else:
                ending_angle = ending_angle % 360
            alpha = ending_angle - angle
            if alpha < 0:
                alpha = round(abs(alpha))
                if alpha < 180:
                    new_location = move(new_location, drone, cw=(alpha))
                else:
                    new_location = move(new_location, drone, ccw=(360 - alpha))
            elif alpha < 180:
                new_location = move(new_location, drone, cw=(round(alpha)))
            else:
                new_location = move(new_location, drone, ccw=(round(360 - alpha)))
    print(f"\nCURRENT LOCATION >>>>>>>>>>{new_location}\n")
    return new_location

if __name__ == "__main__":
    FB_RANGE = [62000, 82000]#[32000, 52000] # preset parameter for detected image boundary size
    W, H = 720, 480 # display size of the screen
    LOCATION = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
    TURBINE_LOCATIONS = [] # List containing the LOCATIONs of found turbines
    DETECTED_OBJECT = 0 # A flag to determine if the camera detected an object in the previous 5 frames
    turbines = {"WindTurbine_2": [1, 0, 0, 0]} # Target: [front, right, back, left]
    drone = Tello()
    drone.connect()
    sleep(0.5)
    print("Current battery remaining: ", drone.get_battery())
    sleep(0.3)
    drone.streamon()
    sleep(0.5)
    drone.takeoff()
    sleep(1.5)
    move(LOCATION, drone, up=40)
    LOCATION = go_to(LOCATION, drone, TURBINE_LOCATIONS, 60, 90)
    LOCATION = go_to(LOCATION, drone, TURBINE_LOCATIONS, 72, 101)
    LOCATION = go_to(LOCATION, drone, TURBINE_LOCATIONS, 58, 103)
    LOCATION = go_to(LOCATION, drone, TURBINE_LOCATIONS, 0, 0, 0)
    drone.land()