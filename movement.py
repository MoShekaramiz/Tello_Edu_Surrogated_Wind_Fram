from djitellopy import Tello
import math
import random
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
        drone.move_forward(fwd)
        new_location[0] += fwd * math.cos(math.radians(new_location[2]))
        new_location[1] += fwd * math.sin(math.radians(new_location[2]))
        sleep(0.5)
    elif fwd == 0 and back != 0:
        # returns x and y coordinates after backwards movement
        drone.move_back(back)
        new_location[0] += -(back) * math.cos(math.radians(new_location[2]))
        new_location[1] += -(back) * math.sin(math.radians(new_location[2]))
        sleep(0.5)
    return new_location

def return_path(new_location, drone):
    '''Takes a list with length 3 of the current x, y coordinates and angle relative to takeoff of the drone [x, y, angle].
    The variable for the drone being used is also required.
    This function will first angle the drone back to zero degrees, rotate towards the origin, then move forward the calculated distance'''
    x = new_location[0]
    y = new_location[1]
    angle = new_location[2]
    # Rotate the drone to zero degrees
    if x > 180:
        drone.rotate_counter_clockwise(360 - angle)
        sleep(0.5)
    else:
        drone.rotate_clockwise(angle)
        sleep(0.5)
    # calculate the return angle
    try:
        return_angle = abs(math.degrees(math.atan(x/y)))
    except ZeroDivisionError:
        return_angle = 180
    # cases for rotation based on current cartesian quadrant of the drone
    if (x > 0) and (y > 0):
        drone.rotate_clockwise(int(90 + return_angle))
        sleep(0.5)
    elif (x > 0) and (y < 0):
        drone.rotate_counter_clockwise(int(90 + return_angle))
        sleep(0.5)
    elif (x < 0) and (y < 0):
        drone.rotate_counter_clockwise(int(90 - return_angle))
        sleep(0.5)
    elif (x < 0) and (y > 0):
        drone.rotate_clockwise(int(90 - return_angle))
        sleep(0.5)
    elif (x == 0) and (y > 0):
        drone.rotate_clockwise(90)
        sleep(0.5)
    elif (x == 0) and (y < 0):
        drone.rotate_counter_clockwise(90)
        sleep(0.5)
    elif (x > 0) and (y == 0):
        drone.rotate_clockwise(180)
        sleep(0.5)
    # calculate return vector distance and break it up into pieces if over 500 centimeters or land if under 20 centimeters.
    return_distance = int(math.sqrt(x**2 + y**2))
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

if __name__ == "__main__":
    # example of how to use the functions, sending random distances and angles to the drone before calling return.
    new_location = [0, 0, 0]
    drone = takeoff()
    for i in range(3):
        new_location = move(new_location, drone, fwd = random.randint(20, 100), cw = random.randint(0, 359))
        print("Fwd & cw step: ", i + 1)
    for i in range(3):
        new_location = move(new_location, drone, fwd = random.randint(20, 100), ccw = random.randint(0, 359))
        print("Fwd & ccw step: ", i + 1)
    for i in range(3):
        new_location = move(new_location, drone, back = random.randint(20, 100), cw = random.randint(0, 359))
        print("Back & cw step: ", i + 1)
    for i in range(3):
        new_location = move(new_location, drone, back = random.randint(20, 100), ccw = random.randint(0, 359))
        print("Back & ccw step: ", i + 1)
    print("Returning to origin")
    return_path(new_location, drone)