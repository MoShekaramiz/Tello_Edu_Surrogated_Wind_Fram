
import cv2 as cv
from time import sleep
import movement as mov
from traversal_image_interface import trackObject
import haar_cascade as hc
import math

def snake_exploration(drone, fly_zone, search_width, move_increment, curve=0):
    '''The drone explores a snaking.
    plt.show() needs to be called after if you want to display a plot of the path
    
    Input:
        drone : movement class initialized from movement.py
        fly_zone : [x_min, y_min, x_max, y_max] four vertices representing area to be explored
        search_width (int): distance (cm) between traversals
        move_increment (int) : distance (cm) for drone to move before checking the area distance between traversals
        curve (1 or 0) : use the curve algorithm or 90 degree angles to turn
    Output:
        total_distance (int) : total distance (cm) traveled by the drone '''

    x_min = fly_zone[0] 
    y_min = fly_zone[2] 
    x_max = fly_zone[1]  
    y_max = fly_zone[3] 

    # Ensure that the drone is in the lower right corner and rotated correctly
    if round(drone.get_x_location()) != x_min or round(drone.get_y_location()) != y_min or drone.get_angle() != 0:
        drone.go_to(x_min, y_min, 0)

    total_distance = 0                                       # tracks distance traveled
    turns = 0                                                # track number of turns
    x_distance = x_max-x_min                                   # distance needed for next horizontal traversal
    y_distance = y_max-y_min                                   # distance needed for next vertical traversal
    turn_direction = 1                                       # 0 for clockwise 1 for counter-clockwise
    x_moves_before_turn = int(x_distance/move_increment)     # number of times the drone will move on the x-axis before turning
    y_traversals = int(y_distance/search_width)              # total number of times the drone will travel in the y-axis
    y_moves_before_turn = int(search_width/move_increment)   # number of times the drone will move on the y-axis before turning
    y = 0
    
    while True:
        # CHECK CAMERA
        check_camera(drone)
        ###################
        for i in range(x_moves_before_turn):
            targetx = drone.get_x_location() + move_increment * math.cos(math.radians(drone.get_angle()))
            targety = drone.get_y_location() + move_increment * math.sin(math.radians(drone.get_angle()))
            if drone.get_angle() < 90 or drone.get_angle() > 270:
                drone.go_to(targetx, targety, 0)
            else:
                drone.go_to(targetx, targety, 180)
            # CHECK CAMERA
            check_camera(drone)
            ###################
            total_distance += move_increment

        if (x_distance % move_increment) > 20:
            targetx = drone.get_x_location() + (x_distance % move_increment) * math.cos(math.radians(drone.get_angle()))
            targety = drone.get_y_location() + (x_distance % move_increment) * math.sin(math.radians(drone.get_angle()))
            if drone.get_angle() < 90 or drone.get_angle() > 270:
                drone.go_to(targetx, targety, 0)
            else:
                drone.go_to(targetx, targety, 180)
            #drone.move(fwd=x_distance%move_increment)
            total_distance += x_distance % move_increment
            # CHECK CAMERA
            check_camera(drone)
            ###################

        if y >= y_traversals:
            break
        if turn_direction:
            drone.move(ccw=90)
            turns += 1
        else:
            drone.move(cw=90)
            turns += 1
        # CHECK CAMERA
        check_camera(drone)
        ###################
        for i in range(y_moves_before_turn):
            targetx = drone.get_x_location() + move_increment * math.cos(math.radians(drone.get_angle()))
            targety = drone.get_y_location() + move_increment * math.sin(math.radians(drone.get_angle()))
            if drone.get_angle() > 0 and drone.get_angle() < 180:
                drone.go_to(targetx, targety, 90)
            else:
                drone.go_to(targetx, targety, 270)
            #drone.move(fwd=move_increment)
            # CHECK CAMERA
            check_camera(drone)
            ###################
            total_distance += move_increment

        if (search_width % move_increment) > 20:
            targetx = drone.get_x_location() + (search_width % move_increment) * math.cos(math.radians(drone.get_angle()))
            targety = drone.get_y_location() + (search_width % move_increment) * math.sin(math.radians(drone.get_angle()))
            if drone.get_angle() > 0 and drone.get_angle() < 180:
                drone.go_to(targetx, targety, 90)
            else:
                drone.go_to(targetx, targety, 270)
            #drone.move(fwd=search_width%move_increment)
            total_distance += search_width % move_increment
            # CHECK CAMERA
            check_camera(drone)
            ###################

        if turn_direction:
            drone.move(ccw=90)
            turns += 1
        else:
            drone.move(cw=90)
            turns += 1
        y += 1
        # CHECK CAMERA
        check_camera(drone)
        ###################
        turn_direction = (turn_direction + 1) % 2 
    
    # return to original location and track the distance
    drone.go_to(0, 0, 0)
    total_distance += int(math.sqrt(drone.get_x_location()**2 + drone.get_y_location()**2))

    return total_distance


def spiral_exploration(drone, fly_zone, search_width, move_increment):
    '''The drone explores a spiral_exploration path. This function needs to be modified to check the camera itself
    plt.show() needs to be called after if you want to display a plot of the path
     
    Input:
        fly_zone : [x_min, y_min, x_max, y_max] four vertices representing area to be explored
        search_width (int): distance (cm) between traversals
        move_increment (int) : distance (cm) for drone to move before checking the area
    Output:
        total_distance (int) : total distance (cm) traveled by the drone '''

    x_min = fly_zone[0]
    y_min = fly_zone[2] 
    x_max = fly_zone[1]  
    y_max = fly_zone[3] 
    total_distance = 0        #keeps track of path distance
    turns = 0            #track # of turnsre

    # Go to correct location if starting location is not in the lower right corner
    if round(drone.get_x_location()) != x_min or round(drone.get_x_location()) != y_min or drone.get_angle() != 0:
        drone.go_to(x_min, y_min, 0)

    x_distance = x_max-x_min                 # distance needed for next horizontal traverse
    y_distance = y_max-y_min                 # distance needed for next vertical traverse
    x_traversed = 0                          # x_traversed
    y_traversed = 0                          # y_traversed
    f = 0                                    # zero until the drone has gone in one straight line

    while True: # Drone cannot travel less than 20 cm
        # travel the y_distance 
        if round(drone.get_angle())==90 or round(drone.get_angle())==270:
            if y_distance < 20:
                break
            ##### CHECK CAMERA
            check_camera(drone)
            ###################
            #sleep(0.2)    
            if y_traversed + move_increment > y_distance:
                if y_distance-y_traversed>=20:
                    target_x = drone.get_x_location() + (y_distance - y_traversed) * math.cos(math.radians(drone.get_angle()))
                    target_y = drone.get_y_location() + (y_distance - y_traversed) * math.sin(math.radians(drone.get_angle()))
                    drone.go_to(target_x, target_y, drone.get_angle())
                    total_distance += y_distance - y_traversed
                y_traversed = 0

                if f != 0:                  # decrease distance to travel each time after first line
                    y_distance -= search_width
                location =drone.move(location,drone, ccw=90)
                turns += 1
                f = 1 

            else:
                y_traversed += move_increment
                target_x = drone.get_x_location() + move_increment * math.cos(math.radians(drone.get_angle()))
                target_y = drone.get_y_location() + move_increment * math.sin(math.radians(drone.get_angle()))
                drone.go_to(target_x, target_y, drone.get_angle())
                total_distance += move_increment

        # Travel the x_distance
        elif round(drone.get_angle())==0 or round(drone.get_angle())==180:
            if x_distance < 20:
                break
            ##### CHECK CAMERA 
            check_camera(drone)
            ###################
            if x_traversed + move_increment > x_distance:
                if x_distance-x_traversed>=20:
                    target_x = drone.get_x_location() + (x_distance - x_traversed) * math.cos(math.radians(drone.get_angle()))
                    target_y = drone.get_y_location() + (x_distance - x_traversed) * math.sin(math.radians(drone.get_angle()))
                    drone.go_to(target_x, target_y, drone.get_angle())
                    total_distance += x_distance - x_traversed
                x_traversed = 0
                if f != 0:
                    x_distance -= search_width
                drone.move(ccw=90)
                turns +=1
                f = 1
            else:
                x_traversed += move_increment
                target_x = drone.get_x_location() + move_increment * math.cos(math.radians(drone.get_angle()))
                target_y = drone.get_y_location() + move_increment * math.sin(math.radians(drone.get_angle()))
                drone.go_to(target_x, target_y, drone.get_angle())
                total_distance += move_increment
    
    # return to original location and track the distance
    drone.go_to(0, 0, 0)
    total_distance += int(math.sqrt(drone.get_x_location()**2 + drone.get_y_location()**2))

    return total_distance

def check_camera(drone):
    w, h = 720, 480 # display size of the screen
    image = drone.get_drone()
    frame = image.get_frame_read()
    img = frame.frame
    img = cv.resize(img, (w, h))
    img, info = hc.findTurbine(img)
    x, y = info[0]
    if x == 0:
        for i in range(4):
            frame = image.get_frame_read()
            sleep(0.2)
            img = frame.frame
            img = cv.resize(img, (w, h))
            img, info = hc.findTurbine(img)
            x, y = info[0]
            if x != 0:
                break
    trackObject(drone, info, turbines, [drone.get_x_location(), drone.get_y_location(), drone.get_angle()])

#bounds = [0, 328, 0, 324]    #actual size of path in drone cage
if __name__ == "__main__":
    turbines = {"WindTurbine_2": [1, 0, 0, 0]}
    drone = mov.movement()
    bounds = [0, 161, 0, 221]#161
    search_width = 50
    move_increment = 75
    dist = snake_exploration(drone, bounds, search_width, move_increment)
    drone.land()
