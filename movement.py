'''The movment module that facilitates drone coordinate tracking and return path algorithms. By Branden Pinney and Shayne Duncan 2022'''

from ast import operator
from djitellopy import Tello
import operator
import math
from time import sleep
from output_video import LiveFeed
import sys
import os

CWD = os.getcwd()

class movement():
    def __init__(self, height=70, stream=True):
        self.drone = Tello()               # Drone class initialized by calling: drone = Tello()
        self.new_location = [0, 0, 0, 0]   # [x location, y location, z location, angle]
        self.turbine_locations = []        # List of all known turbine locations
        self.video_stream = None
        self.takeoff(height, stream)
    
    def takeoff(self, height=70, stream=True): 
        '''Initializes and launches the drone and rises 40 cm as a default.'''
        self.drone.connect()
        sleep(1)
        self.drone.streamon()
        sleep(1)
        if stream:
            self.video()
            sleep(1)
        self.drone.takeoff()
        print("Current battery remaining: ", self.drone.get_battery())
        self.drone.send_command_with_return("downvision 0")
        self.new_location[2] = self.drone.get_height()
        self.move(up=height)
    
    def video(self):
        self.video_stream = LiveFeed(self.drone)
        self.video_stream.start()
    
    def land(self, turn_off = False):
        '''Lands the drone at the end of flight.'''
        print(">>>>>>>>>>>>>>>> BATTERY REMAINING: ", self.drone.get_battery())
        print(">>>>>>>>>>>>>>>> ENDING FLIGHT TIME: ", self.drone.get_flight_time())
        self.drone.land()
        self.video_stream.stop_haar()
        self.video_stream.stop_qr()
        self.video_stream.stop_image()
        self.drone.streamoff()
        print("\nThe drone has succesfully landed. See directory " + CWD + " to view collected data.\n")
        if turn_off == True:
            sys.exit()

    def append_turbine_locations(self, QR, known_locations=None):
        '''Add the location of a found turbine to the list and create the no-fly zone around it.'''
        self.turbine_locations.append([self.new_location[0] - 30, self.new_location[0] + 30, self.new_location[1] - 30, 
                                       self.new_location[1] + 30, QR, self.new_location[0], self.new_location[1]])

    def get_turbine_locations(self):
        '''Returns the list of all known turbines locatoins, their no-fly zones, and their QR code data.'''
        return self.turbine_locations
        
    def get_location(self):
        '''Returns a list containing the current x, y, z coordinates of the drone and the current angle relative to takeoff.'''
        return self.new_location

    def get_x_location(self):
        '''Returns the current x location of the drone.'''
        return self.new_location[0]
    
    def get_y_location(self):
        '''Returns the current y location of the drone.'''
        return self.new_location[1]
    
    def get_z_location(self):
        '''Returns the altitude currently in the location list.'''
        return self.new_location[2]
    
    def get_angle(self):
        '''Returns the current angle of the drone.'''
        return self.new_location[3]
    
    def get_drone(self):
        '''Returns the class controlling the SDK for the drone.'''
        return self.drone
    
    def get_video(self):
        '''Returns the live stream video'''
        return self.video_stream

    def set_coordinates(self, x_location=None, y_location=None, z_location=None, angle=None):
        '''Sets the coordinates of the drone'''
        if x_location != None:
            self.new_location[0] = x_location
        if y_location != None:
            self.new_location[1] = y_location
        if z_location != None:
            self.new_location[2] = z_location
        if angle != None:
            self.new_location[3] = angle

    def move(self, fwd=0, back=0, ccw=0, cw=0, up=0, down=0, left=0, right=0):
        '''Takes a list holding the x, y cartesian coordinates of the drone and the angle relative to takeoff [x, y, angle] (initialized as [0, 0, 0]).
        The variable for the drone being used is also required.
        Then commands for fwd and back will be taken that can range from 20-500, and ccw and cw from 0-359.
        The function will then rotate the drone first with counter-clockwise (ccw) being the priority, then move with forward (fwd) being the priority.
        The function will update the current location of the drone which can be reached with movement.get_location()'''
        sleep(0.3)
        if up != 0:
            # increases drone altitude
            up = round(up)
            self.drone.move_up(up)
            self.new_location[2] += up

        elif up == 0 and down != 0:
            # decreases drone altitiude
            down = round(down)
            if self.new_location[2] > down:
                self.drone.move_down(down)
                self.new_location[2] -= down

        if ccw != 0:
            # counter-clockwise takes priority -- updates angle after counter-clockwise rotation
            ccw = round(ccw)
            self.drone.rotate_counter_clockwise(ccw)
            self.new_location[3] = (self.new_location[3] + ccw) % 360 

        elif ccw == 0 and cw != 0:
            # updates angle after clockwise rotation
            cw = round(cw)
            self.drone.rotate_clockwise(cw)

            if cw > self.new_location[3]:
                # Returned angle if cw angle is greater than the current angle
                self.new_location[3] = 360 - abs((self.new_location[3] - cw)) 

            else:
                # Returned angle if cw angle is less than the current angle
                self.new_location[3] = abs((self.new_location[3] - cw)) 

        if fwd != 0:
            # forward takes priority -- updates x and y coordinates after movement
            fwd = round(fwd)
            self.drone.move_forward(fwd)
            self.new_location[0] += fwd * math.cos(math.radians(self.new_location[3]))
            self.new_location[1] += fwd * math.sin(math.radians(self.new_location[3]))

        elif fwd == 0 and back != 0:
            # updates x and y coordinates after backwards movement
            back = round(back)
            self.drone.move_back(back)
            self.new_location[0] += -(back) * math.cos(math.radians(self.new_location[3]))
            self.new_location[1] += -(back) * math.sin(math.radians(self.new_location[3]))

        if left != 0:
            # left takes priority -- updates x and y coordinates after lateral left movement
            left = round(left)
            self.drone.move_left(left)
            self.new_location[0] += left * math.cos(math.radians((self.new_location[3] + 90) % 360))
            self.new_location[1] += left * math.sin(math.radians((self.new_location[3] + 90) % 360))

        elif left == 0 and right != 0:
            # updates x and y coordinates after lateral right movement
            right = round(right)
            self.drone.move_right(right)
            if self.new_location[3] - 90 >= 0:
                self.new_location[0] += right * math.cos(math.radians(self.new_location[3] - 90))
                self.new_location[1] += right * math.sin(math.radians(self.new_location[3] - 90))
            else:
                self.new_location[0] += right * math.cos(math.radians(360 + (self.new_location[3] - 90)))
                self.new_location[1] += right * math.sin(math.radians(360 + (self.new_location[3] - 90)))

    def curve(self, radius = 50, left_right = 0):
        '''Curve a quarter circle left or right. Currently developed for a radius of 50cm.'''
        velocity = round(1.5 * ((radius * math.pi * 18)/180))
        if left_right == 0: # 0 is to curve left
            self.drone.send_rc_control(0, velocity, 0, -36)
            sleep(4.05)
            self.drone.send_rc_control(0, 0, 0, -36)
            sleep(0.2)
            self.drone.send_rc_control(0, 0, 0, 0)
            sleep(5)
            self.new_location[0] += (2 * radius * math.sin(math.radians(90/2))) * math.cos(math.radians((self.new_location[3] + 45) % 360))
            self.new_location[1] += (2 * radius * math.sin(math.radians(90/2))) * math.sin(math.radians((self.new_location[3] + 45) % 360))
            self.new_location[3] = (self.new_location[3] + 90) % 360

        else: # any other input will curve right
            self.drone.send_rc_control(0, -velocity, 0, 36)
            sleep(4.05)
            self.drone.send_rc_control(0, 0, 0, 36)
            sleep(0.2)
            self.drone.send_rc_control(0, 0, 0, 0)
            sleep(3)
            if self.new_location[3] < 45:
                self.new_location[0] += (2 * radius * math.sin(math.radians(90/2))) * math.cos(math.radians((self.new_location[3] - 45) + 360))
                self.new_location[1] += (2 * radius * math.sin(math.radians(90/2))) * math.sin(math.radians((self.new_location[3] - 45) + 360))
            else:
                self.new_location[0] += (2 * radius * math.sin(math.radians(90/2))) * math.cos(math.radians(self.new_location[3] - 45))
                self.new_location[1] += (2 * radius * math.sin(math.radians(90/2))) * math.sin(math.radians(self.new_location[3] - 45))
            if self.new_location[3] < 90:
                self.new_location[3] = (self.new_location[3] - 90) + 360
            else:
                self.new_location[3] = self.new_location[3] - 90

    def target_angle(self, return_angle, x, y, quadrant=0):
        '''This function helps the drone move the shortest distance to the correct angle
        by taking the intended angle and the x and y coordinates.
        This function is not typically called by the user.'''
        dronex = round(self.new_location[0])
        droney = round(self.new_location[1])
        angle = self.new_location[3]

        # The following cases for rotation are calculated by placing the desired
        # location at the orgin of a cartesian grid and having the drone rotate
        # based on the relative qudrant of the drone and the current angle 
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
                self.move(cw=int(angle + relative_angle + 90))
            elif return_angle <= angle < 180 + return_angle:
                self.move(ccw=int(180 + return_angle - angle))
            elif 180 + return_angle <= angle:
                self.move(cw=int(angle - (180 + return_angle)))

        elif quadrant == 2:# quadrant 2
            relative_angle = 90 + return_angle
            if angle < 90 + relative_angle:
                self.move(cw=int(angle + abs(return_angle)))
            elif 90 + relative_angle <= angle <= 270 + relative_angle:
                self.move(ccw=int(270 - angle + relative_angle))
            elif 270 + relative_angle <= angle:
                self.move(cw=int(angle - 270 - relative_angle))

        elif quadrant == 3:# quadrant 3
            relative_angle = 90 - return_angle
            if angle < return_angle:
                self.move(ccw=int(return_angle - angle))
            elif relative_angle <= angle < return_angle + 180:
                self.move(cw=int(angle - return_angle))
            elif 180 + return_angle <= angle:
                self.move(ccw=int(360 - angle + return_angle))

        elif quadrant == 4:# quadrant 4
            relative_angle = 90 + return_angle
            if angle < 90 + relative_angle:
                self.move(ccw=int(90 - angle + relative_angle))
            elif 90 + relative_angle < angle < 270 + relative_angle:
                self.move(cw=int(angle - relative_angle - 90))
            elif 270 + relative_angle <= angle:
                self.move(ccw=int(450 - angle + relative_angle))

        elif x == dronex and y > droney:# positive Y axis
            if 0 <= angle < 90:
                self.move(ccw=int(90 - angle))
            elif 90 <= angle < 180:
                self.move(cw=int(angle - 90))
            elif 180 <= angle < 270:
                self.move(cw=int(90 + angle))
            elif 270 <= angle <= 360:
                self.move(cw=int(90 + (360 - angle)))

        elif x == dronex and y < droney:# negative Y axis
            if 90 <= angle < 270:
                self.move(ccw=int(270 - angle))
            elif 0 <= angle < 90 or 270 <= angle < 360:
                self.move(cw=int(270 - angle))

        elif x < dronex and y == droney:# positive X axis
            if 0 <= angle <= 180:
                self.move(ccw=int(180 - angle))
            elif 180 < angle <= 360:
                self.move(cw=int(360 - angle))

        elif x > dronex and y == droney:# negative X axis
            if 0 <= angle <= 180:
                self.move(cw=int(angle))
            elif 180 < angle <= 360:
                self.move(ccw=int(360 - angle))
    ########################################################################

    def go_to(self, targetx=0, targety=0, ending_angle=None, targetz=None, rotate_only=False, half_travel=False):
        '''Used to tell the drone to go to a specific cartesian coordinate with a target X and Y value,
        along with the desired ending angle.'''
        x = round(self.new_location[0])
        y = round(self.new_location[1])
        target_x = round(targetx)
        target_y = round(targety)
        angle = self.new_location[3]
        self.new_location[2] = self.drone.get_height()

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
        if half_travel is True:
            point_distance = point_distance-30
        try:
            vector_angle = int(round(math.degrees(math.atan((y-target_y)/(x-target_x)))))
        except ZeroDivisionError:
            vector_angle = 0
        possible_collisions = []

        for i in self.turbine_locations:
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

            elif quadrant == 4: # drone in quadrant 4
                if (x < centerx < target_x) and (y < centery < target_y):
                    possible_collisions.append([i[0], i[1], i[2], i[3], center_distance, centerx, centery])

        if rotate_only is True:
            self.target_angle(vector_angle, target_x, target_y, quadrant)
            return

        if len(possible_collisions) != 0:
            possible_collisions = sorted(possible_collisions, key=operator.itemgetter(4)) # sort the possible collision list by distance from drone
            starting_point = self.new_location
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

                            right_distance = math.sqrt((right_corner[0] - self.new_location[0])**2 + (right_corner[1] - self.new_location[1])**2) # point-distance to the bottom right corner
                            left_distance = math.sqrt((left_corner[0] - self.new_location[0])**2 + (left_corner[1] - self.new_location[1])**2) # point-distance to the bottom left corner

                            if right_distance < left_distance:
                                try:
                                    return_angle = abs(math.degrees(math.atan((x-right_corner[0])/(y-right_corner[1]))))
                                except ZeroDivisionError:
                                    pass
                                self.target_angle(self, return_angle, right_corner[0], right_corner[1], quadrant)
                                self.move(fwd=right_distance)

                            else:
                                try:
                                    return_angle = abs(math.degrees(math.atan((x-left_corner[0])/(y-left_corner[1]))))
                                except ZeroDivisionError:
                                    pass
                                self.target_angle(return_angle, left_corner[0], left_corner[1], quadrant)
                                self.move(fwd=left_distance)

                            self.go_to(target_x, target_y, ending_angle) # call return path again until there are no possible collisions remaining
                            ######### Work on if the no-go zone overlaps an axis
            if self.new_location == starting_point:
                self.target_angle(vector_angle, target_x, target_y, quadrant)
                while (point_distance != 0):
                    if (point_distance >= 500):
                        self.move(fwd=500)
                        point_distance -= 500

                    elif (point_distance < 500) and (point_distance < 20):
                        point_distance = 0

                    else:
                        self.move(fwd=point_distance)
                        sleep(0.5)
                        point_distance = 0
                if ending_angle is not None: # rotate the shortest distance to the ending angle
                    angle = self.new_location[3] # current angle has to be updated
                    if ending_angle < 0:
                        ending_angle = abs(360 + angle) % 360
                    else:
                        ending_angle = ending_angle % 360
                    alpha = ending_angle - angle
                    if alpha < 0:
                        alpha = round(abs(alpha))
                        if alpha < 180:
                           self.move(cw=(alpha))
                        else:
                            self.move(ccw=(360 - alpha))
                    elif alpha < 180:
                        self.move(cw=(round(alpha)))
                    else:
                        self.move(ccw=(round(360 - alpha)))
        else:
            self.target_angle(vector_angle, target_x, target_y, quadrant)
            while (point_distance != 0):
                if (point_distance >= 500):
                    self.move(fwd=500)
                    point_distance -= 500

                elif (point_distance < 500) and (point_distance < 20):
                    point_distance = 0

                else:
                    self.move(fwd=point_distance)
                    sleep(0.5)
                    point_distance = 0

            if ending_angle is not None: # rotate the shortest distance to the ending angle
                angle = round(self.new_location[3]) # current angle has to be updated
                if ending_angle < 0:
                    ending_angle = abs(360 + angle) % 360
                else:
                    ending_angle = ending_angle % 360
                alpha = ending_angle - angle
                if alpha < 0:
                    alpha = round(abs(alpha))
                    if alpha < 180:
                        self.move(cw=(alpha))
                    else:
                        self.move(ccw=(360 - alpha))
                elif alpha < 180:
                    self.move(ccw=(round(alpha)))
                else:
                    self.move(ccw=(round(360 - alpha)))
        if targetz is not None:
            if targetz > self.get_z_location():
                if (targetz - self.get_z_location()) >= 20:
                    self.move(up=targetz - self.get_z_location())
            elif targetz < self.get_z_location():
                if (self.get_z_location() - targetz) >= 20:
                    self.move(down=self.get_z_location() - targetz)
                    
        print(f"\nCURRENT LOCATION >>>>>>>>>>{self.new_location}\n")


