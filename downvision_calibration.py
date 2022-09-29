from djitellopy import Tello
import cv2 as cv
import haar_cascade as hc
from qr_reader import droneReadQR
import movement as mov
from check_camera import check_camera  
from time import sleep, time
import math

fbRange = [62000,82000] # [32000, 52000] # preset parameter for detected image boundary size
w, h = 720, 480         # display size of the screen

def calibrate(drone_class, land=False, x_coordinate=0, y_coordinate=0):
    # with open('OutputLog.csv', 'r') as outFile:
    #             start = int(outFile.readline())
    # with open('OutputLog.csv', 'a') as outFile:
    #             outFile.write(f"Calibration at ({x_coordinate, y_coordinate}) started at: {round(time()-start)}\n")
    drone = drone_class.get_drone()
    drone_class.go_to(x_coordinate, y_coordinate, rotate_only=True)
    frame = drone.get_frame_read()
    img = frame.frame
    img = cv.resize(img, (w, h))
    img, circle, width, center = hc.find_circles(img, green=False)
    cv.imshow("Scanning For Calibration Marker", img)
    cv.waitKey(1)
    found = go_to_helipad(drone_class, width, center)
    cv.destroyWindow("Scanning For Calibration Marker")
    if found == False:
        drone_class.go_to(x_coordinate, y_coordinate)
    print("><><><><><><><><><><><><><>", drone.get_height())
    drone_class.move(down=105)
    print("><><><><><><><><><><><><><>", drone.get_height())
    height = drone.get_height()
    while height > 30:
        drone.send_rc_control(0, 0, -10, 0)
        height = drone.get_height()
    for i in range(5):
        drone.send_rc_control(0, 0, 0, 0)
    print("><><><><><><><><><><><><><>", drone.get_height())

    drone.send_command_with_return("downvision 1")
    location_calibrated = False
    angle_calibrated = False
    img_counter = 0
    found_center = 0
    frames_since_positive = 0
    previous_x = 0
    previous_y = 0
    while location_calibrated is False:
        frame = drone.get_frame_read()
        img = frame.frame
        img, info = hc.find_circles(img, green=False)
        cv.imshow("Downward Output", img)
        cv.waitKey(1)

        if info is not None:
            circle_x = info[0][0][0]
            circle_y = info[0][0][1]
            frames_since_positive = 0
            if circle_x in range(150, 170) and circle_y in range(110, 130): # Calibration in tolerance range
                drone.send_rc_control(0, 0, 0, 0)
                found_center += 1
                if found_center == 15:
                    drone.send_rc_control(0, 0, 0, 0)
                    location_calibrated = True
                    drone_class.set_coordinates(x_coordinate, y_coordinate, 30)
                    drone.send_command_with_return("downvision 0")
                    cv.destroyWindow("Downward Output")
                
            else:
                x = round(-(circle_x-160)/10)
                y = round(-(circle_y-120)/10)
                if found_center == 0:
                    if (-8 < x < -2) or (2 < x < 8):
                        if x < 0: x = -8
                        elif x != 0: x = 8
                    if (-8 < y < -2) or (2 < y < 8):
                        if y < 0: y = -8
                        elif y != 0: y = 8
                drone.send_rc_control(y, x, 0, 0)

        else:
            frames_since_positive += 1
            #drone.send_rc_control(0, 0, 0, 0)
            if frames_since_positive == 5:
                frames_since_positive = 0
                img_counter += 1
                if img_counter == 30:
                    drone_class.move(fwd=60)
                elif img_counter == 60:
                    drone_class.move(left=60)
                elif img_counter == 90 or img_counter == 120:
                    drone_class.move(back=60)
                elif img_counter == 150 or img_counter == 180:
                    drone_class.move(right=60)
                elif img_counter == 210 or img_counter == 240:
                    drone_class.move(fwd=60)
                elif img_counter == 270:
                    drone_class.move(back=60, left=60, down=20)
                    img_counter = 0

    frames_since_positive = 0
    while angle_calibrated is False:
        frame = drone.get_frame_read()
        img = frame.frame
        img, info = hc.find_circles(img, down=False)
        cv.imshow("Angle Recalibration", img)
        cv.waitKey(1)

        if info is not None:
            circle_x = info[0][0][0]
            circle_y = info[0][0][1]
            frames_since_positive = 0

            if(450 < circle_x < 470):
                for i in range(5):
                    drone.send_rc_control(0, 0, 0, 0)
                # The drone is centered on the target and the coordinates and angle are now calibrated
                angle_calibrated = True
                drone_class.set_coordinates(x_coordinate, y_coordinate, 30, 0)
                cv.destroyWindow("Angle Recalibration")
                if land is False:
                    drone_class.move(up=110)
            else:
                print(circle_x)
                x = round((circle_x-460)/15)
                if x in range(-20, -3) or x in range(3, 20):
                        if x < 0: x = -9
                        elif x != 0: x = 9
                drone.send_rc_control(0, 0, 0, x)

        else:
            frames_since_positive += 1
            #drone.send_rc_control(0, 0, 0, 0)
            if frames_since_positive == 30:
                frames_since_positive = 0
                drone.send_rc_control(0, 0, 0, 20)
    
    # with open('OutputLog.csv', 'a') as outFile:
    #             outFile.write(f"Calibration at ({x_coordinate, y_coordinate}) finished at: {round(time()-start)}\n")
        
def go_to_helipad(drone, width, center):
    camera = drone.get_drone()
    if center == 0:
        for i in range(10):
            if center == 0:
                frame = camera.get_frame_read()
                img = frame.frame
                img = cv.resize(img, (w, h))
                img, circle, width, center = hc.find_circles(img, green=False)
                cv.imshow("Scanning For Calibration Marker", img)
                cv.waitKey(1)
            else:
                break
        if center == 0:
            return False
    
    x = center  # The x location of the center of the bounding box in the frame 
    area = width*width # The width of the bounding box
    img_pass = 0    # Flag to determine if the drone is returning from a target to skip point distance calculations

    # object detected
    if(x != 0):
        # TODO: CHANGE 40.64 TO THE WIDTH OF THE CIRCLE IN CM
        distance = int((650 * 40.64) / width) - 30 # (Focal length of camera lense * Real-world width of object)/Width of object in pixels  -  40 centimeters to stop short
        if distance < 20:
            distance = 20

        if(0 < x <= 340):
            # The drone needs to angle to the left to center the target.
            new_angle = int(round(((360 - x) / 360) * 41.3))

            drone.move(ccw=new_angle)  
            img, circle, width, center = check_camera(camera, circles=True)     
            go_to_helipad(drone, width, center)
            img_pass = 1

        elif(x >= 380):
            # The drone needs to angle to the right to center the target.
            new_angle = int(round(((x - 360) / 360) * 41.3))
            target_angle = drone.get_angle()-new_angle
            if target_angle < 0: target_angle += 360

            drone.move(cw=new_angle) 
            img, circle, width, center = check_camera(camera, circles=True)      
            go_to_helipad(drone, width, center)
            img_pass = 1

        if area > fbRange[0] and area < fbRange[1] and img_pass == 0:
            # The drone has approached the target and will scan for a QR code 
            return True

        elif area > fbRange[1] and img_pass == 0:
            # The drone is too close to the target
            drone.move(back=20)
            img, circle, width, center = check_camera(camera, circles=True)
            go_to_helipad(drone, width, center)

        elif area < fbRange[0] and area != 0 and img_pass == 0:
            # The drone is too far from the target
            if distance <= 500:
                drone.move(fwd=distance)
            else:
                while distance != 0:
                    if distance > 500:
                        drone.move(fwd=500)
                        distance -= 500
                    else:
                        drone.move(fwd=distance)
                        distance -= distance
            return True  

if __name__ == "__main__":
    drone = mov.movement()
    drone.go_to(365, 20) 
    # drone.go_to(220, 200)
    # drone.go_to(100, 78)
    # drone.move_down(20)
    # feed = LiveFeed(drone)
    # feed.start()
    # #feed.run()
    # sleep(1)
    calibrate(drone, True, 200, 0)
    drone.land()
