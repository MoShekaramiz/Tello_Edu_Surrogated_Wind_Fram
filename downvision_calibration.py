import cv2 as cv
import haar_cascade as hc
import movement as mov
from check_camera import check_camera  
import numpy
import time
import csv
import sys
import os

fbRange = [62000,82000] # [32000, 52000] # preset parameter for detected image boundary size
w, h = 720, 480         # display size of the screen

def calibrate(drone_class, fileName, start, st, fileFlag, land=False, x_coordinate=0, y_coordinate=0):
    # with open('OutputLog.csv', 'r') as outFile:
    #             start = int(outFile.readline())
    # with open('OutputLog.csv', 'a') as outFile:
    #             outFile.write(f"Calibration at ({x_coordinate, y_coordinate}) started at: {round(time()-start)}\n")
    drone = drone_class.get_drone()
    # drone_class.go_to(x_coordinate, y_coordinate, rotate_only=True)
    frame = drone.get_frame_read()
    img = frame.frame
    img = cv.resize(img, (w, h))
    # img, circle, width, center = hc.find_circles(img, green=False)
    width = 0
    center = 0
    # cv.imshow("Scanning For Calibration Marker", img)
    # cv.waitKey(1)
    # cv.destroyWindow("Scanning For Calibration Marker")
    drone_class.go_to(x_coordinate - 200, y_coordinate)
    drone_class.go_to(drone_class.get_x_location(), drone_class.get_y_location(), 0)
    drone_class.move(down=40)
    found = go_to_helipad(drone_class, width, center)
    #drone_class.go_to(drone_class.get_x_location(), drone_class.get_y_location(), 0)
    if found == False:
        drone_class.go_to(x_coordinate, y_coordinate, 0)
    # found = go_to_helipad(drone_class, width, center)
    # cv.destroyWindow("Scanning For Calibration Marker")
    # if found == False:
    #     print("Not found.")
    #     drone_class.go_to((x_coordinate + drone_class.get_x_location())/2, (y_coordinate + drone_class.get_y_location())/2)
    #     found = go_to_helipad(drone_class, width, center)
    #     if found == False:
    #         drone_class.go_to((x_coordinate + drone_class.get_x_location())/2, (y_coordinate + drone_class.get_y_location())/2)
    #         found = go_to_helipad(drone_class, width, center)
    #         print("Not found, second time.")
    #         if found == False:
    #             drone_class.go_to(x_coordinate, y_coordinate)
    #             print("Not found, third time")
    print("><><><><><><><><><><><><><>", drone.get_height())
    drone_class.move(down=105)
    print("><><><><><><><><><><><><><>", drone.get_height())
    height = drone.get_height()
    while height > 30:
        drone.send_rc_control(0, 0, -20, 0)
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
        img, circle = hc.find_circles(img, green=True)
        cv.imshow("Downward Output", img)
        cv.waitKey(1)

        if circle is not None:
            circle_x = circle[0][0][0]
            circle_y = circle[0][0][1]
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
                    current = time.time()
                    drone.land()
                    numRows = 0          
                    CsvFile = open(fileName, 'r') 
                    csvreader = csv.reader(CsvFile, delimiter=',')
                    for row in csvreader:
                        numRows = numRows + 1
                        previous_time = row[3]
                    previous_time = float(previous_time)
                    CsvFile.close()
                    print('numRows:', numRows)
                    print('previous_time:', previous_time)
                    csvFile = open(fileName, "a")
                    csvwriter = csv.writer(csvFile, lineterminator='\n')
                    if((current-previous_time-start)%60 < 10 and (current-start)%60 < 10):
                        csvwriter.writerow(['Landing helipad', current-previous_time-start, str(int((current-previous_time-start)//60)) + '.0' + str((current-previous_time-start)%60), current-start, str(int((current-start)//60)) + '.0' + str((current-start)%60), str(drone.get_battery()) + ' %'])
                    elif((current-previous_time-start)%60 < 10):
                        csvwriter.writerow(['Landing helipad', current-previous_time-start, str(int((current-previous_time-start)//60)) + '.0' + str((current-previous_time-start)%60), current-start, str(int((current-start)//60)) + '.' + str((current-start)%60), str(drone.get_battery()) + ' %'])
                    elif((current-start)%60 < 10):
                        csvwriter.writerow(['Landing helipad', current-previous_time-start, str(int((current-previous_time-start)//60)) + '.' + str((current-previous_time-start)%60), current-start, str(int((current-start)//60)) + '.0' + str((current-start)%60), str(drone.get_battery()) + ' %'])
                    else:
                        csvwriter.writerow(['Landing helipad', current-previous_time-start, str(int((current-previous_time-start)//60)) + '.' + str((current-previous_time-start)%60), current-start, str(int((current-start)//60)) + '.' + str((current-start)%60), str(drone.get_battery()) + ' %'])
                    csvFile.close()
                    if(fileFlag == 1):
                        fName = 'CSV Files\Successful ' + str(numRows-2) + '-Fan test Data Log ' + st + '.csv'
                    else:
                        fName = 'CSV Files\Failure Data Log' + st + '.csv'
                    fName = os.path.dirname(__file__) + '\\' + fName
                    fileName = os.path.dirname(__file__) + '\\' + fileName
                    csvFile.close()
                    os.rename(fileName, fName)
                    sys.exit()
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

    # frames_since_positive = 0
    # angle = 0
    # up_count = 0
    # while angle_calibrated is False:
    #     frame = drone.get_frame_read()
    #     img = frame.frame
    #     img, info = hc.find_circles(img, down=False, green=True)
    #     cv.imshow("Angle Recalibration", img)
    #     cv.waitKey(1)

    #     if info is not None:
    #         circle_x = info[0][0][0]
    #         circle_y = info[0][0][1]
    #         frames_since_positive = 0

    #         if(450 < circle_x < 470):
    #             for i in range(5):
    #                 drone.send_rc_control(0, 0, 0, 0)
    #             # The drone is centered on the target and the coordinates and angle are now calibrated
    #             angle_calibrated = True
    #             drone_class.set_coordinates(x_coordinate, y_coordinate, 30, 0)
    #             cv.destroyWindow("Angle Recalibration")
    #             if land is False:
    #                 drone_class.move(up=(110 - (up_count * 20)))
    #         else:
    #             print(circle_x)
    #             x = round((circle_x-460)/15)
    #             if x in range(-20, -3) or x in range(3, 20):
    #                     if x < 0: x = -9
    #                     elif x != 0: x = 9
    #             drone.send_rc_control(0, 0, 0, x)

    #     else:
    #         frames_since_positive += 1
    #         #drone.send_rc_control(0, 0, 0, 0)
    #         if frames_since_positive == 30 and (angle % 360 != 0 or angle == 0):
    #             frames_since_positive = 0
    #             drone_class.move(cw=30)
    #             angle = angle + 30
    #         elif angle % 360 == 0 and angle != 0:
    #             drone_class.move(up=20)
    #             up_count += 1
    #         if up_count == 3:
    #             break
            
    
    # with open('OutputLog.csv', 'a') as outFile:
    #             outFile.write(f"Calibration at ({x_coordinate, y_coordinate}) finished at: {round(time()-start)}\n")
# blue circles        
def go_to_helipad(drone, width, center, flag_rotate=0, flag_shift=0, flag_shift_direction = "none"):
    camera = drone.get_drone()
    if center == 0:
        for i in range(100):
            if center == 0:
                frame = camera.get_frame_read()
                img = frame.frame
                img = cv.resize(img, (w, h))
                img, circle, width, center = hc.find_circles(img, down=False, green=False)
                cv.imshow("Scanning For Blue Circle", img)
                cv.waitKey(1)
            else:
                break
        # if we did not find the center after finding it previously and our last command was a rotate. Rotate the same degree in the opposite direction
        if center == 0 and flag_rotate != 0:
            drone.move(ccw=flag_rotate)
            go_to_helipad(drone, width, center, flag_rotate)
        # if we did not find the center after finding it previously and our last command was a shift left. Move right the same distance.
        elif center == 0 and flag_shift != 0 and flag_shift_direction == "left":
            drone.move(right=flag_shift)
            go_to_helipad(drone, width, center, flag_shift)
        # if we did not find the center after finding it previously and our last command was a shift right. Move left the same distance.
        elif center == 0 and flag_shift != 0 and flag_shift_direction == "right":
            drone.move(left=flag_shift)
            go_to_helipad(drone, width, center, flag_shift)
        elif center == 0:
            return False
    
    x = center  # The x location of the center of the bounding box in the frame 
    area = width*width # The width of the bounding box
    img_pass = 0    # Flag to determine if the drone is returning from a target to skip point distance calculations

    # object detected
    if(x != 0):
        print("Blue circle detected")
        # TODO: CHANGE 40.64 TO THE WIDTH OF THE CIRCLE IN CM
        distance = int((650 * 17.28) / width) - 90 # (Focal length of camera lense * Real-world width of object)/Width of object in pixels  -  40 centimeters to stop short
        print("Distance: ", distance)
        #if our distance is set to 
        if distance < 20 and distance > 10:
            print("distance is 0")
            distance = 20
        elif distance <= 10:
            print("distance is 0")
            distance = 0
        if(0 < x <= 350): #maybe try 355
            # The drone sees the blue circle in the left half of the image
            new_angle = int(round(((360 - x) / 360) * 41.3))
            print("new_angle: " , new_angle)
            shift = numpy.abs(distance * numpy.sin(new_angle * numpy.pi/180))
            print("Shift Left: " , shift)
            # If our opposite O found from distance * sin(theta) < 20 rotate the drone counter-clockwise to center on the blue circle.
            if shift < 20:
                drone.move(ccw=new_angle)  
                flag_rotate = new_angle * -1
                flag_shift = 0
            else: 
            # If our opposite O found from distance * sin(theta) > 20 shift the drone left by O to center on the blue circle.
                drone.move(left=shift)
                flag_shift = shift
                flag_rotate = 0
                flag_shift_direction = "left"
            img, circle, width, center = check_camera(camera, circles=True)     
            go_to_helipad(drone, width, center, flag_rotate, flag_shift, flag_shift_direction)
            img_pass = 1

        elif(x >= 370): #maybe try 365
            # The drone sees the blue circle in the right half of the image
            new_angle = int(round(((x - 360) / 360) * 41.3))
            target_angle = drone.get_angle()-new_angle
            if target_angle < 0: target_angle += 360
            print("new_angle: " , new_angle)
            shift = numpy.abs(distance * numpy.sin(new_angle * numpy.pi/180))
            print("Shift Right: " , shift)
            # If our opposite O found from distance * sin(theta) < 20 rotate the drone clockwise to center on the blue circle.
            if shift < 20:
                drone.move(cw=new_angle)
                flag_rotate = new_angle
                flag_shift = 0
            # If our opposite O found from distance * sin(theta) > 20 shift the drone right by O to center on the blue circle.
            else:
                drone.move(right=shift)
                flag_shift = shift
                flag_rotate = 0
                flag_shift_direction = "right" 
            img, circle, width, center = check_camera(camera, circles=True)      
            go_to_helipad(drone, width, center, flag_rotate, flag_shift, flag_shift_direction)
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
            flag_rotate = 0
            if distance <= 250 and distance > 19:
                drone.move(fwd=distance)
                print("First If")
            else:
                while distance != 0:
                    if distance > 250:
                        drone.move(fwd=250)
                        print("While loop IF")
                        distance -= 250
                        if distance < 250:
                            drone.move(down=40)
                        center = 0
                        go_to_helipad(drone, width, center)
                    else: 
                        if distance < 20 and distance > 10:
                            print("distance is 20")
                            distance = 20
                        elif distance <= 10:
                            print("distance is 0")
                            distance = 0
                        print("While loop else")
                        drone.move(fwd=distance)
                        distance -= distance
                        
            return True  

if __name__ == "__main__":
    drone = mov.movement()
    # drone.go_to(365, 20) 
    # drone.go_to(220, 200)
    # drone.go_to(100, 78)
    # drone.move_down(20)
    # feed = LiveFeed(drone)
    # feed.start()
    # #feed.run()
    # sleep(1)
    calibrate(drone, True, 200, 0)
    drone.land()
