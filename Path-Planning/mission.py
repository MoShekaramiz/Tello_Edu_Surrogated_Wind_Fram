'''Script to take video and photos of the fans and place them within a created directory sharing the name of the turbine that is being documented
By Brandon Pinney 2022 and Angel Rodriguez 2023'''

import cv2 as cv
import shutil
from time import sleep
import os
from vars import *
import time
import csv

path = "Default"

def mission0(mv, mission, turbine):
    '''Film directly in front of the fan'''

    # This block of code will change current directory to the directory for the turbine scanned, new directory will be created if not already existing
    parent_directory = os.getcwd()
    directory = str('WindTurbines\\' + turbine)
    path = os.path.join(parent_directory, directory) # The current working directory, it is called the parent since we will go into its child's directory
    if not os.path.isdir(directory): # If the directory does not already exist
        # shutil.rmtree(directory) # Delete directory
        os.mkdir(path) # Make path directory
    os.chdir(path) # Change directory to the path

    # Get x and y coordinates of the turbine
    coordinates = turbines[turbine][1]
    print(f'coordinates of {turbine}: {coordinates}')
    # Go to y-coordinate of fan without changing angle
    # mv.y_go_to(coordinates[1])
    if mission[0] == 1: # Film the fan on the front side
        mv.move(fwd=60) # Move a little closer to the fan
        temp_time = time.time() # Begin tracking time spent for doing the mission around each fan
        temp_battery = mv.get_battery() # Value of battery proceeding the mission
        # img_count = recordVideo(mv, turbine, 'front')
        rectangle_path(mv, turbine, 'front')
    mission[0] = 0
    if mission[1] == 1: # Film the fan on the right side
        mission1(mv, mission, turbine)
    elif mission[3] == 1: # Film the fan on the back side
        mission3(mv, mission, turbine)
    elif mission[2] == 1: # Film the fan on the left side
        mission2(mv, mission, turbine)

    # Return to parent directory, otherwise we will recursively create new folders inside another
    os.chdir(parent_directory)

    # Write mission information to CSV file
    mission_time = time.time() - temp_time
    current_battery = mv.get_battery()
    mission_battery = temp_battery - current_battery
    fields = [turbine, mission_time, mission_battery, current_battery, mv.get_flight_time()]
    with open(mission_filename, 'a') as f:
        writer = csv.writer(f)
        writer.writerow(fields)
    # Set the z location to equal the height determined by the drone sensor
    mv.set_coordinates(mv.get_x_location(), mv.get_y_location(), z_location=mv.get_height())
    # Return to normal traveling height
    mv.go_to(mv.get_x_location(), mv.get_y_location(), targetz=mv.get_traversal_altitude())
    # mv.move(up=40)

def mission1(mv, mission, turbine):
    '''Film the fan on the right side'''
    mv.move(right=60)
    mv.move(fwd=60)
    mv.move(ccw=90)
    if mv.get_z_location() == 0:
        mv.move(up=20)
    recordVideo(mv, turbine, 'right')
    mission[1] = 0
    if mission[2] == 1:
        mission2(mv, mission, turbine, recent_mission = 1)
    elif mission[3] == 1:
        mission3(mv, mission, turbine, recent_mission = 1)
    else:
        mv.move(left=60)
        mv.move(fwd=60)
        mv.move(cw=90)

def mission2(mv, mission, turbine, recent_mission = 0):
    '''Film the back side of the fan'''
    if mv.get_z_location() == 0:
        mv.move(up=50)
    if recent_mission == 0:
        # move around the fan to avoid hitting it
        mv.move(right=60)
        mv.move(fwd=120)
        # This below is going to be more than the value we moved right since rectangle_path ends on the right side of the front of the fan
        mv.move(left=80)
        # Face the back of the fan
        mv.move(ccw=180)
    elif recent_mission == 1:
        mv.move(right=50)
        mv.move(fwd=60)
        mv.move(ccw=90)
    # img_count = recordVideo(mv, turbine, 'back')
    rectangle_path(mv, turbine, 'back')
    mission[2] = 0
    if mission[3] == 1:
        mission3(mv, mission, turbine, recent_mission = 2)
    else: # Go back to the front of the fan
        # mv.move(right=60)
        # mv.move(fwd=110)
        # mv.move(left=60)
        # mv.move(cw = 180)
        pass

def mission3(mv, mission, turbine, recent_mission = 0):
    '''Film the left side of the fan after going up 20cm'''
    if mv.get_z_location() == 0:
        mv.move(up=20)
    if recent_mission == 0:
        mv.move(left=60)
        mv.move(fwd=60)
        mv.move(cw=90)
    elif recent_mission == 1:
        mv.move(left=60)
        mv.move(fwd=120)
        mv.move(right=60)
        mv.move(ccw=180)
    elif recent_mission == 2:
        mv.move(right=60)
        mv.move(fwd=50)
        mv.move(ccw=90)
    recordVideo(mv, turbine, 'left')
    mission[3] = 0
    if mission[2] == 1:
        mission2(mv, mission, turbine)
    else:
        mv.move(right=60)
        mv.move(fwd=60)
        mv.move(ccw=90)

def recordVideo(mv, turbine, step, img_num):
    '''Records video and extracts photos from the video to save into the current directory'''
    # Get unique time stamp for each photo
    drone = mv.get_drone()
    frame_read = drone.get_frame_read()
    height, width, _ = frame_read.frame.shape
    record = 1 # Quantity of frames to save
    # Start video
    # video = cv.VideoWriter(f'{turbine}_{step}_video_{img_num}_{mission_st}.mp4', cv.VideoWriter_fourcc(*'mp4v'), 30, (width, height))
    while record > 0: # Keep saving images until record = 0
        # if record % 10 == 0:  # IF we want to save images for every 10 frames recorded
        cv.imwrite(f'{turbine}_{step}_img{img_num}_{mission_st}.png', frame_read.frame) # Save frame as .png file
        img_num += 1
        # video.write(frame_read.frame) # Add frame to the video
        sleep(1/60)
        record -= 1 
        # video.release() # Release video when finished adding frames to video, otherwise, video will be unplayable
    return img_num

def rectangle_path(drone, turbine, turbine_side, img_count=1):
    '''Makes a standing rectangle path for purposed of recording a fan (should not move in x-direction)'''
    img_count = recordVideo(drone, turbine, turbine_side, img_count)
    drone.move(left=20)
    img_count = recordVideo(drone, turbine, turbine_side, img_count)
    drone.move(down=20)
    img_count = recordVideo(drone, turbine, turbine_side, img_count)
    drone.move(right=20)
    img_count = recordVideo(drone, turbine, turbine_side, img_count)
    drone.move(right=20)
    img_count = recordVideo(drone, turbine, turbine_side, img_count)
    drone.move(up=20)
    img_count = recordVideo(drone, turbine, turbine_side, img_count)

if __name__ == "__main__":
    # turbine = 'WindTurbine_2'
    # '''Test code for changing and/or making directories'''
    # parent_directory = os.getcwd() # The current working directory, it is called the parent since we will go into its child's directory
    # directory = str('WindTurbines\\' + turbine)
    # path = os.path.join(parent_directory, directory)
    # if not os.path.isdir(directory): # If the directory does not already exist
    #     # shutil.rmtree(directory) # Delete directory
    #     os.mkdir(path) # Make path directory
    # os.chdir(path) # Change directory to the path
    # f = open(f"{turbine}demofile2.txt", "a")
    # f.write("Now the file has more content!")
    # f.close()
    # f = open(f"{turbine}demofile3.txt", "a")
    # f.write("Now the file has more content!")
    # f.close()
    pass