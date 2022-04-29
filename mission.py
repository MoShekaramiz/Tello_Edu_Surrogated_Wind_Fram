import cv2 as cv
import movement as mv
import shutil
from time import sleep
from djitellopy import Tello
import os

path = "Default"
def mission0(location, drone, mission, turbine):
    '''Film directly in front of the fan after rising 20cm'''
    mv.move(location, drone, up=20)
    parent_directory = os.getcwd()
    directory = turbine
    path = os.path.join(parent_directory, directory)
    if os.path.isdir(directory):
        shutil.rmtree(directory)
    os.mkdir(path)
    os.chdir(path)
    if mission[0] == 1:
        recordVideo(drone, turbine, 0)
    mission[0] = 0
    if mission[1] == 1:
        mission1(drone, location, mission, turbine)
    elif mission[3] == 1:
        mission3(drone, location, mission, turbine)
    elif mission[2] == 1:
        mission2(drone, location, mission, turbine)

def mission1(drone, location, mission, turbine):
    '''Film the fan on the right side after rising 20cm'''
    drone.move_right(60)
    drone.move_forward(60)
    mv.move(location, drone, ccw=90)
    if location[3] == 0:
        mv.move(location, drone, up=20)
    recordVideo(drone, turbine, 1)
    mission[1] = 0
    if mission[2] == 1:
        mission2(drone, location, mission, turbine, recent_mission = 1)
    elif mission[3] == 1:
        mission3(drone, location, mission, turbine, recent_mission = 1)
    else:
        drone.move_left(60)
        drone.move_forward(60)
        mv.move(location, drone, cw=90)

def mission2(drone, location, mission, turbine, recent_mission = 0):
    '''Film the back side of the fan  after rising 20cm'''
    if location[3] == 0:
        mv.move(location, drone, up=50)
    if recent_mission == 0:
        drone.move_right(60)
        drone.move_forward(110)
        drone.move_left(60)
        mv.move(location, drone, ccw=180)
    elif recent_mission == 1:
        drone.move_right(50)
        drone.move_forward(60)
        mv.move(location, drone, ccw=90)
    recordVideo(drone, turbine, 2)
    mission[2] = 0
    if mission[3] == 1:
        mission3(drone, location, mission, turbine, recent_mission = 2)
    else:
        drone.move_right(60)
        drone.move_forward(110)
        drone.move_left(60)
        mv.move(location, drone, cw = 180)

def mission3(drone, location, mission, turbine, recent_mission = 0):
    '''Film the left side of the fan after going up 20cm'''
    if location[3] == 0:
        mv.move(location, drone, up=20)
    if recent_mission == 0:
        drone.move_left(60)
        drone.move_forward(60)
        mv.move(location, drone, cw=90)
    elif recent_mission == 1:
        drone.move_left(60)
        drone.move_forward(120)
        drone.move_right(60)
        mv.move(location, drone, ccw=180)
    elif recent_mission == 2:
        drone.move_right(60)
        drone.move_forward(50)
        mv.move(location, drone, ccw=90)
    recordVideo(drone, turbine, 3)
    mission[3] = 0
    if mission[2] == 1:
        mission2(drone, location, mission, turbine)
    else:
        drone.move_right(60)
        drone.move_forward(60)
        mv.move(location, drone, ccw=90)

def recordVideo(drone, turbine, step):
    count = 0
    img_num = 1
    frame_read = drone.get_frame_read()
    height, width, _ = frame_read.frame.shape
    record = 300
    video = cv.VideoWriter(f'{turbine}_{step}.mp4', cv.VideoWriter_fourcc(*'XVID'), 30, (width, height))
    while record > 0:
        if count%30 == 0:
            cv.imwrite(f'{turbine}_{step}_img{img_num}.png', frame_read.frame)
            img_num += 1
        count += 1
        video.write(frame_read.frame)
        sleep(1/60)
        record -= 1
    video.release()
