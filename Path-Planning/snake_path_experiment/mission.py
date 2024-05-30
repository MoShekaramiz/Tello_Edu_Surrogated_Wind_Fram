import cv2 as cv
import shutil
from time import sleep
import os

path = "Default"
def mission0(mv, mission, turbine):
    '''Film directly in front of the fan'''
    parent_directory = os.getcwd()
    directory = turbine
    path = os.path.join(parent_directory, directory)
    if os.path.isdir(directory):
        shutil.rmtree(directory)
    os.mkdir(path)
    os.chdir(path)
    if mission[0] == 1:
        recordVideo(mv, turbine, 0)
    mission[0] = 0
    if mission[1] == 1:
        mission1(mv, mission, turbine)
    elif mission[3] == 1:
        mission3(mv, mission, turbine)
    elif mission[2] == 1:
        mission2(mv, mission, turbine)
    else:
        mv.move(up=20)

def mission1(mv, mission, turbine):
    '''Film the fan on the right side'''
    mv.move(right=60)
    mv.move(fwd=60)
    mv.move(ccw=90)
    if mv.get_z_location == 0:
        mv.move(up=20)
    recordVideo(mv, turbine, 1)
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
    if mv.get_z_location == 0:
        mv.move(up=50)
    if recent_mission == 0:
        mv.move(right=60)
        mv.move(fwd=110)
        mv.move(left=60)
        mv.move(ccw=180)
    elif recent_mission == 1:
        mv.move(right=50)
        mv.move(fwd=60)
        mv.move(ccw=90)
    recordVideo(mv, turbine, 2)
    mission[2] = 0
    if mission[3] == 1:
        mission3(mv, mission, turbine, recent_mission = 2)
    else:
        mv.move(right=60)
        mv.move(fwd=110)
        mv.move(left=60)
        mv.move(cw = 180)

def mission3(mv, mission, turbine, recent_mission = 0):
    '''Film the left side of the fan after going up 20cm'''
    if mv.get_z_location == 0:
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
    recordVideo(mv, turbine, 3)
    mission[3] = 0
    if mission[2] == 1:
        mission2(mv, mission, turbine)
    else:
        mv.move(right=60)
        mv.move(fwd=60)
        mv.move(ccw=90)

def recordVideo(mv, turbine, step):
    drone = mv.get_drone()
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
