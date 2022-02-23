import cv2 as cv
import movement as mv
from time import sleep
from djitellopy import Tello
import os

path = "Default"
def mission0(location, drone, mission, turbine):
    '''Film directly in front of the fan after rising 20cm'''
    parent_directory = os.getcwd()
    directory = turbine
    path = os.path.join(parent_directory, directory)
    os.mkdir(path)
    os.chdir(path)
    if mission[0] == 1:
        mv.move(location, drone, up=20)
        recordVideo(drone, turbine, 0)
    mission[0] = 0
    if mission[1] == 1:
        mission1(drone, location, mission, turbine)
    elif mission[3] == 1:
        mission3(drone, location, mission, turbine)
    elif mission[2] == 1:
        mission2(drone, location, mission, turbine)
    else:
        mv.move(location, drone, down=20)

def mission1(drone, location, mission, turbine):
    '''Film the fan on the right side after rising 20cm'''
    drone.move_right(50)
    drone.move_forward(50)
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
        mv.move(location, drone, down=20)
        drone.move_left(50)
        drone.move_forward(50)
        mv.move(location, drone, cw=90)

def mission2(drone, location, mission, turbine, recent_mission = 0):
    '''Film the back side of the fan  after rising 20cm'''
    if location[3] == 0:
        mv.move(location, drone, up=50)
    if recent_mission == 0:
        drone.move_right(50)
        drone.move_forward(100)
        drone.move_left(50)
        mv.move(location, drone, ccw=180)
    elif recent_mission == 1:
        drone.move_right(50)
        drone.move_forward(50)
        mv.move(location, drone, ccw=90)
    recordVideo(drone, turbine, 2)
    mission[2] = 0
    if mission[3] == 1:
        mission3(drone, location, mission, turbine, recent_mission = 2)
    else:
        drone.move_right(50)
        drone.move_forward(100)
        drone.move_left(50)
        location = mv.move(location, drone, down=20)

def mission3(drone, location, mission, turbine, recent_mission = 0):
    '''Film the left side of the fan after going up 20cm'''
    if location[3] == 0:
        mv.move(location, drone, up=20)
    if recent_mission == 0:
        drone.move_left(50)
        drone.move_forward(100)
        mv.move(location, drone, cw=90)
    elif recent_mission == 1:
        drone.move_left(50)
        drone.move_forward(100)
        drone.move_right(50)
        mv.move(location, drone, ccw=180)
    elif recent_mission == 2:
        drone.move_right(50)
        drone.move_forward(50)
        mv.move(location, drone, ccw=90)
    recordVideo(drone, turbine, 3)
    drone.move_down(20)
    mission[3] = 0
    if mission[2] == 1:
        mission2(drone, location, mission, turbine)
    else:
        drone.move_right(50)
        drone.move_forward(50)
        mv.move(location, drone, ccw=90)
        mv.move(location, drone, down=20)

def recordVideo(drone, turbine, step):
    frame_read = drone.get_frame_read()
    height, width, _ = frame_read.frame.shape
    record = 300
    video = cv.VideoWriter(f'{turbine}_{step}.mp4', cv.VideoWriter_fourcc(*'XVID'), 30, (width, height))
    while record > 0:
        video.write(frame_read.frame)
        sleep(1/60)
        record -= 1
    video.release()

if __name__ == "__main__":
    drone = Tello()
    drone.connect()
    sleep (0.3)
    location = [0,0,0,0]
    drone.streamon()
    #drone.takeoff()
    #sleep(2)
    mission = [1, 0, 0, 0]
    location = mission0(location, drone, mission, 'Turbine 1')
