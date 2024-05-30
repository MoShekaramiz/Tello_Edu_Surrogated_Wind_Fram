'''Tutorial demonstration for connecting to the drone, outputing the live camera feed, and sending movement commands
by Angel Rodriguez 2023
DJITelloPy API Reference: https://djitellopy.readthedocs.io/en/latest/tello/'''


# Importing required libraries
from djitellopy import Tello
import cv2 as cv
import threading

def output_video(drone):
    '''Output live video feed of the drone to user'''    
    while True: # Infinite while loop to output the live video feed indefinetly 
            frame_read = drone.get_frame_read()
            img = frame_read.frame
            # Display output window showing the drone's camera frames
            cv.imshow("Output", img)
            cv.waitKey(1)

# Creating a Tello object
tello = Tello()

# Connecting to Tello Drone
tello.connect()

# Display battery level
battery = tello.get_battery()
print(f'>>>>>>>>>> DRONE BATTERY: {battery}')
if battery < 20:
      print('>>>>>>>>>> CHANGE DRONE BATTERY')

# Starting the video stream
tello.streamon()

# Show the user the video stream using threading
video_thread = threading.Thread(target=output_video, kwargs={'drone': tello})
video_thread.start()

# Taking off
tello.takeoff()

# Flying forward
tello.move_forward(50)

# Flying backward
tello.move_back(50)

# Going up
tello.move_up(50)

# Going down
tello.move_down(50)

# Flipping
tello.flip('f')

# Landing
tello.land()

# Closing the video stream
tello.streamoff()

# Disconnecting from Tello Drone
tello.end()