# Tello-Drone-Project
Current python files related to drone flight path planning and object detection for the College of Engineering and Technology at Utah Valley University<br />

![alt text](https://github.com/BrandenPinney/Tello-Drone-Project/blob/main/UVU_engineering.png "UVU Logo")

This work is supported by the Office of the Commissioner of Utah System of Higher Education (USHE)-Deep Technology Initiative Grant 20210016UT

![alt text](https://github.com/BrandenPinney/Tello-Drone-Project/blob/main/USHE-logo.png "USHE Logo")

## Table of Contents
1. [Installations](#Installations)
2. [Usage](#Usage)
3. [movement.py Instructions](#movement.py-Instructions)
    1. [User Commands](#User-Commands)
        1. [Initialization](#Initialization)
        2. [move()](#move())
        3. [curve()](#curve())
        4. [go_to()](#go_to())
        5. [video()](#video())
        6. [get_location()](#get_location())
        7. [get_x_location()](#get_x_location())
        8. [get_y_location()](#get_y_location())
        9. [get_z_location()](#get_z_location())
        10. [get_angle()](#get_angle())
        11. [get_drone()](#get_drone())
        12. [get_video()](#get_video())
    2. [Other Commands](#Other-Commands)
        1. [get_turbine_locations()](#get_turbine_locations())
        2. [append_turbine_locations()](#append_turbine_locations())
        3. [target_angle()](#target_angle())
5. [output.py Instructions](#output.py-Instructions)
    1. [stop_haar()](#stop_haar())
    2. [start_haar()](#start_haar())
    3. [stop_qr()](#stop_qr())
    4. [start_qr()](#start_qr())
    5. [stop_image()](#stop_image())

## Intallations <a name="Installations"></a>
This application uses:<br />
Python 3.10.4<br />
OpenCV 4.5.5<br />
djitellopy
```
pip install opencv-python
pip install djitellopy
```
See [djitellopy.readme.md](https://github.com/damiafuentes/DJITelloPy/blob/master/README.md) for more information on djitellopy.<br />
See [opencv-python 4.5.5.64](https://pypi.org/project/opencv-python/) for more information on OpenCV.<br />
See [Tello SDK 2.0 User Guide](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf) for more information on the Tello EDU SDK.<br />

## Usage <a name="Usage"></a>
### Running the current setup
To run the program in it's current configuration, launch area_exploration.py in your IDE of choice. 
The current itteration of the program will begin a snaking exploration pattern while looking for black metallic fans.
If a fan is detected, the drone will fly up to the fan, lower 80cm and scan the QR code.
The drone will then fly the pre-selected mission which is default to filming the front of the fan for 10 seconds in mp4 format.
After finding the correct target and completing the mission, the drone will fly back to the starting point and land itself.
<br />
<br />
## movement.py Instructions <a name="movement.py-Instructions"></a>
The following instructions are for the use of our movement.py class for other automated drone purposes:
### Initialization <a name="Initialization"></a>
To initialize the drone, the movment class in movement.py will handle most of the required intitialization steps.<br />
The intialization has 2 optional parameters, the altitude adjustment of the drone after takeoff (default to 40cm for a total of 90cm above the ground), and a boolean stream variable that will control if the user wants the live video feed (default to True, see the section on output_video.py for more details).<br />
Import movement.py into your python code and assign a varaible to the class as follows:
```
import movement as mv
# Takeoff with default settings

drone = mv.movement()
```
OR
```
import movement as mv
# Rise to 100cm on takeoff and no video stream

drone = mv.movement(100, False)
```
The user must first power on the Tello EDU and connect to the drone via wi-fi. 
Instantiating the drone this way will begin the live video stream, launch the drone, raise the altitude of the drone (set to a default of 90cm, to choose a different height pass in an integer value), and prepare the drone for further commands.<br />
The drone's initial location is also initialized at the cartesian point (0,0), the current angle of the drone being 0°, and the altitude after takeoff using one of the drone's onboard atitude sensors.

---------------------------------------------------------------------------------------------------------------------
### Commands from the user: <a name="User-Commands"></a>
The following is a comprehensive list of functions available to the user and their meaning:
| Function       | Inputs                                    | Result                                               |
| ---------------|:----------------------------------------- | :----------------------------------------------------|
| land           | none                                      | Lands the drone                                      |
| move           | fwd, back, up, down, left, right, ccw, cw | Moves the drone and tracks location                  |
| curve          | radius, left_right                        | Curves the drone left or right 90°                   |
| go_to          | targetx, targety, ending_angle            | Moves the drone to the chosen coordinates            |
| video          | none                                      | Starts the Livestream                                |
| get_location   | none                                      | Returns a list of the x, y, z coordinates and angle  |
| get_x_location | none                                      | Returns the drone's current x coordinate             |
| get_y_location | none                                      | Returns the drone's current y coordinate             |
| get_z_location | none                                      | Returns the drone's current z coordinate             |
| get_angle      | none                                      | Returns the drone's current angle                    |
| get_drone      | none                                      | Returns the class interfacing the drone with the SDK |
| get_video      | none                                      | Returns the variable controlliing the livestream     |

#### land() <a name="land()"></a>
Lands the drone after turning off the video stream and printing a message directing the user to the directory any collected data has been stored in. The action then exits the program.
```
drone.land()
```
#### move() <a name="move()"></a>
Moves the drone according to the direction or angle given by the user. Movement must been in cm between 20cm and 500cm. The angle movement is in degrees from 0 to 359. Keywords for movement are: fwd(forward), back, up, down, left, right, ccw(counter clock-wise rotation), cw(clock-wise rotation).
```
# Move forward 50cm

drone.move(fwd=50)
```
#### curve() <a name="curve()"></a>
Rotates the drone 90° in a curved path with a given radius and left/right flag to curve to the left or right. The default is 50cm curve to the left. This function is still in work and may not always work as expected.
```
# Curve right 100cm
# left_right=1 sets the curve right, default is left

drone.curve(radius=100, left_right=1) 
```
#### go_to() <a name="go_to()"></a>
Tells the drone to go to a specific set of cartesian coordinates with a desired ending angle. The drone's coordinates after movement are printed for the user. 
```
# Go to (100,150) with an ending angle of 141°

drone.go_to(100, 150, 141)
```
#### video() <a name="video()"></a>
Initializes a live video stream for the user. This function is called during the intialization process by default. If the user doesn't want a video stream, the stream parameter during initialization should be set to false. See the section on using [output.py](#output.py-Instructions) for more information.
```
# Start the livestream

drone.video()
```
#### get_location() <a name="get_location()"></a>
Returns the full list of the drone's location formatted as [x_location, y_location, z_location, angle]. 
```
location = drone.get_location()
```
#### get_x_location() <a name="get_x_location()"></a>
Returns only the x location of the drone.
```
x_location = drone.get_x_location()
```
#### get_y_location() <a name="get_y_location()"></a>
Returns only the y location of the drone.
```
y_location = drone.get_y_location()
```
#### get_z_location() <a name="get_z_location()"></a>
Returns only the z location of the drone.
```
z_location = drone.get_z_location()
```
#### get_angle() <a name="get_angle()"></a>
Returns only the angle of the drone.
```
angle = drone.get_angle()
```
#### get_drone() <a name="get_drone()"></a>
Returns the class controlling the SDK for the drone should the user need it.
```
drone = drone.get_drone()
```
#### get_video() <a name="get_video()"></a>
Returns the variable controlling the video stream for the drone.
```
video = drone.get_video()
```
------------------------------------------------------------------------------------------------------------------------------
### Commands not typically called by the user <a name="Other-Commands"></a>
The following list of commands are functions that are not typically called by the user, but can be should the user decide it is necessary:
| Function                 | Inputs                                    | Result                                               |
| ------------------------ |:----------------------------------------- | :--------------------------------------------------- |
| get_turbine_locations    | none                                      | Returns a list of the turbines found by the drone    |
| append_turbine_locations | QR                                        | Adds a new turbine to the turbine_locations list     |
| target_angle             | return_angle, x, y, quadrant              | Rotates the drone shortest distance to the target    |

#### get_turbine_locations() <a name="get_turbine_locations()"></a>
Returns a list of the gathered locations and no-fly zones of turbines while the drone was in flight.
```
turbines = drone.get_turbine_locations()
```
#### append_turbine_locations() <a name="append_turbine_locations()"></a>
Adds another turbine to the list according to its detected location and QR code data.
```
# QR contains the string data from decrypting a QR code

drone.append_turbine_locations(QR)
```
#### target_angle() <a name="target_angle()"></a>
Takes the angle to the drone from the target location, the x and y coordinates of the target location, and the quadrant of the drone relative to the target location acting as the origin.
The drone will then rotate the shortest posible distance to center on the target location. It is reccomended that the user instead use the go_to() or move() functions to rotate the drone instead of using target_angle().
```
# If the drone were located at a 45 degree angle from the origin and was trying to return

drone.target_angle(45, 0, 0, 1)
```

## output.py Instructions <a name="output.py-Instructions"></a>
A livestream video is created during initialization of the drone unless manually set to False. The algorithm creates the Livestream class found in output.py The video stream can be started later by using the [video()](#video()) command found under the [movement.py instructions](#movement.py-Instructions). The video stream is initialized to operate the Haar cascade detection algorithm and can also scan for QR codes in the frame. The easiest was to currently interact with the Livestream class in output.py is to use the [get_video()](#get_video()) command seen in the [movement.py instructions](#movement.py-Instructions) which returns the Livestream class. Use the commands below to interact with the Livestream:

| Function        | Inputs     | Result                            |
| --------------- |:---------- | :-------------------------------- |
| stop_haar       | none       | Stops the Haar cascade detection  |
| start_haar      | none       | Starts the Haar cascade detection |
| stop_qr         | none       | Stops the QR code detection       |
| start_qr        | none       | Starts the QR code detection      |
| stop_image      | none       | Stops the image output            |

#### stop_haar() <a name="stop_haar()"></a>
Stops the Haar cascade detection algorithm. The detection is set to currently detect black metallic fans. If this is turned off, QR code detection immediately starts. Both the Haar cascade and QR detection must be turned off for a regular livestream.
```
# Assign the class to a variable
video = drone.get_video()

# Disable the Haar cascade
video.stop_haar()
```

#### start_haar() <a name="start_haar()"></a>
Starts the Haar cascade detection algorithm. It will turn on immediately from a regular livestream, but if the QR detection is on it will have to be deactivated to restart the Haar cascade. The Haar cascade is on by default after starting the livestream.
```
# Assign the class to a variable
video = drone.get_video()

# Start the Haar cascade
video.start_haar()
```

#### stop_qr() <a name="stop_qr()"></a>
Stops the QR code detection. Stopping both the Haar cascade and QR detection will result in a regular livestream video. _It is reccomended to use 64-bit python when using QR code detection._
```
# Assign the class to a variable
video = drone.get_video()

# Disable the QR detection
video.stop_qr()
```

#### start_qr() <a name="start_qr()"></a>
Starts the QR detection algorithm. It will turn on immediately from a regular livestream, but if the Haar cascade detection is on it will have to be deactivated to restart the QR detection. The QR detection is on by default after starting the livestream and will show after the Haar cascade is deactivated.
```
# Assign the class to a variable
video = drone.get_video()

# Start the QR detection
video.start_qr()
```

#### stop_image() <a name="stop_image()"></a>
Turns off the livestream image. The Haar cascade and QR detection must also be turned off to fully stop the livestream. This is done automatically when [land()](#land()) is used from [movement.py](#movement.py-Instructions).
```
# Assign the class to a variable
video = drone.get_video()

# Disable the QR detection
video.stop_qr()
# Disable the Haar cascade
video.stop_haar()
# Stop the livestream
video.stop_image()
```
