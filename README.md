# Tello-Drone-Project
Current python files related to drone flight path planning and object detection

## Intallations
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

## Usage
### Running the current setup
To run the program in it's current configuration, launch area_exploration.py in your IDE of choice. 
The current itteration of the program will begin a snaking exploration pattern while looking for black metallic fans.
If a fan is detected, the drone will fly up to the fan, lower 80cm and scan the QR code.
The drone will then fly the pre-selected mission which is default to filming the front of the fan for 10 seconds in mp4 format.
After finding the correct target and completing the mission, the drone will fly back to the starting point and land itself.
<br />
<br />
## movement.py Instructions
The following instructions are for the use of our movement.py class for other automated drone purposes:
### Initialization
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
The drone's initial location is also initialized at the cartesian point (0,0), the current angle of the drone being 0 degrees, and the altitude after takeoff using one of the drone's onboard atitude sensors.

---------------------------------------------------------------------------------------------------------------------
### Commands from the user:
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

#### land()
Lands the drone after turning off the video stream and printing a message directing the user to the directory any collected data has been stored in.
```
drone.land()
```
#### move()
Moves the drone according to the direction or angle given by the user. Movement must been in cm between 20cm and 500cm. The angle movement is in degrees from 0 to 359. Keywords for movement are: fwd(forward), back, up, down, left, right, ccw(counter clock-wise rotation), cw(clock-wise rotation).
```
# move forward 50cm

drone.move(fwd=50)
```
#### curve()
Rotates the drone 90° in a curved path with a given radius and left/right flag to curve to the left or right. The default is 50cm curve to the left. This function is still in work and may not always work as expected.
```
# curve right 100cm
# left_right=1 sets the curve right, default is left

drone.curve(radius=100, left_right=1) 
```
#### go_to()
Tells the drone to go to a specific set of cartesian coordinates with a desired ending angle. The drone's coordinates after movement are printed for the user. 
```
# go to (100,150) with an ending angle of 141 degrees

drone.go_to(100, 150, 141)
```
#### video()
Initializes a live video stream for the user. This function is called during the intialization process by default. If the user doesn't want a video stream, the stream parameter during initialization should be set to false. See the section on using output.py for more information.
```
# Start the livestream

drone.video()
```
#### get_location()
Returns the full list of the drone's location formatted as [x_location, y_location, z_location, angle]. 
```
location = drone.get_location()
```
#### get_x_location()
Returns only the x location of the drone.
```
x_location = drone.get_x_location()
```
#### get_y_location()
Returns only the y location of the drone.
```
y_location = drone.get_y_location()
```
#### get_z_location()
Returns only the z location of the drone.
```
z_location = drone.get_z_location()
```
#### get_angle()
Returns only the angle of the drone.
```
angle = drone.get_angle()
```
#### get_drone()
Returns the class controlling the SDK for the drone should the user need it.
```
drone = drone.get_drone()
```
#### get_video()
Returns the variable controlling the video stream for the drone.
```
video = drone.get_video()
```
------------------------------------------------------------------------------------------------------------------------------
### Commands not typically called by the user
The following list of commands are functions that are not typically called by the user, but can be should the user decide it is necessary:
| Function                 | Inputs                                    | Result                                               |
| ------------------------ |:----------------------------------------- | :--------------------------------------------------- |
| get_turbine_locations    | none                                      | Returns a list of the turbines found by the drone    |
| append_turbine_locations | QR                                        | Adds a new turbine to the turbine_locations list     |
| target_angle             | return_angle, x, y, quadrant              | Rotates the drone shortest distance to the target    |

#### get_turbine_locations()
Returns a list of the gathered locations and no-fly zones of turbines while the drone was in flight.
```
turbines = drone.get_turbine_locations()
```
#### append_turbine_locations()
Adds another turbine to the list according to its detected location and QR code data.
```
# QR contains the string data from decrypting a QR code

drone.append_turbine_locations(QR)
```
#### target_angle()
Takes the angle to the drone from the target location, the x and y coordinates of the target location, and the quadrant of the drone relative to the target location acting as the origin.
The drone will then rotate the shortest posible distance to center on the target location. It is reccomended that the user instead use the go_to() or move() functions to rotate the drone instead of using target_angle().
```
# if the drone were located at a 45 degree angle from the origin and was trying to return

drone.target_angle(45, 0, 0, 1)
```

## output.py Instructions
A livestream video is created during initialization of the drone unless manually set to False. The algorithm creates the Livestream class found in output.py The video stream can be started later by using the video() command found under the movement.py instructions. The video stream is initialized to operate the Haar cascade detection algorithm and can also scan for QR codes in the frame. The easiest was to currently interact with the Livestream class in output.py is to use the get_video() command seen in the movment.py instructions which returns the Livestream class. Use the commands below to interact with the Livestream:
