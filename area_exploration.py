from djitellopy import Tello
from output_video import LiveFeed
import cv2 as cv
from time import sleep
import movement as mv
from traversal_image_interface import trackObject
import haar_cascade as hc
import math
import matplotlib.pyplot as plt
from matplotlib.path import Path
import numpy as np
import matplotlib.patches as patches
from shapely.geometry import Polygon
import time


fbRange = [32000, 52000] # preset parameter for detected image boundary size
w, h = 720, 480 # display size of the screen
location = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
turbine_locations = [] # List containing the locations of found turbines
detected_object = 0 # A flag to determine if the camera detected an object in the previous 5 frames


def snake_exploration(drone, location, flyZone, searchWidth, moveIncr, display=False, curve=0):
    '''The drone explores a snaking.
    plt.show() needs to be called after if you want to display a plot of the path
    
    Input:
        drone (Tello) : drone variable
        location : [x, y, angle] Current coordinates and angle of drone
        flyZone : [xMin, yMin, xMax, yMax] four vertices representing area to be explored
        searchWidth (int): distance (cm) between traversals
        moveIncr (int) : distance (cm) for drone to move before checking the area
        distance between traversals
        display (bool, optional) : default = false. If true a graph of the projected path will be displayed
    Output:
        location : [x, y, angle] updated coordinates and angle of drone
        totalDist (int) : total distance (cm) traveled by the drone '''

    xMin = flyZone[0] 
    yMin = flyZone[2] 
    xMax = flyZone[1]  
    yMax = flyZone[3] 

    # Ensure that the drone is in the lower right corner and rotated correctly, otherwise quit
    if round(location[0]) != xMin or round(location[1]) != yMin or location[2] != 0:
        location = mv.go_to(location, drone, turbine_locations, xMin, yMin, 0)

    totalDist = 0                   # tracks distance traveled
    turns = 0                       # track # turns
    xgraph = []
    ygraph = []
    #maxMove = 30                #how much to move at a time;
    #shortLen = 30               #how far to move on short edge;

    xDist = xMax-xMin               # distance needed for next horizontal traverse
    yDist = yMax-yMin               # distance needed for next vertical traverse
    turnDir = 1                     # 0 for clockwise 1 for counter clockwise
    XMovesBeforeTurn = int(xDist/moveIncr)
    Ytraversals = int(yDist/searchWidth)
    YMovesBeforeTurn = int(searchWidth/moveIncr)
    y=0
    while True:
        # CHECK CAMERA
        location= check_camera(drone, location)
        ###################
        if display:
            xgraph.append(location[0])    
            ygraph.append(location[1])
        for i in range(XMovesBeforeTurn):
            targetx = location[0] + moveIncr * math.cos(math.radians(location[2]))
            targety = location[1] + moveIncr * math.sin(math.radians(location[2]))
            if location[2] < 90 or location[2] > 270:
                location = mv.go_to(location, drone, turbine_locations, targetx, targety, 0)
            else:
                location = mv.go_to(location, drone, turbine_locations, targetx, targety, 180)
            #mv.move(location, drone, fwd=moveIncr)
            # CHECK CAMERA
            location= check_camera(drone, location)
            ###################
            totalDist += moveIncr
            if display:
                xgraph.append(location[0])    
                ygraph.append(location[1])
        if (xDist % moveIncr) > 20:
            targetx = location[0] + (xDist % moveIncr) * math.cos(math.radians(location[2]))
            targety = location[1] + (xDist % moveIncr) * math.sin(math.radians(location[2]))
            if location[2] < 90 or location[2] > 270:
                location = mv.go_to(location, drone, turbine_locations, targetx, targety, 0)
            else:
                location = mv.go_to(location, drone, turbine_locations, targetx, targety, 180)
            #mv.move(location, drone, fwd=xDist%moveIncr)
            totalDist += xDist % moveIncr
            # CHECK CAMERA
            location= check_camera(drone, location)
            ###################
            if display:
                xgraph.append(location[0])    
                ygraph.append(location[1])
        if y >= Ytraversals:
            break
        if turnDir:
            location = mv.move(location, drone, ccw=90)
            turns += 1
        else:
            location = mv.move(location, drone, cw=90)
            turns += 1
        # CHECK CAMERA
        location= check_camera(drone, location)
        ###################
        for i in range(YMovesBeforeTurn):
            targetx = location[0] + moveIncr * math.cos(math.radians(location[2]))
            targety = location[1] + moveIncr * math.sin(math.radians(location[2]))
            if location[2] > 0 and location[2] < 180:
                location = mv.go_to(location, drone, turbine_locations, targetx, targety, 90)
            else:
                location = mv.go_to(location, drone, turbine_locations, targetx, targety, 270)
            #mv.move(location, drone, fwd=moveIncr)
            # CHECK CAMERA
            location= check_camera(drone, location)
            ###################
            totalDist += moveIncr
            if display:
                xgraph.append(location[0])    
                ygraph.append(location[1])
        if (searchWidth % moveIncr) > 20:
            targetx = location[0] + (searchWidth % moveIncr) * math.cos(math.radians(location[2]))
            targety = location[1] + (searchWidth % moveIncr) * math.sin(math.radians(location[2]))
            if location[2] > 0 and location[2] < 180:
                location = mv.go_to(location, drone, turbine_locations, targetx, targety, 90)
            else:
                location = mv.go_to(location, drone, turbine_locations, targetx, targety, 270)
            #mv.move(location, drone, fwd=searchWidth%moveIncr)
            totalDist += searchWidth % moveIncr
            # CHECK CAMERA
            if display:
                xgraph.append(location[0])    
                ygraph.append(location[1])
        if turnDir:
            location = mv.move(location, drone, ccw=90)
            turns += 1
        else:
            location = mv.move(location, drone, cw=90)
            turns += 1
        y += 1
        # CHECK CAMERA
        location= check_camera(drone, location)
        ###################
        turnDir = (turnDir + 1) % 2 
    
    # plot the path
    if display:
        xgraph.append(location[0])    
        ygraph.append(location[1])
        xgraph.append(xgraph[0])
        ygraph.append(ygraph[0])
        plt.figure()
        plt.plot(xgraph, ygraph, '-kx', lw=2, label='backforth')
        
    
    # return to original location and track the distance
    location = mv.go_to(location, drone, turbine_locations, 0, 0, 0)
    totalDist += int(math.sqrt(location[0]**2 + location[1]**2))

    return location, totalDist


def spiral_exploration(drone, location, flyZone, searchWidth, moveIncr, display=False):
    #poly_bound = Polygon(boundary)    
    #cp = poly_bound.centroid
    #poly_bound.exterior.coords
    '''The drone explores a spiral_exploration path. This function needs to be modified to check the camera itself
    plt.show() needs to be called after if you want to display a plot of the path
     
    Input:
        drone (Tello) : drone variable
        location : [x, y, angle] Current coordinates and angle of drone
        flyZone : [xMin, yMin, xMax, yMax] four vertices representing area to be explored
        searchWidth (int): distance (cm) between traversals
        moveIncr (int) : distance (cm) for drone to move before checking the area
        display (bool, optional) : default = false. If true a graph of the projected path will be displayed
    Output:
        location : [x, y, angle] updated coordinates and angle of drone
        totalDist (int) : total distance (cm) traveled by the drone '''

    xMin = flyZone[0]
    yMin = flyZone[2] 
    xMax = flyZone[1]  
    yMax = flyZone[3] 
    totalDist = 0        #keeps track of path distance
    turns = 0            #track # of turnsre

    # Go to correct location if starting location is not in the lower right corner
    if round(location[0]) != xMin or round(location[1]) != yMin or location[2] != 0:
        location = mv.go_to(location, drone, turbine_locations, xMin, yMin, 0)

    xDist = xMax-xMin               # distance needed for next horizontal traverse
    yDist = yMax-yMin               # distance needed for next vertical traverse
    x_traversed = 0                          # x_traversed
    y_traversed = 0                          # y_traversed
    ygraph = []
    xgraph = []
    f = 0                           # zero until the drone has gone in one straight line
    while True: # Drone cannot travel less than 20 cm
        if display:
                xgraph.append(location[0])    
                ygraph.append(location[1])
        # travel the yDist 
        if round(location[2])==90 or round(location[2])==270:
            if yDist < 20:
                break
            ##### CHECK CAMERA
            location= check_camera(drone, location)
            ###################
            #sleep(0.2)    
            if y_traversed + moveIncr > yDist:
                if yDist-y_traversed>=20:
                    target_x = location[0] + (yDist - y_traversed) * math.cos(math.radians(location[2]))
                    target_y = location[1] + (yDist - y_traversed) * math.sin(math.radians(location[2]))
                    location = mv.go_to(location, drone, turbine_locations, target_x, target_y, location[2])
                    totalDist += yDist - y_traversed
                y_traversed = 0
                if f != 0:                  # decrease distance to travel each time after first line
                    yDist -= searchWidth
                location =mv.move(location,drone, ccw=90)
                turns += 1
                f = 1  
            else:
                y_traversed += moveIncr
                target_x = location[0] + moveIncr * math.cos(math.radians(location[2]))
                target_y = location[1] + moveIncr * math.sin(math.radians(location[2]))
                location = mv.go_to(location, drone, turbine_locations, target_x, target_y, location[2])
                totalDist += moveIncr
        # Travel the xDist
        elif round(location[2])==0 or round(location[2])==180:
            if xDist < 20:
                break
            ##### CHECK CAMERA 
            location= check_camera(drone, location)
            ###################
            if x_traversed + moveIncr > xDist:
                if xDist-x_traversed>=20:
                    target_x = location[0] + (xDist - x_traversed) * math.cos(math.radians(location[2]))
                    target_y = location[1] + (xDist - x_traversed) * math.sin(math.radians(location[2]))
                    location = mv.go_to(location, drone, turbine_locations, target_x, target_y, location[2])
                    totalDist += xDist - x_traversed
                x_traversed = 0
                if f != 0:
                    xDist -= searchWidth
                location = mv.move(location, drone, ccw=90)
                turns +=1
                f = 1
            else:
                x_traversed += moveIncr
                target_x = location[0] + moveIncr * math.cos(math.radians(location[2]))
                target_y = location[1] + moveIncr * math.sin(math.radians(location[2]))
                location = mv.go_to(location, drone, turbine_locations, target_x, target_y, location[2])
                totalDist += moveIncr

    # plot the path
    if display:
        xgraph.append(location[0])    
        ygraph.append(location[1])
        xgraph.append(xgraph[0])
        ygraph.append(ygraph[0])
        f = plt.figure()
        plt.plot(xgraph, ygraph, '-kx', lw=2, label='spiral_explorationPath')
        
    
    # return to original location and track the distance
    mv.go_to(location, drone, turbine_locations, 0, 0, 0)
    totalDist += int(math.sqrt(location[0]**2 + location[1]**2))

    return location, totalDist

def check_camera(drone, location):
    frame = drone.get_frame_read()
    sleep(0.2)
    img = frame.frame
    img = cv.resize(img, (w, h))
    img, info = hc.findTurbine(img)
    x, y = info[0]
    if x == 0:
        for i in range(4):
            frame = drone.get_frame_read()
            sleep(0.2)
            img = frame.frame
            img = cv.resize(img, (w, h))
            img, info = hc.findTurbine(img)
            x, y = info[0]
            if x != 0:
                break
    location = trackObject(drone, info, location, turbines, video, [location[0], location[1], location[2]])
    return location
    
def approx_cell_decomp(obstacleList, boundary):
    '''input: array of obstactles, array of boundary coordinates
    Decomposes the area into cells 
    output: waypoints that are in the safe to fly zone.'''
    boundary = np.append(boundary, [boundary[0]], 0)
    w = 30
    xmin = np.amin(boundary[:,0]) + (w/2)
    xmax = np.amax(boundary[:,0]) - (w/2)
    ymin = np.amin(boundary[:,1]) + (w/2)
    ymax = np.amax(boundary[:,1]) - (w/2)
    cells_x = np.arange(xmin, xmax, w)
    cells_y = np.arange(ymin, ymax, w)
    cells = []
    waypoints = []

    for x in cells_x:
        for y in cells_y:
            cells =  np.append(cells, [x,y])
    cells = cells.reshape(-1,2)
    fig, ax = plt.subplots()
    
    for c in cells:
        xmin = c[0] - w/2
        xmax = c[0] + w/2
        ymin = c[1] - w/2
        ymax = c[1] + w/2
        pts = [[xmin, ymin],[xmax,ymin],[xmax, ymax],[xmin, ymax]]
        poly_cell = Polygon(pts)
        p_cell = Path(pts)
        isInside = []
        for ob in obstacleList:
            # This section is used for plotting for now
            codes = [Path.MOVETO]
            for i in range(len(ob.coords)-1):
                codes.append(Path.LINETO)
            codes.append(Path.CLOSEPOLY)
            poly_obst = Polygon(ob.coords)
            ob.coords = np.append(ob.coords, [ob.coords[0]], 0)
            p_obst = Path(ob.coords, codes)
            patch = patches.PathPatch(p_obst, facecolor='blue')
            ax.add_patch(patch)
            #isInside = np.append(isInside, p_obst.contains_points(pts))
            isInside = np.append(isInside, poly_cell.intersects(poly_obst))
        if not(isInside.any()):
            waypoints = np.append(waypoints, c)
            waypoints = waypoints.reshape(-1,2)
    plt.plot(boundary[:,0], boundary[:,1], color='black', lw=3, label='boundary')
    plt.scatter(cells[:,0], cells[:,1], marker='o', color='red', label='cells containing objects')
    plt.scatter(waypoints[:,0], waypoints[:,1], marker='o', color='green', label='waypoints')
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, 0.95))
    plt.show()
    return waypoints
         
def test_cellDecomp():
    '''Example tests of approximate cell decomposition'''
    boundary =  [[0,0],[0,305],[305,305],[305,0]]
    #boundary = [[-270, 0], [-270,270], [0, 270], [0,0]] 

    obstacles = np.array([obstacle('wind_turbine1', [[50,50],[150,50],[100,100],[75,120],[50,100]]),
                          obstacle('wind_turbine1', [[150,200],[300,200],[300,300],[200,250]])])
    approx_cell_decomp(drone, location, obstacles, boundary)

    obstacles = np.array([obstacle('wind_turbine1', [[50,50],[30,130],[80,120],[120,80],[60,40]]),
                          obstacle('wind_turbine1', [[150,200],[300,175],[275,300],[200,250]]),
                          obstacle('wind_tubine3', [[270, 78], [250,20], [280,50]])])
    approx_cell_decomp(obstacles, boundary)

class obstacle: 
    def __init__(self, name, coords): 
        self.name = name 
        self.coords = coords

 

if __name__ == "__main__":
    drone = Tello()
    turbines = {"WindTurbine_2": [0, 0, 0, 0]}
     # COMMENT OUT SECTION IF TESTING W/O PHYSICAL DRONE
    drone.connect()
    sleep(0.5)
    BatteryStart = drone.get_battery()
    print("Current battery remaining: ", BatteryStart)
    sleep(0.3)
    drone.streamon()
    sleep(0.5)
    video = LiveFeed(drone)
    video.start()
    drone.takeoff()
    sleep(1)
    # END OF SECTION TO COMMENT OUT
    location = mv.move(location, drone, up=40)
    bounds = [0, 221, 0, 221]#161
    #bounds = [0, 328, 0, 324]    #actual size of path in drone cage
    start_time = time.time()
    searchWidth = 50
    moveIncr = 75
    [location,dist] = spiral_exploration(drone, location, bounds, searchWidth, moveIncr)
    #plt.xlabel('x (cm)')
    #plt.ylabel('y (cm)')
    #plt.show()
    end_time = time.time()
    print('--- ', round(end_time - start_time, 2), ' seconds ---')
    BatteryEnd = drone.get_battery()
    print("Current battery remaining: ", BatteryEnd)
    print("Total battery used: ", BatteryStart - BatteryEnd)
    drone.land()
    quit()

    # #location = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
    # #bounds = [-328,0, 0, 324]
    # #distBF = testBF(location, bounds, display=False)
    # print(distBF2, distspiral_exploration)
    # plt.show()
    # #test_cellDecomp(location)

    # GRAPHS DISTANCES FOR DIFFERENT SIZES OF SEARCH AREA

    # y_len = 100
    # area = []
    # dspiral_exploration = []
    # dBF = []
    # tspiral_exploration = []
    # tBF = []

    # for i in range(20):
    #     x_len = 100
    #     for j in range(20):
    #         area.append(x_len*y_len)
    #         bounds = [0, x_len, 0, y_len]
    #         location = [0, 0, 0, 0]
    #         [location,distspiral_exploration,turnspiral_exploration] = spiral_exploration(drone, location, bounds, 50, display=True)
    #         location = [0, 0, 0, 0]
    #         [location,distBF2, turnBF] = snake_exploration(drone, location, bounds, 50, display=True)
    #         dspiral_exploration.append(distspiral_exploration)
    #         dBF.append(distBF2)
    #         tspiral_exploration.append(turnspiral_exploration)
    #         tBF.append(turnBF)
    #         x_len += 100
    # y_len += 100

    # plt.figure()
    # # plt.plot(area, dspiral_exploration, 'r.', label='spiral_exploration')
    # # plt.plot(area, dBF, 'b.', label='Back-and-Forth')
    # plt.plot(area, tspiral_exploration, 'r.', label='spiral_exploration')
    # plt.plot(area, tBF, 'b.', label='Back-and-Forth')
    # plt.xlabel("Area (cm)")
    # plt.ylabel("Turns")
    # plt.legend()
    # plt.show()