from re import X
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
from shapely.geometry import box, Polygon
import time


fbRange = [32000, 52000] # preset parameter for detected image boundary size
w, h = 720, 480 # display size of the screen
location = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
turbine_locations = [] # List containing the locations of found turbines

def backForth(drone, location, flyZone, moveIncr, display=False, xgraph=[], ygraph=[]):
    '''The drone explores a back and forth path. Currently the funcion ends after each movement_test to
    allow for an external check of the area.
    
    Input:
        drone (Tello) : drone variable
        location : [x, y, angle] Current coordinates and angle of drone
        flyZone : [xMin, yMin, xMax, yMax] four vertices representing area to be explored
        moveIncr (int) : distance (cm) for drone to move before checking the area, This will also be the 
        distance between traversals
        display (bool, optional) : default = false. If true a graph of the projected path will be displayed
    Output:
        location : [x, y, angle] updated coordinates and angle of drone
        totalDist (int) : total distance (cm) traveled by the drone 
        valid (int) : returns 1 if not end of path, returns 0 if end of path'''

    xMin = flyZone[0] 
    yMin = flyZone[2] 
    xMax = flyZone[1]  
    yMax = flyZone[3] 

    totalDist = 0                   # tracks distance traveled

    #maxMove = 30                #how much to move at a time;
    #shortLen = 30               #how far to move on short edge;

    yNew = location[1] + moveIncr * math.sin(math.radians(location[2]))
    #print(location[0], location[1], location[2])
 
    #straighten out with horizontal closest to angle  <180=90 (left)       else=270 (right)
    if location[2]<90:
        location = mv.move(location, drone, ccw=90-location[2])
    elif location[2]<180:
        location = mv.move(location, drone, cw=location[2]-90)
    elif location[2]<270:
        location = mv.move(location, drone, ccw=270-location[2])
    else: 
        location = mv.move(location, drone, cw=location[2]-270)


    if display:
        xgraph.append(location[0])    
        ygraph.append(location[1])


    # Travel Up
    if location[2]<180:
        if yNew > yMax:
            if round(yMax-location[1])>20:
                totalDist += round(yMax-location[1])
                location = mv.move(location, drone, fwd=round(yMax-location[1]))
                if display:
                    xgraph.append(location[0])    
                    ygraph.append(location[1])
            location = mv.move(location, drone, ccw=90)
            xNew = location[0] + moveIncr * math.cos(math.radians(location[2]))
            if xNew < xMin:
                print("Went through whole area")
                mv.go_to(location, drone, turbine_locations, 0, 0, 0)
                totalDist += int(math.sqrt(location[0]**2 + location[1]**2))
                return location, totalDist, 0
    # Travel Down
    else:
        if yNew < yMin:
            if round(location[1]-yMin)>20:
                totalDist += round(location[1]-yMin)
                location = mv.move(location, drone, fwd=round(location[1]-yMin))
                if display:
                    xgraph.append(location[0])    
                    ygraph.append(location[1])
            location = mv.move(location, drone, cw=91)
            xNew = location[0] + moveIncr * math.cos(math.radians(location[2]))
            if xNew < xMin:
                print("Went through whole area")
                mv.go_to(location, drone, turbine_locations, 0, 0, 0)
                totalDist += int(math.sqrt(location[0]**2 + location[1]**2))
                return location, totalDist, 0
    totalDist += moveIncr
    location = mv.move(location, drone, fwd=moveIncr)
    return location, totalDist, 1

def backForth2(drone, location, flyZone, searchWidth, moveIncr, display=False):
    '''The drone explores a back and forth path.
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
    if location[0] != xMin or location[1] != yMin or location[2] != 0:
        print("Needs to be further developed to support this")
        quit()

    totalDist = 0                   # tracks distance traveled
    turns = 0                       # track number of turns
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
        if display:
            xgraph.append(location[0])    
            ygraph.append(location[1])
        for i in range(XMovesBeforeTurn):
            mv.move(location, drone, fwd=moveIncr)
            # CHECK CAMERA
            location= check_camera(drone, location)
            ###################
            totalDist += moveIncr
            if display:
                xgraph.append(location[0])    
                ygraph.append(location[1])
        if (xDist % moveIncr) > 20:
            mv.move(location, drone, fwd=xDist%moveIncr)
            totalDist += xDist%moveIncr
            # CHECK CAMERA
            location= check_camera(drone, location)
            ###################
            if display:
                xgraph.append(location[0])    
                ygraph.append(location[1])
        if y >= Ytraversals:
            break
        if turnDir:
            mv.move(location, drone, ccw=90)
            turns += 1
        else:
            mv.move(location, drone, cw=90)
            turns += 1
        # CHECK CAMERA
        location= check_camera(drone, location)
        ###################
        for i in range(YMovesBeforeTurn):
            mv.move(location, drone, fwd=moveIncr)
            # CHECK CAMERA
            location= check_camera(drone, location)
            ###################
            totalDist += moveIncr
            if display:
                xgraph.append(location[0])    
                ygraph.append(location[1])
        if (searchWidth % moveIncr) > 20:
            mv.move(location, drone, fwd=searchWidth%moveIncr)
            totalDist += searchWidth%moveIncr
            # CHECK CAMERA
            if display:
                xgraph.append(location[0])    
                ygraph.append(location[1])
        if turnDir:
            mv.move(location, drone, ccw=90)
            turns += 1
        else:
            mv.move(location, drone, cw=90)
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
        plt.plot(xgraph, ygraph, '-kx', lw=2, label='spiralPath')
        plt.show()
    
    # return to original location and track the distance
    mv.go_to(location, drone, turbine_locations, 0, 0, 0)
    totalDist += int(math.sqrt(location[0]**2 + location[1]**2))

    return location, totalDist
    

def spiral(drone, location, flyZone, searchWidth, moveIncr, display=False):
    #poly_bound = Polygon(boundary)    
    #cp = poly_bound.centroid
    #poly_bound.exterior.coords
    '''The drone explores a spiral path. This function needs to be modified to check the camera itself
    
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

    # Go to correct location if starting location is not in the lower right corner
    #if yMax - location[1] > location[1] - yMin:
     #  if xMax - location[0] > location[0] - xMin:
      #    if round(location[2]) !=90:
       #       location = mv.move(location, drone, ccw=)

    # Ensure that the drone is in the lower right corner and rotated correctly, otherwise quit
    if location[0] != xMin or location[1] != yMin or location[2] != 0:
        print("Needs to be further developed to support this")
        quit()

    xDist = xMax-xMin               # distance needed for next horizontal traverse
    yDist = yMax-yMin               # distance needed for next vertical traverse
    xt = 0                          # xtraversed
    yt = 0                          # ytraversed
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
            if yt + moveIncr > yDist:
                if yDist-yt>=20:
                    location = mv.move(location,drone,fwd=yDist-yt)
                    totalDist += yDist-yt
                yt = 0
                if f != 0:                  # decrease distance to travel each time after first line
                    yDist -= searchWidth
                location =mv.move(location,drone, ccw=90)
                turns += 1
                f =1  
            else:
                yt += moveIncr
                location = mv.move(location, drone, fwd=moveIncr)
                totalDist += moveIncr
        # Travel the xDist
        elif round(location[2])==0 or round(location[2])==180:
            if xDist < 20:
                break
            ##### CHECK CAMERA
            location= check_camera(drone, location)
            ################### 
            #sleep(0.2)
            if xt + moveIncr > xDist:
                if xDist-xt>=20:
                    location = mv.move(location,drone,fwd=xDist-xt)
                    totalDist += xDist-xt
                xt = 0
                if f != 0:
                    xDist -= searchWidth
                location = mv.move(location,drone, ccw=90)
                turns +=1
                f = 1
            else:
                xt += moveIncr
                location = mv.move(location,drone, fwd=moveIncr)
                totalDist += moveIncr
        # The code is not developed to work with any drone angle other then 0, 90, 180,n270
        else:
            print("Should not reach this print statement") 
            quit()
    

    # plot the path
    if display:
        xgraph.append(location[0])    
        ygraph.append(location[1])
        xgraph.append(xgraph[0])
        ygraph.append(ygraph[0])
        plt.plot(xgraph, ygraph, '-kx', lw=2, label='spiralPath')
        plt.show()
    
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
    location = trackObject(drone, info, location, turbines, video)
    print("escaped")
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
         
        
def testBF(location, bounds, display=False):
    '''Tests back and for search function given the current location of the drone.
        bounds =  [-275,0, 0, 275] works well for the drone cage'''
    mission_list = [1, 1, 1, 1]
    turbine_list = ["WindTurbine_2"]
    drone = Tello()
    valid = 1
    dist = 0
    ygraph = []
    xgraph = []
    while valid:
        [location, totalDist, valid] = backForth(drone, location, bounds, 50, display, xgraph, ygraph)
        dist += totalDist
        sleep(0.5)
    if display:
        xgraph.append(xgraph[0])
        ygraph.append(ygraph[0])
        plt.figure()
        plt.plot(xgraph, ygraph, '-kx', lw=2, label='spiralPath')
    return dist

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
    print("Current battery remaining: ", drone.get_battery())
    sleep(0.3)
    drone.streamon()
    sleep(0.5)
    video = LiveFeed(drone)
    video.start()
    drone.takeoff()
    sleep(1)
    mv.move(location, drone, up=40)
    sleep(0.5)
    # END OF SECTION TO COMMENT OUT
    
    #bounds = [0,321, 0, 324]
    location = mv.move(location, drone, up=40)
    bounds = [0,160, 0, 221]
    #bounds = [-327,0, 0, 327]
    #bounds = [-150,0,0,150]
    start_time = time.time()
    moveIncr = 100
    searchWidth = 50
    [location,distSpiral] = spiral(drone, location, bounds, searchWidth, moveIncr)
    #location = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
    #[location,distBF2] = backForth2(drone, location, bounds, 50, display=False)
    end_time = time.time()
    print('--- ', round(end_time - start_time, 2), ' seconds ---', end_time - start_time)
    #location = [0, 0, 0, 0] # Initialized list of x, y and angle coordinates for the drone.
    #bounds = [-328,0, 0, 324]
    #distBF = testBF(location, bounds, display=False)
    #print(distBF, distBF2)
    #test_cellDecomp(location)