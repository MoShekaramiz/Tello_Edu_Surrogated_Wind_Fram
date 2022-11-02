from math import floor
from time import sleep
import csv
from tracemalloc import start
import movement as mv
import os

#drone.move(fwd=200)
#drone.go_to(0, 0)
#drone.land()

#drone1 = drone.get_drone()

#while (drone.get_location() != [0 , 0, 0, 0]):
#    print("Battery remaining: ", drone1.get_battery())
#    sleep(2)

if __name__ == "__main__":

    #create CSV file
    
    filename = input("Enter filename: ")
    filename = filename + ".csv"

    os.chdir("C:\\Users\\jsfab\\Documents")
    print("Saving ", filename, " in directory ", os.getcwd)

    f = open(filename, 'w')
    writer = csv.writer(f)
    header = ["Time", "Battery"]
    writer.writerow(header)

    terminated_early = False

    #debugging
    #f.close()

    drone = mv.movement()       
    drone_var = drone.get_drone()
    #start_time = drone_var.get_flight_time()


    print(">>>>>>>>>>>>>>>>START BATTERY: ", drone_var.get_battery(), "%")
    print(">>>>>>>>>>>>>>>>BEGINNING FLIGHT TIME: ", drone_var.get_flight_time())

    row = [0, 0]

    #drone will automatically land if the battery is <= 10%
    #we need to land before that happens to get time data, 15% seems to be sweet spot
    #battery seems to drop anywhere between 1 - 3% every 10 seconds
    
    while (drone_var.get_battery() > 15):
        #drone will time out if we don't send a command once every 15 seconds
        try:
            drone.move(up=20)
        except:
            writer.writerow("Flight Terminated Early")
            f.close()
            print(">>>>>>>>>>>>>>>>CRITICAL: Drone terminated early, writing data to file", filename)
            terminated_early = True
            break
        current_time = drone_var.get_flight_time()
        current_bat = drone_var.get_battery()
        row[0] = current_time
        row[1] = current_bat
        print("Time: ", current_time)
        print("Battery: ", current_bat, "%")
        writer.writerow(row)
        sleep(5)

        current_time = drone_var.get_flight_time()
        current_bat = drone_var.get_battery()
        row[0] = current_time
        row[1] = current_bat
        print("Time: ", current_time)
        print("Battery: ", current_bat, "%")
        writer.writerow(row)
        sleep(5)

        try:
            drone.move(down=20)
        except:
            writer.writerow("Flight Terminated Early")
            f.close()
            print(">>>>>>>>>>>>>>>>CRITICAL: Drone terminated early, writing data to file ", filename)
            terminated_early = True
            break
        current_time = drone_var.get_flight_time()
        current_bat = drone_var.get_battery()
        row[0] = current_time
        row[1] = current_bat
        print("Time: ", current_time)
        print("Battery: ", current_bat, "%")
        try:
            writer.writerow(row)
        except:
            break
        sleep(5)

        current_time = drone_var.get_flight_time()
        current_bat = drone_var.get_battery()
        row[0] = current_time
        row[1] = current_bat
        print("Time: ", current_time)
        print("Battery: ", current_bat, "%")
        try:
            writer.writerow(row)
        except:
            break
        sleep(5)

    end_time = drone_var.get_flight_time()
    print(">>>>>>>>>>>>>>>>ENDING FLIGHT TIME: ", end_time)

    total_minutes = floor(end_time / 60)
    total_seconds = end_time % 60

    time_string = "Total Flight Time: " + str(total_minutes) + " minutes " + str(total_seconds) + " seconds"
    print(time_string)
    writer.writerow(time_string)
    writer.writerow("Successful Flight!")

    print(">>>>>>>>>>>>>>>> SUCCESS: Writing data to file ", filename)
        
    # close out of .csv file
    if (terminated_early == False):
        f.close()
    # land drone
    drone.land()

    
    



