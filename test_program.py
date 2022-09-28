from math import floor
from time import sleep
import csv
from tracemalloc import start
import movement as mv

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
    #filename = filename + ".csv"

    f = open(filename, 'w')
    writer = csv.writer(f)
    header = ["Time", "Battery"]
    writer.writerow(header)

    #debugging
    #f.close()

    drone = mv.movement()
    drone_var = drone.get_drone()

    start_time = drone_var.get_flight_time()

    print(">>>>>>>>>>>>>>>>START BATTERY: ", drone_var.get_battery(), "%")
    print(">>>>>>>>>>>>>>>>BEGINNING FLIGHT TIME: ", start_time)

    row = [0, 0]

    #drone will automatically land if the battery is <= 10%
    #we need to land before that happens to get time data
    while (drone_var.get_battery() > 10):
        drone.move(up=20)
        current_time = drone_var.get_flight_time()
        current_bat = drone_var.get_battery()
        row[0] = current_time
        row[1] = current_bat
        print("Time: ", current_time)
        print("Battery: ", current_bat, "%")
        writer.writerow(row)
        sleep(10)

        drone.move(down=20)
        current_time = drone_var.get_flight_time()
        current_bat = drone_var.get_battery()
        row[0] = current_time
        row[1] = current_bat
        print("Time: ", current_time)
        print("Battery: ", current_bat, "%")
        writer.writerow(row)
        sleep(10)

    end_time = drone_var.get_flight_time()
    print(">>>>>>>>>>>>>>>>ENDING FLIGHT TIME: ", end_time)

    total_time = end_time - start_time
    total_minutes = floor(total_time / 60)
    total_seconds = total_time % 60

    time_string = "Total Flight Time: ", total_minutes, " minutes ", total_seconds, " seconds"
    print(time_string)
    writer.writerow(time_string)
    
    # close out of .csv file
    f.close()

    drone.land()
    



