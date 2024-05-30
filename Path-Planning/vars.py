'''Variables used in traveling_salesman_geometric, salesman_image_interface and mission
By Angel Rodriguez 2023'''

from datetime import datetime

# Used to update the drone's coordinates in qr_detection
fans_x_coordinates = [360, 832, 550, 613, 161, 832, 188]
fans_y_coordinates = [48, 409, 224, 460, 227, 150, 457]

# Turbine dictionary contaiing QR identification strings as well as x/y coordinates, target turbines should always be within here (with already found ones being popped)
turbines = {"WindTurbine_1": [[1, 0, 1, 0], [fans_x_coordinates[0], fans_y_coordinates[0]]], 
            "WindTurbine_2": [[1, 0, 1, 0], [fans_x_coordinates[1], fans_y_coordinates[1]]], 
            "WindTurbine_3": [[1, 0, 1, 0], [fans_x_coordinates[2], fans_y_coordinates[2]]],
            "WindTurbine_4": [[1, 0, 1, 0], [fans_x_coordinates[3], fans_y_coordinates[3]]], 
            "WindTurbine_5": [[1, 0, 1, 0], [fans_x_coordinates[4], fans_y_coordinates[4]]],
            "WindTurbine_6": [[1, 0, 1, 0], [fans_x_coordinates[5], fans_y_coordinates[5]]], 
            "WindTurbine_7": [[1, 0, 1, 0], [fans_x_coordinates[6], fans_y_coordinates[6]]]}

# 100 cm is about the distance the drone can scan the QR code
x_distance_cutoff = 100 # Declare variable to be the distance we want to stop short in the x-axis


# The original turbines dictionary gets popped when a new turbine is found
# We need a new dictionary which always has all turbines for the purpose of when the drone detects the wrong one
turbines_copy = turbines.copy()

# Demonstration purposes
# fans_x_coordinates = [239, 239, 550, 613, 161, 832, 188]
# fans_y_coordinates = [91, -91, 224, 460, 227, 150, 457]

# Demonstration purposes          
# turbines = {"WindTurbine_8": [[0, 0, 0, 0], [fans_x_coordinates[0], fans_y_coordinates[0]]], 
#             "WindTurbine_9": [[0, 0, 0, 0], [fans_x_coordinates[1], fans_y_coordinates[1]]],
#             "WindTurbine_10": [[0, 0, 0, 0], [fans_x_coordinates[2], fans_y_coordinates[2]]]}

mission_st = datetime.now().strftime('%B %d,%Y %H,%M,%S')
mission_filename = 'mission_log.csv'

