'''Variables used in snake_path_experiment and salesman_image_interface
By Angel Rodriguez 2023'''

# Used to determine if we want to go back to previous recorded y_coordinate
shift_cutoff = 90

# Fan coordinate stuff we will not use but have since alot of functions created expect these parameters
fans_x_coordinates = [360, 832, 550, 613, 161, 832, 188]
fans_y_coordinates = [48, 409, 224, 460, 227, 150, 457]

# Keeping this code for now to avoid errors
turbines = {"WindTurbine_1": [[0, 0, 0, 0], [fans_x_coordinates[0], fans_y_coordinates[0]]], "WindTurbine_2": [[0, 0, 0, 0], [fans_x_coordinates[1], fans_y_coordinates[1]]], "WindTurbine_3": [[0, 0, 0, 0], [fans_x_coordinates[2], fans_y_coordinates[2]]],
                "WindTurbine_4": [[0, 0, 0, 0], [fans_x_coordinates[3], fans_y_coordinates[3]]], "WindTurbine_5": [[0, 0, 0, 0], [fans_x_coordinates[4], fans_y_coordinates[4]]],
                "WindTurbine_6": [[0, 0, 0, 0], [fans_x_coordinates[5], fans_y_coordinates[5]]], "WindTurbine_7": [[0, 0, 0, 0], [fans_x_coordinates[6], fans_y_coordinates[6]]]}
