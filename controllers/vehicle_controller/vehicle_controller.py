"""vehicle_controller is the vehicle controller known to Webots.
It calls the actual controller which is given to it as a parameter."""
import os
import sys
FILE_PATH = os.path.dirname(os.path.realpath(__file__))
sys.path.append(FILE_PATH + "/../controller_commons")

if len(sys.argv) > 1:
    controller_name = sys.argv[1]

    if controller_name is None or type(controller_name) is not str:
        print("Controller name is not given")
    else:
        if sys.platform == 'win32':  # Windows
            path_to_controller = FILE_PATH + "\\..\\" + controller_name
        else:  # Linux / Mac OS
            path_to_controller = FILE_PATH + "/../" + controller_name
        print("Controller: {}, Path:{}".format(controller_name, path_to_controller))
        sys.path.append(path_to_controller)
        controller = __import__(controller_name)
        print controller
        methodToCall = getattr(controller, controller_name)
        print methodToCall
        print sys.argv[2:]
        robot = methodToCall(sys.argv[2:])
        robot.run()
