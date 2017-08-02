"""webots_vehicle_testing is the entry point for the supervisor controller in Webots."""

import os
import sys
from sim_controller import SimController

FILE_PATH = os.path.dirname(os.path.realpath(__file__))
sys.path.append(FILE_PATH + "/../controller_commons")

if len(sys.argv) >= 1:
    try:
        PORT_NO = int(sys.argv[1])
    except ValueError:
        PORT_NO = 10020
else:
    PORT_NO = 10020

DEBUG_MODE = 0
SUPERVISOR = SimController()
SUPERVISOR.init(DEBUG_MODE, PORT_NO)
SUPERVISOR.run()
