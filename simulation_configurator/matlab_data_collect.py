import sys
import os
filePath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(filePath+"/../controllers/webots_vehicle_testing")
sys.path.append(filePath+"/../controllers/controller_commons")
sys.path.append("./examples")
from single_car_acc_follow import *


def collect_data(acc_target_list, acc_target_time_list):
    return execute_single_car_acc_follow_data_collection(acc_target_list, acc_target_time_list)
