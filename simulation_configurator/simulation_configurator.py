import multiprocessing as mp
import sys
import os
filePath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(filePath+"/../controllers/webots_vehicle_testing")
sys.path.append(filePath+"/../controllers/controller_commons")
sys.path.append(filePath+"/examples")
#from mpc_controller_test import *
#from itsc_example_repeat import *
#from single_car_acc_follow import *
from truck_merging_1 import *


if __name__ == "__main__":
    main_start_time = time.time()
    print('Started at : {}'.format(main_start_time))
    mp.freeze_support()

    #execute_mpc_controller_test()
    #execute_itsc_example_repeat()
    #execute_single_car_acc_follow_data_collection_test()
    execute_truck_merging_test()
    main_elapsed_time = time.time() - main_start_time
    print('Overall Execution Time: {}'.format(main_elapsed_time))
