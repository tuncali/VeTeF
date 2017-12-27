import math
import time

from heart_beat import HeartBeat
from heart_beat import HeartBeatConfig
from item_description import ItemDescription
from sim_config_tools import *
from sim_data import SimData
from simulation_communication_interface import SimulationCommunicationInterface
from webots_controller_parameter import WebotsControllerParameter
from webots_sensor import WebotsSensor


def single_car_acc_follow_data_collection(sim_config, comm_interface, acc_target_time_list=None, acc_target_list=None):
    acc_list = [0.0]
    time_list = [0.0]
    collected_data = []
    road_list = []
    vhc_list = []
    next_data_send_index = 0
    next_data_send_time = 0
    for cur_run_config in sim_config.run_config_arr:
        # Define ROADS:
        road_obj = create_road_object()
        road_obj.number_of_lanes = 3
        road_obj.right_border_bounding_object = False
        road_obj.left_border_bounding_object = False
        road_obj.width = float(road_obj.number_of_lanes) * 5.0
        road_obj.position = [-50.0, 0.0, 0.0]
        road_obj.length = 3000.0
        road_list.append(road_obj)

        # Define VEHICLES:
        vhc_obj = create_vehicle_object()
        vhc_obj.current_position = [0.0, 0.4, 0.0]
        vhc_obj.current_rotation = [0.0, 1.0, 0.0, math.pi / 2.0]
        vhc_obj.id = 1
        vhc_obj.vehicle_model = 'CitroenCZero'
        vhc_obj.controller = 'simple_acceleration_follower'
        vhc_obj.controller_arguments.append(str(vhc_obj.id))
        temp_str = str(acc_list)
        temp_str = temp_str.replace(' ', '')
        vhc_obj.controller_arguments.append(temp_str)
        temp_str = str(time_list)
        temp_str = temp_str.replace(' ', '')
        vhc_obj.controller_arguments.append(temp_str)
        vhc_obj.sensor_array.append(WebotsSensor())
        vhc_obj.sensor_array[0].sensor_location = WebotsSensor.CENTER
        vhc_obj.sensor_array[0].sensor_type = 'Compass'
        vhc_obj.sensor_array[0].add_sensor_field('name', '"vut_compass"')
        vhc_obj.sensor_array[0].add_sensor_field('yAxis', 'FALSE')
        vhc_obj.sensor_array.append(WebotsSensor())
        vhc_obj.sensor_array[-1].sensor_location = WebotsSensor.CENTER
        vhc_obj.sensor_array[-1].sensor_type = 'GPS'
        vhc_obj.sensor_array[-1].add_sensor_field('name', '"vut_gps"')
        vhc_obj.sensor_array.append(WebotsSensor())
        vhc_obj.sensor_array[-1].sensor_location = WebotsSensor.CENTER
        vhc_obj.sensor_array[-1].sensor_type = 'Receiver'
        vhc_obj.sensor_array[-1].add_sensor_field('name', '"receiver"')
        vhc_obj.sensor_array[-1].add_sensor_field('channel', '1')
        vhc_list.append(vhc_obj)

        # Communicate:
        # try:
        if comm_interface is None:
            comm_interface = SimulationCommunicationInterface(server_address=sim_config.server_ip,
                                                              server_port=sim_config.server_port,
                                                              max_connection_retry=100)
            time.sleep(0.1)
        if comm_interface is not None:
            for road_obj in road_list:
                if not comm_interface.add_road_to_simulation(road_obj, True):
                    print('ADD ROAD error')
                time.sleep(0.1)

            for vhc_obj in vhc_list:
                if not comm_interface.add_vehicle_to_simulation(vhc_obj, False, True):
                    print('ADD VEHICLE error')
                time.sleep(0.1)

            if not comm_interface.set_heart_beat_config(HeartBeatConfig.WITH_SYNC, 1000):
                print('SET HEART BEAT error')
            if not comm_interface.set_view_follow_point(ItemDescription.ITEM_TYPE_VEHICLE, 0):
                print('SET VIEW FOLLOW POINT error')
            if not comm_interface.add_data_log_description(ItemDescription.ITEM_TYPE_TIME, 0, 0):
                print('ADD DATA LOG error')
            if not comm_interface.add_data_log_description(ItemDescription.ITEM_TYPE_VEHICLE,
                                                           0,
                                                           WebotsVehicle.STATE_ID_ACCELERATION):
                print('ADD DATA LOG error')
            if not comm_interface.set_data_log_period_ms(100):
                print('SET DATA LOG PERIOD error')

            (acc_target_time_list_to_send, acc_target_list_to_send, next_data_send_time, next_data_send_index) = \
                get_data_to_send(acc_target_time_list, acc_target_list, next_data_send_time, next_data_send_index, 0)
            if len(acc_target_list_to_send) > 0:
                acc_target_parameter.set_parameter_data(acc_target_list_to_send)
                acc_target_time_parameter.set_parameter_data(acc_target_time_list_to_send)
                comm_interface.send_controller_parameter(acc_target_parameter)
                comm_interface.send_controller_parameter(acc_target_time_parameter)

            if not comm_interface.start_simulation(sim_config.sim_duration_ms,
                                                   sim_config.sim_step_size,
                                                   cur_run_config.simulation_run_mode):
                print('START SIMULATION error')
            simulation_continues = True
            acc_target_parameter = WebotsControllerParameter(vehicle_id=1,
                                                             parameter_name='acc_target_value',
                                                             parameter_data=[])
            acc_target_time_parameter = WebotsControllerParameter(vehicle_id=1,
                                                                  parameter_name='acc_target_time',
                                                                  parameter_data=[])
            while simulation_continues:
                received_heart_beat = comm_interface.receive_heart_beat()
                if received_heart_beat is not None:
                    if received_heart_beat.simulation_status == HeartBeat.SIMULATION_STOPPED:
                        simulation_continues = False
                    else:
                        (acc_target_time_list_to_send, acc_target_list_to_send, next_data_send_time,
                         next_data_send_index) = \
                            get_data_to_send(acc_target_time_list, acc_target_list, next_data_send_time,
                                             next_data_send_index, received_heart_beat.simulation_time_ms)
                        if len(acc_target_list_to_send) > 0:
                            acc_target_parameter.set_parameter_data(acc_target_list_to_send)
                            acc_target_time_parameter.set_parameter_data(acc_target_time_list_to_send)
                            comm_interface.send_controller_parameter(acc_target_parameter)
                            comm_interface.send_controller_parameter(acc_target_time_parameter)
                        comm_interface.send_continue_sim_command()
            collected_data = comm_interface.get_data_log()
            print('collected_data shape: {}'.format(collected_data.shape))
            if not comm_interface.restart_simulation():
                print('RESTART SIMULATION error')
            time.sleep(0.1)
            comm_interface.disconnect_from_simulator()
        # except:
        #     pass
    return collected_data


def get_data_to_send(acc_target_time_list, acc_target_list, next_data_send_time, next_data_send_index, cur_time):
    acc_target_list_to_send = []
    acc_target_time_list_to_send = []
    if acc_target_list is not None and acc_target_time_list is not None:
        if next_data_send_index < len(acc_target_list) == len(acc_target_list):
            if cur_time > next_data_send_time:
                last_index = min(next_data_send_index+10, len(acc_target_list))
                acc_target_list_to_send = acc_target_list[next_data_send_index:last_index]
                acc_target_time_list_to_send = acc_target_time_list[next_data_send_index:last_index]
                next_data_send_index = last_index
                next_data_send_time = acc_target_time_list[max(0, last_index-2)]
    return acc_target_time_list_to_send, acc_target_list_to_send, next_data_send_time, next_data_send_index


def execute_single_car_acc_follow_data_collection(acc_target_list, acc_target_time_list):
    sim_config = SimulationConfig(1)
    sim_config.run_config_arr.append(RunConfig())
    sim_config.run_config_arr[0].simulation_run_mode = SimData.SIM_TYPE_REAL_TIME
    sim_config.sim_duration_ms = 50000
    sim_config.sim_step_size = 10
    # First, tries to connect to the Webots. IF it can't connect, runs webots.
    comm_interface = SimulationCommunicationInterface(sim_config.server_ip, sim_config.server_port, 1)
    simulator_instance = None
    if comm_interface.comm_module is None:
        comm_interface = None
        simulator_instance = start_webots(sim_config.world_file, False)
    data = single_car_acc_follow_data_collection(sim_config, comm_interface, acc_target_time_list, acc_target_list)
    if simulator_instance is not None:
        kill_webots_pid(simulator_instance.pid)
    return data

def execute_single_car_acc_follow_data_collection_test():
    acc_target_list = [0.0, 1.0, 2.0, 0.0, -1.0, 1.0, 0.0]
    acc_target_time_list = [0.0, 5.0, 10.0, 15.0, 25.0, 30.0, 35.0]
    execute_single_car_acc_follow_data_collection(acc_target_list, acc_target_time_list)


if __name__ == "__main__":
    main_start_time = time.time()
    print('Started at : {}'.format(main_start_time))

    acc_target_list = [0.0, 1.0, 2.0, 0.0, 2.0, 1.0, 0.0]
    acc_target_time_list = [0.0, 5.0, 10.0, 15.0, 25.0, 30.0, 35.0]

    execute_single_car_acc_follow_data_collection(acc_target_list, acc_target_time_list)
    main_elapsed_time = time.time() - main_start_time
    print('Overall Execution Time: {}'.format(main_elapsed_time))

