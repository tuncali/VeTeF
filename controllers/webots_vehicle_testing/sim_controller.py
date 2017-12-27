"""Defines SimController class"""

import time
import numpy as np
import sys
import os
from simulation_message_interface import SimulationMessageInterface
from communication_server import CommunicationServer
from robustness_computation import RobustnessComputation
from sim_object_generator import SimObjectGenerator
from supervisor_controls import SupervisorControls
from vehicles_manager import VehiclesManager
from environment_manager import EnvironmentManager
from heart_beat import HeartBeatConfig, HeartBeat
from data_logger import DataLogger
from item_description import ItemDescription
FILE_PATH = os.path.dirname(os.path.realpath(__file__))
sys.path.append(FILE_PATH + "/../controller_commons")
from controller_communication_interface import ControllerCommunicationInterface
# For Data Logging and Plotting:
# from DataLogger import *


class SimController(object):
    """SimController class controls the flow of the simulation."""
    def __init__(self):
        self.heart_beat_config = HeartBeatConfig()
        self.sim_data = None
        self.sim_obj_generator = SimObjectGenerator()
        self.message_interface = SimulationMessageInterface()
        self.comm_server = None
        self.client_socket = None
        self.comm_port_number = 10021
        self.supervisor_control = SupervisorControls()
        self.vehicles_manager = None
        self.environment_manager = EnvironmentManager()
        self.current_sim_time_ms = 0
        self.robustness_function = None
        self.debug_mode = 0
        self.data_logger = None
        self.view_follow_item = None
        self.controller_comm_interface = ControllerCommunicationInterface()

    def init(self, debug_mode, supervisor_params):
        """Initialize the simulation controller."""
        if self.debug_mode:
            print("Starting initialization")
            sys.stdout.flush()
        self.comm_port_number = supervisor_params
        self.supervisor_control.init(supervisor_params)
        self.vehicles_manager = VehiclesManager(self.supervisor_control)
        self.debug_mode = debug_mode
        self.vehicles_manager.debug_mode = self.debug_mode
        if self.debug_mode:
            # self.robustnessFunction.set_debug_mode(self.debug_mode)
            print("Initialization: OK")
            sys.stdout.flush()

    def set_debug_mode(self, mode):
        """Set debug mode."""
        self.debug_mode = mode

    def generate_road_network(self, road_list, sim_obj_generator, road_network_id):
        """Generate and add the road network to the simulator."""
        road_network_string = sim_obj_generator.generate_road_network_string(road_list, road_network_id)
        if self.debug_mode == 2:
            print road_network_string
        self.supervisor_control.add_obj_to_sim_from_string(road_network_string)

    def generate_vehicle(self, vhc, sim_obj_generator):
        """Generate and add the vehicle to the simulator."""
        vehicle_string = sim_obj_generator.generate_vehicle_string(vhc)
        if self.debug_mode == 2:
            print vehicle_string
        self.supervisor_control.add_obj_to_sim_from_string(vehicle_string)

    def receive_and_execute_commands(self):
        """Read all incoming commands until START SIMULATION Command."""
        if self.debug_mode:
            print("Waiting Commands")
        road_segments_to_add = []
        add_road_network_to_world = False
        v_u_t_to_add = []
        add_v_u_t_to_world = []
        dummy_vhc_to_add = []
        add_dummy_vhc_to_world = []
        continue_simulation = False
        remote_command = None
        while continue_simulation is False or self.sim_data is None:
            rcv_msg = self.comm_server.receive_blocking(self.client_socket)
            remote_command = self.message_interface.interpret_message(rcv_msg)
            remote_response = []
            if self.debug_mode:
                print("Received command : {}".format(remote_command.command))
            if remote_command.command == self.message_interface.START_SIM:
                self.sim_data = remote_command.object
                continue_simulation = True
                remote_response = self.message_interface.generate_ack_message()
            elif remote_command.command == self.message_interface.RELOAD_WORLD:
                continue_simulation = True
                remote_response = self.message_interface.generate_ack_message()
            elif remote_command.command == self.message_interface.CONTINUE_SIM:
                continue_simulation = True
                remote_response = self.message_interface.generate_ack_message()
            elif remote_command.command == self.message_interface.SET_DATA_LOG_PERIOD_MS:
                if self.data_logger is None:
                    self.data_logger = DataLogger()
                    self.data_logger.set_environment_manager(self.environment_manager)
                    self.data_logger.set_vehicles_manager(self.vehicles_manager)
                self.data_logger.set_log_period(remote_command.object)
                remote_response = self.message_interface.generate_ack_message()
            elif remote_command.command == self.message_interface.SET_CONTROLLER_PARAMETER:
                if self.vehicles_manager is not None:
                    emitter = self.vehicles_manager.get_emitter()
                    if emitter is not None:
                        self.controller_comm_interface.transmit_set_controller_parameters_message(
                            emitter,
                            remote_command.object.vehicle_id,
                            remote_command.object.parameter_name,
                            remote_command.object.parameter_data)
                remote_response = self.message_interface.generate_ack_message()
            elif remote_command.command in (self.message_interface.SURROUNDINGS_DEF,
                                            self.message_interface.SURROUNDINGS_ADD):
                road_segments_to_add.append(remote_command.object)
                add_road_network_to_world = (remote_command.command == self.message_interface.SURROUNDINGS_ADD)
                remote_response = self.message_interface.generate_ack_message()
            elif remote_command.command in (self.message_interface.DUMMY_ACTORS_DEF,
                                            self.message_interface.DUMMY_ACTORS_ADD):
                dummy_vhc_to_add.append(remote_command.object)
                if remote_command.command == self.message_interface.DUMMY_ACTORS_ADD:
                    add_dummy_vhc_to_world.append(True)
                else:
                    add_dummy_vhc_to_world.append(False)
                remote_response = self.message_interface.generate_ack_message()
            elif remote_command.command in (self.message_interface.VUT_DEF, self.message_interface.VUT_ADD):
                v_u_t_to_add.append(remote_command.object)
                if remote_command.command == self.message_interface.VUT_ADD:
                    add_v_u_t_to_world.append(True)
                else:
                    add_v_u_t_to_world.append(False)
                remote_response = self.message_interface.generate_ack_message()
            elif remote_command.command == self.message_interface.SET_HEART_BEAT_CONFIG:
                self.heart_beat_config = remote_command.object
                remote_response = self.message_interface.generate_ack_message()
            elif remote_command.command == self.message_interface.SET_ROBUSTNESS_TYPE:
                robustness_type = remote_command.object
                self.robustness_function = RobustnessComputation(robustness_type,
                                                                 self.supervisor_control,
                                                                 self.vehicles_manager,
                                                                 self.environment_manager)
                remote_response = self.message_interface.generate_ack_message()
            elif remote_command.command == self.message_interface.SET_VIEW_FOLLOW_ITEM:
                self.view_follow_item = remote_command.object
                remote_response = self.message_interface.generate_ack_message()
            elif remote_command.command == self.message_interface.ADD_DATA_LOG_DESCRIPTION:
                if self.data_logger is None:
                    self.data_logger = DataLogger()
                    self.data_logger.set_environment_manager(self.environment_manager)
                    self.data_logger.set_vehicles_manager(self.vehicles_manager)
                self.data_logger.add_data_log_description(remote_command.object)
                if self.sim_data is not None:
                    self.data_logger.set_expected_simulation_time(self.sim_data.simulation_duration_ms)
                    self.data_logger.set_simulation_step_size(self.sim_data.simulation_step_size_ms)
                remote_response = self.message_interface.generate_ack_message()
            elif remote_command.command == self.message_interface.GET_ROBUSTNESS:
                if self.robustness_function is not None:
                    rob = self.robustness_function.get_robustness()
                else:
                    rob = 0.0
                remote_response = self.message_interface.generate_robustness_msg(rob)
            elif remote_command.command == self.message_interface.GET_DATA_LOG_INFO:
                if self.data_logger is not None:
                    log_info = self.data_logger.get_log_info()
                else:
                    log_info = (0, 0)
                remote_response = self.message_interface.generate_log_info_message(log_info)
            elif remote_command.command == self.message_interface.GET_DATA_LOG:
                requested_log_start_index = remote_command.object[0]
                requested_log_end_index = remote_command.object[1]
                if self.data_logger is not None:
                    data_log = self.data_logger.get_log(requested_log_start_index, requested_log_end_index)
                else:
                    data_log = np.empty(0)
                remote_response = self.message_interface.generate_data_log_message(data_log)
            if len(remote_response) > 0:
                self.comm_server.send_blocking(self.client_socket, remote_response)

        # Generate simulation environment (add VUT, Dummy Actors and Surroundings)
        if road_segments_to_add and self.debug_mode:
            print("Number of road segments: {}".format(len(road_segments_to_add)))
        if add_road_network_to_world:
            self.generate_road_network(road_segments_to_add,
                                       self.sim_obj_generator,
                                       self.environment_manager.get_num_of_road_networks() + 1)
        if road_segments_to_add:
            self.environment_manager.record_road_network(road_segments_to_add)

        if v_u_t_to_add and self.debug_mode:
            print("Number of VUT: {}".format(len(v_u_t_to_add)))
        for i in range(len(v_u_t_to_add)):
            vhc = v_u_t_to_add[i]
            if add_v_u_t_to_world[i]:
                self.generate_vehicle(vhc, self.sim_obj_generator)
            self.vehicles_manager.record_vehicle(vhc, self.vehicles_manager.VHC_VUT)

        if dummy_vhc_to_add and self.debug_mode:
            print("Number of Dummy vehicles: {}".format(len(dummy_vhc_to_add)))
        for i in range(len(dummy_vhc_to_add)):
            vhc = dummy_vhc_to_add[i]
            if add_dummy_vhc_to_world[i]:
                self.generate_vehicle(vhc, self.sim_obj_generator)
            self.vehicles_manager.record_vehicle(vhc, self.vehicles_manager.VHC_DUMMY)
        if remote_command is not None and remote_command.command == self.message_interface.RELOAD_WORLD:
            time.sleep(1.0)
            try:
                print('Closing connection!')
                self.comm_server.close_connection()
            except:
                print('Could not close connection!')
                pass
            time.sleep(0.5)
            self.comm_server = None
            print('Reverting Simulation!')
            self.supervisor_control.revert_simulation()
            time.sleep(1)

    def prepare_sim_environment(self):
        """Prepares the simulation environment based on the communication with simulation controller."""
        if self.debug_mode:
            print("Will Prepare Sim Environment")
        self.supervisor_control.initialize_creating_simulation_environment()
        self.comm_server = CommunicationServer(True, self.comm_port_number, self.debug_mode)
        self.client_socket = self.comm_server.get_connection()

        # Read all incoming commands until START SIMULATION Command
        self.receive_and_execute_commands()

        # Set viewpoint
        view_point_vehicle_index = 0
        if self.view_follow_item is not None:
            if self.view_follow_item.item_type == ItemDescription.ITEM_TYPE_VEHICLE:
                view_point_vehicle_index = self.view_follow_item.item_index
        if self.vehicles_manager.vehicles:
            if len(self.vehicles_manager.vehicles) <= view_point_vehicle_index:
                view_point_vehicle_index = 0
            viewpoint = self.supervisor_control.getFromDef('VIEWPOINT')
            if viewpoint is not None:
                follow_point = viewpoint.getField('follow')
                if follow_point is not None:
                    follow_point.setSFString(
                        self.vehicles_manager.vehicles[view_point_vehicle_index].name.getSFString())

        # Set time parameters for the objects where necessary.
        if self.data_logger is not None:
            self.data_logger.set_expected_simulation_time(self.sim_data.simulation_duration_ms)
            self.data_logger.set_simulation_step_size(self.sim_data.simulation_step_size_ms)

        self.vehicles_manager.set_time_step(self.sim_data.simulation_step_size_ms / 1000.0)

        # Reflect changes to the simulation environment
        self.supervisor_control.finalize_creating_simulation_environment()

    def run(self):
        """The overall execution of the simulation."""
        # Prepare Simulation Environment
        self.prepare_sim_environment()

        # Start simulation
        print 'Simulation Duration = {}, step size = {}'.format(self.sim_data.simulation_duration_ms,
                                                                self.sim_data.simulation_step_size_ms)
        sys.stdout.flush()
        if self.sim_data.simulation_execution_mode == self.sim_data.SIM_TYPE_RUN:
            self.supervisor_control.set_simulation_mode(self.supervisor_control.SIMULATION_MODE_RUN)
        elif self.sim_data.simulation_execution_mode == self.sim_data.SIM_TYPE_FAST_NO_GRAPHICS:
            self.supervisor_control.set_simulation_mode(self.supervisor_control.SIMULATION_MODE_FAST)
        else:
            self.supervisor_control.set_simulation_mode(self.supervisor_control.SIMULATION_MODE_REAL_TIME)

        # Execute simulation
        while self.current_sim_time_ms < self.sim_data.simulation_duration_ms:
            cur_sim_time_s = self.current_sim_time_ms / 1000.0
            self.vehicles_manager.simulate_vehicles(cur_sim_time_s)
            if self.robustness_function is not None:
                self.robustness_function.compute_robustness(cur_sim_time_s)
            if self.data_logger is not None:
                self.data_logger.log_data(self.current_sim_time_ms)
            if (self.heart_beat_config is not None
                    and self.heart_beat_config.sync_type in (HeartBeatConfig.WITH_SYNC, HeartBeatConfig.WITHOUT_SYNC)
                    and self.current_sim_time_ms % self.heart_beat_config.period_ms == 0):
                heart_beat_message = self.message_interface.generate_heart_beat_message(self.current_sim_time_ms,
                                                                                        HeartBeat.SIMULATION_RUNNING)
                self.comm_server.send_blocking(self.client_socket, heart_beat_message)
                if self.heart_beat_config.sync_type == HeartBeatConfig.WITH_SYNC:
                    # This means it has to wait for a new command.
                    self.receive_and_execute_commands()
            self.supervisor_control.step_simulation(self.sim_data.simulation_step_size_ms)
            self.current_sim_time_ms = self.current_sim_time_ms + self.sim_data.simulation_step_size_ms

        # End Simulation
        self.supervisor_control.set_simulation_mode(self.supervisor_control.SIMULATION_MODE_PAUSE)
        heart_beat_message = self.message_interface.generate_heart_beat_message(self.current_sim_time_ms,
                                                                                HeartBeat.SIMULATION_STOPPED)
        self.comm_server.send_blocking(self.client_socket, heart_beat_message)
        for i in range(100):  # Maximum 100 messages are accepted after the simulation. Against lock / memory grow etc.
            if self.comm_server is not None:
                self.receive_and_execute_commands()

        # if self.debug_mode and (self.robustness_function is not None):
        #     print("Robustness: {}".format(self.robustness_function.get_robustness()))

        # Post Simulation Actions
        # self.perform_post_simulation_actions()
