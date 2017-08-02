"""Defines SimulationCommunicationInterface class."""
import numpy as np
from communication_client import CommunicationClient
from simulation_message_interface import SimulationMessageInterface


class SimulationCommunicationInterface(object):
    """SimulationCommunicationInterface class handles the communication with the server.
    Uses CommunicationClient and SimulationControlMessage classes."""
    def __init__(self, server_address='127.0.0.1', server_port=10021, max_connection_retry=100):
        self.server_address = server_address
        self.server_port = server_port
        self.comm_module = self.connect_to_simulator(max_connection_retry)
        self.simulation_message_interface = SimulationMessageInterface()
        self.max_response_time = 600

    def connect_to_simulator(self, max_connection_retry):
        """Creates a client object and connects to the simulation server"""
        comm_module = CommunicationClient(self.server_address, self.server_port)
        if not comm_module.connect_to_server(max_connection_retry):
            comm_module = None
        return comm_module

    def disconnect_from_simulator(self):
        """Disconnects from the simulation server."""
        if self.comm_module is not None:
            self.comm_module.disconnect_from_server()

    def start_simulation(self, duration, step_size, sim_type):
        """Asks the simulation server to start the simulation."""
        command = self.simulation_message_interface.generate_start_simulation_command(duration, step_size, sim_type)
        self.comm_module.send_to_server(command)
        return self.receive_ack()

    def restart_simulation(self):
        """Asks the simulation server to restart the simulation."""
        command = self.simulation_message_interface.generate_restart_simulation_command()
        self.comm_module.send_to_server(command)
        self.comm_module.set_socket_timeout(30)
        return self.receive_ack()

    def add_road_to_simulation(self, road_object, is_create):
        """Asks the simulation server to add a road to the simulation."""
        command = self.simulation_message_interface.generate_add_road_to_simulation_command(road_object, is_create)
        self.comm_module.send_to_server(command)
        return self.receive_ack()

    def add_vehicle_to_simulation(self, vehicle_object, is_dummy, is_create):
        """Asks the simulation server to add a vehicle to the simulation."""
        command = self.simulation_message_interface.generate_add_vehicle_to_simulation_command(vehicle_object,
                                                                                               is_dummy,
                                                                                               is_create)
        self.comm_module.send_to_server(command)
        return self.receive_ack()

    def set_view_follow_point(self, item_type, item_index):
        command = self.simulation_message_interface.generate_set_view_follow_item_command(item_type, item_index)
        self.comm_module.send_to_server(command)
        return self.receive_ack()

    def add_data_log_description(self, item_type, item_index, item_state_index):
        command = self.simulation_message_interface.generate_add_data_log_description_command(item_type,
                                                                                              item_index,
                                                                                              item_state_index)
        self.comm_module.send_to_server(command)
        return self.receive_ack()

    def set_data_log_period_ms(self, data_log_period_ms):
        command = self.simulation_message_interface.generate_set_data_log_period_ms_command(data_log_period_ms)
        self.comm_module.send_to_server(command)
        return self.receive_ack()

    def set_heart_beat_config(self, sync_type, heart_beat_period_ms):
        command = \
            self.simulation_message_interface.generate_set_heart_beat_config_command(sync_type, heart_beat_period_ms)
        self.comm_module.send_to_server(command)
        return self.receive_ack()

    def receive_status_message(self):
        command = self.receive_message()
        if command.command == self.simulation_message_interface.HEART_BEAT:
            cur_sim_time = command.object
        else:
            cur_sim_time = None
        self.send_ack()
        return cur_sim_time

    def receive_heart_beat(self):
        command = self.receive_message()
        if command.command == self.simulation_message_interface.HEART_BEAT:
            heart_beat = command.object
        else:
            heart_beat = None
        return heart_beat

    def set_max_response_time(self, max_response_time):
        self.max_response_time = max_response_time

    def receive_ack(self):
        is_ack = False
        message = self.receive_message()
        if message.command == SimulationMessageInterface.ACK:
            is_ack = True
        return is_ack

    def send_ack(self):
        message = self.simulation_message_interface.generate_ack_message()
        self.comm_module.send_to_server(message)

    def receive_robustness_value(self):
        """Receives computed robustness value from Webots."""
        # self.comm_module.set_socket_timeout(self.max_response_time)
        message = self.receive_message()
        if message.command == SimulationMessageInterface.ROBUSTNESS:
            rob = message.object
        else:
            rob = None
        return rob

    def receive_message(self):
        raw_data = self.comm_module.receive_blocking()
        message = self.simulation_message_interface.interpret_message(raw_data)
        return message

    def send_continue_sim_command(self):
        command = self.simulation_message_interface.generate_continue_sim_command()
        self.comm_module.send_to_server(command)
        return self.receive_ack()

    def send_controller_parameter(self, controller_param):
        command = self.simulation_message_interface.generate_set_controller_parameter_command(
            vhc_id=controller_param.vehicle_id,
            parameter_name=controller_param.parameter_name,
            parameter_data=controller_param.parameter_data)
        self.comm_module.send_to_server(command)
        return self.receive_ack()

    def get_data_log(self):
        log = np.empty(0)
        command = self.simulation_message_interface.generate_get_log_info_command()
        self.comm_module.send_to_server(command)
        response = self.receive_message()
        if response.command == self.simulation_message_interface.DATA_LOG_INFO:
            num_of_records = response.object[0]
            size_of_record = response.object[1]
            print('numofrecords: {} sizeof records: {}'.format(num_of_records, size_of_record))
            total_log_size = num_of_records
            cur_log_index = 0
            if size_of_record >= 64:
                increment_size = 1
            else:
                increment_size = 64/size_of_record
            while cur_log_index < total_log_size:
                command = self.simulation_message_interface.generate_get_data_log_command(
                    cur_log_index, min(cur_log_index + increment_size, total_log_size))
                cur_log_index = min(cur_log_index + increment_size, total_log_size)
                self.comm_module.send_to_server(command)
                response = self.receive_message()
                if response.command == self.simulation_message_interface.DATA_LOG:
                    log = np.append(log, response.object)
            log = np.reshape(log, (num_of_records, size_of_record))
        return log
