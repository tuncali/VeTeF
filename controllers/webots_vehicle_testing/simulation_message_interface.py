"""Defines the SimulationMessageInterface class"""
import struct
import numpy as np
from webots_road import WebotsRoad
from webots_vehicle import WebotsVehicle
from webots_sensor import WebotsSensor
from staliro_signal import STaliroSignal
from simulation_command import SimulationCommand
from sim_data import SimData
from heart_beat import HeartBeatConfig
from heart_beat import HeartBeat
from item_description import ItemDescription
import sys
import os
FILE_PATH = os.path.dirname(os.path.realpath(__file__))
sys.path.append(FILE_PATH + "/../controller_commons")
from controller_communication_interface import ControllerCommunicationInterface


class SimulationMessageInterface(object):
    """SimulationMessageInterface class understands the command types,
     and acts as an interpreter between the application and the communication server."""
    START_SIM = 1
    RELOAD_WORLD = 2
    SET_HEART_BEAT_CONFIG = 3
    CONTINUE_SIM = 4
    SET_ROBUSTNESS_TYPE = 5
    ADD_DATA_LOG_DESCRIPTION = 6
    SET_VIEW_FOLLOW_ITEM = 7
    GET_ROBUSTNESS = 8
    GET_DATA_LOG_INFO = 9
    GET_DATA_LOG = 10
    SET_CONTROLLER_PARAMETER = 11
    SET_DATA_LOG_PERIOD_MS = 12
    SURROUNDINGS_ADD = 100
    SURROUNDINGS_DEF = 101
    S_ROAD = 102
    STRAIGHT_ROAD = 103
    DUMMY_ACTORS_ADD = 120
    DUMMY_ACTORS_DEF = 121
    D_VHC = 122
    VUT_ADD = 130
    VUT_DEF = 131
    VUT_VHC = 132
    ROBUSTNESS = 201
    HEART_BEAT = 202
    DATA_LOG_INFO = 203
    DATA_LOG = 204
    ACK = 250

    def __init__(self):
        self.debug_mode = 0
        self.controller_comm_interface = ControllerCommunicationInterface()

    def interpret_message(self, msg):
        """Extracts the command and object information from the given raw msg."""
        obj = None
        (command, ) = struct.unpack('B', msg[0:struct.calcsize('B')])
        cur_msg_index = struct.calcsize('B')
        if self.debug_mode:
            print("SimulationMessageInterface : msg length:{}".format(len(msg)))
        if command == self.ACK:
            obj = None
        elif command == self.CONTINUE_SIM:
            obj = None
        elif command == self.HEART_BEAT:
            (sim_status,) = struct.unpack('B', msg[cur_msg_index:cur_msg_index + struct.calcsize('B')])
            cur_msg_index += struct.calcsize('B')
            (sim_time_ms, ) = struct.unpack('I', msg[cur_msg_index:cur_msg_index + struct.calcsize('I')])
            obj = HeartBeat(simulation_status=sim_status, simulation_time_ms=sim_time_ms)
        elif command == self.SET_CONTROLLER_PARAMETER:
            obj = self.interpret_set_controller_parameter_command(msg)
        elif command == self.SET_DATA_LOG_PERIOD_MS:
            (obj, ) = struct.unpack('I', msg[cur_msg_index:cur_msg_index + struct.calcsize('I')])
        elif command in (self.SURROUNDINGS_DEF, self.SURROUNDINGS_ADD):
            (item_type, ) = struct.unpack('B', msg[cur_msg_index:cur_msg_index + struct.calcsize('B')])
            cur_msg_index += struct.calcsize('B')
            if item_type == self.S_ROAD:
                if self.debug_mode:
                    print("Road")
                obj = WebotsRoad()
                (obj.position[0],
                 obj.position[1],
                 obj.position[2],
                 obj.rotation[0],
                 obj.rotation[1],
                 obj.rotation[2],
                 obj.rotation[3],
                 obj.length,
                 obj.width,
                 road_type,
                 obj.number_of_lanes,
                 obj.right_border_bounding_object,
                 obj.left_border_bounding_object) = \
                    struct.unpack('dddddddddBB??', msg[cur_msg_index:cur_msg_index + struct.calcsize('dddddddddBB??')])
                cur_msg_index += struct.calcsize('dddddddddBB??')
                if self.debug_mode:
                    print("SimulationMessageInterface: rd length: {}".format(obj.length))
                if road_type == self.STRAIGHT_ROAD:
                    obj.road_type = "StraightRoadSegment"
            else:
                print("SimulationMessageInterface: Unknown SURROUNDINGS DEF {}".format(item_type))
        elif command in (self.DUMMY_ACTORS_DEF, self.DUMMY_ACTORS_ADD, self.VUT_DEF, self.VUT_ADD):
            (item_type, ) = struct.unpack('B', msg[cur_msg_index:cur_msg_index + struct.calcsize('B')])
            cur_msg_index += struct.calcsize('B')
            if (((command in (self.DUMMY_ACTORS_DEF, self.DUMMY_ACTORS_ADD)) and item_type == self.D_VHC)
                    or ((command in (self.VUT_DEF, self.VUT_ADD)) and item_type == self.VUT_VHC)):
                obj = WebotsVehicle()
                (obj.current_position[0],
                 obj.current_position[1],
                 obj.current_position[2],
                 obj.current_rotation[0],
                 obj.current_rotation[1],
                 obj.current_rotation[2],
                 obj.current_rotation[3],
                 obj.id,
                 vhc_model,
                 obj.controller) = struct.unpack('dddddddB25s30s',
                                                 msg[cur_msg_index:cur_msg_index + struct.calcsize('dddddddB25s30s')])
                cur_msg_index += struct.calcsize('dddddddB25s30s')
                vhc_model = vhc_model.rstrip(' \t\r\n\0')  # Remove null characters at the end
                vhc_model = vhc_model.strip()  # Remove space characters
                obj.set_vehicle_model(vhc_model)
                if command in (self.DUMMY_ACTORS_DEF, self.DUMMY_ACTORS_ADD):
                    obj.def_name = "DVHC_" + str(obj.id)
                else:
                    obj.def_name = "VUT_" + str(obj.id)

                obj.controller = obj.controller.rstrip('\0')
                if self.debug_mode:
                    print("SimulationMessageInterface : Adding vhc: Controller: {}, length: {}".format(
                        obj.controller, len(obj.controller)))

                # Read Vehicle Parameters to be used in proto settings
                (num_of_params, ) = struct.unpack('B', msg[cur_msg_index:cur_msg_index + struct.calcsize('B')])
                cur_msg_index += struct.calcsize('B')
                if self.debug_mode:
                    print("SimulationMessageInterface: Adding vhc: numOf vehicle Params {}".format(num_of_params))
                for i in range(num_of_params):
                    (length_of_setting, ) = struct.unpack('B', msg[cur_msg_index:cur_msg_index + struct.calcsize('B')])
                    cur_msg_index += struct.calcsize('B')
                    param_name_str = ''
                    if length_of_setting > 0:
                        param_name_str += msg[cur_msg_index:cur_msg_index + length_of_setting]
                        cur_msg_index += length_of_setting
                    (length_of_setting, ) = struct.unpack('B', msg[cur_msg_index:cur_msg_index + struct.calcsize('B')])
                    cur_msg_index += struct.calcsize('B')
                    param_str = ''
                    if length_of_setting > 0:
                        param_str += msg[cur_msg_index:cur_msg_index + length_of_setting]
                        cur_msg_index += length_of_setting
                    obj.vehicle_parameters.append((param_name_str, param_str))

                # Read Controller Arguments additional to vehicle type
                (num_of_contr_arguments, ) = struct.unpack('B', msg[cur_msg_index:cur_msg_index + struct.calcsize('B')])
                cur_msg_index += struct.calcsize('B')
                if self.debug_mode:
                    print("SimulationMessageInterface: Adding vhc: num_of_contr_arguments {}".format(
                        num_of_contr_arguments))
                for i in range(num_of_contr_arguments):
                    (length_of_setting, ) = struct.unpack('I', msg[cur_msg_index:cur_msg_index + struct.calcsize('I')])
                    cur_msg_index += struct.calcsize('I')
                    if length_of_setting > 0:
                        argument_str = msg[cur_msg_index:cur_msg_index + length_of_setting]
                        cur_msg_index += length_of_setting
                        obj.controller_arguments.append(argument_str)

                # Read signals
                (num_of_signals, ) = struct.unpack('B', msg[cur_msg_index:cur_msg_index + struct.calcsize('B')])
                cur_msg_index += struct.calcsize('B')
                obj.signal = []
                if self.debug_mode:
                    print("SimulationMessageInterface: Adding vhc: num_of_signals {}".format(num_of_signals))
                for i in range(0, num_of_signals):
                    (signal_type, interpolation_type, signal_ref_index, signal_ref_field, signal_val_count) = \
                        struct.unpack('BBBBh', msg[cur_msg_index:cur_msg_index + struct.calcsize('BBBBh')])
                    cur_msg_index += struct.calcsize('BBBBh')
                    signal_values = []
                    reference_values = []
                    for j in range(0, signal_val_count):
                        (sig_val, ) = struct.unpack('d', msg[cur_msg_index:cur_msg_index + struct.calcsize('d')])
                        signal_values.append(sig_val)
                        cur_msg_index += struct.calcsize('d')
                    for j in range(0, signal_val_count):
                        ref_val = struct.unpack('d', msg[cur_msg_index:cur_msg_index + struct.calcsize('d')])
                        reference_values.append(ref_val)
                        cur_msg_index += struct.calcsize('d')
                    obj.signal.append(STaliroSignal(signal_type,
                                                    interpolation_type,
                                                    signal_ref_index,
                                                    signal_ref_field,
                                                    signal_values,
                                                    reference_values))
                    if self.debug_mode:
                        print("SimulationMessageInterface: Added Signal")

                # Read Sensors
                (num_of_sensors, ) = struct.unpack('B', msg[cur_msg_index:cur_msg_index + struct.calcsize('B')])
                if self.debug_mode:
                    print("SimulationMessageInterface: Adding vhc: num_of_sensors {}".format(num_of_sensors))
                cur_msg_index += struct.calcsize('B')
                obj.sensor_array = [WebotsSensor() for i in range(num_of_sensors)]
                for i in range(0, num_of_sensors):
                    (obj.sensor_array[i].sensor_location, ) = \
                        struct.unpack('B', msg[cur_msg_index:cur_msg_index + struct.calcsize('B')])
                    cur_msg_index += struct.calcsize('B')
                    (len_of_type, ) = struct.unpack('B', msg[cur_msg_index:cur_msg_index + struct.calcsize('B')])
                    cur_msg_index += struct.calcsize('B')
                    obj.sensor_array[i].sensor_type = msg[cur_msg_index:cur_msg_index + len_of_type]
                    cur_msg_index += len_of_type
                    (len_of_field_name, ) = struct.unpack('B', msg[cur_msg_index:cur_msg_index + struct.calcsize('B')])
                    cur_msg_index += struct.calcsize('B')
                    field_index = 0
                    while len_of_field_name > 0:
                        temp_field_name = msg[cur_msg_index:cur_msg_index + len_of_field_name]
                        cur_msg_index += len_of_field_name
                        (len_of_field_val, ) = \
                            struct.unpack('B', msg[cur_msg_index:cur_msg_index + struct.calcsize('B')])
                        cur_msg_index += struct.calcsize('B')
                        temp_field_val = msg[cur_msg_index:cur_msg_index + len_of_field_val]
                        cur_msg_index += len_of_field_val
                        obj.sensor_array[i].add_sensor_field(temp_field_name, temp_field_val)
                        (len_of_field_name, ) = \
                            struct.unpack('B', msg[cur_msg_index:cur_msg_index + struct.calcsize('B')])
                        cur_msg_index += struct.calcsize('B')
                        field_index += 1

                # Read Controller Parameters (NOT arguments!)
                (num_of_control_params, ) = struct.unpack('B', msg[cur_msg_index:cur_msg_index + struct.calcsize('B')])
                cur_msg_index += struct.calcsize('B')
                obj.controller_parameters = []
                if self.debug_mode:
                    print("SimulationMessageInterface: Adding vhc : num_of_control_params {}".format(
                        num_of_control_params))
                for i in range(0, num_of_control_params):
                    (controller_param, param_msg_size) = \
                        self.controller_comm_interface.interpret_controller_parameter_message(msg[cur_msg_index:])
                    cur_msg_index += param_msg_size
                    controller_param.set_vehicle_id(obj.id)
                    obj.controller_parameters.append(controller_param)
                    if self.debug_mode:
                        print("SimulationMessageInterface: Added Controller Parameter.")
            else:
                print('SimulationMessageInterface: UNEXPECTED DUMMY ACTOR')
        elif command == self.SET_ROBUSTNESS_TYPE:
            (obj, ) = struct.unpack('I', msg[cur_msg_index:cur_msg_index + struct.calcsize('I')])
        elif command == self.ADD_DATA_LOG_DESCRIPTION:
            obj = ItemDescription()
            (obj.item_type, obj.log_item_index, obj.item_state_index) = \
                struct.unpack('BBB', msg[cur_msg_index:cur_msg_index + struct.calcsize('BBB')])
        elif command == self.START_SIM:
            obj = SimData()
            (obj.simulation_duration_ms, obj.simulation_step_size_ms, obj.simulation_execution_mode) = \
                struct.unpack('IIB', msg[cur_msg_index:cur_msg_index + struct.calcsize('IIB')])
            print("SimulationMessageInterface: Simulation Duration: {} step size: {} type: {}".format(
                obj.simulation_duration_ms,
                obj.simulation_step_size_ms,
                obj.simulation_execution_mode))
        elif command == self.RELOAD_WORLD:
            obj = None
            print("SimulationMessageInterface: Revert world")
        elif command == self.GET_ROBUSTNESS:
            obj = None
        elif command == self.GET_DATA_LOG_INFO:
            obj = None
        elif command == self.GET_DATA_LOG:
            (log_start_index, log_end_index) = \
                struct.unpack('II', msg[cur_msg_index:cur_msg_index + struct.calcsize('II')])
            obj = (log_start_index, log_end_index)
        elif command == self.DATA_LOG_INFO:
            (num_log, size_of_each_log) = struct.unpack('II', msg[cur_msg_index:cur_msg_index + struct.calcsize('II')])
            obj = (num_log, size_of_each_log)
        elif command == self.DATA_LOG:
            (num_data, ) = struct.unpack('I', msg[cur_msg_index:cur_msg_index + struct.calcsize('I')])
            cur_msg_index += struct.calcsize('I')
            obj = np.fromstring(msg[cur_msg_index:], dtype='d%s' % num_data)
        elif command == self.SET_HEART_BEAT_CONFIG:
            obj = HeartBeatConfig()
            (obj.sync_type, obj.period_ms) = \
                struct.unpack('II', msg[cur_msg_index:cur_msg_index + struct.calcsize('II')])
            print("Heart Beat Type: {} Period: {}".format(obj.sync_type, obj.period_ms))
        elif command == self.SET_VIEW_FOLLOW_ITEM:
            obj = ItemDescription()
            (obj.item_type, obj.item_index) = \
                struct.unpack('BB', msg[cur_msg_index:cur_msg_index + struct.calcsize('BB')])
        elif command == self.ROBUSTNESS:
            (obj, ) = struct.unpack('d', msg[cur_msg_index:cur_msg_index + struct.calcsize('d')])
        else:
            print("SimulationMessageInterface: Unknown COMMAND {}".format(command))

        ret_cmd = SimulationCommand(command, obj)
        return ret_cmd

    def generate_ack_message(self):
        """Creates ACKNOWLEDGEMENT message to be sent."""
        msg = struct.pack('B', self.ACK)
        return msg

    def generate_continue_sim_command(self):
        command = struct.pack('B', self.CONTINUE_SIM)
        return command

    def generate_robustness_msg(self, robustness):
        """Creates robustness message with the given robustness value."""
        msg = struct.pack('B', self.ROBUSTNESS)
        msg += struct.pack('d', robustness)
        return msg

    def generate_set_heart_beat_config_command(self, sync_type, heart_beat_period_ms):
        command = struct.pack('B', self.SET_HEART_BEAT_CONFIG)
        command += struct.pack('II', sync_type, heart_beat_period_ms)
        return command

    def generate_set_data_log_period_ms_command(self, data_log_period_ms):
        command = struct.pack('B', self.SET_DATA_LOG_PERIOD_MS)
        command += struct.pack('I', data_log_period_ms)
        return command

    def generate_heart_beat_message(self, current_simulation_time_ms, simulation_status):
        """Creates heartbeat message with the given simulation time."""
        msg = struct.pack('B', self.HEART_BEAT)
        msg += struct.pack('B', simulation_status)
        msg += struct.pack('I', current_simulation_time_ms)
        return msg

    def generate_start_simulation_command(self, duration, step_size, sim_type):
        command = struct.pack('B', self.START_SIM)
        command += struct.pack('IIB', duration, step_size, sim_type)
        return command

    def generate_restart_simulation_command(self):
        command = struct.pack('B', self.RELOAD_WORLD)
        return command

    def generate_set_robustness_type_command(self, robustness_type):
        command = struct.pack('B', self.SET_ROBUSTNESS_TYPE)
        command += struct.pack('I', robustness_type)
        return command

    def generate_set_view_follow_item_command(self, item_type, item_index):
        command = struct.pack('B', self.SET_VIEW_FOLLOW_ITEM)
        command += struct.pack('BB', item_type, item_index)
        return command

    def generate_add_data_log_description_command(self, item_type, item_index, item_state_index):
        command = struct.pack('B', self.ADD_DATA_LOG_DESCRIPTION)
        command += struct.pack('BBB', item_type, item_index, item_state_index)
        return command

    def generate_get_log_info_command(self):
        command = struct.pack('B', self.GET_DATA_LOG_INFO)
        return command

    def generate_log_info_message(self, log_info):
        command = struct.pack('B', self.DATA_LOG_INFO)
        command += struct.pack('II', log_info[0], log_info[1])
        return command

    def generate_get_data_log_command(self, start_index, end_index):
        command = struct.pack('B', self.GET_DATA_LOG)
        command += struct.pack('II', start_index, end_index)
        return command

    def generate_data_log_message(self, data_log):
        command = struct.pack('B', self.DATA_LOG)
        command += struct.pack('I', data_log.size)
        command += data_log.astype('d').tostring()
        return command

    def generate_add_road_to_simulation_command(self, road_object, is_create):
        if is_create:
            cmd = self.SURROUNDINGS_ADD
        else:
            cmd = self.SURROUNDINGS_DEF
        msg = struct.pack('B', cmd)
        msg += struct.pack('B', self.S_ROAD)
        msg += struct.pack('dddddddddBB??',
                           road_object.position[0],
                           road_object.position[1],
                           road_object.position[2],
                           road_object.rotation[0],
                           road_object.rotation[1],
                           road_object.rotation[2],
                           road_object.rotation[3],
                           road_object.length,
                           road_object.width,
                           self.STRAIGHT_ROAD,
                           road_object.number_of_lanes,
                           road_object.right_border_bounding_object,
                           road_object.left_border_bounding_object)
        return msg

    def generate_add_vehicle_to_simulation_command(self, vehicle_object, is_dummy, is_create):
        if is_dummy and is_create:
            cmd = self.DUMMY_ACTORS_ADD
            vhc_type = self.D_VHC
        elif is_dummy and not is_create:
            cmd = self.DUMMY_ACTORS_DEF
            vhc_type = self.D_VHC
        elif not is_dummy and is_create:
            cmd = self.VUT_ADD
            vhc_type = self.VUT_VHC
        else:
            cmd = self.VUT_DEF
            vhc_type = self.VUT_VHC

        msg = struct.pack('B', cmd)
        msg += struct.pack('B', vhc_type)
        # Vehicle main structure:
        msg += struct.pack('dddddddB25s30s', vehicle_object.current_position[0],
                           vehicle_object.current_position[1],
                           vehicle_object.current_position[2],
                           vehicle_object.current_rotation[0],
                           vehicle_object.current_rotation[1],
                           vehicle_object.current_rotation[2],
                           vehicle_object.current_rotation[3],
                           vehicle_object.id,
                           vehicle_object.vehicle_model,
                           vehicle_object.controller)
        # Vehicle Parameters to be used in proto settings. As string.
        num_params = len(vehicle_object.vehicle_parameters)
        msg += struct.pack('B', num_params)
        for (par_name, par_val) in vehicle_object.vehicle_parameters:
            msg += struct.pack('B', len(par_name))
            msg += struct.pack("%ds" % (len(par_name),), par_name)
            msg += struct.pack('B', len(par_val))
            msg += struct.pack("%ds" % (len(par_val),), par_val)
        # Controller Arguments other than vehicle type. As string.
        num_of_controller_arguments = len(vehicle_object.controller_arguments)
        msg += struct.pack('B', num_of_controller_arguments)
        for s in vehicle_object.controller_arguments:
            msg += struct.pack('I', len(s))
            msg += struct.pack("%ds" % (len(s),), s)
        # Signals related to the vehicle:
        num_signals = len(vehicle_object.signal)
        msg += struct.pack('B', num_signals)
        for s in vehicle_object.signal:
            msg += struct.pack('BBBBh',
                               s.signal_type,
                               s.interpolation_type,
                               s.ref_index,
                               s.ref_field,
                               len(s.signal_values))
            for j in range(len(s.signal_values)):
                msg += struct.pack('d', s.signal_values[j])
            for j in range(len(s.ref_values)):
                msg += struct.pack('d', s.ref_values[j])
        # Sensors related to the vehicle:
        num_sensors = len(vehicle_object.sensor_array)
        msg += struct.pack('B', num_sensors)
        for s in vehicle_object.sensor_array:
            msg += struct.pack('B', s.sensor_location)
            msg += struct.pack('B', len(s.sensor_type))
            msg += struct.pack("%ds" % (len(s.sensor_type),), s.sensor_type)
            for f in s.sensor_fields:
                msg += struct.pack('B', len(f.field_name))
                msg += struct.pack("%ds" % (len(f.field_name),), f.field_name)
                msg += struct.pack('B', len(f.field_val))
                msg += struct.pack("%ds" % (len(f.field_val),), f.field_val)
            msg += struct.pack('B', 0)  # Length of next field name is 0. This finishes the sensor message
        # Controller parameters related to the vehicle:
        num_of_controller_parameters = len(vehicle_object.controller_parameters)
        msg += struct.pack('B', num_of_controller_parameters)
        for c_param in vehicle_object.controller_parameters:
            msg += self.controller_comm_interface.generate_controller_parameter_message(c_param.parameter_name,
                                                                                        c_param.parameter_data)
        return msg

    def generate_set_controller_parameter_command(self, vhc_id=0, parameter_name='N/A', parameter_data=None):
        command = struct.pack('B', self.SET_CONTROLLER_PARAMETER)
        command += struct.pack('I', vhc_id)
        command += self.controller_comm_interface.generate_controller_parameter_message(parameter_name=parameter_name,
                                                                                        parameter_data=parameter_data)
        return command

    def interpret_set_controller_parameter_command(self, message):
        cur_msg_index = struct.calcsize('B')
        (vhc_id, ) = struct.unpack('I', message[cur_msg_index:cur_msg_index+struct.calcsize('I')])
        cur_msg_index += struct.calcsize('I')
        (data, data_length) = \
            self.controller_comm_interface.interpret_controller_parameter_message(message[cur_msg_index:])
        data.vehicle_id = vhc_id
        return data
