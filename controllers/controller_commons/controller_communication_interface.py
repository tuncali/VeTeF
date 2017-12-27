"""Defines ControllerCommunicationInterface class"""
import struct

from webots_controller_parameter import WebotsControllerParameter


class ControllerCommunicationInterface(object):
    """ControllerCommunicationInterface class handles the messaging between supervisor and the vehicle controllers."""
    VHC_POSITION_MESSAGE = 1
    SET_CONTROLLER_PARAMETERS_MESSAGE = 2

    def __init__(self):
        pass

    def get_receiver_message(self, receiver_device):
        is_received = False
        if receiver_device.getQueueLength() > 0:
            received_message = []
            # Receive message
            while receiver_device.getQueueLength() > 0:
                received_message += receiver_device.getData()
                receiver_device.nextPacket()
            received_message = ''.join(received_message)
            is_received = True
        else:
            received_message = ''
        return is_received, received_message

    def interpret_message(self, message):
        (command, ) = struct.unpack('B', message[0:struct.calcsize('B')])
        if command == self.VHC_POSITION_MESSAGE:
            (data, data_size) = self.interpret_vehicle_position_message(message)
        elif command == self.SET_CONTROLLER_PARAMETERS_MESSAGE:
            (data, data_size) = self.interpret_set_controller_parameters_message(message)
        else:
            data = None
            data_size = len(message)
        return command, data, data_size

    def receive_all_commands(self, receiver):
        # returns an array of commands, and corresponding data
        command_array = []
        data_array = []
        (is_received, received_message) = self.get_receiver_message(receiver)
        if is_received:
            cur_msg_index = 0
            while cur_msg_index < len(received_message):
                (command, data, data_size) = self.interpret_message(received_message[cur_msg_index:])
                command_array.append(command)
                data_array.append(data)
                cur_msg_index += data_size
        return command_array, data_array

    def extract_all_vehicle_positions(self, command_array, data_array):
        vhc_pos_array = []
        for (cmd_ind, cmd) in enumerate(command_array):
            if cmd == self.VHC_POSITION_MESSAGE:
                vhc_pos_array.append(data_array[cmd_ind])
        return vhc_pos_array

    def generate_vehicle_position_message(self, vhc_id, vhc_position):
        message = struct.pack('B', self.VHC_POSITION_MESSAGE)
        message += struct.pack('I', vhc_id)
        message += struct.pack('ddd', vhc_position[0], vhc_position[1], vhc_position[2])
        return message

    def transmit_vehicle_position_message(self, emitter, vhc_id, vhc_position):
        if emitter is not None:
            message = self.generate_vehicle_position_message(vhc_id, vhc_position)
            emitter.send(message)

    def interpret_vehicle_position_message(self, message):
        cur_msg_index = struct.calcsize('B')  # Command is already read
        vhc_position = [0.0, 0.0, 0.0]
        (vhc_id,) = struct.unpack('I', message[cur_msg_index:cur_msg_index + struct.calcsize('I')])
        cur_msg_index += struct.calcsize('I')
        (vhc_position[0], vhc_position[1], vhc_position[2]) = \
            struct.unpack('ddd', message[cur_msg_index:cur_msg_index + struct.calcsize('ddd')])
        cur_msg_index += struct.calcsize('ddd')
        data = VehiclePosition(vhc_id=vhc_id, vhc_position=vhc_position)
        return data, cur_msg_index

    def generate_controller_parameter_message(self, parameter_name='N/A', parameter_data=None):
        """Generates controller parameter message to be used inside add vehicle or set controller params messages.
        Message structure: Length of parameter name string(1),
        parameter name string(?), Parameter data type character(1), Length of parameter data(4), parameter data(?)"""
        message = struct.pack('B', len(parameter_name))
        message += struct.pack("%ds" % (len(parameter_name),), parameter_name)
        data_type_name = 'x'
        data_length = 0
        if type(parameter_data) == list:
            data_length = len(parameter_data)
            if len(parameter_data) > 0:
                if type(parameter_data[0]) is bool:
                    data_type_name = '?'
                elif type(parameter_data[0]) is int:
                    data_type_name = 'I'
                elif type(parameter_data[0]) is float:
                    data_type_name = 'd'
        elif type(parameter_data) == str:
            data_length = len(parameter_data)
            data_type_name = 's'
        elif type(parameter_data) is bool:
            data_length = 1
            data_type_name = '?'
        elif type(parameter_data) is int:
            data_length = 1
            data_type_name = 'I'
        elif type(parameter_data) is float:
            data_length = 1
            data_type_name = 'd'
        message += struct.pack('s', data_type_name)
        message += struct.pack('I', data_length)
        pack_str = '%s{}'.format(data_type_name)
        message += struct.pack(pack_str % data_length, *parameter_data)
        return message

    def interpret_controller_parameter_message(self, message):
        cur_msg_index = 0
        (param_name_length, ) = struct.unpack('B', message[cur_msg_index:cur_msg_index + struct.calcsize('B')])
        cur_msg_index += struct.calcsize('B')
        (parameter_name, ) = \
            struct.unpack('%ds' % param_name_length,
                          message[cur_msg_index:cur_msg_index + struct.calcsize('%ds' % param_name_length)])
        cur_msg_index += struct.calcsize('%ds' % param_name_length)
        (data_type_name, ) = struct.unpack('s', message[cur_msg_index:cur_msg_index + struct.calcsize('s')])
        cur_msg_index += struct.calcsize('s')
        (data_length, ) = struct.unpack('I', message[cur_msg_index:cur_msg_index + struct.calcsize('I')])
        cur_msg_index += struct.calcsize('I')
        unpack_str = '%s{}'.format(data_type_name)
        parameter_data = \
            list(struct.unpack(unpack_str % data_length,
                               message[cur_msg_index:cur_msg_index + struct.calcsize(unpack_str % data_length)]))
        cur_msg_index += struct.calcsize(unpack_str % data_length)
        data = WebotsControllerParameter(parameter_name=parameter_name, parameter_data=parameter_data)
        return data, cur_msg_index

    def generate_set_controller_parameters_message(self, vhc_id=0, parameter_name='N/A', parameter_data=None):
        """Generates SET_CONTROLLER_PARAMETERS message.
        Message structure: Command(1), Applicable vehicle id(4), Length of parameter name string(1),
        parameter name string(?), Parameter data type character(1), Length of parameter data(4), parameter data(?)"""
        message = struct.pack('B', self.SET_CONTROLLER_PARAMETERS_MESSAGE)
        message += struct.pack('I', int(vhc_id))
        message += self.generate_controller_parameter_message(parameter_name, parameter_data)
        return message

    def transmit_set_controller_parameters_message(self, emitter, vhc_id=0, parameter_name='N/A', parameter_data=None):
        if emitter is not None:
            message = self.generate_set_controller_parameters_message(vhc_id=vhc_id,
                                                                      parameter_name=parameter_name,
                                                                      parameter_data=parameter_data)
            emitter.send(message)

    def interpret_set_controller_parameters_message(self, message):
        cur_msg_index = struct.calcsize('B')
        (vhc_id, ) = struct.unpack('I', message[cur_msg_index:cur_msg_index + struct.calcsize('I')])
        cur_msg_index += struct.calcsize('I')
        (data, data_size) = self.interpret_controller_parameter_message(message[cur_msg_index:])
        data.set_vehicle_id(vhc_id)
        return data, data_size + struct.calcsize('I') + struct.calcsize('B')


class VehiclePosition(object):
    def __init__(self, vhc_id=None, vhc_position=None):
        if vhc_id is None:
            self.vehicle_id = 0
        else:
            self.vehicle_id = vhc_id
        if vhc_position is None:
            self.position = [0.0, 0.0, 0.0]
        else:
            self.position = vhc_position

    def set_vehicle_id(self, vhc_id):
        self.vehicle_id = vhc_id

    def get_vehicle_id(self):
        return self.vehicle_id

    def set_vehicle_position(self, position):
        self.position = position

    def get_vehicle_position(self):
        return self.position
