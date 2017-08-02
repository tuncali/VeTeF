import os
import sys
filePath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(filePath+"/../base_controller")
sys.path.append(filePath+"/../generic_pid_controller")
sys.path.append(filePath+"/../generic_stanley_controller")
sys.path.append(filePath+"/../controller_commons")
sys.path.append(filePath+"/../webots_vehicle_testing")
from base_controller import BaseCarController
from generic_pid_controller import *
from generic_stanley_controller import *
from controller_commons import *
from controller_communication_interface import ControllerCommunicationInterface
from coordinate_system import CoordinateSystem


# **********************************************************************************************
# This controller tries to match the target acceleration values.
# **********************************************************************************************
class simple_acceleration_follower(BaseCarController):
    def __init__(self, (car_model, self_vhc_id, target_acc_list, target_time_list)):
        BaseCarController.__init__(self, car_model)
        self.EMITTER_DEVICE_NAME = "emitter"
        self.RECEIVER_DEVICE_NAME = "receiver"
        self.COMPASS_DEVICE_NAME = "vut_compass"
        self.STEP_TIME = 10
        self.TOUCH_SENSOR_NAME = "touch sensor"
        self.MAX_NEGATIVE_THROTTLE_CHANGE = -float(self.STEP_TIME)/3000.0
        self.MAX_POSITIVE_THROTTLE_CHANGE = float(self.STEP_TIME)/1000.0  # I want to give full throttle from 0 in 1 sec
        self.STANLEY_K = 1.4   # 0.25
        self.STANLEY_K2 = 1.0  # 0.3
        self.STANLEY_K3 = 1.0
        self.ref_orientation = 0.0
        self.received_message = []
        self.SELF_VHC_ID = int(self_vhc_id)
        self.touch_sensor = None
        self.compass_device = None
        self.receiver_device = None
        self.emitter_device = None
        self.GPS_DEVICE_NAME = "vut_gps"
        self.GPS_DEVICE_PERIOD = 10
        self.prev_long_control = 0.0
        self.prev_lat_control = 0.0
        self.last_throttle = 0.0
        self.LONG_PID_P = 0.06125  # 1.5
        self.LONG_PID_I = 0.000005  # 0.1
        self.LONG_PID_D = 0.1  # 8.5
        self.PID_INTEGRATOR_MIN = -500.0
        self.PID_INTEGRATOR_MAX = 500.0
        self.STEP_TIME = 10
        self.prev_speed = None
        self.MAX_NEGATIVE_THROTTLE_CHANGE = -float(self.STEP_TIME)/100.0
        self.MAX_POSITIVE_THROTTLE_CHANGE = float(self.STEP_TIME)/100.0
        self.longitudinal_pid = GenericPIDController()
        self.lateral_controller = GenericStanleyController()
        import ast
        print('list: {}'.format(target_acc_list))
        self.target_acc_list = ast.literal_eval(target_acc_list)
        print('Target Acc: {}'.format(self.target_acc_list))
        self.target_time_list = ast.literal_eval(target_time_list)
        self.contr_comm_interface = ControllerCommunicationInterface()
    
    def run(self):
        self_vhc_pos = None
        target_lat_pos = 0.0
        self.longitudinal_pid.set_parameters(self.LONG_PID_P, self.LONG_PID_I, self.LONG_PID_D)
        self.longitudinal_pid.set_output_range(self.MAX_NEGATIVE_THROTTLE_CHANGE, self.MAX_POSITIVE_THROTTLE_CHANGE)
        self.longitudinal_pid.set_integrator_value_range(self.PID_INTEGRATOR_MIN, self.PID_INTEGRATOR_MAX)
        self.lateral_controller.set_parameters(self.STANLEY_K, self.STANLEY_K2, self.STANLEY_K3)
        self.receiver_device = self.getReceiver(self.RECEIVER_DEVICE_NAME)
        if self.receiver_device is not None:
            self.receiver_device.enable(self.STEP_TIME)
        # Enable compass device.
        self.compass_device = self.getCompass(self.COMPASS_DEVICE_NAME)
        if self.compass_device is not None:
            self.compass_device.enable(self.STEP_TIME)
        gps_device = self.getGPS(self.GPS_DEVICE_NAME)
        if gps_device is not None:
            gps_device.enable(self.GPS_DEVICE_PERIOD)
        self.start_car()
        target_ind = 0

        while True:
            self.step()
            cur_sim_time = self.get_sim_time()
            while target_ind < len(self.target_time_list)-1 and self.target_time_list[target_ind+1] < cur_sim_time:
                target_ind += 1

            cur_speed = self.get_current_speed()
            if math.isnan(cur_speed):
                cur_speed = 0.0
            desired_acc = self.target_acc_list[target_ind]
            if self.prev_speed is not None:
                cur_acc = 100.0*(cur_speed - self.prev_speed) / 3.6
            else:
                cur_acc = 0.0
            if math.isnan(cur_acc):
                cur_acc = 0.0
            if cur_acc < -8.0:
                cur_acc = -8.0
            elif cur_acc > 3.0:
                cur_acc = 3.0
            if not math.isnan(cur_speed):
                self.prev_speed = cur_speed
            acc_err = desired_acc - cur_acc
            if math.isnan(acc_err):
                acc_err = 0.0
            throttle_delta = self.longitudinal_pid.compute_no_derivative_kick(acc_err, cur_acc)
            self.last_throttle += throttle_delta
            self.last_throttle = min(max(-1.0, self.last_throttle), 1.0)
            # print('desired_acc: {} acc_err: {}, cur_acc: {} throttle_delta:{}: throttle: {}'.format(
            # desired_acc, acc_err, cur_acc, throttle_delta, self.last_throttle))

            # Receive message
            if self.receiver_device is not None:
                (is_rcv, temp_message) = get_receiver_message(self.receiver_device)
                if is_rcv:
                    self.received_message = temp_message
                    cur_msg_index = 0
                    while cur_msg_index < len(temp_message):
                        (command, data, data_size) = \
                            self.contr_comm_interface.interpret_message(self.received_message[cur_msg_index:])
                        if command == self.contr_comm_interface.SET_CONTROLLER_PARAMETERS_MESSAGE:
                            if data.vehicle_id == self.SELF_VHC_ID or data.vehicle_id == 0:
                                if data.parameter_name == 'acc_target':
                                    self.target_time_list.extend(data.parameter_data[0])
                                    self.target_acc_list.extend(data.parameter_data[1])
                                elif data.parameter_name == 'acc_target_time':
                                    self.target_time_list.extend(data.parameter_data)
                                    # print self.target_time_list
                                elif data.parameter_name == 'acc_target_value':
                                    self.target_acc_list.extend(data.parameter_data[:])
                                    # print self.target_acc_list
                        elif command == self.contr_comm_interface.VHC_POSITION_MESSAGE:
                            if gps_device is None:
                                self_vhc_pos = data
                        if data_size > 0:
                            cur_msg_index += data_size
                        else:
                            cur_msg_index = len(temp_message)

            if gps_device is not None:
                self_vhc_pos = gps_device.getValues()
            if self_vhc_pos is None:
                lat_distance = 0.0
            else:
                lat_distance = target_lat_pos - self_vhc_pos[CoordinateSystem.LAT_AXIS]

            if self.compass_device is not None:
                orientation = get_bearing(self.compass_device)
                if not math.isnan(orientation):
                    orient_err = orientation - self.ref_orientation
                    if orient_err > math.pi:
                        orient_err -= 2.0 * math.pi
                    if orient_err < -math.pi:
                        orient_err += 2.0 * math.pi
                else:
                    orient_err = 0.0
            else:
                orient_err = 0.0
            if not math.isnan(cur_speed):
                lat_control = self.lateral_controller.compute(orient_err, lat_distance, cur_speed)
            else:
                lat_control = 0.0

            # cur_speed = self.get_current_speed()
            # print('curr_speed: {}'.format(cur_speed))
            self.set_control_actions_throttle_angle(self.last_throttle, lat_control)
            self.prev_lat_control = lat_control
