"""Defines VehiclesManager class"""
import math
import struct
import time
from webots_controller_parameter import WebotsControllerParameter
from staliro_signal import STaliroSignal
from controller_communication_interface import ControllerCommunicationInterface
# from coordinate_system import CoordinateSystem


class VehiclesManager(object):
    """VehiclesManager keeps track of the vehicles in the simulation environment."""
    VHC_DUMMY = 0
    VHC_VUT = 1

    def __init__(self, supervisor_controller):
        self.EMITTER_NAME = "emitter"
        self.debug_mode = 0
        self.vehicles = []
        self.vehicle_dictionary = {}
        self.VUT_dictionary = {}
        self.dummy_vhc_dictionary = {}
        self.total_vhc_count = 0
        self.has_sent_controller_config = False
        self.supervisorControl = supervisor_controller
        self.time_step = 0.01
        self.current_sim_time = 0.0
        self.step_counter = 0
        self.num_of_steps_for_jerk_calc = 10
        self.acc_arr = [0.0]*(self.num_of_steps_for_jerk_calc + 3)
        self.cur_jerk_compute_index = 0
        self.controller_comm_interface = ControllerCommunicationInterface()

    def record_vehicle(self, vehicle_object, vehicle_type):
        """Add the vehicle in the records. vehicle_type can be VUT / Dummy"""
        self.vehicles.append(vehicle_object)
        self.vehicles[self.total_vhc_count].node = \
            self.supervisorControl.get_obj_node(self.vehicles[self.total_vhc_count])
        self.vehicles[self.total_vhc_count].translation = \
            self.supervisorControl.get_obj_field(self.vehicles[self.total_vhc_count], "translation")
        self.vehicles[self.total_vhc_count].rotation = \
            self.supervisorControl.get_obj_field(self.vehicles[self.total_vhc_count], "rotation")
        self.vehicles[self.total_vhc_count].name = \
            self.supervisorControl.get_obj_field(self.vehicles[self.total_vhc_count], "name")
        self.vehicles[self.total_vhc_count].front_right_wheel_angular_velocity = \
            self.supervisorControl.get_obj_field(self.vehicles[self.total_vhc_count],
                                                 "front_right_wheel_angular_velocity")
        self.vehicles[self.total_vhc_count].front_left_wheel_angular_velocity = \
            self.supervisorControl.get_obj_field(self.vehicles[self.total_vhc_count],
                                                 "front_left_wheel_angular_velocity")
        self.vehicles[self.total_vhc_count].rear_right_wheel_angular_velocity = \
            self.supervisorControl.get_obj_field(self.vehicles[self.total_vhc_count],
                                                 "rear_right_wheel_angular_velocity")
        self.vehicles[self.total_vhc_count].rear_left_wheel_angular_velocity = \
            self.supervisorControl.get_obj_field(self.vehicles[self.total_vhc_count],
                                                 "rear_left_wheel_angular_velocity")
        self.vehicles[self.total_vhc_count].current_position = \
            self.supervisorControl.get_obj_position_3D(self.vehicles[self.total_vhc_count])
        # We use current vehicle index as its id as well:
        self.vehicle_dictionary[self.vehicles[self.total_vhc_count].def_name] = self.total_vhc_count
        if vehicle_type == self.VHC_VUT:
            self.VUT_dictionary[self.vehicles[self.total_vhc_count].def_name] = self.total_vhc_count
        if vehicle_type == self.VHC_DUMMY:
            self.dummy_vhc_dictionary[self.vehicles[self.total_vhc_count].def_name] = self.total_vhc_count
        self.total_vhc_count += 1

    def update_vehicle_states(self, vhc):
        """Update the current states of the vehicle."""
        vhc.current_position = self.supervisorControl.get_obj_position_3D(vhc)
        if vhc.previous_position is None:
            curr_velocity = 0.0
        else:
            curr_velocity = ((vhc.current_position[0] - vhc.previous_position[0])
                             / (self.time_step * float(self.num_of_steps_for_jerk_calc)))
        curr_acc = (curr_velocity - vhc.previous_velocity) / (self.time_step * float(self.num_of_steps_for_jerk_calc))
        self.acc_arr[self.cur_jerk_compute_index] = curr_acc
        if self.step_counter > self.num_of_steps_for_jerk_calc + 3:
            cur_acc_avg = (self.acc_arr[self.cur_jerk_compute_index]
                           + self.acc_arr[(self.cur_jerk_compute_index-1) % len(self.acc_arr)]
                           + self.acc_arr[(self.cur_jerk_compute_index-1) % len(self.acc_arr)]) / 3.0
            prev_acc_avg = (self.acc_arr[self.cur_jerk_compute_index-self.num_of_steps_for_jerk_calc]
                            + self.acc_arr[(self.cur_jerk_compute_index-self.num_of_steps_for_jerk_calc-1)
                                           % len(self.acc_arr)] +
                            self.acc_arr[(self.cur_jerk_compute_index-self.num_of_steps_for_jerk_calc-2)
                                         % len(self.acc_arr)]) / 3.0
            curr_jerk = (cur_acc_avg - prev_acc_avg) / (self.time_step * float(self.num_of_steps_for_jerk_calc))
        else:
            curr_jerk = 0.0
        self.cur_jerk_compute_index = (self.cur_jerk_compute_index + 1) % (self.num_of_steps_for_jerk_calc + 3)
        vhc.previous_position = vhc.current_position[:]
        vhc.previous_velocity = curr_velocity
        vhc.previous_acceleration = curr_acc
        vhc.current_acceleration = curr_acc
        vhc.current_jerk = curr_jerk
        vhc.current_velocity = self.supervisorControl.get_obj_velocity(vhc)
        # print 'computed velocity: {} read velocity: {}'.format(temp_velocity, vhc.current_velocity[0])
        # print 'vhc.current_velocity:{} previous_velocity: {}'.format(vhc.current_velocity, previous_velocity)
        # np.subtract(vhc.current_velocity[0:3], previous_velocity)/self.time_step
        # vhc.current_acceleration = list(vhc.current_acceleration)
        # (vhc.current_acceleration[0] - prev_acc[0])/self.time_step
        # print 'vhc: {}, current_acceleration = {} prev_acc = {} current_jerk = {}'.format(vhc.def_name,
        # vhc.current_acceleration[0], prev_acc[0], vhc.current_jerk)
        # vhc.current_jerk = list(vhc.current_jerk)
        vhc.speed = math.sqrt(vhc.current_velocity[0]**2 + vhc.current_velocity[1]**2 + vhc.current_velocity[2]**2)
        vhc.state_record_time = self.current_sim_time

    def update_all_vehicles_states(self):
        """Updates the state of the all vehicles."""
        for vhc in self.vehicles:
            self.update_vehicle_states(vhc)

    def get_reference_value(self, ref_index, ref_field, current_sim_time):
        """Get value of the reference field of the indexed vehicle at the given time."""
        if ref_index == 0:  # reference is time
            ret_val = current_sim_time
        else:
            vhc = self.vehicles[ref_index-1]
            if ref_field == 0:
                ret_val = vhc.speed
            elif ref_field == 1:
                pos = self.supervisorControl.get_obj_position_3D(vhc)
                ret_val = pos[0]
            elif ref_field == 2:
                pos = self.supervisorControl.get_obj_position_3D(vhc)
                ret_val = pos[1]
            elif ref_field == 3:
                pos = self.supervisorControl.get_obj_position_3D(vhc)
                ret_val = pos[2]
            else:
                ret_val = 0.0
        return ret_val

    def transmit_all_vhc_positions(self, emitter):
        """Transmit all vehicle positions through emitter."""
        for vhc in self.vehicles:
            self.controller_comm_interface.transmit_vehicle_position_message(emitter, vhc.id, vhc.current_position)

    def transmit_init_controller_params(self, emitter):
        """Transmit the neural network controller parameters."""
        if self.has_sent_controller_config is False:
            for vhc in self.vehicles:
                for c_param in vhc.controller_parameters:
                    self.controller_comm_interface.transmit_set_controller_parameters_message(
                        emitter=emitter,
                        vhc_id=c_param.vehicle_id,
                        parameter_name=c_param.parameter_name,
                        parameter_data=c_param.parameter_data)
                    time.sleep(0.1)
            self.has_sent_controller_config = True

    def apply_manual_position_control(self, vhc_id):
        """Manually control the position of the vehicle."""
        vhc = self.vehicles[self.dummy_vhc_dictionary[vhc_id]]
        pos = self.supervisorControl.get_obj_position_3D(vhc)
        for s in vhc.signal:
            reference_value = self.get_reference_value(s.ref_index, s.ref_field, self.current_sim_time)
            signal_value = s.get_signal_value_corresponding_to_value_of_reference(reference_value,
                                                                                  STaliroSignal.INTERPOLATION_TYPE_NONE)
            # print("reference_value: {} s.ref_index: {} s.ref_field: {} signal_value: {}".format(reference_value,
            # s.ref_index, s.ref_field, signal_value));
            if s.signal_type == s.SIGNAL_TYPE_SPEED:
                # When I change speed aggressively, the vehicle rolls over. Look into this.
                # spd = self.get_velocity(vhc)
                # spd[0] = signal_value
                # self.set_velocity(vhc, spd)
                pos[0] = pos[0] + signal_value * self.time_step
                # print("SIGNAL_TYPE_SPEED : signal_value : {} pos: {}".format(signal_value, pos));
                self.supervisorControl.set_obj_position_3D(vhc, pos)
            if s.signal_type == s.SIGNAL_TYPE_Y_POSITION:
                pos[2] = signal_value
                # print("SIGNAL_TYPE_Y_POSITION : signal_value: {} pos: {}".format(signal_value, pos));
                self.supervisorControl.set_obj_position_3D(vhc, pos)

    def set_time_step(self, time_step):
        """Set the time_step."""
        self.time_step = time_step
        self.num_of_steps_for_jerk_calc = int(math.ceil(0.05 / time_step))
        print('Num of steps for jerk calculation = {}'.format(self.num_of_steps_for_jerk_calc))
        self.acc_arr = [0.0]*(self.num_of_steps_for_jerk_calc+3)

    def get_emitter(self):
        """Returns the supervisor emitter"""
        supervisor_emitter = self.supervisorControl.get_emitter(self.EMITTER_NAME)
        return supervisor_emitter

    def simulate_vehicles(self, current_sim_time_s):
        """Simulation vehicles for one time step."""
        self.current_sim_time = current_sim_time_s
        control_type = 0
        supervisor_emitter = self.get_emitter()
        self.update_all_vehicles_states()
        self.transmit_all_vhc_positions(supervisor_emitter)
        self.transmit_init_controller_params(supervisor_emitter)
        if control_type == 0:
            for vhc_id in self.dummy_vhc_dictionary:
                self.apply_manual_position_control(vhc_id)
        self.step_counter += 1
