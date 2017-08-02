import math
import os
import sys
import struct
import numpy as np

filePath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(filePath+"/../base_controller")
sys.path.append(filePath+"/../generic_pid_controller")
sys.path.append(filePath+"/../generic_stanley_controller")
from base_controller import BaseCarController
from generic_pid_controller import *
from generic_stanley_controller import *
from coordinate_system import CoordinateSystem

# **********************************************************************************************
# This controller is a simple controller for vehicles. Drives the car straight with 0.5 throttle
# **********************************************************************************************
class distance_keep_pid(BaseCarController):
    def __init__(self, (car_model, self_vhc_id, target_vhc_id, ref_long_distance, ref_lat_distance)):
        BaseCarController.__init__(self, car_model)
        self.EMITTER_DEVICE_NAME = "emitter"
        self.RECEIVER_DEVICE_NAME = "dummy_vhc_receiver"
        self.COMPASS_DEVICE_NAME = "dummy_vhc_compass"
        self.STEP_TIME = 10
        self.TOUCH_SENSOR_NAME = "touch sensor"
        self.target_throttle = 0.3
        self.LONG_PID_P = 0.0002
        self.LONG_PID_I = 0.0000005
        self.LONG_PID_D = 0.28
        self.MAX_NEGATIVE_THROTTLE_CHANGE = -float(self.STEP_TIME)/3000.0
        self.MAX_POSITIVE_THROTTLE_CHANGE = float(self.STEP_TIME)/1000.0 #I want to give full throttle from 0 in 1 sec.
        self.STANLEY_K = 0.25
        self.STANLEY_K2 = 0.3
        self.STANLEY_K3 = 1.0
        self.ref_long_distance = float(ref_long_distance)
        self.ref_lat_distance = float(ref_lat_distance)
        self.ref_orientation = 0.0
        self.received_message = []
        self.TARGET_VHC_ID = int(target_vhc_id)
        self.SELF_VHC_ID = int(self_vhc_id)
        self.touch_sensor = None
        self.compass_device = None
        self.receiver_device = None
        self.emitter_device = None
        self.prev_long_control = 0.0
        self.prev_lat_control = 0.0
        self.longitudinal_pid = GenericPIDController()
        self.lateral_controller = GenericStanleyController()

    def set_target_throttle(self, throttle):
        self.target_throttle = throttle

    def set_ref_long_distance(self, distance):
        self.ref_long_distance = distance

    def set_ref_lat_distance(self, distance):
        self.ref_lat_distance = distance

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
        return (is_received, received_message)

    def receive_vhc_pos(self, target_vhc_id):
        pos = [0.0, 0.0, 0.0]
        # Evaluate message
        if len(self.received_message) > 0:
            cmd = struct.unpack('B', self.received_message[0:struct.calcsize('B')])[0] # TODO: Check cmd for different type of messages
            cur_index = struct.calcsize('B')
            num_vehicles = struct.unpack('h', self.received_message[cur_index:cur_index+struct.calcsize('h')])[0]
            cur_index += struct.calcsize('h')
            for i in range(num_vehicles):
                (vehicle_id, pos[0], pos[1], pos[2]) = struct.unpack("Bddd", self.received_message[cur_index:cur_index+struct.calcsize("Bddd")])
                if vehicle_id == target_vhc_id:
                    break
                cur_index += struct.calcsize("Bddd")
        return pos    
    
    def get_bearing(self):
        # Return the vehicle's heading in radians. pi/2 is straight up, 0 is straight right.
        compass_data = [0.0, 0.0, 0.0]
        if self.compass_device is not None:
            compass_data = self.compass_device.getValues()
        radians = math.atan2(compass_data[CoordinateSystem.LAT_AXIS], compass_data[CoordinateSystem.LONG_AXIS])
        radians += math.pi/2.0
        if radians > 2.0*math.pi:
            radians -= 2.0*math.pi
        return radians

    def transmit_collision_info(self):
        if self.touch_sensor is not None:
            touch_sensor_type = self.touch_sensor.getType()
            if touch_sensor_type == 'force-3d':
                touch_sensor_val = self.touch_sensor.getValues()
                # print touch_sensor_val
                message = struct.pack("ddd", touch_sensor_val[0], touch_sensor_val[1], touch_sensor_val[2])
                if self.emitter_device is not None:
                    self.emitter_device.send(message)
            else:
                touch_sensor_val = self.touch_sensor.getValue()
                # print touch_sensor_val
                message = struct.pack("ddd", -100, -100, touch_sensor_val)
                if self.emitter_device is not None:
                    self.emitter_device.send(message)

    def run(self):
        self.longitudinal_pid.set_parameters(self.LONG_PID_P, self.LONG_PID_I, self.LONG_PID_D)
        self.longitudinal_pid.set_output_range(self.MAX_NEGATIVE_THROTTLE_CHANGE, self.MAX_POSITIVE_THROTTLE_CHANGE)
        self.longitudinal_pid.set_integrator_value_range(-500.0, 500.0);
        self.lateral_controller.set_parameters(self.STANLEY_K, self.STANLEY_K2, self.STANLEY_K3)
        # Get the emitter device for later access.
        self.emitter_device = self.getEmitter(self.EMITTER_DEVICE_NAME)
        # Get the touch sensor for later access.
        self.touch_sensor = self.getTouchSensor(self.TOUCH_SENSOR_NAME)
        if self.touch_sensor is not None:
            self.touch_sensor.enable(self.STEP_TIME)
        self.receiver_device = self.getReceiver(self.RECEIVER_DEVICE_NAME)
        if self.receiver_device is not None:
            self.receiver_device.enable(self.STEP_TIME)
        # Enable compass device.
        self.compass_device = self.getCompass(self.COMPASS_DEVICE_NAME)
        if self.compass_device is not None:
            self.compass_device.enable(self.STEP_TIME)
        self.start_car()
        while True:
            self.step()
            # Receive message
            (is_rcv, temp_message) = self.get_receiver_message(self.receiver_device)
            if is_rcv:
                self.received_message = temp_message
            target_vhc_pos = self.receive_vhc_pos(self.TARGET_VHC_ID)
            self_vhc_pos = self.receive_vhc_pos(self.SELF_VHC_ID)
            if len(target_vhc_pos) > 2 and len(self_vhc_pos) > 2:
                distance = np.subtract(target_vhc_pos, self_vhc_pos)
                long_distance = distance[CoordinateSystem.LONG_AXIS]
                #if long_distance < -8.0:
                #    self.ref_long_distance = 0.0
                #    self.ref_lat_distance = 0.0
                long_control_delta = self.longitudinal_pid.compute(long_distance - self.ref_long_distance)
                long_control = self.prev_long_control + long_control_delta
                long_control = max(min(long_control, 1.0), -1.0)
                #print('long_distance: {}, long_control: {} long_control_delta: {}'.format(long_distance, long_control, long_control_delta))
                lat_distance = distance[CoordinateSystem.LAT_AXIS]
                orientation = self.get_bearing()
                cur_speed = self.get_current_speed()
                orient_err = orientation - self.ref_orientation
                if orient_err > math.pi:
                    orient_err -= 2.0*math.pi
                if orient_err < -math.pi:
                    orient_err += 2.0*math.pi
                if not math.isnan(cur_speed):
                    lat_control = self.lateral_controller.compute(orient_err, lat_distance - self.ref_lat_distance, cur_speed)
                else:
                    lat_control = 0.0
            else:
                long_control = self.prev_long_control
                lat_control = self.prev_lat_control

            #print('long_dist: {}, lat_dist: {} orientation_err: {} long_cntr: {}, lat_cntr: {}'.format(long_distance, lat_distance, orient_err, long_control, lat_control))
            self.set_control_actions_throttle_angle(long_control, lat_control)
            self.prev_long_control = long_control
            self.prev_lat_control = lat_control
            self.transmit_collision_info()
