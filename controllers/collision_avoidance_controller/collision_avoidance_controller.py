import math
import os
import sys
import struct
import numpy as np

filePath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(filePath+"/../base_controller")
sys.path.append(filePath+"/../controller_commons")

from base_controller import BaseCarController
from controller_commons import *

class VehicleStates:
    def __init__(self):
        self.BRAKE_SYSTEM_DELAY = 0.1
        self.CONTROLLER_DELAY = 0.1
        self.MAX_DECELERATION = 8.0
        self.long_pos = 0.0
        self.lat_pos = 0.0
        self.long_vel = 0.0
        self.lat_vel = 0.0

class collision_avoidance_controller(BaseCarController):
    def __init__(self, (car_model,)):
        BaseCarController.__init__(self, car_model)
        #self.RECEIVER_DEVICE_NAME = "vut_receiver"
        #self.RECEIVER_PERIOD = 10
        self.LIDAR_DEVICE_NAME = "vut_lidar"
        self.LIDAR_PERIOD = 10
        self.debug_mode = False
        self.last_throttle = 0.0
        self.last_steering = 0.0

    def compute_friction_scaling(cur_friction):
        # Normally this function must compute the following:
        # f(u) = f(u_min) = u_norm/u_min, if u <= u_min
        # f(u) = f(u_norm) = 1, if u >= u_norm
        # f(u) = f(u_min) + (u - u_min)*((f(u_norm) - f(u_min))/(u_norm - u_min)), if u_min < u < u_norm
        # However, we don't have these values for Webots for now. So, we consider a constant friction u = u_norm
        # Hence, the function will just return 1 for now.
        return 1

    def compute_warning_index(self, ego_vhc_state, target_vhc_state):
        ROAD_FRICTION = 1.0
        c = target_vhc_state.long_pos - ego_vhc_state.long_pos
        v_rel = ego_vhc_state.long_vel - target_vhc_state.long_vel
        d_br = v_rel*ego_vhc_state.BRAKE_SYSTEM_DELAY + compute_friction_scaling(ROAD_FRICTION)*((ego_vhc_state.long_vel**2 - target_vhc_state.long_vel**2)/(2*ego_vhc_state.MAX_DECELERATION))
        d_w = d_br + ego_vhc_state.long_vel*ego_vhc_state.CONTROLLER_DELAY
        return (c-d_br)/(d_w - d_br)

    def run(self):
        receiver_device = self.getReceiver(self.RECEIVER_DEVICE_NAME)
        receiver_device.enable(self.RECEIVER_PERIOD)
        #dist_sensor = self.getDistanceSensor("distance")
        #dist_sensor.enable(10)
        #self.lidar_sensor = self.getLidar(self.LIDAR_DEVICE_NAME)
        #self.lidar_sensor.enable(self.LIDAR_PERIOD)
        self.start_car()
        while True:
            self.step()
            #dist_val = dist_sensor.getValue()
            #print("distance: {}".format(dist_val))
            # Receive Message from Receiver:
            #message = []
            #while receiver_device.getQueueLength() > 0:
            #    message += receiver_device.getData()
            #    receiver_device.nextPacket()
            #message = ''.join(message)

