import math
import os
import sys
import numpy as np
import copy

filePath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(filePath+"/../base_controller")
sys.path.append(filePath+"/../controller_commons")
sys.path.append(filePath+"/../generic_pid_controller")
sys.path.append(filePath+"/../generic_stanley_controller")

from base_controller import BaseCarController
from controller_commons import *
from generic_pid_controller import *
from generic_stanley_controller import *
from side_collision_avoidance import *
import dssm

# ----------------------------------------------------------------------------
# Cumhur Erkan Tuncali, Arizona State University
# Ref:     A Study on the Traffic Predictive Cruise Control Strategy with 
#          Downstream Traffic Information
# Authors: Sehyun Tak, Sunghoon Kim, Hwasoo Yeo
# ----------------------------------------------------------------------------

class VehiclesStates:
    def __init__(self):
        self.leader_acc = 0.0
        self.leader_speed = 0.0
        self.leader_long_pos = 0.0
        self.acc = 0.0
        self.speed = 0.0
        self.long_pos = 0.0
        self.lat_pos = 0.0
        self.follower_acc = 0.0
        self.follower_speed = 0.0
        self.follower_long_pos = 0.0
        self.record_time = 0.0

class acr_controller(BaseCarController):
    def __init__(self, (car_model, id, desired_velocity)):
        BaseCarController.__init__(self, car_model)
        self.LIDAR_DEVICE_NAME = "vut_lidar"
        self.LIDAR_PERIOD = 10
        self.RADAR_DEVICE_NAME = "vut_radar"
        self.RADAR_PERIOD = 10
        self.WEIGHT_RECEIVER_DEVICE_NAME = "vut_nn_receiver"
        self.WEIGHT_RECEIVER_PERIOD = 10
        self.RECEIVER_DEVICE_NAME = "vut_receiver"
        self.RECEIVER_PERIOD = 10
        self.LEFTFRONT_SENSOR_NAME = "vut_lf_sens"
        self.LEFTFRONT_SENSOR_PERIOD = 10
        self.LEFTREAR_SENSOR_NAME = "vut_lr_sens"
        self.LEFTREAR_SENSOR_PERIOD = 10
        self.RIGHTFRONT_SENSOR_NAME = "vut_rf_sens"
        self.RIGHTFRONT_SENSOR_PERIOD = 10
        self.RIGHTREAR_SENSOR_NAME = "vut_rr_sens"
        self.RIGHTREAR_SENSOR_PERIOD = 10
        self.FRONT_DIST_SENSOR_NAME = "vut_front_sens"
        self.FRONT_DIST_SENSOR_PERIOD = 200
        self.REAR_DIST_SENSOR_NAME = "vut_rear_sens"
        self.REAR_DIST_SENSOR_PERIOD = 200
        self.GPS_DEVICE_NAME = "vut_gps"
        self.GPS_DEVICE_PERIOD = 10
        self.COMPASS_DEVICE_NAME = "vut_compass"
        self.COMPASS_PERIOD = 10
        self.ACCELEROMETER_NAME = "accelerometer"
        self.ACCELEROMETER_PERIOD = 10
        self.debug_mode = False
        self.last_throttle = 0.0
        self.last_steering = 0.0
        self.DESIRED_MAX_ACCELERATION = 2.2 # CitroenCZero can not accelerate more
        self.DESIRED_MIN_ACCELERATION = -9.0
        self.MAXIMUM_STEERING = 0.5
        self.MAXIMUM_STEERING_DELTA = 0.01 # 0 to full steering at around 0.5 seconds
        self.DESIRED_SIDE_DIST = 1.5
        self.MINIMUM_SIDE_DIST = 1.0
        self.SIDE_TTC_THRESHOLD = 2.0
        self.desired_min_spacing = 3.0
        self.desired_velocity = float(desired_velocity)
        self.desired_time_gap = 1.2
        self.desired_lateral_pos = 0.0
        self.prev_left_coll_avoid_lat_target = self.desired_lateral_pos
        self.ref_orientation = 0.0
        self.FORWARD_SENSING_RANGE = 60.0
        self.longitudinal_pid = GenericPIDController()
        self.lateral_controller = GenericStanleyController()
        self.LONG_PID_P = 1.5
        self.LONG_PID_I = 0.1
        self.LONG_PID_D = 8.5
        self.PID_INTEGRATOR_MIN = -500.0
        self.PID_INTEGRATOR_MAX = 500.0
        self.STEP_TIME = None
        self.STANLEY_K = 2.5
        self.STANLEY_K2 = 1.0
        self.STANLEY_K3 = 1.0
        self.touch_sensor = None
        self.compass_device = None
        self.leftfront_sensor = None
        self.leftrear_sensor = None
        self.rightfront_sensor = None
        self.rightrear_sensor = None
        self.frontdist_sensor = None
        self.prev_left_target = None
        self.prev_right_target = None
        self.receiver_device = None
        self.weight_receiver_device = None
        self.collAvoidance = None
        self.gps_device = None
        self.accelerometer = None
        self.radar_device = None
        self.my_id = int(id)
        self.acr_acc_threshold = 0.7
        self.acr_dec_threshold = 0.85
        self.a_max_constant_c = 3.68
        self.a_max_constant_d = -0.03
        self.b_max = -6.0
        self.max_acc_variation = 10.0
        self.reaction_time = self.FRONT_DIST_SENSOR_PERIOD / 1000.0 # Converting to ms.
        self.desired_min_rear_spacing = 2.0
        self.ego_dssm_weight = 5.0
        self.target_time_gap_weight = 2.0
        self.prev_states = None
        self.curr_states = None
        self.MIN_ACC = -6.0
        self.MAX_ACC = 2.5
        self.MIN_SPEED = 0.0
        self.MAX_SPEED = 50.0 # m/s
        self.EXTERNAL_VHC_MIN_ACC = -8.0
        self.EXTERNAL_VHC_MAX_ACC = 4.0
        self.EXTERNAL_VHC_MIN_SPEED = 0.0
        self.EXTERNAL_VHC_MAX_SPEED = 70.0 # m/s

    def set_step_time_dependent_parameters(self):
        self.MAX_NEGATIVE_THROTTLE_CHANGE = -self.STEP_TIME / 0.1 #I want to give full break in 0.1 sec.
        self.MAX_POSITIVE_THROTTLE_CHANGE = self.STEP_TIME / 1.0 #I want to give full throttle from 0 in 1 sec.
        self.longitudinal_pid.set_output_range(self.MAX_NEGATIVE_THROTTLE_CHANGE, self.MAX_POSITIVE_THROTTLE_CHANGE)

    def record_states(self):
        self.prev_states = copy.copy(self.curr_states)
        if self.gps_device is not None:
            if self.curr_states is None:
                self.curr_states = VehiclesStates()
            self.curr_states.record_time = self.get_sim_time()
            if self.prev_states is not None:
                time_diff = self.curr_states.record_time - self.prev_states.record_time

            pos = self.gps_device.getValues()
            self.curr_states.long_pos = pos[0]
            self.curr_states.lat_pos = pos[2]
            self.curr_states.speed = np.linalg.norm(self.gps_device.getSpeed()) # GPS speed values are m/s
            if self.prev_states is not None and time_diff > 0.0:
                self.curr_states.acc = (self.curr_states.speed - self.prev_states.speed) / time_diff
                #print 'self.curr_states.speed: {} self.curr_states.record_time :{} self.prev_states.record_time: {}'.format(self.curr_states.speed, self.curr_states.record_time, self.prev_states.record_time)
            else:
                self.curr_states.acc = 0.0

            front_dist = self.frontdist_sensor.getValue()
            rear_dist = self.reardist_sensor.getValue()
            #print 'front_dist : {}, rear_dist = {} speed: {}'.format(front_dist, rear_dist, self.gps_device.getSpeed())
            self.curr_states.leader_long_pos = self.curr_states.long_pos + 1.6 + front_dist
            self.curr_states.follower_long_pos = self.curr_states.long_pos - 1.6 - rear_dist
            if self.prev_states is not None and time_diff > 0.0:
                self.curr_states.leader_speed = (self.curr_states.leader_long_pos - self.prev_states.leader_long_pos) / time_diff
                self.curr_states.follower_speed = (self.curr_states.follower_long_pos - self.prev_states.follower_long_pos) / time_diff
                self.curr_states.leader_acc = (self.curr_states.leader_speed - self.prev_states.leader_speed) / time_diff
                self.curr_states.follower_acc = (self.curr_states.follower_speed - self.prev_states.follower_speed) / time_diff
            else:
                self.curr_states.leader_speed = 0.0
                self.curr_states.follower_speed = 0.0
                self.curr_states.leader_acc = 0.0
                self.curr_states.follower_acc = 0.0

    def compute_a_max(self, speed): # Eqn. (6)
        a_max = self.a_max_constant_c  + self.a_max_constant_d * speed
        return a_max

    def compute_desired_acceleration(self, dssm, a_max, b_max): # Eqn. (5)
        if dssm < 0.0:
            dssm = 0.0
        if self.acr_acc_threshold <= dssm <= self.acr_dec_threshold:
            a_desired = 0.0
        elif dssm < self.acr_acc_threshold:
            a_desired = a_max * (self.acr_acc_threshold - dssm) / self.acr_acc_threshold
        else:
            a_desired = -b_max * (self.acr_dec_threshold - dssm) / (1.2 - self.acr_dec_threshold)
        #print 'dssm = {}, desired_Acc = {}'.format(dssm, a_desired)
        return a_desired

    def do_device_initializations(self):
        # Device and sensor initializations:
        self.accelerometer = self.getAccelerometer(self.ACCELEROMETER_NAME)
        if self.accelerometer is not None:
            self.accelerometer.enable(self.ACCELEROMETER_PERIOD)
        self.compass_device = self.getCompass(self.COMPASS_DEVICE_NAME)
        if self.compass_device is not None:
            self.compass_device.enable(self.COMPASS_PERIOD)
        self.weight_receiver_device = self.getReceiver(self.WEIGHT_RECEIVER_DEVICE_NAME)
        if self.weight_receiver_device is not None:
            self.weight_receiver_device.enable(self.WEIGHT_RECEIVER_PERIOD)
        self.receiver_device = self.getReceiver(self.RECEIVER_DEVICE_NAME)
        if self.receiver_device is not None:
            self.receiver_device.enable(self.RECEIVER_PERIOD)
            if self.collAvoidance is None:
                self.collAvoidance = SideCollisionAvoidance()
            self.collAvoidance.register_vhc_pos_receiver(self.receiver_device, self.my_id)
        self.radar_device = self.getRadar(self.RADAR_DEVICE_NAME)
        if self.radar_device is not None:
            self.radar_device.enable(self.RADAR_PERIOD)
            if self.collAvoidance is None:
                self.collAvoidance = SideCollisionAvoidance()
            self.collAvoidance.register_sensor()
            self.collAvoidance.sensor_array[-1].location = CollisionAvoidanceSensor.FRONT
            self.collAvoidance.sensor_array[-1].type = CollisionAvoidanceSensor.RADAR
            self.collAvoidance.sensor_array[-1].device = self.radar_device
        self.frontdist_sensor = self.getDistanceSensor(self.FRONT_DIST_SENSOR_NAME)
        if self.frontdist_sensor is not None:
            self.frontdist_sensor.enable(self.FRONT_DIST_SENSOR_PERIOD)
            if self.collAvoidance is None:
                self.collAvoidance = SideCollisionAvoidance()
            self.collAvoidance.register_sensor()
            self.collAvoidance.sensor_array[-1].location = CollisionAvoidanceSensor.FRONT
            self.collAvoidance.sensor_array[-1].type = CollisionAvoidanceSensor.SONAR
            self.collAvoidance.sensor_array[-1].device = self.frontdist_sensor
        self.reardist_sensor = self.getDistanceSensor(self.REAR_DIST_SENSOR_NAME)
        if self.reardist_sensor is not None:
            self.reardist_sensor.enable(self.REAR_DIST_SENSOR_PERIOD)
            if self.collAvoidance is None:
                self.collAvoidance = SideCollisionAvoidance()
            self.collAvoidance.register_sensor()
            self.collAvoidance.sensor_array[-1].location = CollisionAvoidanceSensor.REAR
            self.collAvoidance.sensor_array[-1].type = CollisionAvoidanceSensor.SONAR
            self.collAvoidance.sensor_array[-1].device = self.reardist_sensor
        self.leftfront_sensor = self.getDistanceSensor(self.LEFTFRONT_SENSOR_NAME)
        if self.leftfront_sensor is not None:
            self.leftfront_sensor.enable(self.LEFTFRONT_SENSOR_PERIOD)
            if self.collAvoidance is None:
                self.collAvoidance = SideCollisionAvoidance()
            self.collAvoidance.register_sensor()
            self.collAvoidance.sensor_array[-1].location = CollisionAvoidanceSensor.LEFTFRONT
            self.collAvoidance.sensor_array[-1].type = CollisionAvoidanceSensor.SONAR
            self.collAvoidance.sensor_array[-1].device = self.leftfront_sensor
        self.leftrear_sensor = self.getDistanceSensor(self.LEFTREAR_SENSOR_NAME)
        if self.leftrear_sensor is not None:
            self.leftrear_sensor.enable(self.LEFTREAR_SENSOR_PERIOD)
            if self.collAvoidance is None:
                self.collAvoidance = SideCollisionAvoidance()
            self.collAvoidance.register_sensor()
            self.collAvoidance.sensor_array[-1].location = CollisionAvoidanceSensor.LEFTREAR
            self.collAvoidance.sensor_array[-1].type = CollisionAvoidanceSensor.SONAR
            self.collAvoidance.sensor_array[-1].device = self.leftrear_sensor
        self.rightfront_sensor = self.getDistanceSensor(self.RIGHTFRONT_SENSOR_NAME)
        if self.rightfront_sensor is not None:
            self.rightfront_sensor.enable(self.RIGHTFRONT_SENSOR_PERIOD)
            if self.collAvoidance is None:
                self.collAvoidance = SideCollisionAvoidance()
            self.collAvoidance.register_sensor()
            self.collAvoidance.sensor_array[-1].location = CollisionAvoidanceSensor.RIGHTFRONT
            self.collAvoidance.sensor_array[-1].type = CollisionAvoidanceSensor.SONAR
            self.collAvoidance.sensor_array[-1].device = self.rightfront_sensor
        self.rightrear_sensor = self.getDistanceSensor(self.RIGHTREAR_SENSOR_NAME)
        if self.rightrear_sensor is not None:
            self.rightrear_sensor.enable(self.RIGHTREAR_SENSOR_PERIOD)
            if self.collAvoidance is None:
                self.collAvoidance = SideCollisionAvoidance()
            self.collAvoidance.register_sensor()
            self.collAvoidance.sensor_array[-1].location = CollisionAvoidanceSensor.RIGHTREAR
            self.collAvoidance.sensor_array[-1].type = CollisionAvoidanceSensor.SONAR
            self.collAvoidance.sensor_array[-1].device = self.rightrear_sensor
        self.gps_device = self.getGPS(self.GPS_DEVICE_NAME)
        if self.gps_device is not None:
            self.gps_device.enable(self.GPS_DEVICE_PERIOD)

    def run(self):
        contr_weights_received = False

        self.do_device_initializations()
        self.longitudinal_pid.set_parameters(self.LONG_PID_P, self.LONG_PID_I, self.LONG_PID_D)
        self.longitudinal_pid.set_integrator_value_range(self.PID_INTEGRATOR_MIN, self.PID_INTEGRATOR_MAX)
        self.lateral_controller.set_parameters(self.STANLEY_K, self.STANLEY_K2, self.STANLEY_K3)

        self.start_car()
        while True:
            self.step()
            if self.STEP_TIME is None:
                self.STEP_TIME = self.get_sim_time() # In seconds. So, this will be something like 0.01
                self.set_step_time_dependent_parameters()
            if self.collAvoidance is not None:
                self.collAvoidance.collisionObjectsArr = []
                self.collAvoidance.find_risky_objects()
            self.record_states() # This has to be called after collAvoidance.find_risky_objects

            if contr_weights_received is False:
                (is_recv, recv_msg) = get_receiver_message(self.weight_receiver_device)
                if is_recv:
                    contr_weights = receive_nn_weights(recv_msg)
                    if self.debug_mode:
                        print 'received weights: {}'.format(contr_weights)
                    self.longitudinal_pid.set_parameters(contr_weights[0], contr_weights[1], contr_weights[2])
                    contr_weights_received = True

            if contr_weights_received and self.prev_states is not None:
                # --- Compute Lateral Control ---
                if self.collAvoidance is not None:
                    position = [self.curr_states.long_pos, 0.32, self.curr_states.lat_pos]
                    (target_lat_pos, target_velocity) = self.collAvoidance.compute_side_coll_avoidance_maneuver(position, self.desired_velocity, self.desired_lateral_pos)
                else:
                    target_lat_pos = self.desired_lateral_pos
                    target_velocity = self.desired_velocity
                lateral_err = target_lat_pos - self.curr_states.lat_pos

                orientation = get_bearing(self.compass_device)
                if not math.isnan(orientation):
                    orient_err = orientation - self.ref_orientation
                    if orient_err > math.pi:
                        orient_err -= 2.0 * math.pi
                    if orient_err < -math.pi:
                        orient_err += 2.0 * math.pi
                else:
                    orient_err = 0.0

                cur_speed = self.get_current_speed() # This is in km/h
                if not math.isnan(cur_speed):
                    # Limiting steering to avoid rolling over:
                    if cur_speed < 40.0:
                        self.MAXIMUM_STEERING = 0.5
                    else:
                        self.MAXIMUM_STEERING = max(0.5 - (cur_speed - 40.0) / 100.0, 0.1)
                    self.MAXIMUM_STEERING_DELTA = self.STEP_TIME * self.MAXIMUM_STEERING / 0.5 # Reach max steering in 0.5 secs
                    lat_control = self.lateral_controller.compute(orient_err, lateral_err, cur_speed)
                    if lat_control - self.last_steering > self.MAXIMUM_STEERING_DELTA:
                        lat_control = self.last_steering + self.MAXIMUM_STEERING_DELTA
                    elif self.last_steering - lat_control > self.MAXIMUM_STEERING_DELTA:
                        lat_control = self.last_steering - self.MAXIMUM_STEERING_DELTA
                else:
                    lat_control = self.last_steering
                lat_control = min(self.MAXIMUM_STEERING, max(-self.MAXIMUM_STEERING, lat_control))
                #print('orient_err: {}. lateral_err: {}, cur_Speed:{}, lat_control:{}'.format(orient_err, lateral_err, cur_speed, lat_control))

                # --- Compute Longitudinal Control ---
                if self.debug_mode:
                    x = np.array([self.curr_states.leader_acc, self.curr_states.leader_speed, self.curr_states.leader_long_pos, \
                                  self.curr_states.acc, self.curr_states.speed, self.curr_states.long_pos, \
                                  self.curr_states.follower_acc, self.curr_states.follower_speed, self.curr_states.follower_long_pos])
                    print list(x)

                spacing_e_l = self.curr_states.leader_long_pos - self.curr_states.long_pos - self.desired_min_spacing 
                (dssm_e, b_e) = dssm.compute_dssm_and_braking(spacing_e_l, self.curr_states.speed, self.curr_states.acc, self.b_max, self.max_acc_variation, self.reaction_time,
                                                            self.curr_states.leader_speed, self.curr_states.leader_acc, self.b_max, self.max_acc_variation)
                desired_acc = self.compute_desired_acceleration(dssm_e, self.MAX_ACC, self.b_max)

                acc_err = desired_acc - self.curr_states.acc
                if math.isnan(acc_err):
                    acc_err = 0.0

                throttle_delta = self.longitudinal_pid.compute_no_derivative_kick(acc_err, self.curr_states.acc)
                if self.debug_mode:
                    print 'acc_err: {}, cur_acc: {}, throttle_delta: {}, prev_throttle: {}'.format(acc_err, self.curr_states.acc, throttle_delta, self.last_throttle)
                self.last_throttle += throttle_delta
                self.last_throttle = min(max(-1.0, self.last_throttle), 1.0)
                if desired_acc < 0.0 and self.last_throttle > 0.0:
                    self.last_throttle = 0.0
                #print 'acc_target: {} acc err: {}, throttle: {}, throttle_delta: {}'.format(desired_acc, acc_err, self.last_throttle, throttle_delta)

                self.set_control_actions_throttle_angle(self.last_throttle, lat_control)
                self.last_steering = lat_control
