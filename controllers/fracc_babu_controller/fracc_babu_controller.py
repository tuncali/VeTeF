import math
import os
import sys
#import struct
import numpy as np

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

# ----------------------------------------------------------------------------
# Ref:     Design and Analysis of Full Range Adaptive Cruise Control with
#          Integrated Collision A voidance Strategy,
# Authors: Freddy A. Mullakkal-Babu, Meng Wang, Bart van Arem, Riender Happee
# ----------------------------------------------------------------------------

class VehicleStates:
    def __init__(self):
        self.BRAKE_SYSTEM_DELAY = 0.1
        self.CONTROLLER_DELAY = 0.1
        self.MAX_DECELERATION = 8.0
        self.long_pos = 0.0
        self.lat_pos = 0.0
        self.long_vel = 0.0
        self.lat_vel = 0.0

class fracc_babu_controller(BaseCarController):
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
        self.GPS_DEVICE_NAME = "vut_gps"
        self.GPS_DEVICE_PERIOD = 10
        self.COMPASS_DEVICE_NAME = "vut_compass"
        self.COMPASS_PERIOD = 10
        self.debug_mode = False
        self.last_throttle = 0.0
        self.last_steering = 0.0
        self.prev_speed = None
        self.prev_left_dist = None
        self.prev_right_dist = None
        self.DESIRED_MAX_ACCELERATION = 2.2 # CitroenCZero can not accelerate more
        self.DESIRED_MIN_ACCELERATION = -9.0
        self.MAXIMUM_STEERING = 0.5
        self.MAXIMUM_STEERING_DELTA = 0.01 # 0 to full steering at around 0.5 seconds
        self.DESIRED_SIDE_DIST = 1.5
        self.MINIMUM_SIDE_DIST = 1.0
        self.SIDE_TTC_THRESHOLD = 2.0
        #self.param_K1 = 0.18
        self.param_K1 = 0.54
        #self.param_K2 = 1.93
        self.param_K2 = 1.84
        #self.param_P = 100.0
        self.param_P = 150.0
        #self.param_Q = 1.0
        self.param_Q = 0.75
        self.desired_min_spacing = 3.0
        self.desired_velocity = float(desired_velocity)
        self.desired_time_gap = 1.2
        self.desired_lateral_pos = 0.0
        self.prev_left_coll_avoid_lat_target = self.desired_lateral_pos
        self.coll_avoid_counter = -1
        self.ref_orientation = 0.0
        self.FORWARD_SENSING_RANGE = 60.0
        self.longitudinal_pid = GenericPIDController()
        self.lateral_controller = GenericStanleyController()
        '''
        self.LONG_PID_P = 0.015
        self.LONG_PID_I = 0.001
        self.LONG_PID_D = 8.5
        '''
        self.LONG_PID_P = 1.5
        self.LONG_PID_I = 0.1
        self.LONG_PID_D = 8.5
        self.PID_INTEGRATOR_MIN = -500.0
        self.PID_INTEGRATOR_MAX = 500.0
        self.STEP_TIME = 10
        self.MAX_NEGATIVE_THROTTLE_CHANGE = -float(self.STEP_TIME)/100.0
        self.MAX_POSITIVE_THROTTLE_CHANGE = float(self.STEP_TIME)/1000.0 #I want to give full throttle from 0 in 1 sec.
        #self.MAX_NEGATIVE_THROTTLE_CHANGE = -0.5
        #self.MAX_POSITIVE_THROTTLE_CHANGE = 0.5
        #self.STANLEY_K = 0.25
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
        self.sideCollAvoidance = None
        self.my_id = int(id)

    def compute_spacing_error(self, spacing, velocity, desired_velocity): # Eqn. (3)
        if self.debug_mode:
            if spacing - self.desired_min_spacing - max(velocity, 0.5)*self.desired_time_gap < (desired_velocity - velocity)*self.desired_time_gap:
                print 'min is spacing (spacing: {})'.format(spacing)
            else:
                print 'min is velocity'
            print 'spacing err: {}, velocity err: {}'.format(spacing - self.desired_min_spacing - velocity*self.desired_time_gap, (desired_velocity - velocity)*self.desired_time_gap)
        
        return min((spacing - self.desired_min_spacing - max(velocity, 0.5)*self.desired_time_gap, 
                   (desired_velocity - velocity)*self.desired_time_gap)) # at very low speeds spacing error is not very dependable. So, I have made it 0.5 at minimum.

    def compute_error_response(self, spacing): # Eqn. (4)
        r = 1.0 + (-1.0/(1.0 + self.param_Q*math.exp(-spacing/self.param_P)))
        return r

    def compute_desired_acceleration(self, spacing, velocity, preceding_velocity, desired_velocity): # Eqn. (2)
        if spacing <= self.FORWARD_SENSING_RANGE:
            u = self.param_K1*self.compute_spacing_error(spacing, velocity, desired_velocity)
            u += self.param_K2*(preceding_velocity - velocity)*self.compute_error_response(spacing)
            if self.debug_mode:
                print 'u_desired: {} + {} = {}'.format(self.param_K1*self.compute_spacing_error(spacing, velocity, desired_velocity), self.param_K2*(preceding_velocity - velocity)*self.compute_error_response(spacing), u)
        else:
            u = self.param_K1*(desired_velocity - velocity)*self.desired_time_gap
        u = max(min(u, self.DESIRED_MAX_ACCELERATION), self.DESIRED_MIN_ACCELERATION)
        return u

    def get_bearing(self):
        # Return the vehicle's heading in radians. pi/2 is straight up, 0 is straight right.
        compass_data = [0.0, 0.0, 0.0]
        if self.compass_device is not None:
            compass_data = self.compass_device.getValues()
            radians = math.atan2(compass_data[2], compass_data[0])
            radians += math.pi/2.0
            if radians > 2.0*math.pi:
                radians -= 2.0*math.pi
        else:
            radians = 0.0
        return radians

    def run(self):
        accelerometer = self.getAccelerometer('accelerometer')
        if accelerometer is not None:
            accelerometer.enable(10);
        self.compass_device = self.getCompass(self.COMPASS_DEVICE_NAME)
        if self.compass_device is not None:
            self.compass_device.enable(self.COMPASS_PERIOD)
        self.longitudinal_pid.set_parameters(self.LONG_PID_P, self.LONG_PID_I, self.LONG_PID_D)
        self.longitudinal_pid.set_output_range(self.MAX_NEGATIVE_THROTTLE_CHANGE, self.MAX_POSITIVE_THROTTLE_CHANGE)
        self.longitudinal_pid.set_integrator_value_range(self.PID_INTEGRATOR_MIN, self.PID_INTEGRATOR_MAX);
        self.lateral_controller.set_parameters(self.STANLEY_K, self.STANLEY_K2, self.STANLEY_K3)
        weight_receiver_device = self.getReceiver(self.WEIGHT_RECEIVER_DEVICE_NAME)
        if weight_receiver_device is not None:
            weight_receiver_device.enable(self.WEIGHT_RECEIVER_PERIOD)
        receiver_device = self.getReceiver(self.RECEIVER_DEVICE_NAME)
        if receiver_device is not None:
            receiver_device.enable(self.RECEIVER_PERIOD)
            if self.sideCollAvoidance is None:
                self.sideCollAvoidance = SideCollisionAvoidance()
            self.sideCollAvoidance.register_vhc_pos_receiver(receiver_device, self.my_id)

        radar_device = self.getRadar(self.RADAR_DEVICE_NAME)
        if radar_device is not None:
            radar_device.enable(self.RADAR_PERIOD)
            if self.sideCollAvoidance is None:
                self.sideCollAvoidance = SideCollisionAvoidance()
            self.sideCollAvoidance.register_sensor()
            self.sideCollAvoidance.sensor_array[-1].location = CollisionAvoidanceSensor.FRONT
            self.sideCollAvoidance.sensor_array[-1].type = CollisionAvoidanceSensor.RADAR
            self.sideCollAvoidance.sensor_array[-1].device = radar_device
        self.frontdist_sensor = self.getDistanceSensor(self.FRONT_DIST_SENSOR_NAME)
        if self.frontdist_sensor is not None:
            self.frontdist_sensor.enable(self.FRONT_DIST_SENSOR_PERIOD)
            if self.sideCollAvoidance is None:
                self.sideCollAvoidance = SideCollisionAvoidance()
            self.sideCollAvoidance.register_sensor()
            self.sideCollAvoidance.sensor_array[-1].location = CollisionAvoidanceSensor.FRONT
            self.sideCollAvoidance.sensor_array[-1].type = CollisionAvoidanceSensor.SONAR
            self.sideCollAvoidance.sensor_array[-1].device = self.frontdist_sensor
        self.leftfront_sensor = self.getDistanceSensor(self.LEFTFRONT_SENSOR_NAME)
        if self.leftfront_sensor is not None:
            self.leftfront_sensor.enable(self.LEFTFRONT_SENSOR_PERIOD)
            if self.sideCollAvoidance is None:
                self.sideCollAvoidance = SideCollisionAvoidance()
            self.sideCollAvoidance.register_sensor()
            self.sideCollAvoidance.sensor_array[-1].location = CollisionAvoidanceSensor.LEFTFRONT
            self.sideCollAvoidance.sensor_array[-1].type = CollisionAvoidanceSensor.SONAR
            self.sideCollAvoidance.sensor_array[-1].device = self.leftfront_sensor
        self.leftrear_sensor = self.getDistanceSensor(self.LEFTREAR_SENSOR_NAME)
        if self.leftrear_sensor is not None:
            self.leftrear_sensor.enable(self.LEFTREAR_SENSOR_PERIOD)
            if self.sideCollAvoidance is None:
                self.sideCollAvoidance = SideCollisionAvoidance()
            self.sideCollAvoidance.register_sensor()
            self.sideCollAvoidance.sensor_array[-1].location = CollisionAvoidanceSensor.LEFTREAR
            self.sideCollAvoidance.sensor_array[-1].type = CollisionAvoidanceSensor.SONAR
            self.sideCollAvoidance.sensor_array[-1].device = self.leftrear_sensor
        self.rightfront_sensor = self.getDistanceSensor(self.RIGHTFRONT_SENSOR_NAME)
        if self.rightfront_sensor is not None:
            self.rightfront_sensor.enable(self.RIGHTFRONT_SENSOR_PERIOD)
            if self.sideCollAvoidance is None:
                self.sideCollAvoidance = SideCollisionAvoidance()
            self.sideCollAvoidance.register_sensor()
            self.sideCollAvoidance.sensor_array[-1].location = CollisionAvoidanceSensor.RIGHTFRONT
            self.sideCollAvoidance.sensor_array[-1].type = CollisionAvoidanceSensor.SONAR
            self.sideCollAvoidance.sensor_array[-1].device = self.rightfront_sensor
        self.rightrear_sensor = self.getDistanceSensor(self.RIGHTREAR_SENSOR_NAME)
        if self.rightrear_sensor is not None:
            self.rightrear_sensor.enable(self.RIGHTREAR_SENSOR_PERIOD)
            if self.sideCollAvoidance is None:
                self.sideCollAvoidance = SideCollisionAvoidance()
            self.sideCollAvoidance.register_sensor()
            self.sideCollAvoidance.sensor_array[-1].location = CollisionAvoidanceSensor.RIGHTREAR
            self.sideCollAvoidance.sensor_array[-1].type = CollisionAvoidanceSensor.SONAR
            self.sideCollAvoidance.sensor_array[-1].device = self.rightrear_sensor
        gps_device = self.getGPS(self.GPS_DEVICE_NAME)
        if gps_device is not None:
            gps_device.enable(self.GPS_DEVICE_PERIOD)
        #dist_sensor = self.getDistanceSensor("distance")
        #dist_sensor.enable(10)
        #self.lidar_sensor = self.getLidar(self.LIDAR_DEVICE_NAME)
        #self.lidar_sensor.enable(self.LIDAR_PERIOD)
        contr_weights_received = False
        self.start_car()

        des_acceleration = []
        cur_acceleration = []
        throttle_delta_arr = []
        throttle_arr = []
        step_counter = 0
        prev_acc = 0.0
        max_jerk = 0.0
        jerk_i = 0
        jerk_arr = [0.0]*10
        while True:
            self.step()
            if accelerometer is not None:
                temp_acc = accelerometer.getValues()
            else:
                temp_acc = [0.0, 0.0, 0.0]
            jerk_arr[jerk_i] = (temp_acc[0]-prev_acc)/0.01
            #print 'acc = {} ---- Jerk = {}'.format(temp_acc, jerk_arr[jerk_i])
            jerk_i = jerk_i+1
            cur_sim_time = self.get_sim_time()
            if jerk_i == len(jerk_arr):
                if cur_sim_time > 10.0:
                    #print '------------------------------------------------------------------------------------------ jerk = {} max jerk = {}'.format(sum(jerk_arr)/len(jerk_arr), max_jerk)
                    if abs(sum(jerk_arr)/len(jerk_arr)) > max_jerk:
                        max_jerk = abs(sum(jerk_arr)/len(jerk_arr))
                        #print '************************************************************************************** max jerk = {}'.format(max_jerk)
                jerk_i = 0
            prev_acc = temp_acc[0]
            if contr_weights_received is False:
                (is_recv, recv_msg) = get_receiver_message(weight_receiver_device)
                if is_recv:
                    contr_weights = receive_nn_weights(recv_msg)
                    if self.debug_mode:
                        print 'received weights: {}'.format(contr_weights)
                    self.longitudinal_pid.set_parameters(contr_weights[0], contr_weights[1], contr_weights[2])
                    contr_weights_received = True

            if contr_weights_received:
                # --- Compute Lateral Control ---
                if gps_device is not None:
                    position = gps_device.getValues()
                else:
                    position = [0.0, 0.0, 0.0]
                if self.sideCollAvoidance is not None:
                    self.sideCollAvoidance.collisionObjectsArr = []
                    self.sideCollAvoidance.find_risky_objects()
                    (target_lat_pos, target_velocity) = self.sideCollAvoidance.compute_side_coll_avoidance_maneuver(position, self.desired_velocity, self.desired_lateral_pos)
                else:
                    target_lat_pos = self.desired_lateral_pos
                    target_velocity = self.desired_velocity
                lateral_err = target_lat_pos - position[2]

                orientation = self.get_bearing()
                if not math.isnan(orientation):
                    orient_err = orientation - self.ref_orientation
                    if orient_err > math.pi:
                        orient_err -= 2.0*math.pi
                    if orient_err < -math.pi:
                        orient_err += 2.0*math.pi
                else:
                    orient_err = 0.0

                cur_speed = self.get_current_speed()
                if not math.isnan(cur_speed):
                    if cur_speed < 40.0:
                        self.MAXIMUM_STEERING = 0.5
                    else:
                        self.MAXIMUM_STEERING = max(0.5 - (cur_speed - 40.0)/100.0, 0.1)
                    self.MAXIMUM_STEERING_DELTA = self.MAXIMUM_STEERING / 50.0
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
                (spacing, rel_speed) = self.sideCollAvoidance.find_preceding_vehicle()
                if receiver_device is None:
                    preceding_velocity = cur_speed + rel_speed
                else:
                    preceding_velocity = rel_speed

                '''
                preceding_target = None
                left_target = None
                right_target = None
                targets = radar_device.getTargets()
                #if len(targets) > 0:
                #    print targets
                for target in targets:
                    if abs(target.azimuth) < 0.2:
                        #print('t.dist: {}, t.azimuth: {}'.format(target.distance, target.azimuth))
                        if abs(math.sin(target.azimuth)*target.distance) < 2.0: # We ignore targets further than 2m in lateral distance
                            if preceding_target is None:
                                preceding_target = target
                            elif abs(math.cos(target.azimuth)*target.distance) < abs(math.cos(preceding_target.azimuth)*preceding_target.distance):
                                preceding_target = target
                        elif target.azimuth <= 0.0:
                            if left_target is None:
                                left_target = target
                            elif abs(math.cos(target.azimuth)*target.distance) < abs(math.cos(left_target.azimuth)*left_target.distance):
                                left_target = target
                        else:
                            if right_target is None:
                                right_target = target
                            elif abs(math.cos(target.azimuth)*target.distance) < abs(math.cos(right_target.azimuth)*right_target.distance):
                                right_target = target
                if left_target is not None:
                    if self.prev_left_target is not None:
                        left_old_dist = abs(math.sin(self.prev_left_target.azimuth)*self.prev_left_target.distance)
                        left_new_dist = abs(math.sin(left_target.azimuth)*left_target.distance)
                        left_delta = left_old_dist - left_new_dist
                        print 'approaching left target in {}'.format(((left_new_dist - 2.0) / left_delta))
                        if 0.0 < ((left_new_dist - 2.0) / left_delta) < 25.0: # The target on left will be in front of us in 2 seconds
                            if preceding_target is not None:
                                if abs(math.cos(left_target.azimuth)*left_target.distance) < abs(math.cos(preceding_target.azimuth)*preceding_target.distance):
                                    preceding_target = left_target
                                    print('t.dist: {}, t.azimuth: {}'.format(preceding_target.distance, preceding_target.azimuth))
                            else:
                                preceding_target = left_target
                                print('t.dist: {}, t.azimuth: {}'.format(preceding_target.distance, preceding_target.azimuth))

                self.prev_left_target = left_target
                if right_target is not None:
                    if self.prev_right_target is not None:
                        right_old_dist = abs(math.sin(self.prev_right_target.azimuth)*self.prev_right_target.distance)
                        right_new_dist = abs(math.sin(right_target.azimuth)*right_target.distance)
                        right_delta = right_old_dist - right_new_dist
                        if 0.0 < ((right_new_dist - 2.0) / right_delta) < 25.0: # The target on right will be in front of us in 2 seconds
                            #print 'approaching right target in {}'.format(((right_new_dist - 2.0) / right_delta))
                            if preceding_target is not None:
                                if abs(math.cos(right_target.azimuth)*right_target.distance) < abs(math.cos(preceding_target.azimuth)*preceding_target.distance):
                                    preceding_target = right_target
                                    print('t.dist: {}, t.azimuth: {}'.format(preceding_target.distance, preceding_target.azimuth))
                            else:
                                preceding_target = right_target
                                print('t.dist: {}, t.azimuth: {}'.format(preceding_target.distance, preceding_target.azimuth))
                    
                self.prev_right_target = right_target
                if preceding_target is None:
                    preceding_velocity = cur_speed
                    spacing = 2.0*self.FORWARD_SENSING_RANGE
                else:
                    preceding_velocity = cur_speed + preceding_target.speed
                    spacing = abs(math.cos(preceding_target.azimuth)*preceding_target.distance)
                    #print 'spacing: {}'.format(spacing)
                '''
                desired_acc = self.compute_desired_acceleration(spacing, cur_speed/3.6, preceding_velocity/3.6, target_velocity/3.6) #Division by 3.6 converts km/h to m/s
                #print 'spacing: {}, cur_speed: {}, preceding_velocity: {}, desired_acc: {}'.format(spacing, cur_speed, preceding_velocity, desired_acc)
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
                self.prev_speed = cur_speed
                acc_err = desired_acc - cur_acc
                if math.isnan(acc_err):
                    acc_err = 0.0

                throttle_delta = self.longitudinal_pid.compute_no_derivative_kick(acc_err, cur_acc)
                #if desired_acc < 0.0 and throttle_delta > 0.0:
                #    throttle_delta = 0.0
                if self.debug_mode:
                    print 'acc_err: {}, cur_acc: {}, throttle_delta: {}, prev_throttle: {}'.format(acc_err, cur_acc, throttle_delta, self.last_throttle)
                self.last_throttle += throttle_delta
                self.last_throttle = min(max(-1.0, self.last_throttle), 1.0)
                if desired_acc < 0.0 and self.last_throttle > 0.0:
                    self.last_throttle = 0.0
                #print 'acc_target: {} acc err: {}, cur_acc: {}, throttle: {}, throttle_delta: {}'.format(desired_acc, acc_err, cur_acc, self.last_throttle, throttle_delta)

                #if desired_acc < -2.0:
                #    self.set_control_actions_throttle_angle(-1.0, lat_control)
                #else:
                self.set_control_actions_throttle_angle(self.last_throttle, lat_control)
                self.last_steering = lat_control
                #self.set_control_actions_throttle_angle(1.0, lat_control)
                #dist_val = dist_sensor.getValue()
                #print("distance: {}".format(dist_val))
                # Receive Message from Receiver:
                #message = []
                #while receiver_device.getQueueLength() > 0:
                #    message += receiver_device.getData()
                #    receiver_device.nextPacket()
                #message = ''.join(message)

                '''
                step_counter += 1
                des_acceleration.append(desired_acc)
                cur_acceleration.append(cur_acc)
                throttle_delta_arr.append(throttle_delta)
                throttle_arr.append(self.last_throttle)
                if step_counter >= 4980:
                    import matplotlib.pyplot as plt
                    # red dashes, blue squares and green triangles
                    #t = np.arange(0., 45., 0.1)
                    #plt.plot(t, des_acceleration, 'r--', t, cur_acceleration, 'bs') #, t, t**3, 'g^')
                    t = np.arange(0.0, 0.01*len(des_acceleration), 0.01)
                    #plt.plot(t, np.array(des_acceleration))
                    plt.plot(t, np.array(des_acceleration), 'r--', t, np.array(cur_acceleration), 'b--', t, np.array(throttle_delta_arr), 'g+', t, np.array(throttle_arr), 'b^') #, t, t**3, 'g^')
                    plt.show()
                '''
                    

