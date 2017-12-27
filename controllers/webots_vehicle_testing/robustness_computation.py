"""Defines RobustnessComputation class"""
import math
import struct

import numpy as np

import dssm
from coordinate_system import CoordinateSystem


class RobustnessComputation(object):
    """Handles the robustness computation"""
    ROB_TYPE_TTC_COLL_SPEED = 0
    ROB_TYPE_DUMMY_TRAIN_COLL = 1
    ROB_TYPE_DUMMY_TRAIN_DRIVE = 2
    ROB_TYPE_DUMMY_TRAIN_FRONT_COLL = 3
    ROB_TYPE_STAY_IN_THE_MIDDLE = 4
    ROB_TYPE_FOLLOW = 5
    ROB_TYPE_TRAFFIC_WAVE = 6
    ROB_TYPE_MAX_ABS_JERK = 7
    ROB_TYPE_DSSM_FRONT_REAR = 8
    ROB_TYPE_NONE = 100
    MAX_ROBUSTNESS = 100
    MIN_COLL_MAGNITUDE = 0.0
    MAX_COLL_MAGNITUDE = 30.0
    TOUCH_SENSOR_BUMPER = -100
    TOUCH_SENSOR_FORCE = -200
    TOUCH_SENSOR_FORCE_3D = 0
    TOUCH_SENSOR_UNKNOWN_TYPE = -1

    def __init__(self, robustness_type, supervisor_control, vehicles_manager, environment_manager):
        self.receiver_name = "receiver"
        self.receiver_period_ms = 10
        self.debug_mode = 0
        if robustness_type is None:
            robustness_type = self.ROB_TYPE_NONE
        self.robustness_type = robustness_type
        self.supervisor_control = supervisor_control
        self.vehicles_manager = vehicles_manager
        self.environment_manager = environment_manager
        self.minimum_robustness = self.MAX_ROBUSTNESS
        self.minimum_robustness_instant = 0
        self.collision_detected = False
        self.supervisor_control.enable_receiver(self.receiver_name, self.receiver_period_ms)
        self.touch_sensor_type = self.TOUCH_SENSOR_UNKNOWN_TYPE
        self.extra_punishment = 0.0
        self.started_computing_robustness = False
        self.traffic_wave_threshold_speed = 8.33  # Corresponds to 30 km/h #5.56 #: 20 km/h
        self.min_dummy_speed = self.MAX_ROBUSTNESS

    def set_debug_mode(self, mode):
        """Sets the debug mode for this object."""
        self.debug_mode = mode

    def compute_ang_wrt_pos_lat_axis(self, vector):
        """Computes the vector angle, i.e., is the angle of the vector wrt positive lateral axis"""
        if -0.00001 < vector[CoordinateSystem.LAT_AXIS] < 0.00001:
            if vector[CoordinateSystem.LONG_AXIS] >= 0:
                vector_ang = math.pi / 2.0
            else:
                vector_ang = -math.pi / 2.0
        elif vector[CoordinateSystem.LAT_AXIS] > 0:
            vector_ang = math.atan(vector[CoordinateSystem.LONG_AXIS] / vector[CoordinateSystem.LAT_AXIS])
        elif vector[CoordinateSystem.LONG_AXIS] < 0:  # vector[CoordinateSystem.LAT_AXIS] < 0
            vector_ang = -math.pi + math.atan(vector[CoordinateSystem.LONG_AXIS] / vector[CoordinateSystem.LAT_AXIS])
        else:  # vector[CoordinateSystem.LONG_AXIS] >= 0 and vector[CoordinateSystem.LAT_AXIS] < 0
            vector_ang = math.pi + math.atan(vector[CoordinateSystem.LONG_AXIS] / vector[CoordinateSystem.LAT_AXIS])
        return vector_ang

    def check_collision_course(self, ego_vhc_pos, ego_vhc_pts, ego_vhc_ang_velocity, ego_vhc_velocity,
                               intruder_pts, intruder_velocity):
        """ Check if two vehicles are on a collision course.
         "Vehicle Collision Probability Calculation for General Traffic Scenarios Under Uncertainty",
         J. Ward, G. Agamennoni, S. Worrall, E. Nebot"""
        is_collision_path = False
        num_of_ego_pts = len(ego_vhc_pts)
        num_of_int_pts = len(intruder_pts)

        for i in range(num_of_ego_pts):
            # v_i_lin is a numpy array of linear velocity of the loom point 
            v_i_lin = ego_vhc_velocity + np.cross(ego_vhc_ang_velocity, np.subtract(ego_vhc_pts[i], ego_vhc_pos))
            min_ang = np.inf
            max_ang = -np.inf
            dist_of_min_pt = np.inf
            dist_of_max_pt = -np.inf
            for j in range(num_of_int_pts):
                distance_intruder_to_ego = np.subtract(ego_vhc_pts[i], intruder_pts[j])
                angle_intruder_to_ego = self.compute_ang_wrt_pos_lat_axis(distance_intruder_to_ego)
                if angle_intruder_to_ego < min_ang:
                    min_ang = angle_intruder_to_ego
                    dist_of_min_pt = distance_intruder_to_ego
                if angle_intruder_to_ego > max_ang:
                    max_ang = angle_intruder_to_ego
                    dist_of_max_pt = distance_intruder_to_ego

            if np.linalg.norm(dist_of_min_pt) != 0 and np.linalg.norm(dist_of_max_pt) != 0:
                loom_rate_of_min_pt = np.cross(dist_of_min_pt, v_i_lin) + np.cross(dist_of_min_pt, intruder_velocity)
                loom_rate_of_min_pt = loom_rate_of_min_pt / (np.linalg.norm(dist_of_min_pt) ** 2)
                loom_rate_of_max_pt = np.cross(dist_of_max_pt, v_i_lin) + np.cross(dist_of_max_pt, intruder_velocity)
                loom_rate_of_max_pt = loom_rate_of_max_pt / (np.linalg.norm(dist_of_max_pt) ** 2)
                if (loom_rate_of_min_pt[CoordinateSystem.LAT_AXIS] <= 0 <=
                        loom_rate_of_max_pt[CoordinateSystem.LAT_AXIS]):
                    is_collision_path = True
                    break
            else:
                is_collision_path = True
                break

        return is_collision_path

    def compute_ttc(self, pt_1, pt_2, vel_1, vel_2):
        """Computes the time to collision between points pt_1 and pt_2 given the velocities vel_1 and vel_2"""
        epsilon = 0.00001
        d = math.sqrt(np.dot(np.transpose(np.subtract(pt_1, pt_2)), np.subtract(pt_1, pt_2)))
        d_dot = np.dot(np.transpose(np.subtract(pt_1, pt_2)), np.subtract(vel_1, vel_2))[0] / d
        # d_2dot would be used for second-order TTC computation.
        # d_2dot = (np.dot(np.transpose(np.subtract(vel_1, vel_2)), np.subtract(vel_1, vel_2)) - d_dot ** 2) / d
        if abs(d_dot) > epsilon:
            ttc_1 = -d / d_dot
            if ttc_1 < 0:
                ttc_1 = None
        else:
            ttc_1 = None

        ttc = ttc_1
        return ttc

    def get_robustness(self):
        """Returns the minimum robustness (including any extra punishment)"""
        return self.minimum_robustness + self.extra_punishment

    def get_current_robustness(self, ego_vhc_pos, ego_vhc_pts, ego_vhc_ang_velocity, ego_vhc_velocity, intruder_pos,
                               intruder_pts, intruder_velocity, collision_magnitude):
        """Computes and returns the robustness value for current time step only."""
        current_robustness = self.MAX_ROBUSTNESS
        if (self.touch_sensor_type == self.TOUCH_SENSOR_FORCE_3D or self.touch_sensor_type == self.TOUCH_SENSOR_FORCE) \
                and (collision_magnitude > self.MIN_COLL_MAGNITUDE):
            current_robustness = collision_magnitude
            is_collision = True
        elif (self.touch_sensor_type == self.TOUCH_SENSOR_BUMPER) and (collision_magnitude > 0.0):
            current_robustness = abs(np.linalg.norm(np.array(ego_vhc_velocity) - np.array(intruder_velocity)))
            is_collision = True
        else:
            is_collision = False

        if (is_collision and self.robustness_type == self.ROB_TYPE_DUMMY_TRAIN_FRONT_COLL
                and (ego_vhc_pos[CoordinateSystem.LONG_AXIS] < intruder_pos[CoordinateSystem.LONG_AXIS])):
            is_collision = False  # Cancel the collision if we are looking for a front collision and we are behind.
            current_robustness = 10000
        elif (is_collision and self.robustness_type == self.ROB_TYPE_DUMMY_TRAIN_FRONT_COLL
              and (ego_vhc_pos[CoordinateSystem.LONG_AXIS] > intruder_pos[CoordinateSystem.LONG_AXIS])):
            current_robustness = -10000
        elif not is_collision:
            # TODO: This computation should be done on the points that are closest to a collision, not central points
            if self.check_collision_course(ego_vhc_pos, ego_vhc_pts, ego_vhc_ang_velocity, ego_vhc_velocity,
                                           intruder_pts, intruder_velocity):
                ttc = self.compute_ttc(ego_vhc_pos, intruder_pos, ego_vhc_velocity, intruder_velocity)
                current_robustness = ttc + self.MAX_COLL_MAGNITUDE
            else:
                current_robustness = self.MAX_ROBUSTNESS + self.MAX_COLL_MAGNITUDE
                # We are penalizing too much the cases where vehicles are not in collision course
                # current_robustness = 50 + self.MAX_COLL_MAGNITUDE
        return current_robustness, is_collision

    def compute_robustness(self, current_sim_time_s):
        """Computes the Robustness value for current time"""
        if self.robustness_type == self.ROB_TYPE_DSSM_FRONT_REAR:
            # TODO: Vehicle indices are hardcoded: Find the related dummy vehicles in a smart way without hardcoding.
            leader_pos = self.vehicles_manager.vehicles[2].current_position[CoordinateSystem.LONG_AXIS]
            ego_pos = self.vehicles_manager.vehicles[0].current_position[CoordinateSystem.LONG_AXIS]
            follower_pos = self.vehicles_manager.vehicles[1].current_position[CoordinateSystem.LONG_AXIS]
            leader_speed = self.vehicles_manager.vehicles[2].current_velocity[CoordinateSystem.LONG_AXIS]
            ego_speed = self.vehicles_manager.vehicles[0].current_velocity[CoordinateSystem.LONG_AXIS]
            follower_speed = self.vehicles_manager.vehicles[1].current_velocity[CoordinateSystem.LONG_AXIS]
            leader_acc = self.vehicles_manager.vehicles[2].current_acceleration
            ego_acc = self.vehicles_manager.vehicles[0].current_acceleration
            follower_acc = self.vehicles_manager.vehicles[1].current_acceleration
            spacing_e = leader_pos - ego_pos - 3.2
            spacing_f = ego_pos - follower_pos - 3.2
            # print 'leader_pos: {}, ego_pos :{} follower pos {}'.format(leader_pos, ego_pos, follower_pos)
            ego_b_max = -4.0
            ego_max_acc_variation = 10.0
            ego_reaction_time = 0.2
            leader_b_max = -6.0
            leader_max_acc_variation = 10.0
            follower_b_max = -4.0
            follower_max_acc_variation = 10.0
            follower_reaction_time = 0.2

            (dssm_e, b_e) = dssm.compute_dssm_and_braking(spacing_e, ego_speed, ego_acc,
                                                          ego_b_max, ego_max_acc_variation, ego_reaction_time,
                                                          leader_speed, leader_acc, leader_b_max,
                                                          leader_max_acc_variation)
            (dssm_f, b_f) = dssm.compute_dssm_and_braking(spacing_f, follower_speed, follower_acc,
                                                          follower_b_max, follower_max_acc_variation,
                                                          follower_reaction_time, ego_speed, ego_acc,
                                                          b_e, ego_max_acc_variation)

            k1 = 5.0
            k2 = 3.0
            k3 = 5.0
            k4 = 1.0
            d_safe = 3.0
            # print 'spacing_ e: {}, leader_Speed: {} ego_speed: {}'.format(spacing_e, leader_speed, ego_speed)
            # print 'Rob : {} + {} + {} + {} = {}'.format(k1*(max(0.0, dssm_e - 0.8))**2 ,
            # k2*(max(0.0, dssm_f - 0.8))**2 ,
            # k3*(max(0.0, d_safe - spacing_e))**2 ,
            # k4*(leader_speed - ego_speed)**2 ,
            # (k1*(max(0.0, dssm_e - 0.8))**2 +
            # k2*(max(0.0, dssm_f - 0.8))**2 +
            # k3*(max(0.0, d_safe - spacing_e))**2 +
            # k4*(leader_speed - ego_speed)**2))
            dssm_value = (k1 * (max(0.0, dssm_e - 0.8)) ** 2 +
                          k2 * (max(0.0, dssm_f - 0.8)) ** 2 +
                          k3 * (max(0.0, d_safe - spacing_e)) ** 2 +
                          k4 * (leader_speed - ego_speed) ** 2)
            self.extra_punishment += dssm_value
        elif self.robustness_type == self.ROB_TYPE_MAX_ABS_JERK:
            if not self.started_computing_robustness:
                if current_sim_time_s > 1.0:
                    self.started_computing_robustness = True
                    # print 'Starting Robustness Computation!'
            else:
                maximum_jerk_perf_value = 100.0
                max_jerk = 0.0
                for vhc_id in self.vehicles_manager.VUT_dictionary:
                    vut_jerk = \
                        abs(self.vehicles_manager.vehicles[self.vehicles_manager.VUT_dictionary[vhc_id]].current_jerk)
                    if vut_jerk > 15.0:
                        vut_jerk = 0.0

                    if vut_jerk > max_jerk:
                        max_jerk = vut_jerk
                rob_val = maximum_jerk_perf_value - max_jerk
                # print("Robustness: Robustness Value = {}, max_jerk = {}, min rob = {}".format(rob_val,
                # max_jerk, self.minimum_robustness))
                if rob_val < self.minimum_robustness:
                    self.minimum_robustness = rob_val
                    self.minimum_robustness_instant = current_sim_time_s
                    # print("Robustness: Robustness Value = {}, max_jerk: {} at time: {}".format(
                    # rob_val, max_jerk, self.minimum_robustness_instant))
        elif self.robustness_type == self.ROB_TYPE_TRAFFIC_WAVE:
            # self.extra_punishment = 0.0
            if not self.started_computing_robustness:
                less_than_threshold_exists = False
                for vhc_id in self.vehicles_manager.VUT_dictionary:
                    if self.vehicles_manager.vehicles[self.vehicles_manager.VUT_dictionary[vhc_id]].speed \
                            < self.traffic_wave_threshold_speed:
                        less_than_threshold_exists = True
                        break
                if not less_than_threshold_exists:
                    self.started_computing_robustness = True
            else:
                # for dum_id in self.vehicles_manager.dummy_vhc_dictionary:
                #    if self.vehicles_manager.vehicles[self.vehicles_manager.dummy_vhc_dictionary[dum_id]].speed \
                # < self.min_dummy_speed:
                #        self.min_dummy_speed = \
                # self.vehicles_manager.vehicles[self.vehicles_manager.dummy_vhc_dictionary[dum_id]].speed
                #        #print 'Dummy vehicles speed dropped to : {}'.format(
                # self.vehicles_manager.vehicles[self.vehicles_manager.dummy_vhc_dictionary[dum_id]].speed)
                for vhc_id in self.vehicles_manager.VUT_dictionary:
                    # if self.vehicles_manager.vehicles[self.vehicles_manager.VUT_dictionary[vhc_id]].speed \
                    # < self.minimum_robustness:
                    #    print 'Speed of vhc_id: {} is {}'.format(vhc_id,
                    # self.vehicles_manager.vehicles[self.vehicles_manager.VUT_dictionary[vhc_id]].speed)
                    self.minimum_robustness = \
                        min(self.minimum_robustness,
                            self.vehicles_manager.vehicles[self.vehicles_manager.VUT_dictionary[vhc_id]].speed)
        elif self.robustness_type == self.ROB_TYPE_FOLLOW:
            self.minimum_robustness = 0.0
            self.extra_punishment += (self.vehicles_manager.vehicles[1].current_position[CoordinateSystem.LONG_AXIS] -
                                      self.vehicles_manager.vehicles[0].current_position[CoordinateSystem.LONG_AXIS] -
                                      13.5) ** 2
        elif self.robustness_type == self.ROB_TYPE_STAY_IN_THE_MIDDLE:
            self.minimum_robustness = 0.0
            for vhc_id in self.vehicles_manager.VUT_dictionary:
                vut_pos = self.vehicles_manager.vehicles[self.vehicles_manager.VUT_dictionary[vhc_id]].current_position
                # TODO: Vehicle indices are hardcoded: Find the related dummy vehicles in a smart way without hardcoding
                dist_lon1 = (self.vehicles_manager.vehicles[1].current_position[CoordinateSystem.LONG_AXIS] -
                             vut_pos[CoordinateSystem.LONG_AXIS])
                dist_lon2 = (vut_pos[CoordinateSystem.LONG_AXIS] -
                             self.vehicles_manager.vehicles[4].current_position[CoordinateSystem.LONG_AXIS])
                self.extra_punishment += abs(dist_lon1 - dist_lon2)
                dist_lat1 = (self.vehicles_manager.vehicles[2].current_position[CoordinateSystem.LAT_AXIS] -
                             vut_pos[CoordinateSystem.LAT_AXIS])
                dist_lat2 = (vut_pos[CoordinateSystem.LAT_AXIS] -
                             self.vehicles_manager.vehicles[3].current_position[CoordinateSystem.LAT_AXIS])
                self.extra_punishment += abs(dist_lat1 - dist_lat2)
        elif self.robustness_type == self.ROB_TYPE_DUMMY_TRAIN_DRIVE:
            if len(self.vehicles_manager.vehicles) > 1:
                dist = np.linalg.norm(np.array(self.vehicles_manager.vehicles[0].current_position) -
                                      np.array(self.vehicles_manager.vehicles[1].current_position))
                self.extra_punishment += dist
            self.minimum_robustness = 0.0
        elif self.robustness_type == self.ROB_TYPE_TTC_COLL_SPEED:
            touch_sensor_mag = 0.0
            while self.supervisor_control.receiver.getQueueLength() > 0:
                message = self.supervisor_control.receiver.getData()
                touch_sensor_val = struct.unpack("ddd", message)
                if touch_sensor_val[0] == touch_sensor_val[1] == self.TOUCH_SENSOR_BUMPER:
                    touch_sensor_mag = touch_sensor_val[2]
                    self.touch_sensor_type = self.TOUCH_SENSOR_BUMPER
                elif touch_sensor_val[0] == touch_sensor_val[1] == self.TOUCH_SENSOR_FORCE:
                    touch_sensor_mag = touch_sensor_val[2]
                    self.touch_sensor_type = self.TOUCH_SENSOR_FORCE
                else:
                    touch_sensor_mag = np.linalg.norm(touch_sensor_val)
                    self.touch_sensor_type = self.TOUCH_SENSOR_FORCE_3D
                if self.debug_mode:
                    print("RobustnessComputation: Touch sensor magnitude = {}, value: {}".format(touch_sensor_mag,
                                                                                                 touch_sensor_val))
                self.supervisor_control.receiver.nextPacket()
            rob_val = 0.0
            if not self.collision_detected:
                ego_pts = self.vehicles_manager.vehicles[0].get_vehicle_critical_points()
                intruder_points = self.vehicles_manager.vehicles[1].get_vehicle_critical_points()
                rob_val, self.collision_detected = self.get_current_robustness(
                    self.vehicles_manager.vehicles[0].current_position,
                    ego_pts,
                    self.vehicles_manager.vehicles[0].current_velocity[3:6],
                    self.vehicles_manager.vehicles[0].current_velocity[0:3],
                    self.vehicles_manager.vehicles[1].current_position,
                    intruder_points,
                    self.vehicles_manager.vehicles[1].current_velocity[0:3],
                    touch_sensor_mag)
                if rob_val < self.minimum_robustness:
                    self.minimum_robustness = rob_val
                    self.minimum_robustness_instant = current_sim_time_s
                    if self.debug_mode:
                        print("Robustness: Robustness Value = {}".format(rob_val))

            if self.robustness_type == self.ROB_TYPE_DUMMY_TRAIN_COLL:
                for vhc_id in self.vehicles_manager.dummy_vhc_dictionary:
                    dist = self.environment_manager.get_distance_from_pt_to_road_network(
                        self.vehicles_manager.vehicles[
                            self.vehicles_manager.dummy_vhc_dictionary[vhc_id]].current_position)
                    self.extra_punishment += dist
                    self.extra_punishment += rob_val
                    # if self.debug_mode:
                    #    print("Robustness: Extra punishment for vehicle {} = {}".format(vhc_id, dist))
            elif self.robustness_type == self.ROB_TYPE_DUMMY_TRAIN_FRONT_COLL:
                self.minimum_robustness = 0.0
                for vhc_id in self.vehicles_manager.dummy_vhc_dictionary:
                    offset = np.zeros(3)
                    offset[CoordinateSystem.LONG_AXIS] = 4.0
                    # We are forcing dummy vehicle to 4m ahead of the VUT
                    dist = \
                        np.linalg.norm(
                            np.array(
                                self.vehicles_manager.vehicles[0].current_position) +
                            offset -
                            np.array(
                                self.vehicles_manager.vehicles[
                                    self.vehicles_manager.dummy_vhc_dictionary[vhc_id]].current_position))
                    self.extra_punishment += dist
                    dist = \
                        self.environment_manager.get_distance_from_pt_to_road_network(
                            self.vehicles_manager.vehicles[
                                self.vehicles_manager.dummy_vhc_dictionary[vhc_id]].current_position)
                    self.extra_punishment += dist ** 2
                    # Canceled collision detection for now:
                    # self.extra_punishment += rob_val
        else:
            self.minimum_robustness = self.MAX_ROBUSTNESS
            self.extra_punishment = 0.0
