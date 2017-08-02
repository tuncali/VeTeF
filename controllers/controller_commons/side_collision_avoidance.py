import math
import numpy as np

from controller_commons import *


class CollisionAvoidanceSensor:
    FRONT = 0
    CENTER = 1
    LEFT = 2
    RIGHT = 3
    TOP = 4
    LEFTFRONT = 5
    LEFTREAR = 6
    RIGHTFRONT = 7
    RIGHTREAR = 8
    REAR = 9
    RADAR = 0
    SONAR = 1
    LIDAR = 2

    def __init__(self):
        self.location = self.LEFT
        self.type = self.SONAR
        self.device = None


class CollisionObject:
    ABSOLUTE_POSITION = 0
    RELATIVE_POSITION = 1
    AREA_POSITION = 2

    def __init__(self):
        self.position_type = self.AREA_POSITION
        self.position_area = CollisionAvoidanceSensor.LEFT
        self.position_value = [0.0, 0.0, 0.0]  # LONG, VERTICAL, LAT
        self.velocity = [0.0, 0.0]  # LONG, LAT


class SideCollisionAvoidance:
    LEFT = 0
    RIGHT = 1

    def __init__(self):
        self.sensor_array = []
        self.collisionObjectsArr = []
        self.prevCollisionObjectsArr = []
        self.time_step = 10.0
        self.prev_left_dist = None
        self.prev_right_dist = None
        self.coll_avoid_counter = -1
        self.coll_avoid_state = self.LEFT
        self.prev_side_coll_avoid_lat_target = None
        self.desired_side_dist = 1.5
        self.minimum_side_dist = 1.0
        self.side_ttc_threshold = 2.0
        self.prev_forward_pos = None
        self.prev_forward_left_pos = None
        self.prev_forward_right_pos = None
        self.vhc_pos_receiver = None
        self.prev_vhc_pos_dict = None
        self.my_id = None
        self.my_pos = [0.0, 0.0, 0.0]

    def register_vhc_pos_receiver(self, receiver, my_id):
        self.vhc_pos_receiver = receiver
        self.my_id = my_id

    def register_sensor(self):
        self.sensor_array.append(CollisionAvoidanceSensor())

    def find_risky_vehicles_from_receiver(self, receiver_device):
        (is_recv, recv_msg) = get_receiver_message(receiver_device)
        if is_recv:
            vhc_pos_dict = receive_all_vhc_pos(recv_msg)
            self.my_pos = vhc_pos_dict[self.my_id]
            for vhc_id in vhc_pos_dict:
                if vhc_id is not self.my_id:
                    pos = vhc_pos_dict[vhc_id]
                    if pos[0] > self.my_pos[0] + 2.0:  # at front
                        vel = [0.0, 0.0]
                        if self.prev_vhc_pos_dict is not None:
                            previous_position = self.prev_vhc_pos_dict[vhc_id]
                            vel[0] = (pos[0] - previous_position[0]) * 360.0
                            vel[1] = (pos[2] - previous_position[2]) * 360.0
                            if np.linalg.norm(vel) > 180.0:
                                vel = [0.0, 0.0]
                        if (abs(pos[2] - self.my_pos[2]) < (self.desired_side_dist + 1.5) or
                                ((self.prev_vhc_pos_dict is not None) and (pos[2] != previous_position[2]) and
                                     (0.0 < (self.my_pos[2] - pos[2]) / (
                                                 (pos[2] - previous_position[2]) * 100.0) < 2.5))):
                            self.collisionObjectsArr.append(CollisionObject())
                            self.collisionObjectsArr[-1].position_type = CollisionObject.ABSOLUTE_POSITION
                            self.collisionObjectsArr[-1].position_area = CollisionAvoidanceSensor.FRONT
                            self.collisionObjectsArr[-1].position_value = pos[:]
                            self.collisionObjectsArr[-1].velocity = vel[:]
            self.prev_vhc_pos_dict = vhc_pos_dict.copy()

    def find_risky_objects(self):
        if self.vhc_pos_receiver is not None:
            self.find_risky_vehicles_from_receiver(self.vhc_pos_receiver)

        for sensor in self.sensor_array:
            if sensor.type == sensor.SONAR:
                dist = sensor.device.getValue()
                self.collisionObjectsArr.append(CollisionObject())
                self.collisionObjectsArr[-1].position_type = CollisionObject.AREA_POSITION
                self.collisionObjectsArr[-1].position_area = sensor.location
                if sensor.location in [sensor.LEFT, sensor.LEFTFRONT, sensor.LEFTREAR]:
                    self.collisionObjectsArr[-1].position_value = [0.0, 0.0, -dist]
                elif sensor.location in [sensor.RIGHT, sensor.RIGHTFRONT, sensor.RIGHTREAR]:
                    self.collisionObjectsArr[-1].position_value = [0.0, 0.0, dist]
                elif sensor.location in [sensor.FRONT, sensor.TOP, sensor.CENTER]:
                    speed = 0.0
                    for coll_obj in self.prevCollisionObjectsArr:
                        if (coll_obj.position_type == CollisionObject.AREA_POSITION and
                                    coll_obj.position_area == sensor.location):
                            prev_dist = coll_obj.position_value[0]
                            speed = (dist - prev_dist) * 1000.0 / sensor.device.getSamplingPeriod()
                            # print 'front... dist: {}, prev_dist: {}, period_ms: {} speed: {}'.format(dist, prev_dist, sensor.device.getSamplingPeriod(), speed)
                    self.collisionObjectsArr[-1].position_value = [dist, 0.0, 0.0]
                    self.collisionObjectsArr[-1].velocity = [speed, 0.0, 0.0]
                else:
                    speed = 0.0
                    for coll_obj in self.prevCollisionObjectsArr:
                        if (coll_obj.position_type == CollisionObject.AREA_POSITION and
                                    coll_obj.position_area == sensor.location):
                            prev_dist = coll_obj.position_value[0]
                            speed = (dist - prev_dist) * 1000.0 / sensor.device.getSamplingPeriod()
                            # print 'rear... dist: {}, prev_dist: {}, period_ms: {} speed: {}'.format(dist, prev_dist, sensor.device.getSamplingPeriod(), speed)
                    self.collisionObjectsArr[-1].position_value = [-dist, 0.0, 0.0]
                    self.collisionObjectsArr[-1].velocity = [speed, 0.0, 0.0]
            elif sensor.type == sensor.RADAR:
                min_forward = None
                min_forward_left = None
                min_forward_right = None
                targets = sensor.device.getTargets()
                for target in targets:
                    if sensor.location in [sensor.FRONT, sensor.TOP, sensor.CENTER]:
                        if sensor.location == sensor.FRONT:
                            target_rel_pos = [math.cos(target.azimuth) * target.distance, 0.0,
                                              math.sin(target.azimuth) * target.distance]
                        else:  # TODO: The -2.0m is specific to CitroenCZero:
                            target_rel_pos = [math.cos(target.azimuth) * target.distance - 2.0, 0.0,
                                              math.sin(target.azimuth) * target.distance]
                        # target_rel_vel = [math.cos(target.azimuth)*target.speed, math.sin(target.azimuth)*target.speed]
                        target_pos_type = sensor.FRONT
                    elif sensor.location in [sensor.LEFT, sensor.LEFTFRONT, sensor.LEFTREAR]:
                        target_rel_pos = [math.sin(target.azimuth) * target.distance, 0.0,
                                          -math.cos(target.azimuth) * target.distance]
                        # target_rel_vel = [math.sin(target.azimuth)*target.speed, -math.cos(target.azimuth)*target.speed]
                        target_pos_type = sensor.location
                    elif sensor.location in [sensor.RIGHT, sensor.RIGHTFRONT, sensor.RIGHTREAR]:
                        target_rel_pos = [-math.sin(target.azimuth) * target.distance, 0.0,
                                          math.cos(target.azimuth) * target.distance]
                        # target_rel_vel = [-math.sin(target.azimuth)*target.speed, math.cos(target.azimuth)*target.speed]
                        target_pos_type = sensor.location
                    else:
                        target_rel_pos = [-math.cos(target.azimuth) * target.distance, 0.0,
                                          -math.sin(target.azimuth) * target.distance]
                        # target_rel_vel = [-math.cos(target.azimuth)*target.speed, -math.sin(target.azimuth)*target.speed]
                        target_pos_type = sensor.location
                    if target_pos_type == sensor.FRONT:
                        if abs(target_rel_pos[2]) < self.desired_side_dist + 2.7:
                            rel_velocity = [0.0, 0.0, 0.0]
                            self.collisionObjectsArr.append(CollisionObject())
                            self.collisionObjectsArr[-1].position_type = CollisionObject.RELATIVE_POSITION
                            self.collisionObjectsArr[-1].position_value = target_rel_pos[:]
                            self.collisionObjectsArr[-1].position_area = target_pos_type
                            if self.prev_forward_pos is not None:
                                long_delta = target_rel_pos[0] - self.prev_forward_pos[0]
                                rel_velocity[0] = long_delta * 100 / 3.6
                                lat_delta = target_rel_pos[2] - self.prev_forward_pos[2]
                                rel_velocity[1] = lat_delta * 100 / 3.6
                                if np.linalg.norm(rel_velocity) < 180.0:  # Larger than 180 km/h will be ignored
                                    self.collisionObjectsArr[-1].velocity = rel_velocity[:]
                                else:
                                    self.collisionObjectsArr[-1].velocity = [0.0, 0.0]
                            else:
                                self.collisionObjectsArr[-1].velocity = [0.0, 0.0]
                            if min_forward is None:
                                min_forward = target_rel_pos[:]
                            elif target_rel_pos[0] < min_forward[0]:
                                min_forward = target_rel_pos[:]
                        elif target_rel_pos[2] <= -(self.desired_side_dist + 0.7):
                            rel_velocity = [0.0, 0.0, 0.0]
                            if self.prev_forward_left_pos is not None:
                                long_delta = target_rel_pos[0] - self.prev_forward_left_pos[0]
                                rel_velocity[0] = long_delta * 100 / 3.6
                                lat_delta = target_rel_pos[2] - self.prev_forward_left_pos[2]
                                rel_velocity[1] = lat_delta * 100 / 3.6
                                if np.linalg.norm(rel_velocity) < 180.0:  # Larger than 180 km/h will be ignored
                                    if lat_delta > 0.0 and (
                                                -target_rel_pos[2] - (self.desired_side_dist + 0.7)) / lat_delta < 25:
                                        self.collisionObjectsArr.append(CollisionObject())
                                        self.collisionObjectsArr[-1].position_type = CollisionObject.RELATIVE_POSITION
                                        self.collisionObjectsArr[-1].position_value = target_rel_pos[:]
                                        self.collisionObjectsArr[-1].position_area = target_pos_type
                                        self.collisionObjectsArr[-1].velocity = rel_velocity[:]
                            if min_forward_left is None:
                                min_forward_left = target_rel_pos[:]
                            elif target_rel_pos[0] < min_forward_left[0]:
                                min_forward_left = target_rel_pos[:]
                        elif target_rel_pos[2] >= self.desired_side_dist + 0.7:
                            rel_velocity = [0.0, 0.0, 0.0]
                            if self.prev_forward_right_pos is not None:
                                long_delta = target_rel_pos[0] - self.prev_forward_right_pos[0]
                                rel_velocity[0] = long_delta * 100 / 3.6
                                lat_delta = target_rel_pos[2] - self.prev_forward_right_pos[2]
                                rel_velocity[1] = lat_delta * 100 / 3.6
                                if np.linalg.norm(rel_velocity) < 180.0:  # Larger than 180 km/h will be ignored
                                    if lat_delta > 0.0 and (
                                                target_rel_pos[2] - (self.desired_side_dist + 0.7)) / lat_delta < 25:
                                        self.collisionObjectsArr.append(CollisionObject())
                                        self.collisionObjectsArr[-1].position_type = CollisionObject.RELATIVE_POSITION
                                        self.collisionObjectsArr[-1].position_value = target_rel_pos[:]
                                        self.collisionObjectsArr[-1].position_area = target_pos_type
                                        self.collisionObjectsArr[-1].velocity = rel_velocity[:]
                            if min_forward_right is None:
                                min_forward_right = target_rel_pos[:]
                            elif target_rel_pos[0] < min_forward_right[0]:
                                min_forward_right = target_rel_pos[:]

                        '''
                        if (abs(target_rel_pos[2]) < self.desired_side_dist + 0.7 or 
                            ((target_rel_vel[1] is not 0.0) and 
                                (0.0 < (target_rel_pos[2] / target_rel_vel[1]) < (self.desired_side_dist + 0.7)))):
                            print('target pos: {}, vel: {}, pow: {} read_speed:{}'.format(target_rel_pos, target_rel_vel, target.received_power, target.speed))
                            self.collisionObjectsArr.append(CollisionObject())
                            self.collisionObjectsArr[-1].position_type = CollisionObject.RELATIVE_POSITION
                            self.collisionObjectsArr[-1].position_value = target_rel_pos
                            self.collisionObjectsArr[-1].position_area = target_pos_type
                            self.collisionObjectsArr[-1].velocity = target_rel_vel
                        '''
                if min_forward is None:
                    self.prev_forward_pos = None
                else:
                    self.prev_forward_pos = min_forward[:]
                if min_forward_left is None:
                    self.prev_forward_left_pos = None
                else:
                    self.prev_forward_left_pos = min_forward_left[:]
                if min_forward_right is None:
                    self.prev_forward_right_pos = None
                else:
                    self.prev_forward_right_pos = min_forward_right[:]
        self.prevCollisionObjectsArr = self.collisionObjectsArr

    def find_preceding_vehicle(self):
        min_dist = np.inf
        speed = 0.0
        for coll_object in self.collisionObjectsArr:
            if coll_object.position_area == CollisionAvoidanceSensor.FRONT:
                if coll_object.position_type == coll_object.ABSOLUTE_POSITION:
                    if 0.0 < coll_object.position_value[0] - self.my_pos[0] - 3.2 < min_dist:
                        min_dist = coll_object.position_value[0] - self.my_pos[0] - 3.2
                        speed = coll_object.velocity[0]
                else:
                    if 0.0 < coll_object.position_value[0] < min_dist:
                        min_dist = coll_object.position_value[0]
                        speed = coll_object.velocity[0]
        # print 'preceding: {}, {}'.format(min_dist, speed)
        return (min_dist, speed)

    def find_following_vehicle(self):
        min_dist = np.inf
        speed = 0.0
        for coll_object in self.collisionObjectsArr:
            if coll_object.position_area == CollisionAvoidanceSensor.REAR:
                if coll_object.position_type == coll_object.ABSOLUTE_POSITION:
                    if 0.0 < self.my_pos[0] - coll_object.position_value[0] - 3.2 < min_dist:
                        min_dist = self.my_pos[0] - coll_object.position_value[0] - 3.2
                        speed = coll_object.velocity[0]
                else:
                    if 0.0 < -coll_object.position_value[0] < min_dist:
                        min_dist = -coll_object.position_value[0]
                        speed = coll_object.velocity[0]
        # print 'preceding: {}, {}'.format(min_dist, speed)
        return (min_dist, speed)

    def compute_side_risk(self, direction):
        min_dist = np.inf
        min_ttc = np.inf
        min_dist_area = CollisionAvoidanceSensor.LEFT
        min_ttc_area = CollisionAvoidanceSensor.LEFT
        if direction is self.LEFT:
            prev_dist = self.prev_left_dist
            pos_area_list = [CollisionAvoidanceSensor.LEFT, CollisionAvoidanceSensor.LEFTFRONT,
                             CollisionAvoidanceSensor.LEFTREAR]
        else:
            prev_dist = self.prev_right_dist
            pos_area_list = [CollisionAvoidanceSensor.RIGHT, CollisionAvoidanceSensor.RIGHTFRONT,
                             CollisionAvoidanceSensor.RIGHTREAR]

        for coll_object in self.collisionObjectsArr:
            if (coll_object.position_type == CollisionObject.AREA_POSITION
                and coll_object.position_area in pos_area_list):
                dist = abs(coll_object.position_value[2])
                if dist == 5.0:
                    dist = np.inf
                # print('dist: {}'.format(dist))
                if dist < min_dist:
                    min_dist = dist
                    min_dist_area = coll_object.position_area
                if (prev_dist is not None and
                        ((direction is self.LEFT and self.prev_left_min_dist_area == min_dist_area) or
                             (direction is self.RIGHT and self.prev_right_min_dist_area == min_dist_area))):
                    # print('prev_dist:{}'.format(prev_dist))
                    ttc = (dist / (
                        prev_dist - dist)) * 0.01  # *0.01 is to convert from simulation step size to s. Assuming 10ms step size
                    if 0.0 < ttc < min_ttc:
                        min_ttc = ttc
                        min_ttc_area = coll_object.position_area
                        # print('min_ttc: {}'.format(min_ttc))

        if direction is self.LEFT:
            self.prev_left_dist = min_dist
            self.prev_left_min_dist_area = min_dist_area
        elif direction is self.RIGHT:
            self.prev_right_dist = min_dist
            self.prev_right_min_dist_area = min_dist_area

        return (min_dist, min_ttc, min_dist_area, min_ttc_area)

    def compute_side_coll_avoidance_maneuver(self, cur_position, desired_velocity, desired_lat_pos):
        target_velocity = desired_velocity
        target_lat_pos = desired_lat_pos
        cur_lat_position = cur_position[2]
        (min_left_dist, min_left_ttc, min_left_dist_area, min_left_ttc_area) = self.compute_side_risk(self.LEFT)
        (min_right_dist, min_right_ttc, min_right_dist_area, min_right_ttc_area) = self.compute_side_risk(self.RIGHT)
        maneuver_computed = False

        # BOTH RIGHT AND LEFT IS UNDER RISK:
        if (min_left_dist < self.desired_side_dist
            and min_right_dist < self.desired_side_dist):
            target_lat_pos = cur_lat_position + ((min_right_dist - min_left_dist) / 2.0)
            if ((min_left_dist < min_right_dist and min_left_dist_area == CollisionAvoidanceSensor.LEFTFRONT)
                or (min_left_dist > min_right_dist and min_right_dist_area == CollisionAvoidanceSensor.RIGHTFRONT)):
                target_velocity = target_velocity * 0.1
            elif ((min_left_dist < min_right_dist and min_left_dist_area == CollisionAvoidanceSensor.LEFTREAR)
                  or (min_left_dist > min_right_dist and min_right_dist_area == CollisionAvoidanceSensor.RIGHTREAR)):
                target_velocity = target_velocity * 1.1
            maneuver_computed = True
        elif 0.0 < min_left_ttc < self.side_ttc_threshold and 0.0 < min_right_ttc < self.side_ttc_threshold:
            if min_left_ttc < min_right_ttc:
                target_lat_pos = cur_lat_position + min_right_dist - self.minimum_side_dist
                if min_left_ttc_area == CollisionAvoidanceSensor.LEFTREAR:
                    target_velocity = target_velocity * 1.1
                else:
                    target_velocity = target_velocity * 0.1
            else:
                target_lat_pos = cur_lat_position - (min_left_dist - self.minimum_side_dist)
                if min_right_ttc_area == CollisionAvoidanceSensor.RIGHTREAR:
                    target_velocity = target_velocity * 1.1
                else:
                    target_velocity = target_velocity * 0.1
            maneuver_computed = True
        elif 0.0 < min_left_ttc < self.side_ttc_threshold:
            # print('min_left_ttc: {}, min_left_dist:{}'.format(min_left_ttc, min_left_dist))
            dist_err = self.desired_side_dist - (
                min_left_ttc - 2.0 * min_left_dist / min_left_ttc)  # Expected dist_err after 2s
            dist_err = min(dist_err, 3.0)
            target_lat_pos = cur_lat_position + min(max(1.0, dist_err),
                                                    min_right_dist - self.minimum_side_dist)  # Target at least 1 m if right side allows
            if min_left_ttc_area == CollisionAvoidanceSensor.LEFTREAR:
                target_velocity = target_velocity * 1.1
            else:
                target_velocity = target_velocity * 0.1
            # print('LEFT TTC! target pos:{}, 2s dist_err:{}'.format(target_lat_pos, dist_err))
            maneuver_computed = True
        elif 0.0 < min_right_ttc < self.side_ttc_threshold:
            dist_err = self.desired_side_dist - (
                min_right_ttc - 2.0 * min_right_dist / min_right_ttc)  # Expected dist_err after 2s
            dist_err = min(dist_err, 3.0)
            target_lat_pos = cur_lat_position - min(max(1.0, dist_err),
                                                    min_left_dist - self.minimum_side_dist)  # Target at least 1 m if left side allows
            if min_right_ttc_area == CollisionAvoidanceSensor.RIGHTREAR:
                target_velocity = target_velocity * 1.1
            else:
                target_velocity = target_velocity * 0.1
            maneuver_computed = True
        elif self.desired_side_dist > min_left_dist:
            dist_err = self.desired_side_dist - min_left_dist
            if min_left_dist < self.minimum_side_dist:
                # print('LEFT MINIMUM DISTANCE!')
                if 2.0 * dist_err < min_right_dist - self.minimum_side_dist:
                    target_lat_pos = cur_lat_position + 2.0 * dist_err
                else:
                    target_lat_pos = cur_lat_position + min_right_dist - self.minimum_side_dist
            else:
                if dist_err < min_right_dist - self.minimum_side_dist:
                    # print ('here: target_lat_pos {}, cur_lat_position {}, dist_err {}'.format(target_lat_pos, cur_lat_position, dist_err))
                    target_lat_pos = cur_lat_position + dist_err + 0.5
                else:
                    target_lat_pos = cur_lat_position + min_right_dist - self.minimum_side_dist
            if min_left_dist_area == CollisionAvoidanceSensor.LEFTFRONT:
                target_velocity = target_velocity * 0.5
            elif min_left_dist_area == CollisionAvoidanceSensor.LEFTREAR:
                target_velocity = target_velocity * 1.1
            # print('LEFT Distance! target pos:{} cur_lat_position: {} dist_err:{}'.format(target_lat_pos, cur_lat_position, dist_err))
            maneuver_computed = True
        elif self.desired_side_dist > min_right_dist:
            dist_err = self.desired_side_dist - min_right_dist
            if min_right_dist < self.minimum_side_dist:
                if 2.0 * dist_err < min_left_dist - self.minimum_side_dist:
                    target_lat_pos = cur_lat_position - 2.0 * dist_err
                else:
                    target_lat_pos = cur_lat_position - (min_left_dist - self.minimum_side_dist)
            else:
                if dist_err < min_left_dist - self.minimum_side_dist:
                    target_lat_pos = cur_lat_position - dist_err - 0.5
                else:
                    target_lat_pos = cur_lat_position - (min_left_dist - self.minimum_side_dist)
            if min_right_dist_area == CollisionAvoidanceSensor.RIGHTFRONT:
                target_velocity = target_velocity * 0.5
            elif min_right_dist_area == CollisionAvoidanceSensor.RIGHTREAR:
                target_velocity = target_velocity * 1.1
            maneuver_computed = True

        if maneuver_computed:
            self.coll_avoid_counter = 0
            self.prev_side_coll_avoid_lat_target = target_lat_pos
        elif self.coll_avoid_counter >= 0:
            if self.prev_side_coll_avoid_lat_target is not None:
                target_lat_pos = self.prev_side_coll_avoid_lat_target
            self.coll_avoid_counter += 1
            if self.coll_avoid_counter >= 200:
                print '--------- COLLISION AVOIDANCE IS OVER -----------'
                self.coll_avoid_counter = -1
        return (target_lat_pos, target_velocity)
