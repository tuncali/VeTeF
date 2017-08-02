import numpy as np
import math
import struct
from coordinate_system import CoordinateSystem


def find_vehicles_in_lidar_data(lidar_data):
    found_vehicles = []
    vhc_started = False
    vhc_start_ind = 0

    # if lidar_data[0] < 10.0:
    #    vhc_started = True
    for i in range(1, len(lidar_data) - 1):
        if lidar_data[i] < lidar_data[i - 1] - 10.0 and lidar_data[i] < 60.0:
            vhc_started = True
            vhc_start_ind = i
        if lidar_data[i] < lidar_data[i + 1] - 10.0 and lidar_data[i] < 60.0:
            if vhc_started:
                found_vehicles.append((vhc_start_ind, i))
    if vhc_started and lidar_data[-1] < 60.0:
        found_vehicles.append((vhc_start_ind, len(lidar_data) - 1))
    return found_vehicles


def compute_ang_wrt_pos_lat_axis(self, vector):
    # vector_ang is the angle of the vector wrt positive lat_axis
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


def angle_between_vectors(v1, v2):
    # Returns the angle in radians between vectors 'v1' and 'v2'::
    #         angle_between((1, 0, 0), (0, 1, 0)): 1.5707963267948966
    #         angle_between((1, 0, 0), (1, 0, 0)): 0.0
    #         angle_between((1, 0, 0), (-1, 0, 0)): 3.141592653589793

    return math.atan2(v2[0], v2[2]) - math.atan2(v1[0], v1[2])


def compute_loom_rate(intruder_point, ego_point, intruder_velocity, ego_linear_velocity):
    vect_pt2ego = np.subtract(intruder_point, ego_point)
    loom_rate = ((np.cross(vect_pt2ego, ego_linear_velocity) + np.cross(vect_pt2ego, intruder_velocity))
                 / (np.linalg.norm(vect_pt2ego) ** 2))
    return loom_rate


def check_collision_course(ego_vhc_pos, ego_vhc_pts, ego_vhc_ang_velocity, ego_vhc_velocity, intruder_pts,
                           intruder_velocity):
    # Check if two vehicles are on a collision course.
    # "Vehicle Collision Probability Calculation for General Traffic Scenarios Under Uncertainty",
    # J. Ward, G. Agamennoni, S. Worrall, E. Nebot
    is_coll_path = False

    for ego_pt in ego_vhc_pts:
        # v_i_lin is a numpy array of linear velocity of the loom point 
        v_i_lin = ego_vhc_velocity + np.cross(ego_vhc_ang_velocity, np.subtract(ego_pt, ego_vhc_pos))

        # Find points of intruder at the minimum and maximum angle wrt ego vehicle:
        min_ang = np.inf
        max_ang = -np.inf
        min_ang_pt = None
        max_ang_pt = None
        for intruder_point in intruder_pts:
            vector_ego_to_intruder = np.subtract(intruder_point, ego_pt)
            angle_ego_to_intruder = angle_between_vectors(vector_ego_to_intruder, v_i_lin)
            if angle_ego_to_intruder < min_ang:
                min_ang_pt = intruder_point
                min_ang = angle_ego_to_intruder
            if angle_ego_to_intruder > max_ang:
                max_ang_pt = intruder_point
                max_ang = angle_ego_to_intruder

        # Compute the loom rates for min and max angle points:
        loom_rate_min_ang_pt = compute_loom_rate(min_ang_pt, ego_pt, intruder_velocity, v_i_lin)
        loom_rate_max_ang_pt = compute_loom_rate(max_ang_pt, ego_pt, intruder_velocity, v_i_lin)
        if loom_rate_min_ang_pt[1] <= 0.0 <= loom_rate_max_ang_pt[1]:
            is_coll_path = True
            break
    return is_coll_path


def get_receiver_message(receiver_device):
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


def receive_vhc_pos(received_message, target_vhc_id):
    pos = [0.0, 0.0, 0.0]
    # Evaluate message
    if len(received_message) > 0:
        cmd = struct.unpack('B', received_message[0:struct.calcsize('B')])[0]
        # TODO: Check cmd for different type of messages
        cur_index = struct.calcsize('B')
        num_vehicles = struct.unpack('h', received_message[cur_index:cur_index + struct.calcsize('h')])[0]
        cur_index += struct.calcsize('h')
        for i in range(num_vehicles):
            (vehicle_id, pos[0], pos[1], pos[2]) = struct.unpack(
                "Bddd", received_message[cur_index:cur_index + struct.calcsize("Bddd")])
            if vehicle_id == target_vhc_id:
                break
            cur_index += struct.calcsize("Bddd")
    return pos


def receive_all_vhc_pos(received_message):
    ret_dict = {}
    # Evaluate message
    if len(received_message) > 0:
        cmd = struct.unpack('B', received_message[0:struct.calcsize('B')])[0]
        # TODO: Check cmd for different type of messages
        cur_index = struct.calcsize('B')
        num_vehicles = struct.unpack('h', received_message[cur_index:cur_index + struct.calcsize('h')])[0]
        cur_index += struct.calcsize('h')
        for i in range(num_vehicles):
            pos = [0.0, 0.0, 0.0]
            (vehicle_id, pos[0], pos[1], pos[2]) = struct.unpack(
                "Bddd", received_message[cur_index:cur_index + struct.calcsize("Bddd")])
            ret_dict[vehicle_id] = [pos[0], pos[1], pos[2]]
            cur_index += struct.calcsize("Bddd")
    return ret_dict


def receive_nn_weights(received_message):
    nn_weights = []
    if len(received_message) > 0:
        cmd = struct.unpack('B', received_message[0:struct.calcsize('B')])[0]
        # TODO: Check cmd for different type of messages
        if cmd == 1:
            (temp, length) = struct.unpack('Bh', received_message[0:struct.calcsize('Bh')])
            cur_index = struct.calcsize('Bh')
            nn_weights = list(struct.unpack('%sd' % length, received_message[cur_index:]))
    return nn_weights

def ttc_unit_test():
    ego_vhc_pos = [0.0, 0.0, 0.0]
    ego_vhc_pts = [[0.0, 0.0, 0.0]]
    ego_vhc_velocity = [10.0, 0.0, 0.0]
    ego_vhc_ang_velocity = [0.0, 0.0, 0.0]
    intruder_pts = [[5.0, 0.0, 1.0], [8.0, 0.0, 1.0], [8.0, 0.0, -1.0], [5.0, 0.0, -1.0]]
    intruder_velocity = [9.0, 0.0, 0.0]
    coll_course1 = check_collision_course(ego_vhc_pos,
                                          ego_vhc_pts,
                                          ego_vhc_ang_velocity,
                                          ego_vhc_velocity,
                                          intruder_pts,
                                          intruder_velocity)
    print coll_course1
    if coll_course1:
        ttc = compute_TTC(ego_vhc_pos, [6.5, 0.0, 0.0], ego_vhc_velocity, intruder_velocity)
        print('TTC: {}'.format(ttc))


def kmh_to_ms(kmh):
    return kmh / 3.6


def get_bearing(compass_device):
    # Return the vehicle's heading in radians. pi/2 is straight up, 0 is straight right.
    if compass_device is not None:
        compass_data = compass_device.getValues()
        radians = math.atan2(compass_data[2], compass_data[0])
        radians += math.pi / 2.0
        if radians > 2.0 * math.pi:
            radians -= 2.0 * math.pi
    else:
        radians = 0.0
    return radians

