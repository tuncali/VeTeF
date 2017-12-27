import os
import struct
import sys

filePath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(filePath+"/../base_controller")
sys.path.append(filePath+"/../generic_pid_controller")
sys.path.append(filePath+"/../generic_stanley_controller")
from base_controller import BaseCarController
from generic_stanley_controller import *
from coordinate_system import CoordinateSystem

# **********************************************************************************************
# This controller is a simple controller for vehicles. Drives the car straight with 0.5 throttle
# **********************************************************************************************
class simple_trajectory_follower(BaseCarController):
    def __init__(self, (car_model, self_vhc_id)):
        BaseCarController.__init__(self, car_model)
        self.EMITTER_DEVICE_NAME = "emitter"
        self.RECEIVER_DEVICE_NAME = "receiver"
        self.COMPASS_DEVICE_NAME = "compass"
        self.STEP_TIME = 10
        self.TOUCH_SENSOR_NAME = "touch sensor"
        self.MAX_NEGATIVE_THROTTLE_CHANGE = -float(self.STEP_TIME)/3000.0
        self.MAX_POSITIVE_THROTTLE_CHANGE = float(self.STEP_TIME)/1000.0 #I want to give full throttle from 0 in 1 sec.
        self.STANLEY_K = 1.4  #0.25
        self.STANLEY_K2 = 1.0 #0.3
        self.STANLEY_K3 = 1.0
        self.ref_orientation = 0.0
        self.received_message = []
        self.SELF_VHC_ID = int(self_vhc_id)
        self.touch_sensor = None
        self.compass_device = None
        self.receiver_device = None
        self.emitter_device = None
        self.prev_long_control = 0.0
        self.prev_lat_control = 0.0
        self.lateral_controller = GenericStanleyController()

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
    
    def run(self):
        #TARGETS = [(50.0, 0.0), (40.0, 3.5), (70.0, -3.5), (30.0, 3.5), (80.0, 0.0)]
        #TARGETS = [(80.0, -5.0), (80.0, -5.0), (70.0, 4.5), (0.0, -4.5), (70.0, 0.0)]
        TARGETS = [(40.0, 0.0), (40.0, 0.0), (0.0, 0.0), (30.0, 0.0), (30.0, 0.0),(90.0, 0.0), (90.0, 0.0), (0.0, 0.0), (0.0, 0.0), (30.0, 0.0)]
        #TARGETS = [(80.0, 0.0), (80.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]
        self.lateral_controller.set_parameters(self.STANLEY_K, self.STANLEY_K2, self.STANLEY_K3)
        # Get the emitter device for later access.
        #self.emitter_device = self.getEmitter(self.EMITTER_DEVICE_NAME)
        # Get the touch sensor for later access.
        self.receiver_device = self.getReceiver(self.RECEIVER_DEVICE_NAME)
        if self.receiver_device is not None:
            self.receiver_device.enable(self.STEP_TIME)
        # Enable compass device.
        self.compass_device = self.getCompass(self.COMPASS_DEVICE_NAME)
        if self.compass_device is not None:
            self.compass_device.enable(self.STEP_TIME)
        #self.start_car()
        count = 0
        target_ind = 0
        while True:
            self.step()
            count += 1
            if count > 1000:
                count = 0
                target_ind += 1
                if target_ind >= len(TARGETS):
                    target_ind = 0
            target_speed = TARGETS[target_ind][0]
            target_lat_pos = TARGETS[target_ind][1]
            # Receive message
            #(is_rcv, temp_message) = self.get_receiver_message(self.receiver_device)
            #if is_rcv:
            #    self.received_message = temp_message
            #self_vhc_pos = self.receive_vhc_pos(self.SELF_VHC_ID)
            #lat_distance = target_lat_pos - self_vhc_pos[CoordinateSystem.LAT_AXIS]
            lat_distance = 0.0
            orientation = self.get_bearing()
            cur_speed = self.get_current_speed()
            orient_err = 0.0  # orientation - self.ref_orientation
            if orient_err > math.pi:
                orient_err -= 2.0*math.pi
            if orient_err < -math.pi:
                orient_err += 2.0*math.pi
            if not math.isnan(cur_speed):
                lat_control = self.lateral_controller.compute(orient_err, lat_distance, cur_speed)
            else:
                lat_control = 0.0

            #print('target_speed: {}, steering: {}'.format(target_speed, lat_control))
            cur_speed = self.get_current_speed()
            #print('curr_speed: {}'.format(cur_speed))
            self.set_control_actions_speed_angle(target_speed, lat_control)
            self.prev_lat_control = lat_control
