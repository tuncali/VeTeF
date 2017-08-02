import math
import os
import sys
import struct
import numpy as np
from keras.models import Sequential, Model
from keras.layers import Dense, Dropout, Activation
from keras.optimizers import SGD
#from controller import *

filePath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(filePath+"/../base_controller")

from base_controller import BaseCarController

class dummy_vhc_nn_control(BaseCarController):
    COMMAND_NN_WEIGHTS = 1
    COMMAND_CONTROL_FEATURES = 2

    def __init__(self, (car_model,)):
        BaseCarController.__init__(self, car_model)
        self.RECEIVER_DEVICE_NAME = "dummy_vhc_receiver"
        self.RECEIVER_PERIOD = 10
        self.debug_mode = False
        self.last_throttle = 0.0
        self.last_steering = 0.0

    def create_nn(self, num_of_features):
        self.model = Sequential()
        self.model.add(Dense(16, init = 'glorot_uniform', input_shape=(num_of_features,)))
        self.model.add(Activation('sigmoid'))

        self.model.add(Dense(16, init = 'glorot_uniform'))
        self.model.add(Activation('sigmoid'))

        self.model.add(Dense(2, init = 'glorot_uniform'))
        # Since we're using binary_crossentropy, don't use a softmax function. Use sigmoid or something else.
        self.model.add(Activation('sigmoid'))

        # try using different optimizers and different optimizer configs
        self.model.compile(loss='mse', optimizer='sgd', metrics=['accuracy'])

        return self.model

    def get_model(self):
        return self.model

    def set_model_weights(self, weights):
        self.model.set_weights(weights)

    def get_model_weights(self):
        return self.model.get_weights()

    def get_control_actions_from_model(self, features):
        normalized_features = features
        normalized_features[0][0] = normalized_features[0][0] / 5.0
        normalized_features[0][1] = normalized_features[0][1] / 50.0
        normalized_features[0][3] = normalized_features[0][3] / 10.0
        normalized_features[0][4] = normalized_features[0][4] / 5.0
        normalized_features[0][5] = normalized_features[0][5] / 10.0
        #print('normalized_features: {}'.format(normalized_features))
        pred = self.model.predict(normalized_features)
        #print('pred: {}'.format(pred))
        delta_throttle = (pred[0][0] - 0.5) / 5.0 # Will return a value between -0.1 and +0.1
        delta_steering = (pred[0][1] - 0.5) / 5.0 # Will return a value between -0.1 and +0.1
        throttle = self.last_throttle + delta_throttle
        if throttle > 1.0:
            throttle = 1.0
        elif throttle < -1.0:
            throttle = -1.0
        steering = self.last_steering + delta_steering
        if steering > 0.3:
            steering = 0.3
        elif steering < -0.3:
            steering = -0.3
        if self.debug_mode:
            print("throttle: {}, steering: {} delta_t: {} delta_s: {}".format(throttle, steering, delta_throttle, delta_steering))
        return (throttle, steering)

    def run(self):
        receiver_device = self.getReceiver(self.RECEIVER_DEVICE_NAME)
        receiver_device.enable(self.RECEIVER_PERIOD)
        self.create_nn(6)
        weights = self.get_model_weights()

        #dist_sensor = self.getDistanceSensor("distance")
        #dist_sensor.enable(10)
        #lidar_sensor = self.getLidar("velodyne")
        #lidar_sensor.enable(10)
        self.start_car()
        while True:
            self.step()
            #val = dist_sensor.getValue()
            #lidar_vals = lidar_sensor.getLayerRangeImage(0)
            #print("lidar vals: {}".format(lidar_vals[len(lidar_vals)/2:len(lidar_vals)/2 + 10]))
            #print("distance: {}".format(val))
            received_features = 0
            message = []
            while receiver_device.getQueueLength() > 0:
                message += receiver_device.getData()
                receiver_device.nextPacket()

            message = ''.join(message)
            if len(message) > 0:
                message_cmd = ord(message[0])
                if message_cmd == self.COMMAND_CONTROL_FEATURES:
                    cur_index = 8
                    if self.debug_mode:
                        print("VEHICLE IS RECEIVING COMMAND_CONTROL_FEATURES !!!")
                    control_features = struct.unpack("dddddd", message[cur_index:])
                    received_features = 1
                elif message_cmd == self.COMMAND_NN_WEIGHTS:
                    if self.debug_mode:
                        print("VEHICLE IS RECEIVING COMMAND_NN_WEIGHTS !!!")
                    (temp, length) = struct.unpack('bh', message[:4])
                    cur_index = 4
                    nn_weights = []
                    for i in range(length):
                        new_weight = struct.unpack("d", message[cur_index:cur_index+8])[0]
                        nn_weights.append(new_weight)
                        cur_index += 8
                    if self.debug_mode:
                        print("Received new NN weights. length: {}!".format(length))
                    weights = self.get_model_weights()
                    cur_w_ind = 0
                    for i in range(len(weights)):
                        shape = weights[i].shape
                        temp_size = weights[i].size
                        weights[i] = np.array(nn_weights[cur_w_ind:cur_w_ind + temp_size])
                        cur_w_ind += temp_size
                        weights[i].shape = shape
                    self.set_model_weights(weights)
                else:
                    print("UNKNOWN Command ({}) for vehicle".format(message_cmd))

            if received_features == 1:
                control_features = np.array(control_features)
                control_features.shape = (1L, 6L)
                if self.debug_mode:
                    print("Control Features: {}".format(control_features))
                (throttle, steering) = self.get_control_actions_from_model(control_features)
                self.set_control_actions_throttle_angle(throttle, steering)
                self.last_throttle = throttle
                self.last_steering = steering

