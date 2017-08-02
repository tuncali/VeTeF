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
sys.path.append(filePath+"/../controller_commons")

from base_controller import BaseCarController
from controller_commons import *

class stay_middle_contr(BaseCarController):
    COMMAND_NN_WEIGHTS = 1
    COMMAND_CONTROL_FEATURES = 2

    def __init__(self, (car_model,)):
        BaseCarController.__init__(self, car_model)
        self.RECEIVER_DEVICE_NAME = "vut_receiver"
        self.RECEIVER_PERIOD = 10
        self.NN_RECEIVER_DEVICE_NAME = "vut_nn_receiver"
        self.NN_RECEIVER_PERIOD = 10
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
        try:
            receiver_device = self.getReceiver(self.RECEIVER_DEVICE_NAME)
            receiver_device.enable(self.RECEIVER_PERIOD)
            nn_receiver_device = self.getReceiver(self.NN_RECEIVER_DEVICE_NAME)
            nn_receiver_device.enable(self.NN_RECEIVER_PERIOD)
            self.create_nn(8)
            #weights = self.get_model_weights()
            #for layer in weights:
            #    print('number of weights in layer: {}'.format(layer.shape))

            #dist_sensor = self.getDistanceSensor("distance")
            #dist_sensor.enable(10)
            #lidar_sensor = self.getLidar("velodyne")
            #lidar_sensor.enable(10)
            prev_dist_front = None
            prev_dist_right = None
            prev_dist_left = None
            prev_dist_behind = None
            nn_weights_received = False
            self.start_car()
            while True:
                self.step()
                (is_recv, recv_msg) = get_receiver_message(receiver_device)
                if is_recv:
                    vhc_pos_dict = receive_all_vhc_pos(recv_msg)
                    dist_front = vhc_pos_dict[1][0] - vhc_pos_dict[5][0]
                    dist_right = vhc_pos_dict[2][2] - vhc_pos_dict[5][2]
                    dist_left = vhc_pos_dict[3][2] - vhc_pos_dict[5][2]
                    dist_behind = vhc_pos_dict[4][0] - vhc_pos_dict[5][0]
                    if prev_dist_front is None:
                        v_front = 0.0
                        v_right = 0.0
                        v_left = 0.0
                        v_behind = 0.0
                    else:
                        v_front = (dist_front - prev_dist_front)*50.0
                        v_right = (dist_right - prev_dist_right)*50.0
                        v_left = (dist_left - prev_dist_left)*50.0
                        v_behind = (dist_behind - prev_dist_behind)*50.0
                    prev_dist_front = dist_front
                    prev_dist_right = dist_right
                    prev_dist_left = dist_left
                    prev_dist_behind = dist_behind

                (is_nn_recv, recv_msg) = get_receiver_message(nn_receiver_device)
                if is_nn_recv:
                    weights = self.get_model_weights()
                    nn_weights = receive_nn_weights(recv_msg)
                    cur_w_ind = 0
                    for i in range(len(weights)):
                        shape = weights[i].shape
                        temp_size = weights[i].size
                        weights[i] = np.array(nn_weights[cur_w_ind:cur_w_ind + temp_size])
                        cur_w_ind += temp_size
                        weights[i].shape = shape
                    nn_weights_received = True
                    self.set_model_weights(weights)

                if is_recv and nn_weights_received:
                    control_features = list([dist_front, dist_right, dist_left, dist_behind, v_front, v_right, v_left, v_behind])
                    control_features = np.array(control_features)
                    control_features.shape = (1L, 8L)
                    if self.debug_mode:
                        print("Control Features: {}".format(control_features))
                    (throttle, steering) = self.get_control_actions_from_model(control_features)
                    self.set_control_actions_throttle_angle(throttle, steering)
                    self.last_throttle = throttle
                    self.last_steering = steering
        except Exception as e: 
            print e

