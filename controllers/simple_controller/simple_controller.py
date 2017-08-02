import math
import os
import sys
import struct

filePath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(filePath+"/../base_controller")

from base_controller import BaseCarController

# **********************************************************************************************
# This controller is a simple controller for vehicles. Drives the car straight with 0.5 throttle
# **********************************************************************************************
class simple_controller(BaseCarController):
    def __init__(self, (car_model, target_throttle)):
        BaseCarController.__init__(self, car_model)
        self.EMITTER_DEVICE_NAME = "emitter"
        self.TOUCH_SENSOR_NAME = "touch sensor"
        self.target_throttle = float(target_throttle)
        print("simple_controller Initialized: {}, {}".format(car_model, self.target_throttle))

    def set_target_throttle(self, throttle):
        self.target_throttle = throttle

    def run(self):
        # Get the emitter device for later access.
        self.emitter_device = self.getEmitter(self.EMITTER_DEVICE_NAME)
        # Get the touch sensor for later access.
        self.touch_sensor = self.getTouchSensor(self.TOUCH_SENSOR_NAME)
        if self.touch_sensor is not None:
            self.touch_sensor.enable(10)
        self.start_car()
        while True:
            self.step()
            self.set_control_actions_throttle_angle(self.target_throttle, 0.0)
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

