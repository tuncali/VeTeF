"""Defines BaseCarController class"""
import math
import sys
import os


try:
    libraryPath = os.environ.get("WEBOTS_HOME") + "/projects/automobile/libraries/python"
    libraryPath.replace('/', os.sep)
    sys.path.append(libraryPath)
    from automobile import Car, Driver
except ImportError:
    sys.stderr.write("Warning: 'automobile' or 'controller' module not found.\n")
    sys.exit(0)


# ********************************************************************************
# This controller is the base vehicle controller
# ********************************************************************************
class BaseCarController(Car):
    """BaseCarController is the base class for a vehicle controller."""
    CONTROL_MODE_SPEED = 0
    CONTROL_MODE_TORQUE = 1

    ENG_TYPE_COMBUSTION = 0
    ENG_TYPE_ELECTRIC = 1
    ENG_TYPE_PARALLEL_HYBRID = 2
    ENG_TYPE_SPLIT_HYBRID = 3

    def __init__(self, car_model):
        self.debug_mode = False
        print car_model
        Car.__init__(self)
        self.car_model = car_model
        self.set_car_specific_parameters()
        if self.debug_mode:
            print("BaseCarController: initialized")

    def set_car_specific_parameters(self):
        if 'Citroen' in self.car_model:
            self.engineType = 'e'  # 'e'lectric, 'c'ombustion, 'h'ybrid
            self.engSplitRatio = 0.0
            self.engSplitRpm = 0.0
            self.eng_a = 0.0
            self.eng_b = 0.0
            self.eng_c = 0.0
            self.engMinRpm = 0.0
            self.engMaxRpm = 8000.0
            self.tireRadius = 0.27
            self.gearRatio = [6.2329]
            self.engMaxTorque = 150.0
            self.engMaxPower = 75000.0
            self.brakeCoef = 500.0
            self.delta_u_max = 3000.0
            self.u_max = 3462.0
            self.u_min = -2000.0
            self.alpha = 100.0
            self.mass = 1200.0
            self.d_safe = 2.0
            self.length = 3.5
            self.wheelbase = 2.6
            self.length_front = self.wheelbase / 2
            self.length_rear = self.wheelbase - self.length_front
            self.inertia = 2534.0
            self.tire_stiffness = 25000.0  # 867.0 #49675.5
            if self.debug_mode:
                print("BaseCarController: Citroen Settings")
        elif 'Toyota' in self.car_model:
            self.engineType = 'h'  # 'e'lectric, 'c'ombustion, 'h'ybrid
            self.engSplitRatio = 0.2778
            self.engSplitRpm = 3000.0
            self.eng_a = 65.0
            self.eng_b = 0.0225
            self.eng_c = -0.0000025
            self.engMinRpm = 1200.0
            self.engMaxRpm = 6500.0
            self.tireRadius = 0.27
            self.gearRatio = [6.0]
            self.engMaxTorque = 350.0
            self.engMaxPower = 33000.0
            self.brakeCoef = 500.0
            self.delta_u_max = 3000.0
            self.u_max = 7777.0
            self.u_min = -2000.0
            self.alpha = 100.0
            self.mass = 1805.0
            self.d_safe = 3.0
            self.length = 4.0
            self.wheelbase = 3.2
            self.length_front = self.wheelbase / 2
            self.length_rear = self.wheelbase - self.length_front
            self.inertia = 3400.0
            self.tire_stiffness = 25000.0
            if self.debug_mode:
                print("BaseCarController: Prius Settings")
        elif 'Bmw' in self.car_model:
            self.engineType = 'c'  # 'e'lectric, 'c'ombustion, 'h'ybrid
            self.engSplitRatio = 0.0
            self.engSplitRpm = 0.0
            self.eng_a = 150.0
            self.eng_b = 0.1
            self.eng_c = 0.0
            self.engMinRpm = 1000.0
            self.engMaxRpm = 4000.0
            self.tireRadius = 0.27
            self.gearRatio = [10.0, 7.0, 5.0, 2.5, 1.0]
            self.engMaxTorque = 250.0
            self.engMaxPower = 50000.0
            self.brakeCoef = 500.0
            self.delta_u_max = 3000.0
            self.u_max = 9259.0
            self.u_min = -2000.0
            self.alpha = 120.0
            self.mass = 2000.0
            self.d_safe = 3.0
            self.length = 5.0
            self.wheelbase = 3.0
            self.length_front = self.wheelbase / 2
            self.length_rear = self.wheelbase - self.length_front
            self.inertia = 4346.1
            self.tire_stiffness = 25000.0
            if self.debug_mode:
                print("BaseCarController: BMW X5 Settings")
        else:
            print('BaseCarController: CAR MODEL MUST BE GIVEN TO CONTROLLER AS SECOND PARAMETER. '
                  'OPTIONS: citroen prius bmw')
            sys.stdout.flush()

    def get_torque(self):
        torque = 0.0
        if self.getControlMode() == self.CONTROL_MODE_TORQUE:
            rpm = self.getRpm()
            if (self.engineType == 'e') or (self.engineType == 'h'):
                if rpm > 1.0:
                    torque = min(self.engMaxTorque, self.engMaxPower*60.0/(2.0*math.pi*rpm))
                else:
                    torque = float(self.engMaxTorque)
            if self.engineType == 'c':
                if rpm < self.engMinRpm:
                    rpm = self.engMinRpm
                if rpm > self.engMaxRpm:
                    torque = 0.0
                else:
                    torque = self.eng_c*(rpm**2) + self.eng_b*rpm + self.eng_a
            if self.engineType == 'h':
                if rpm < self.engMinRpm:
                    torque_c = 0.0
                else:
                    torque_c = self.eng_c*(self.engSplitRpm**2) + self.eng_b*self.engSplitRpm + self.eng_a
                torque = torque + (1.0-self.engSplitRatio)*torque_c
        return torque

    def set_control_actions_throttle_angle(self, throttle, angle):
        self.setSteeringAngle(angle)
        if throttle >= 0.0:
            throttle_setting = min(throttle, 1.0)
            self.setThrottle(throttle_setting)
            self.setBrakeIntensity(0.0)
        else:
            self.setThrottle(0.0)
            self.setBrakeIntensity(min(-throttle, 1.0))

    def set_control_actions_speed_angle(self, speed, angle):
        self.setSteeringAngle(angle)
        self.setCruisingSpeed(speed)

    def set_gear(self, gear):
        self.setGear(gear)

    def set_throttle(self, throttle):
        self.setThrottle(throttle)

    def get_gear(self):
        return self.getGear()

    def get_steering_angle(self):
        return self.getSteeringAngle()

    def get_current_speed(self):
        return self.getCurrentSpeed()

    def get_throttle(self):
        return self.getThrottle()

    def get_brake(self):
        return self.getBrakeIntensity()

    def get_rpm(self):
        if self.getControlMode() == self.CONTROL_MODE_TORQUE:
            ret_val = self.getRpm()
        else:
            ret_val = 0.0
        return ret_val

    def get_control_mode(self):
        return self.getControlMode()

    def get_sim_time(self):
        return self.getTime()

    def start_car(self):
        self.setThrottle(0.0)
        self.setGear(1)
        gear = self.getGear()
        if self.debug_mode:
            print("Starting CAR: Gear: {}".format(gear))
        
    def run(self):
        self.start_car()
        self.setBrakeIntensity(0.0)

        while True:
            self.setThrottle(1.0)
            self.step()
