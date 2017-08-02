import math

class GenericStanleyController:
    def __init__(self):
        self.k = 0.15
        self.k2 = 0.3
        self.k3 = 1.1
        self.MAX_OUTPUT_VALUE = 0.5
        self.MIN_OUTPUT_VALUE = -0.5

    def set_parameters(self, param_k, param_k2, param_k3):
        self.k = param_k
        self.k2 = param_k2
        self.k3 = param_k3

    def set_output_range(self, min_output, max_output):
        self.MIN_OUTPUT_VALUE = min_output
        self.MAX_OUTPUT_VALUE = max_output

    def compute(self, angle_err, distance_err, speed):
        #print('angle_err: {}, distance_err: {}, speed: {}'.format(angle_err, distance_err, speed))
        control_output = self.k2 * angle_err + math.atan(self.k * distance_err / (speed + self.k3))
        control_output = max(min(control_output, self.MAX_OUTPUT_VALUE), self.MIN_OUTPUT_VALUE)
        return control_output
