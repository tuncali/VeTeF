import math

class GenericPIDController:
    def __init__(self):
        self.P_GAIN = 0.1
        self.I_GAIN = 0.0
        self.D_GAIN = 0.0
        self.prev_err = None
        self.prev_f_value = None
        self.i_state = 0.0
        self.I_MAX = float("inf")
        self.I_MIN = -float("inf")
        self.MAX_OUTPUT_VALUE = 1.0
        self.MIN_OUTPUT_VALUE = -1.0

    def set_parameters(self, param_p, param_i, param_d):
        self.P_GAIN = param_p
        self.I_GAIN = param_i
        self.D_GAIN = param_d

    def set_output_range(self, min_output, max_output):
        self.MIN_OUTPUT_VALUE = min_output
        self.MAX_OUTPUT_VALUE = max_output

    def set_integrator_value_range(self, i_min, i_max):
        self.I_MIN = i_min
        self.I_MAX = i_max

    def compute(self, err_input):
        if self.prev_err is None:
            self.prev_err = err_input
        
        self.i_state += err_input
        self.i_state = max(min(self.i_state, self.I_MAX), self.I_MIN)

        p_term = err_input * self.P_GAIN
        i_term = self.i_state * self.I_GAIN
        d_term = (err_input - self.prev_err) * self.D_GAIN
        #print('p_term: {} i_term: {} d_term: {}'.format(p_term, i_term, d_term))
        control_output = p_term + i_term + d_term
        control_output = max(min(control_output, self.MAX_OUTPUT_VALUE), self.MIN_OUTPUT_VALUE)
        self.prev_err = err_input

        return control_output

    def compute_no_derivative_kick(self, err_input, f_value):
        if self.prev_f_value is None:
            self.prev_f_value = f_value
        
        self.i_state += err_input
        self.i_state = max(min(self.i_state, self.I_MAX), self.I_MIN)

        p_term = err_input * self.P_GAIN
        i_term = self.i_state * self.I_GAIN
        d_term = (f_value - self.prev_f_value) * self.D_GAIN
        control_output = p_term + i_term - d_term
        #print('p_term: {} i_term: {} d_term: {} total: {}'.format(p_term, i_term, -d_term, control_output))
        control_output = max(min(control_output, self.MAX_OUTPUT_VALUE), self.MIN_OUTPUT_VALUE)
        self.prev_err = err_input
        self.prev_f_value = f_value

        return control_output
