"""Defines SimData class"""


class SimData(object):
    """SimData class is used for defining simulation parameters like simulation duration and type"""
    SIM_TYPE_REAL_TIME = 1
    SIM_TYPE_RUN = 2
    SIM_TYPE_FAST_NO_GRAPHICS = 3

    def __init__(self):
        self.simulation_duration_ms = 30000
        self.simulation_step_size_ms = 10
        self.simulation_execution_mode = 1
