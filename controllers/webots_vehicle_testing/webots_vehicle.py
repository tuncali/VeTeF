"""Defines WebotsVehicle Class"""
from coordinate_system import CoordinateSystem


class WebotsVehicle(object):
    """WebotsVehicle class defines user configurable vehicle to use in Webots environment"""
    CITROEN_C_ZERO = 201
    CITROEN_C_ZERO_SIMPLE = 202
    TOYOTA_PRIUS = 203
    TOYOTA_PRIUS_SIMPLE = 204
    BMW_X5 = 205
    BMW_X5_SIMPLE = 206
    BUS = 207
    ACKERMANN_VEHICLE = 255

    STATE_ID_VELOCITY_X = 1
    STATE_ID_VELOCITY_Y = 2
    STATE_ID_VELOCITY_Z = 3
    STATE_ID_ACCELERATION_X = 4
    STATE_ID_ACCELERATION_Y = 5
    STATE_ID_ACCELERATION_Z = 6
    STATE_ID_JERK_X = 7
    STATE_ID_JERK_Y = 8
    STATE_ID_JERK_Z = 9
    STATE_ID_ACCELERATION = 10

    def __init__(self):
        self.front_right_wheel_angular_velocity = None
        self.front_left_wheel_angular_velocity = None
        self.rear_right_wheel_angular_velocity = None
        self.rear_left_wheel_angular_velocity = None
        self.node = None
        self.translation = None
        self.rotation = None
        self.name = None
        self.id = 0
        self.def_name = ""
        self.vehicle_model = "AckermannVehicle"
        self.vehicle_model_id = self.ACKERMANN_VEHICLE
        self.color = [1, 1, 1]
        self.current_position = [0, 0.5, 0]
        self.current_rotation = [0, 1, 0, 0]
        # self.target_position = [0, 0.5, 0]
        # self.target_rotation = [0, 1, 0, 0]
        # self.current_angles = [0, 0, 0]
        self.current_velocity = [0, 0, 0]
        self.current_acceleration = [0, 0, 0]
        self.current_jerk = 0.0
        self.roll = 0
        self.pitch = 0
        self.speed = 0
        self.controller = "void"
        self.vehicle_parameters = []
        self.controller_parameters = []
        self.controller_arguments = []
        self.sensor_array = []
        self.signal = []
        self.previous_position = None
        self.previous_velocity = 0.0
        self.previous_acceleration = 0.0

    def set_vehicle_model(self, vhc_model_name):
        """Sets the model of the vehicle per the given vhc_model_name"""
        self.vehicle_model = vhc_model_name[:]
        if 'Citroen' in self.vehicle_model:
            self.vehicle_model_id = self.CITROEN_C_ZERO
        elif 'Bmw' in self.vehicle_model:
            self.vehicle_model_id = self.BMW_X5
        elif 'Toyota' in self.vehicle_model:
            self.vehicle_model_id = self.TOYOTA_PRIUS
        elif 'Bus' in self.vehicle_model:
            self.vehicle_model_id = self.BUS
        else:
            self.vehicle_model_id = self.ACKERMANN_VEHICLE

    def get_vehicle_critical_points_by_model(self, vehicle_model_id):
        """Returns the critical points defining the vehicle shape according to the given vehicle_model_id"""
        pts = {}
        # Vehicles 0 point is the midpoint of the rear axle.
        # Axises: [x, y, z]
        # x: towards right of the screen, y: from ground towards air, z, towards bottom of the screen
        # Vehicle's non-rotated positions: (front sides are looking towards bottom of the screen)
        if vehicle_model_id == self.CITROEN_C_ZERO or vehicle_model_id == self.CITROEN_C_ZERO_SIMPLE:
            pts[0] = [-0.74, 0.05, 3.075]  # front-right corner
            pts[1] = [-0.74, 0.05, -0.425]  # rear-right corner
            pts[2] = [0.74, 0.05, 3.075]  # front-left corner
            pts[3] = [0.74, 0.05, -0.425]  # rear-left corner
            pts[4] = [0, 0.05, -0.425]  # Mid rear
            pts[5] = [0, 0.05, 3.075]  # Mid front
            pts[6] = [-0.74, 0.05, 1.325]  # Mid right
            pts[7] = [0.74, 0.05, 1.325]  # Mid left
        elif vehicle_model_id == self.TOYOTA_PRIUS or vehicle_model_id == self.TOYOTA_PRIUS_SIMPLE:
            pts[0] = [-0.875, 0.05, 3.635]  # front-right corner
            pts[1] = [-0.875, 0.05, -0.85]  # rear-right corner
            pts[2] = [0.875, 0.05, 3.635]  # front-left corner
            pts[3] = [0.875, 0.05, -0.85]  # rear-left corner
            pts[4] = [0, 0.05, -0.85]  # Mid rear
            pts[5] = [0, 0.05, 3.635]  # Mid front
            pts[6] = [-0.875, 0.05, 1.3925]  # Mid right
            pts[7] = [0.875, 0.05, 1.3925]  # Mid left
        elif vehicle_model_id == self.BMW_X5 or vehicle_model_id == self.BMW_X5_SIMPLE:
            pts[0] = [-1.1, 0.05, 3.85]  # front-right corner
            pts[1] = [-1.1, 0.05, -1.01]  # rear-right corner
            pts[2] = [1.1, 0.05, 3.85]  # front-left corner
            pts[3] = [1.1, 0.05, -1.01]  # rear-left corner
            pts[4] = [0, 0.05, -1.01]  # Mid rear
            pts[5] = [0, 0.05, 3.85]  # Mid front
            pts[6] = [-1.1, 0.05, 1.425]  # Mid right
            pts[7] = [1.1, 0.05, 1.425]  # Mid left
        elif vehicle_model_id == self.BUS:
            pts[0] = [-1.37, 0.05, 11.5]  # front-right corner
            pts[1] = [-1.37, 0.05, -6.2]  # rear-right corner
            pts[2] = [1.37, 0.05, 11.5]  # front-left corner
            pts[3] = [1.37, 0.05, -6.2]  # rear-left corner
            pts[4] = [0, 0.05, -6.2]  # Mid rear
            pts[5] = [0, 0.05, 11.5]  # Mid front
            pts[6] = [-1.37, 0.05, 2.65]  # Mid right
            pts[7] = [1.37, 0.05, 2.65]  # Mid left
        else:  # ACKERMANN_VEHICLE
            pts[0] = [-0.95, 0.05, 4.4]  # front-right corner
            pts[1] = [-0.95, 0.05, -0.4]  # rear-right corner
            pts[2] = [0.95, 0.05, 4.4]  # front-left corner
            pts[3] = [0.95, 0.05, -0.4]  # rear-left corner
            pts[4] = [0, 0.05, -0.4]  # Mid rear
            pts[5] = [0, 0.05, 4.4]  # Mid front
            pts[6] = [-0.95, 0.05, 2.0]  # Mid right
            pts[7] = [0.95, 0.05, 2.0]  # Mid left
        return pts

    def get_vehicle_critical_points(self):
        """Returns the critical points defining the vehicle shape for the current vehicle"""
        pts = self.get_vehicle_critical_points_by_model(self.vehicle_model_id)
        return pts

    def get_vehicle_state_with_id(self, state_id):
        """Returns the value of the current state which corresponds to the given state_id"""
        state_value = 0.0
        if state_id == self.STATE_ID_VELOCITY_X:
            state_value = self.current_velocity[CoordinateSystem.X_AXIS]
        elif state_id == self.STATE_ID_VELOCITY_Y:
            state_value = self.current_velocity[CoordinateSystem.Y_AXIS]
        elif state_id == self.STATE_ID_VELOCITY_Z:
            state_value = self.current_velocity[CoordinateSystem.Z_AXIS]
        elif state_id == self.STATE_ID_ACCELERATION:
            state_value = self.current_acceleration
        elif state_id == self.STATE_ID_JERK_X:
            state_value = self.current_jerk
        return state_value
