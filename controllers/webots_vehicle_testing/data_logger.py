"""Defines DataLogger class."""
import numpy as np
from item_description import ItemDescription


class DataLogger(object):
    """DataLogger class handles logs the requested data."""
    def __init__(self):
        self.log = []
        self.data_to_log = []
        self.vehicles_manager = None
        self.environment_manager = None
        self.current_log_size = 0
        self.simulation_time_ms = 1000
        self.simulation_step_size_ms = 10
        self.temp_current_log = None
        self.log_period_ms = 1

    def set_vehicles_manager(self, vehicles_manager):
        """Sets the VehiclesManager that this class will use to log vehicle data."""
        self.vehicles_manager = vehicles_manager

    def set_environment_manager(self, environment_manager):
        """Sets the EnvironmentManager that this class will use to log vehicle data."""
        self.environment_manager = environment_manager

    def add_data_log_description(self, data_log_description):
        """Adds description of data to log."""
        self.data_to_log.append(data_log_description)

    def set_expected_simulation_time(self, simulation_time_ms):
        """Sets the expected total simulation time in ms.
        This is used to compute size of the pre-allocated log space."""
        self.simulation_time_ms = simulation_time_ms

    def set_simulation_step_size(self, simulation_step_size_ms):
        """Sets the simulation step size in ms
        This is used to compute size of the pre-allocated log space."""
        self.simulation_step_size_ms = simulation_step_size_ms

    def set_log_period(self, log_period_ms):
        """Sets the period to log new data."""
        self.log_period_ms = log_period_ms

    def log_data(self, current_time_ms):
        """Add the log for the current time into the data log."""
        if len(self.log) == 0:
            # This the first call to the log_data function. First allocate some log space
            expected_num_of_logs = int(self.simulation_time_ms / self.simulation_step_size_ms)
            size_of_a_log = len(self.data_to_log)
            self.log = np.zeros((expected_num_of_logs, size_of_a_log), dtype=float)
            self.temp_current_log = np.zeros((1, size_of_a_log), dtype=float)
            if self.log_period_ms == 1:
                self.log_period_ms = self.simulation_step_size_ms
            print('log_data: log_count: {} single log size: {}'.format(expected_num_of_logs, size_of_a_log))
        if current_time_ms % self.log_period_ms == 0:
            item_index = 0
            if self.data_to_log is not None:
                for data_log_description in self.data_to_log:
                    if data_log_description.item_type == ItemDescription.ITEM_TYPE_TIME:
                        self.temp_current_log[0][item_index] = float(current_time_ms)
                    elif data_log_description.item_type == ItemDescription.ITEM_TYPE_VEHICLE:
                        vehicle_index = data_log_description.log_item_index
                        if len(self.vehicles_manager.vehicles) > vehicle_index:
                            state_index = data_log_description.item_state_index
                            self.temp_current_log[0][item_index] = \
                                self.vehicles_manager.vehicles[vehicle_index].get_vehicle_state_with_id(state_index)
                    item_index += 1
                if self.current_log_size < len(self.log):
                    self.log[self.current_log_size] = self.temp_current_log
                else:
                    # We have run out of log space. Let's add one more space. (This is not expected to happen)
                    self.log = np.append(self.log, self.temp_current_log, axis=0)
                    print('Log size was unexpectedly small.')
                self.current_log_size += 1

    def get_log_info(self):
        """Returns number of log and """
        if len(self.log) > 0:
            log_info = (self.current_log_size, self.temp_current_log.shape[1])
        else:
            log_info = (0, 0)
        return log_info

    def get_log(self, start_index, end_index):
        """Returns the requested part of the data log."""
        if start_index == end_index:
            start_index = 0
            end_index = self.current_log_size
        elif end_index > self.current_log_size:
            end_index = self.current_log_size
        return self.log[start_index:end_index]
