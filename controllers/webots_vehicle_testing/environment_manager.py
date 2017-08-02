"""Defines EnvironmentManager class"""
from coordinate_system import CoordinateSystem


class EnvironmentManager(object):
    """Records road definitions for the current Webots environment."""
    def __init__(self):
        self.debug_mode = 0
        self.MAX_LEFT = 4.5
        self.MAX_RIGHT = -4.5
        self.road_network = []

    def record_road_segment(self, road_segment):
        """Adds road_segment into the record."""
        self.road_network.append(road_segment)

    def get_num_of_road_networks(self):
        """Returns the number of recorded road segments"""
        return len(self.road_network)

    def record_road_network(self, road_segment_list):
        """Adds all the road segments given as a list into the record."""
        for road_segment in road_segment_list:
            self.record_road_segment(road_segment)

    def get_distance_from_pt_to_road_network(self, pt):
        """Computes the distance from a point to the boundaries of the road.
        !!! Not implemented completely and correctly!!!"""
        # TODO: Compute the actual distance to any kind of road segment
        dist = 0.0
        if pt[CoordinateSystem.LAT_AXIS] > self.MAX_LEFT:
            dist = pt[CoordinateSystem.LAT_AXIS] - self.MAX_LEFT
        elif pt[CoordinateSystem.LAT_AXIS] < self.MAX_RIGHT:
            dist = -pt[CoordinateSystem.LAT_AXIS] + self.MAX_RIGHT

        return dist
