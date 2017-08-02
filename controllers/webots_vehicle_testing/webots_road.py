"""Defines WebotsRoad Class"""

import math


class WebotsRoad(object):
    """User Configurable Road Structure to use in Webots environment"""
    def __init__(self):
        self.def_name = "STRROAD"
        self.road_type = "StraightRoadSegment"
        self.rotation = [0, 1, 0, math.pi/2]
        self.position = [0, 0, 0]
        self.number_of_lanes = 2
        self.width = self.number_of_lanes * 6
        self.length = 1000
        self.right_border_bounding_object = True
        self.left_border_bounding_object = True
