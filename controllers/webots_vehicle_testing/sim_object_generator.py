"""Defines SimObjectGenerator class"""
from webots_sensor import WebotsSensor


class SimObjectGenerator(object):
    """SimObjectGenerator class translate between simulation objects and the strings declaring the objects in Webots."""
    VHC_DUMMY = 0
    VHC_VUT = 1

    def __init__(self):
        pass

    def generate_road_network_string(self, road_list, road_network_id):
        """Generates the Webots string for a list of road segments. These road segments go under a road segment."""
        road_network_string = "#VRML_OBJ R2018a utf8\n"
        road_network_string += "DEF ROAD_NETWORK_" + str(road_network_id) + " Solid {\n"
        road_network_string += "  children ["
        for i in range(0, len(road_list)):
            road_network_string += "    DEF " + road_list[i].def_name + " " + road_list[i].road_type + " {"
            road_network_string += "      translation " + str(road_list[i].position[0]) + " " + str(road_list[i].position[1]) + " " + str(road_list[i].position[2]) + " " + "\n"
            road_network_string += "      rotation " + str(road_list[i].rotation[0]) + " " + str(road_list[i].rotation[1]) + " " + str(road_list[i].rotation[2]) + " " + str(road_list[i].rotation[3]) + " " + "\n"
            road_network_string += "      width " + str(road_list[i].width) + "\n"
            road_network_string += "      numberOfLanes " + str(road_list[i].number_of_lanes) + "\n"
            road_network_string += "      length " + str(road_list[i].length) + "\n"
            road_network_string += "      rightBorderBoundingObject " + str(road_list[i].right_border_bounding_object).upper() + "\n"
            road_network_string += "      leftBorderBoundingObject " + str(road_list[i].left_border_bounding_object).upper() + "\n"
            road_network_string += "    }\n"
        road_network_string += "  ]\n"
        road_network_string += "}\n"
        return road_network_string

    def generate_vehicle_string(self, vhc_object):
        """Generates the Webots string for a vehicle."""
        vehicle_string = "#VRML_OBJ R2018a utf8\n"
        vehicle_string += "DEF " + vhc_object.def_name + " " + vhc_object.vehicle_model + " {\n"
        vehicle_string += "  translation " + str(vhc_object.current_position[0]) + " " + str(vhc_object.current_position[1]) + " " + str(vhc_object.current_position[2]) + " " + "\n"
        vehicle_string += "  rotation " + str(vhc_object.current_rotation[0]) + " " + str(vhc_object.current_rotation[1]) + " " + str(vhc_object.current_rotation[2]) + " " + str(vhc_object.current_rotation[3]) + " " + "\n"
        vehicle_string += "  color " + str(vhc_object.color[0]) + " " + str(vhc_object.color[1]) + " " + str(vhc_object.color[2]) + "\n"
        vehicle_string += '  name \"' + vhc_object.def_name + '\"\n'
        for (param_name, param_val) in vhc_object.vehicle_parameters:
            vehicle_string += '  ' + param_name + ' ' + param_val + '\n'
        if vhc_object.controller != "void":
            vehicle_string += "  controller \"vehicle_controller\"\n"
            vehicle_string += "  controllerArgs \"" + vhc_object.controller + " " + vhc_object.vehicle_model
            for argument_string in vhc_object.controller_arguments:
                vehicle_string += ' '
                vehicle_string += argument_string
            vehicle_string += "\"\n"
        else:
            vehicle_string += "  controller \"void\"\n"

        # Add text for Front Sensors if there is any
        vehicle_string += self.generate_sensor_string(vhc_object.sensor_array, WebotsSensor.FRONT)
        # Add text for Center Sensors if there is any
        vehicle_string += self.generate_sensor_string(vhc_object.sensor_array, WebotsSensor.CENTER)
        # Add text for Left Sensors if there is any
        vehicle_string += self.generate_sensor_string(vhc_object.sensor_array, WebotsSensor.LEFT)
        vehicle_string += self.generate_sensor_string(vhc_object.sensor_array, WebotsSensor.LEFT_FRONT)
        vehicle_string += self.generate_sensor_string(vhc_object.sensor_array, WebotsSensor.LEFT_REAR)
        # Add text for Right Sensors if there is any
        vehicle_string += self.generate_sensor_string(vhc_object.sensor_array, WebotsSensor.RIGHT)
        vehicle_string += self.generate_sensor_string(vhc_object.sensor_array, WebotsSensor.RIGHT_FRONT)
        vehicle_string += self.generate_sensor_string(vhc_object.sensor_array, WebotsSensor.RIGHT_REAR)
        # Add text for Rear Sensors if there is any
        vehicle_string += self.generate_sensor_string(vhc_object.sensor_array, WebotsSensor.REAR)
        # Add text for Top Sensors if there is any
        vehicle_string += self.generate_sensor_string(vhc_object.sensor_array, WebotsSensor.TOP)
        vehicle_string += "}\n"
        return vehicle_string

    def get_sensor_string(self, sensor, tab_size):
        """Generates the part of the sensor string which contains sensor type and parameters."""
        tab_str = " " * tab_size
        sensor_str = tab_str + sensor.sensor_type + " {\n"
        for sensor_field in sensor.sensor_fields:
            sensor_str = sensor_str + tab_str + "  " + sensor_field.field_name + " " + sensor_field.field_val + "\n"
        sensor_str = sensor_str + tab_str + "}\n"
        return sensor_str

    def generate_sensor_string(self, sensor_array, sensor_location):
        """Generates the Webots string for a sensor."""
        # Add text for Left Sensors if there is any
        sensor_count = 0
        sensor_string = ""
        for sensor in sensor_array:
            if sensor.sensor_location == sensor_location:
                if sensor_count == 0:
                    if sensor_location == WebotsSensor.LEFT:
                        sensor_string += "  sensorsSlotLeft [\n"
                    elif sensor_location == WebotsSensor.RIGHT:
                        sensor_string += "  sensorsSlotRight [\n"
                    elif sensor_location == WebotsSensor.LEFT_FRONT:
                        sensor_string += "  sensorsSlotLeftFront [\n"
                    elif sensor_location == WebotsSensor.LEFT_REAR:
                        sensor_string += "  sensorsSlotLeftRear [\n"
                    elif sensor_location == WebotsSensor.RIGHT_FRONT:
                        sensor_string += "  sensorsSlotRightFront [\n"
                    elif sensor_location == WebotsSensor.RIGHT_REAR:
                        sensor_string += "  sensorsSlotRightRear [\n"
                    elif sensor_location == WebotsSensor.CENTER:
                        sensor_string += "  sensorsSlotCenter [\n"
                    elif sensor_location == WebotsSensor.TOP:
                        sensor_string += "  sensorsSlotTop [\n"
                    elif sensor_location == WebotsSensor.REAR:
                        sensor_string += "  sensorsSlotRear [\n"
                    elif sensor_location == WebotsSensor.FRONT:
                        sensor_string += "  sensorsSlotFront [\n"
                    elif sensor_location == WebotsSensor.LEFT:
                        sensor_string += "  sensorsSlotUnknown [\n"
                sensor_count += 1
                sensor_string += self.get_sensor_string(sensor, 4)
        if sensor_count > 0:
            sensor_string += "  ]\n"
        return sensor_string
