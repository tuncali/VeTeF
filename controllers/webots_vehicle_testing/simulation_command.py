"""Defines SimulationCommand class"""


class SimulationCommand(object):
    """SimulationCommand class is a structure keeping control command and the related object received."""
    def __init__(self, command, command_object):
        self.command = command
        self.object = command_object
