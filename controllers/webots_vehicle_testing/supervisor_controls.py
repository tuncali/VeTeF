"""Defines SupervisorControls class"""
from controller import Supervisor, Robot, Node, Field


class SupervisorControls(Supervisor):
    """SupervisorControls class inherits the Supervisor class defined in Webots Python controller interface,
     and carries direct supervisor actions for controlling the simulation."""
    DEBUG_MODE = 1
    # If we add _init_() function, Webots give error.

    def init(self, params):
        self.roots_children = self.getRoot().getField("children")

    def initialize_creating_simulation_environment(self):
        """Initialize the simulation environment creation process. (Just sets the simulation mode to PAUSE)"""
        self.set_simulation_mode(self.SIMULATION_MODE_PAUSE)

    def finalize_creating_simulation_environment(self):
        """Finalizes the simulation environment creation process. (Just steps the simulation)"""
        self.step_simulation(0)

    def enable_receiver(self, receiver_name, period_ms):
        """Enables the receiver."""
        receiver = self.getReceiver(receiver_name)
        receiver.enable(period_ms)

    def get_receiver(self, receiver_name):
        """Returns the receiver object."""
        receiver = self.getReceiver(receiver_name)
        return receiver

    def enable_emitter(self, emitter_name, period_ms):
        """Enables the given emitter."""
        emitter = self.getEmitter(emitter_name)
        emitter.enable(period_ms)

    def get_emitter(self, emitter_name):
        """Returns the emitter object."""
        emitter = self.getEmitter(emitter_name)
        return emitter

    def get_obj_node(self, obj):
        """Returns the object node using the def_name"""
        return self.getFromDef(obj.def_name)

    def get_obj_field(self, obj, field_name):
        """Returns the requested field."""
        return obj.node.getField(field_name)

    def add_obj_to_sim_from_string(self, object_string):
        """Adds the given string as an object to the simulation."""
        self.roots_children.importMFNodeFromString(-1, object_string)
        self.step(0)

    def step_simulation(self, simulation_step_size_ms):
        """Steps the simulation."""
        self.step(simulation_step_size_ms)

    def set_simulation_mode(self, mode):
        """Sets the simulation mode."""
        self.simulationSetMode(mode)

    def revert_simulation(self):
        """Reverts the simulation. Before reloading the world,
        we will first change simulation state to "RUN" so that supervisor starts again after revert."""
        self.set_simulation_mode(self.SIMULATION_MODE_FAST)
        self.step_simulation(0)
        self.simulationRevert()

    def set_obj_position_3D(self, obj, position):
        """Sets the position of the object in 3D."""
        obj.translation.setSFVec3f(position)

    def get_obj_position_3D(self, obj):
        """Gets the position of the object in 3D."""
        pos = obj.translation.getSFVec3f()
        return pos

    def set_obj_velocity(self, obj, velocity):
        """Sets the velocity of the object."""
        obj.node.setVelocity(velocity)

    def get_obj_velocity(self, obj):
        """Gets the current velocity of the object."""
        vel = obj.node.getVelocity()
        return vel

    def set_obj_rotation(self, obj, rotation):
        """Sets the rotation of the object."""
        obj.rotation.setSFRotation(rotation)

    def get_obj_rotation(self, obj):
        """Sets the rotation of the object."""
        rotation = obj.rotation.getSFRotation()
        return rotation

    def reset_obj_physics(self, obj):
        """Resets the physics for the given simulation object."""
        obj.node.resetPhysics()
