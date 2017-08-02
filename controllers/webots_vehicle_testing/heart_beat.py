"""Defines HeartBeatConfig and HeartBeat classes"""


class HeartBeatConfig(object):
    """HeartBeatConfig class is a structure defining the period of the heart beat, and type of heart beat
    (whether heart beat should be sent, whether a command should be waited after heart beat)"""
    WITH_SYNC = 1
    WITHOUT_SYNC = 2
    NO_HEART_BEAT = 3

    def __init__(self):
        self.sync_type = self.NO_HEART_BEAT
        self.period_ms = 10  # in ms.


class HeartBeat(object):
    """HeartBeat class is a structure defining the current heart beat information"""
    SIMULATION_STOPPED = 0
    SIMULATION_RUNNING = 1

    def __init__(self, simulation_time_ms=0, simulation_status=9):
        self.simulation_time_ms = simulation_time_ms
        self.simulation_status = simulation_status
