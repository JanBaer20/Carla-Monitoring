from enum import Enum


class DataTrafficLightState(Enum):
    """
    Enum class for TrafficLightState values
    """
    RED = 0
    YELLOW = 1
    GREEN = 2
    OFF = 3
    UNKNOWN = 4
    SIZE = 5
