from enum import Enum


class DataLandmarkType(Enum):
    """
    Enum class for LandmarkType values
    """
    INVALID = 0
    UNKNOWN = 1
    TRAFFIC_SIGN = 2
    TRAFFIC_LIGHT = 3
    POLE = 4
    GUIDE_POST = 5
    TREE = 6
    STREET_LAMP = 7
    POSTBOX = 8
    MANHOLE = 9
    POWERCABINET = 10
    FIRE_HYDRANT = 11
    BOLLARD = 12
    OTHER = 13

    def __eq__(self, other):
        return self.name == other.name
