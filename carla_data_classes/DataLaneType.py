from enum import Enum


class DataLaneType(Enum):
    """
    Enum class for LaneType values
    """
    INVALID = 0
    UNKNOWN = 1
    NORMAL = 2
    INTERSECTION = 3
    SHOULDER = 4
    EMERGENCY = 5
    MULT = 6
    PEDESTRIAN = 7
    OVERTAKING = 8
    TURN = 9

    def __eq__(self, other):
        return self.name == other.name
