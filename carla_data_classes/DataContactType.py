from enum import Enum


class DataContactType(Enum):
    """
    Enum class for all ContactType values
    """
    INVALID = 0
    UNKNOWN = 1
    FREE = 2
    LANE_CHANGE = 3
    LANE_CONTINUATION = 4
    LANE_END = 5
    SINGLE_POINT = 6
    STOP = 7
    STOP_ALL = 8
    YIELD = 9
    GATE_BARRIER = 10
    GATE_TOLLBOOTH = 11
    GATE_SPIKES = 12
    GATE_SPIKES_CONTRA = 12
    CURB_UP = 13
    CURB_DOWN = 14
    SPEED_BUMP = 15
    TRAFFIC_LIGHT = 16
    CROSSWALK = 17
    PRIO_TO_RIGHT = 18
    RIGHT_OF_WAY = 19
    PRO_TO_RIGHT_AND_STRAIGHT = 20

    def __eq__(self, other):
        return self.name == other.name
