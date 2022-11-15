from dataclasses import dataclass

from restriction import SpeedLimit


@dataclass
class DataSpeedLimit:
    """
    Class that stores SpeedLimit information of roads.
    Each SpeedLimit only counts for specific segments of a road.
    The specific speed_limit is therefore specified for each segment.
    Each segment is given as a from/to distance from the start of the road
    """
    speed_limit: float
    from_distance_from_start: float
    to_distance_from_start: float

    def __init__(self, speed_limit_obj: SpeedLimit, lane_length: float):
        self.speed_limit = float(speed_limit_obj.speedLimit)
        self.from_distance_from_start = float(float(speed_limit_obj.lanePiece.minimum) * lane_length)
        self.to_distance_from_start = float(float(speed_limit_obj.lanePiece.maximum) * lane_length)
