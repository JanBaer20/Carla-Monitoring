from dataclasses import dataclass
from typing import List, Optional

from carla_data_classes.DataActor import DataActor
from carla_data_classes.DataLocation import DataLocation
from carla_data_classes.DataTrafficLight import DataTrafficLight


@dataclass
class DataWaypoint:
    """
    Wrapper class for carla Waypoints
    """
    ad_map_lane_id: int
    road_id: int
    lane_id: int
    location: DataLocation
    distance_from_lane_start: float  # Distance to start of lane
    actor: DataActor
    traffic_light: Optional[DataTrafficLight]
