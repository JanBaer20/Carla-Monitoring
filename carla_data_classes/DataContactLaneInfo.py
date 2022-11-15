from dataclasses import dataclass
from enum import Enum

import ad_map_access as ad

from carla_data_classes.DataContactLocation import DataContactLocation
from carla_data_classes.DataContactType import DataContactType


@dataclass
class DataContactLaneInfo:
    """
    This class wraps the carla ContactLaneInfo class
    """
    ad_map_lane_id: int
    road_id: int
    lane_id: int
    contact_type: DataContactType
    contact_location: DataContactLocation
    traffic_light_id: int

    def __init__(self, ad_map_lane_id, road_id, lane_id, contact_type, contact_location, traffic_light_id=None):
        if type(ad_map_lane_id) == ad.map.lane.LaneId:
            self.ad_map_lane_id = int(str(ad_map_lane_id))
        else:
            self.ad_map_lane_id = ad_map_lane_id
        self.road_id = road_id
        self.lane_id = lane_id
        self.contact_location = contact_location
        self.contact_type = contact_type
        self.traffic_light_id = traffic_light_id
