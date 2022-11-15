from dataclasses import dataclass

from carla_data_classes.DataLane import DataLane
from carla_data_classes.DataLocation import DataLocation

CRITICAL_SECTION_MARGIN: float = 3.0


@dataclass
class DataCriticalSection:
    """
    A critical section defines the area between two lanes.
    It is calculated by the contact (crossing) point of the lanes.
    The distance of the contact point to the start of the lane is calculated.
    The CRITICAL_SECTION_MARGIN is then moved to both sides (start and end) of the lane.
    The resulting 4 points
    (distance(contact_point(lane_1))+CRITICAL_SECTION_MARGIN;
    distance(contact_point(lane_1))-CRITICAL_SECTION_MARGIN);
    (distance(contact_point(lane_2))+CRITICAL_SECTION_MARGIN;
    distance(contact_point(lane_2))-CRITICAL_SECTION_MARGIN);
    then span the critical section. The critical section is used for right of way calculations later on
    """
    id: str  # combination of road_id_1+lane_id_1+road_id_2+lane_id_2 # Smaller road id comes first
    contact_point: DataLocation
    ad_map_lane_id_1: int
    road_id_1: int
    lane_id_1: int
    start_pos_1: float
    end_pos_1: float

    ad_map_lane_id_2: int
    road_id_2: int
    lane_id_2: int
    start_pos_2: float
    end_pos_2: float

    def __init__(self, contact_point: DataLocation, lane_1: DataLane, start_pos_lane_1: float, lane_2: DataLane,
                 start_pos_lane_2: float):
        if lane_2.road_id < lane_1.road_id:
            save = lane_1
            lane_1 = lane_2
            lane_2 = save
            save = start_pos_lane_1
            start_pos_lane_1 = start_pos_lane_2
            start_pos_lane_2 = save
        self.id = f"{lane_1.ad_map_lane_id}0{lane_2.ad_map_lane_id}"
        self.contact_point = contact_point
        self.ad_map_lane_id_1 = lane_1.ad_map_lane_id
        self.road_id_1 = lane_1.road_id
        self.lane_id_1 = lane_1.lane_id
        self.start_pos_1 = float(max(0.0, start_pos_lane_1 - CRITICAL_SECTION_MARGIN))
        self.end_pos_1 = float(min(lane_1.lane_length, start_pos_lane_1 + CRITICAL_SECTION_MARGIN))

        self.ad_map_lane_id_2 = lane_2.ad_map_lane_id
        self.road_id_2 = lane_2.road_id
        self.lane_id_2 = lane_2.lane_id
        self.start_pos_2 = float(max(0.0, start_pos_lane_2 - CRITICAL_SECTION_MARGIN))
        self.end_pos_2 = float(min(lane_2.lane_length, start_pos_lane_2 + CRITICAL_SECTION_MARGIN))

    def is_for(self, lane_1: DataLane, lane_2: DataLane) -> bool:
        if lane_1.ad_map_lane_id == self.ad_map_lane_id_1 and lane_2.ad_map_lane_id == self.ad_map_lane_id_2:
            return True
        if lane_2.ad_map_lane_id == self.ad_map_lane_id_1 and lane_1.ad_map_lane_id == self.ad_map_lane_id_2:
            return True
        return False
