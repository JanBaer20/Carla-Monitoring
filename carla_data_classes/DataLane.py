from dataclasses import dataclass
from typing import List

from carla_data_classes.DataContactLaneInfo import DataContactLaneInfo
from carla_data_classes.DataLandmark import DataLandmark
from carla_data_classes.DataLaneType import DataLaneType
from carla_data_classes.DataLocation import DataLocation
from carla_data_classes.DataSpeedLimit import DataSpeedLimit


@dataclass
class DataLane:
    """
    Stores all important information for Lanes
    """
    ad_map_lane_id: int
    lane_id: int
    road_id: int
    lane_type: DataLaneType
    lane_width: float
    lane_length: float
    direction: int
    speed_limits: List[DataSpeedLimit]
    predecessor_lanes: List[DataContactLaneInfo]
    successor_lanes: List[DataContactLaneInfo]
    intersecting_lanes: List[DataContactLaneInfo]
    lane_midpoints: List[DataLocation]
    landmarks: List[DataLandmark]

    @staticmethod
    def calculate_midpoints(left_edge, right_edge) -> List[DataLocation]:
        # Initialize list for midpoints of current lane
        lane_midpoints: List[DataLocation] = []

        for edge_position_index in range(len(left_edge)):
            # Get current edge point
            left_edge_point = left_edge[edge_position_index]
            right_edge_point = right_edge[edge_position_index]
            # Convert to locations
            left_edge_location = DataLocation(x=float(left_edge_point.x), y=float(-left_edge_point.y),
                                              z=float(left_edge_point.z))
            right_edge_location = DataLocation(x=float(right_edge_point.x), y=float(-right_edge_point.y),
                                               z=float(right_edge_point.z))
            # Get midpoint (the point is then located in the middle of the lane)
            midpoint = DataLane.__calculate_midpoint(left_edge_location, right_edge_location)
            # Add to list of all midpoints
            lane_midpoints.append(midpoint)

        return lane_midpoints

    @staticmethod
    def __calculate_midpoint(data_location_1: DataLocation, data_location_2: DataLocation) -> DataLocation:
        """
        Calculate the midpoint between the given locations
        :param data_location_1: Location 1
        :param data_location_2: Location 2
        :return: Midpoint of given locations
        """
        mid_point_x = (data_location_1.x + data_location_2.x) / 2
        mid_point_y = (data_location_1.y + data_location_2.y) / 2
        mid_point_z = (data_location_1.z + data_location_2.z) / 2
        return DataLocation(x=float(mid_point_x), y=float(mid_point_y), z=float(mid_point_z))
