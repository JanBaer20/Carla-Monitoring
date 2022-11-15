from dataclasses import dataclass
from typing import Optional

from carla_data_classes.DataLandmarkType import DataLandmarkType
from carla_data_classes.DataTrafficSignType import DataTrafficSignType


@dataclass
class DataLandmark(object):
    """
    Class for Landmarks with their types and optional TrafficLightIds
    """
    landmark_type: DataLandmarkType
    traffic_sign_type: DataTrafficSignType
    traffic_light_id: Optional[int] = None  # Nullable
