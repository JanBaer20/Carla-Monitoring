from dataclasses import dataclass
from typing import List

from carla_data_classes.DataWaypoint import DataWaypoint


@dataclass
class DataBlock(object):
    """
    Stores the information of the blocks from the rasterizer
    """
    block_id: str
    waypoints: List[DataWaypoint]
