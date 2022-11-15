from dataclasses import dataclass
from typing import List

from carla_data_classes.DataCriticalSection import DataCriticalSection
from carla_data_classes.DataLane import DataLane


@dataclass
class DataStaticBlock(object):
    """
    Class for information that is within one block calculated by the rasterizer
    """
    block_id: str
    lanes: List[DataLane]
    critical_sections: List[DataCriticalSection]
