from dataclasses import dataclass
from typing import List

from carla_data_classes.DataStaticBlock import DataStaticBlock


@dataclass
class DataStaticMapInformation(object):
    """
    Class for information that is obtainable from map data and is not
    dependent on simulation or dynamic data
    """
    static_blocks: List[DataStaticBlock]
