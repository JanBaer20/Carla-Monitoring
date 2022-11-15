from dataclasses import dataclass

from carla_data_classes.DataBlock import DataBlock


@dataclass
class DataTick:
    """
    Represents the data for one tick
    """
    tick: float
    data: DataBlock
