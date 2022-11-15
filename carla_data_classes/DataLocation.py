from dataclasses import dataclass
from typing import Tuple

from carla import Location


@dataclass
class DataLocation:
    """
    Stores simulation coordinates (x, y, z)
    """
    x: float
    y: float
    z: float

    def __init__(self, location: Location = None, x: float = None, y: float = None, z: float = None):
        if location is not None:
            self.x = location.x
            self.y = location.y
            self.z = location.z
        else:
            self.x = x
            self.y = y
            self.z = z

    def to_location(self) -> Location:
        return Location(x=self.x, y=self.y, z=self.z)

    def to_tuple(self) -> Tuple[float, float]:
        return self.x, self.y
