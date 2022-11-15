from dataclasses import dataclass

from carla import Vector3D


@dataclass
class DataVector3D:
    """
    Stores all important information for 3D vectors
    """
    x: float
    y: float
    z: float

    def __init__(self, location: Vector3D):
        self.x = location.x
        self.y = location.y
        self.z = location.z

    def to_location(self) -> Vector3D:
        return Vector3D(x=self.x, y=self.y, z=self.z)
