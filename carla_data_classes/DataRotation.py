from dataclasses import dataclass

from carla import Rotation


@dataclass
class DataRotation:
    """
    Stores the Rotation for Actors (pitch, yaw, roll)
    """
    pitch: float
    yaw: float
    roll: float

    def __init__(self, rotation: Rotation):
        self.pitch = rotation.pitch
        self.yaw = rotation.yaw
        self.roll = rotation.roll

    def to_rotation(self) -> Rotation:
        return Rotation(pitch=self.pitch, yaw=self.yaw, roll=self.roll)
