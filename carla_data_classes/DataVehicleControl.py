from dataclasses import dataclass

from carla import VehicleControl


@dataclass
class DataVehicleControl:
    """
    Stores all important information for VehicleControls, such as gear, brake, hand_brake, etc.
    """
    gear: int
    steer: float
    brake: float
    hand_brake: bool
    manual_gear_shift: bool
    reverse: bool
    throttle: float

    def __init__(self, vehicle_control: VehicleControl):
        self.gear = vehicle_control.gear
        self.steer = vehicle_control.steer
        self.brake = vehicle_control.brake
        self.hand_brake = vehicle_control.hand_brake
        self.manual_gear_shift = vehicle_control.manual_gear_shift
        self.reverse = vehicle_control.reverse
        self.throttle = vehicle_control.throttle

    def to_vehicle_control(self) -> VehicleControl:
        return VehicleControl(gear=self.gear, steer=self.steer, brake=self.brake, hand_brake=self.hand_brake,
                              manual_gear_shift=self.manual_gear_shift, reverse=self.reverse, throttle=self.throttle)
