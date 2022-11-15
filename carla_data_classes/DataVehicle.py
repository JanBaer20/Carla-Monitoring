from dataclasses import dataclass

from carla import Vehicle, GeoLocation

from carla_data_classes.DataActor import DataActor
from carla_data_classes.DataTrafficLight import DataTrafficLight
from carla_data_classes.DataVehicleControl import DataVehicleControl


@dataclass
class DataVehicle(DataActor):
    """
    Wrapper class for carla Vehicles (extends DataActor)
    """
    vehicle_control: DataVehicleControl
    affecting_traffic_light: DataTrafficLight
    affecting_traffic_light_state: int
    light_state: int
    affecting_speed_limit: float
    ego_vehicle: bool

    def __init__(self, vehicle: Vehicle, ad_map_lane_id: int = None, lane_id: int = None, road_id: int = None,
                 geo_location: GeoLocation = None,
                 ego_vehicle: bool = False):
        super().__init__(actor=vehicle, ad_map_lane_id=ad_map_lane_id, lane_id=lane_id, road_id=road_id,
                         geo_location=geo_location)
        self.vehicle_control = DataVehicleControl(vehicle.get_control())
        self.affecting_traffic_light = None
        self.affecting_traffic_light_state = 0
        self.light_state = 0
        self.affecting_speed_limit = 30.0
        self.ego_vehicle = ego_vehicle
