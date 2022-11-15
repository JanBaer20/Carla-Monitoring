from dataclasses import dataclass

from carla import TrafficLight

from carla_data_classes.DataActor import DataActor
from carla_data_classes.DataGeoLocation import DataGeoLocation
from carla_data_classes.DataTrafficLightState import DataTrafficLightState


@dataclass
class DataTrafficLight(DataActor):
    """
    Represents the data class for Traffic Lights
    """
    state: DataTrafficLightState

    def __init__(self, traffic_light: TrafficLight):
        super().__init__(traffic_light)
        self.state = DataTrafficLightState(int(traffic_light.state))
        self.geo_location = DataGeoLocation()
