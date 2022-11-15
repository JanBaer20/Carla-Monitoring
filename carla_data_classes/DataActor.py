from dataclasses import dataclass
from typing import Optional

from carla import Actor, GeoLocation

from carla_data_classes.DataGeoLocation import DataGeoLocation
from carla_data_classes.DataLocation import DataLocation
from carla_data_classes.DataRotation import DataRotation
from carla_data_classes.DataVector3D import DataVector3D


@dataclass
class DataActor:
    """
    Stores all important information for Actors
    """
    id: int
    ad_map_lane_id: int
    road_id: int
    lane_id: int
    ego_vehicle: bool
    type_id: str
    location: DataLocation
    rotation: DataRotation
    forward_vector: DataVector3D
    velocity: DataVector3D
    acceleration: DataVector3D
    angular_velocity: DataVector3D
    geo_location: Optional[DataGeoLocation]

    def __init__(self, actor: Actor, ad_map_lane_id: int = None, lane_id: int = None, road_id: int = None,
                 geo_location: GeoLocation = None):
        self.id = actor.id
        self.ad_map_lane_id = ad_map_lane_id
        self.road_id = road_id
        self.lane_id = lane_id
        self.type_id = actor.type_id
        self.actor_state = actor.actor_state
        if geo_location is not None:
            self.geo_location = DataGeoLocation(geo_location=geo_location)
        self.location = DataLocation(actor.get_location())
        self.rotation = DataRotation(actor.get_transform().rotation)
        self.forward_vector = DataVector3D(actor.get_transform().get_forward_vector())
        self.velocity = DataVector3D(actor.get_velocity())
        self.acceleration = DataVector3D(actor.get_acceleration())
        self.angular_velocity = DataVector3D(actor.get_angular_velocity())
        self.ego_vehicle = False

    def update_lane_information(self, ad_map_lane_id: int, lane_id: int, road_id: int):
        self.ad_map_lane_id = ad_map_lane_id
        self.lane_id = lane_id
        self.road_id = road_id
