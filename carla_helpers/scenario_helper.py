import logging
import re
from typing import List, Tuple

import carla
from agents.navigation.local_planner import RoadOption
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import WaypointFollower

logger = logging.getLogger(__name__)


class ScenarioHelper:

    def __init__(self, carla_map: carla.Map):
        self._map = carla_map

    def get_actor_by_role_name(self, actors: List[carla.Actor], rolename: str) -> carla.Actor:
        objs = list(filter(lambda s: re.match(rolename, s.attributes['role_name']), actors))
        # TODO Problem: VEH1 matches VEH1, VEH10, VEH11
        if len(objs) < 1:
            logger.warning(f'No actor found for role name {rolename}')
            return None
        if len(objs) > 1:
            logger.warning(f'Mulitple actors found for role name {rolename}. Using first actor in list!')
        return objs[0]

    def get_waypoint_plan(self, locations: List[Tuple[carla.Location, RoadOption]], lane_type=carla.LaneType.Driving) -> \
            List[Tuple[carla.Waypoint, RoadOption]]:
        waypoints = [(self._map.get_waypoint(w, lane_type=lane_type), o) for w, o in locations]
        return waypoints

    def get_waypoint_follower_for_actor(self, role_name: str, actors: List[carla.Actor], speed: int,
                                        waypoints: List[Tuple[carla.Waypoint, RoadOption]],
                                        avoid_collision: bool = True) -> WaypointFollower:
        veh1 = self.get_actor_by_role_name(actors, role_name)
        if veh1 is None:
            raise ValueError(f'No Actor found for role name {role_name}')
        return WaypointFollower(veh1, speed, waypoints, avoid_collision=avoid_collision)
