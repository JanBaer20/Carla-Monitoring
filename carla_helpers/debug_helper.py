import random
from typing import List

import carla
from carla import Junction, Waypoint, Actor, Vector3D, Location

from carla_data_classes.DataActor import DataActor
from carla_helpers.world_helper import WorldHelper


class DebugHelper(object):
    """
    This class can be used the easily debug information into the carla simulation
    """
    BOUNDING_BOX_THICKNESS = 0.05

    def __init__(self, world, debug_lifespan=10.0, text_only=False):
        super(DebugHelper, self).__init__()

        self._world = world
        self._debug = world.debug
        self._world_helper: WorldHelper = WorldHelper(world)
        self._text_only: bool = text_only
        self._debug_lifespan: float = debug_lifespan  # Time in seconds for the debug output to last

        random.seed(0)

    def draw_debug_output_for_junction(self, junction: Junction,
                                       color: carla.Color = carla.Color(0, random.randrange(0, 255), 0, 0)) -> None:
        if not self._text_only:
            # Draw bounding box of current junction
            self._debug.draw_box(junction.bounding_box, junction.bounding_box.rotation, self.BOUNDING_BOX_THICKNESS,
                                 color=color,
                                 life_time=float(self._debug_lifespan))
        if junction is not None:
            # Get all waypoints for the current junction
            junction_waypoints = self._world_helper.get_all_waypoints_for_junction(junction)
            for lane_id, waypoints in junction_waypoints:
                # Choose different color for each lane
                color = carla.Color(random.randrange(0, 255), random.randrange(0, 255), random.randrange(0, 255))
                # Print box for each waypoint of the lane
                self.draw_debug_output_for_waypoints(waypoints, color)

    def draw_debug_output_for_waypoints(self, waypoints: List[Waypoint],
                                        color: carla.Color = carla.Color(0, random.randrange(0, 255), 0, 0)) -> None:
        for waypoint in waypoints:
            self.draw_debug_output_for_waypoint(waypoint, color)

    def draw_debug_output_for_waypoint(self, waypoint: Waypoint,
                                       color: carla.Color = carla.Color(0, random.randrange(0, 255), 0, 0)) -> None:
        if not self._text_only:
            self._debug.draw_box(carla.BoundingBox(waypoint.transform.location, carla.Vector3D(0.1, 0.1, 1)),
                                 waypoint.transform.rotation, self.BOUNDING_BOX_THICKNESS, color=color,
                                 life_time=self._debug_lifespan)
        self._debug.draw_string(waypoint.transform.location, waypoint.lane_id.__str__(), color=color,
                                life_time=self._debug_lifespan)

    def draw_debug_output_for_actor(self, actor: Actor,
                                    color: carla.Color = carla.Color(0, random.randrange(0, 255), 0, 0)) -> None:
        if not self._text_only:
            self._debug.draw_box(carla.BoundingBox(actor.get_transform().location, carla.Vector3D(0.5, 0.5, 20)),
                                 actor.get_transform().rotation, self.BOUNDING_BOX_THICKNESS, color=color,
                                 life_time=self._debug_lifespan)

    def draw_debug_output_for_data_actor(self, actor: DataActor,
                                    color: carla.Color = carla.Color(0, random.randrange(0, 255), 0, 0)) -> None:
        if not self._text_only:
            self._debug.draw_box(carla.BoundingBox(actor.location.to_location(), carla.Vector3D(0.5, 0.5, 20)),
                                 actor.rotation.to_rotation(), self.BOUNDING_BOX_THICKNESS, color=color,
                                 life_time=self._debug_lifespan)

    def draw_debug_output_for_lines(self, line: (Vector3D, Vector3D),
                                    color: carla.Color = carla.Color(random.randrange(0, 255), 0, 0, 0)) -> None:
        if not self._text_only:
            (point1, point2) = line
            location_left = Location(point1)
            location_right = Location(point2)
            self._debug.draw_line(location_left, location_right, self.BOUNDING_BOX_THICKNESS, color=color,
                                  life_time=self._debug_lifespan)
