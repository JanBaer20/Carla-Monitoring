import datetime
import logging
import os
from dataclasses import dataclass
from enum import Enum
from os.path import dirname
from typing import List

import py_trees
from carla import World, Vehicle, Map

import carla_noise_generator
from carla_data_classes.DataActor import DataActor
from carla_data_classes.DataBlock import DataBlock
from carla_data_classes.DataTrafficLight import DataTrafficLight
from carla_helpers.map_helper import MapHelper
from carla_helpers.world_helper import WorldHelper
from carla_monitors.base_monitor import BaseMonitor
from carla_noise_generator import normal_noise_generator
from scenario_rasterizer import InfrastructureRasterizer, ScenarioSegmentVisualizer
import socket

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 65000  # The port used by the server

logger = logging.getLogger(__name__)


class VehicleOnRoute(BaseMonitor):
    """
    This class contains the monitor to output information about the ego vehicle
    """

    def __init__(self, actor: Vehicle, world: World, waypoints : List, turningPoints : List, debug_mode: bool, name="VehicleOnRoute",
                 generate_static_map_info: bool = False, terminate_on_failure=False,
                 scenario_name="scenario"):
        super(VehicleOnRoute, self).__init__(actor, world, debug_mode, name,
                                                terminate_on_failure=terminate_on_failure)

        self.logger.debug("%s.__init__()" % self.__class__.__name__)
        self._world = world
        self._map: Map = self._world.get_map()
        self._scenario_name = scenario_name
        self.start_time = datetime.datetime.now()

        self._actor: Vehicle = self.actor

        self._waypoints = waypoints
        self._turningPoints = turningPoints
        self._turn_index = 0
        self._leftRoute = False

    # Gets called each tick and checks the implemented test
    def update(self):
        """
        Monitor current information about the ego vehicle
        :return: Current status of the test
        """

        new_test_status = py_trees.common.Status.RUNNING

        # Get all carla Actors as DataActors from carla
        all_actors = self._world_helper.get_actors()
        # Create DataActors from carla.Actors
        data_actors = [self._map_helper.get_data_actor_from_actor(current_actor) for current_actor in all_actors]
        logger.debug(f'data actors: {data_actors}')

        # Get ego vehicle Data Actor by ID
        data_ego_vehicles = list(filter(lambda veh: veh.id == self.actor.id, data_actors))
        if len(data_ego_vehicles) != 1:
            raise ValueError(f'Got wrong amount of ego vehicles: len(data_ego_vehicles) - {data_ego_vehicles}')
        data_ego_vehicle = data_ego_vehicles[0]
        logger.debug(f'ego actors: {data_ego_vehicle}')
        
        actor_waypoint = self._world_helper.get_nearest_waypoint_for_data_actor(data_ego_vehicle)

        if self._turn_index < len(self._turningPoints):
            if self._turningPoints[self._turn_index][0].distance(actor_waypoint.transform.location) < 5:
                self.send_violation(str.encode(self._turningPoints[self._turn_index][1]))
                self._turn_index += 1

        self.test_status = py_trees.common.Status.FAILURE

        for wp in self._waypoints:
            if wp.distance(actor_waypoint.transform.location) < 10:
                self._leftRoute = False
                self.test_status = py_trees.common.Status.RUNNING

        if (not self._leftRoute) and (self.test_status == py_trees.common.Status.FAILURE):
            self.send_violation(b"Vehicle left Route")
            self._leftRoute = True
                
        if self._terminate_on_failure and (self.test_status == py_trees.common.Status.FAILURE):
            new_test_status = py_trees.common.Status.FAILURE
        
        return new_test_status


    def send_violation(self, msg):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            self._socket = s
            s.sendall(msg)
