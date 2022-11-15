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
from carla_noise_generator.normal_noise_generator import NoiseTypeEnum
from scenario_rasterizer import InfrastructureRasterizer, ScenarioSegmentVisualizer

logger = logging.getLogger(__name__)


class NoiseTargetEnum(Enum):
    ALL = 0,
    EGO = 1,
    OTHERS = 2


@dataclass()
class NoiseData:
    type: List[NoiseTypeEnum]
    target: NoiseTargetEnum
    mean: float
    deviation: float
    shape: int

    def __init__(self, noise_type: List[NoiseTypeEnum] = None, target: NoiseTargetEnum = NoiseTargetEnum.ALL,
                 mean: float = 0.0, deviation: float = 0.0, shape: int = 2):
        if noise_type is None:
            self.type = [NoiseTypeEnum.NO_NOISE]
        else:
            self.type = noise_type
        self.type = noise_type
        if shape < 0 or shape > 3:
            raise ValueError(f'Shape: {shape} is not suitable for the noise in 3d.')
        self.mean = mean
        self.deviation = deviation
        self.shape = shape
        self.target = target
        self.shutdown = False


class MonitorEgoVehicle(BaseMonitor):
    """
    This class contains the monitor to output information about the ego vehicle
    """

    def __init__(self, actor: Vehicle, world: World, debug_mode: bool, name="MonitorEgoVehicle",
                 generate_static_map_info: bool = False, terminate_on_failure=False, noise: NoiseData = None,
                 scenario_name="scenario"):
        super(MonitorEgoVehicle, self).__init__(actor, world, debug_mode, name,
                                                terminate_on_failure=terminate_on_failure)

        self.logger.debug("%s.__init__()" % self.__class__.__name__)
        self._world = world
        self._map: Map = self._world.get_map()
        self._scenario_name = scenario_name
        self.start_time = datetime.datetime.now()

        # just gather data for noising of actions
        self.noise = noise

        self._rasterizer = InfrastructureRasterizer()
        open_drive = world.get_map().to_opendrive()
        # Analyze blocks for current map
        logger.info("Analyze blocks for current map.")
        blocks = self._rasterizer.analyze_map_from_xodr_content(open_drive)
        logger.info("All blocks have been calculated")

        self._map_helper = MapHelper(self._rasterizer)
        self._world_helper = WorldHelper(world, self._rasterizer)

        # Analyze map and output static information into json
        if generate_static_map_info:
            logger.info("Calculate static map information")
            static_map_information = self._map_helper.get_static_map_information(blocks=blocks)
            logger.info("Static map information has been calculated")

            # Write static map information in to JSON file
            file_path = self.get_log_file_path(f"StaticData_{self._map.name.replace('/', '-')}_{self.name}")
            self.log_data(static_map_information, file_path)
        else:
            logger.info("No static map information generated per user request")

        if debug_mode:
            block_visualizer = ScenarioSegmentVisualizer()
            block_visualizer.visualize_segments_in_carla(blocks, debug_lifespan=0, altitude_offset=15.0,
                                                         draw_block_ids=False, line_width=.15)

        self._actor: Vehicle = self.actor

    # Gets called each tick and checks the implemented test
    def update(self):
        """
        Monitor current information about the ego vehicle
        :return: Current status of the test
        """
        if not self.calculate_tick():
            return py_trees.common.Status.RUNNING

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

        # add noise to data actors
        if self.noise:
            self.logger.debug('adding noise')
            targets = []
            if self.noise.target is NoiseTargetEnum.ALL:
                targets = list(filter(lambda act: "walker" in act.type_id or "vehicle" in act.type_id, data_actors))
            elif self.noise.target is NoiseTargetEnum.EGO:
                targets = [data_ego_vehicle]
            else:  # OTHERS
                targets = list(
                    filter(lambda t: t is not data_ego_vehicle and ("walker" in t.type_id or "vehicle" in t.type_id,),
                           data_actors))  # TODO This might fail

            parent_directory = dirname(dirname(os.path.abspath(__file__)))
            # Path where logs folder should go
            log_directory = os.path.join(parent_directory, "logs")
            logger.debug(f'Actors before Noising: {targets}')
            normal_noise_generator.apply_normal_noise(actors=targets,
                                                     noise_type=self.noise.type,
                                                     mean=self.noise.mean,
                                                     deviation=self.noise.deviation,
                                                     shape=self.noise.shape,
                                                     log_data=True,
                                                     log_folder=log_directory,
                                                     file_pattern=f'noising_{self._scenario_name}_{self.noise.mean}_{self.noise.deviation}_{self.start_time.strftime("%Y-%m-%d %H:%M:%S")}')
            logger.debug(f'Actors after Noising: {targets}')

        # Get Road and Lane Information for Data Actors
        for current_actor in data_actors:
            # Get lane for current actor
            actor_waypoint = self._world_helper.get_nearest_waypoint_for_data_actor(current_actor)
            ad_map_lane_id = self._world_helper.get_ad_map_lane_id(actor_waypoint)
            open_drive_lane_id = self._map_helper.get_open_drive_lane_id_for_ad_map_lane_id(ad_map_lane_id)
            road_id = self._map_helper.get_road_id_for_ad_map_lane_id(ad_map_lane_id)
            # Add Lane and Road information to DataActors
            current_actor.update_lane_information(ad_map_lane_id=ad_map_lane_id, lane_id=open_drive_lane_id,
                                                  road_id=road_id)

        # Get the ego block
        ego_block = self._rasterizer.get_block_for_lane(data_ego_vehicle.ad_map_lane_id)

        # Filter all actors be the ego_block
        actors_in_block: List[DataActor] = self._map_helper.get_actors_in_block(ego_block, data_actors)
        waypoints = []
        for actor in actors_in_block:
            if actor.id == self.actor.id:
                actor.ego_vehicle = True
            else:
                actor.ego_vehicle = False
            # Get TrafficLight (if existing)
            logger.debug(f'[{actor.type_id}]{actor.ad_map_lane_id} - {actor.location}')
            current_lane = self._map_helper.get_lane_for_lane_id(actor.ad_map_lane_id)
            traffic_light_landmark = self._map_helper.get_traffic_light_for_lane(current_lane)
            if traffic_light_landmark is not None:
                traffic_light = self._world.get_traffic_light_from_opendrive_id(
                    str(traffic_light_landmark.traffic_light_id))
                data_traffic_light = DataTrafficLight(traffic_light)
            else:
                data_traffic_light = None
            data_waypoint = self._map_helper.get_data_waypoint_for_data_actor(actor, data_traffic_light)
            waypoints.append(data_waypoint)
        data_block = DataBlock(block_id=ego_block.id, waypoints=waypoints)
        if self.noise is not None:
            scenario_name = f"{self._scenario_name}_{self.noise.mean}_{self.noise.deviation}"
        else:
            scenario_name = self._scenario_name
        self.log(data_obj=data_block, scenario_name=scenario_name)
        # There is no failure criterion. Therefore, keep running
        return py_trees.common.Status.RUNNING
