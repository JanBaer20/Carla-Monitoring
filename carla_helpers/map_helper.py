import concurrent.futures
import logging
from typing import List, Set

import ad_map_access as ad
import lane
import numpy as np
from carla import TrafficLight, Actor, GeoLocation, Vehicle, ServerSideSensor, Rotation
from match import MapMatchedPosition

from carla_data_classes.DataActor import DataActor
from carla_data_classes.DataContactLaneInfo import DataContactLaneInfo
from carla_data_classes.DataContactLaneInfo import DataContactLocation
from carla_data_classes.DataContactLaneInfo import DataContactType
from carla_data_classes.DataCriticalSection import DataCriticalSection
from carla_data_classes.DataGeoLocation import DataGeoLocation
from carla_data_classes.DataLandmark import DataLandmark
from carla_data_classes.DataLandmarkType import DataLandmarkType
from carla_data_classes.DataLane import DataLane
from carla_data_classes.DataLaneType import DataLaneType
from carla_data_classes.DataLocation import DataLocation
from carla_data_classes.DataSpeedLimit import DataSpeedLimit
from carla_data_classes.DataStaticBlock import DataStaticBlock
from carla_data_classes.DataStaticMapInformation import DataStaticMapInformation
from carla_data_classes.DataTrafficLight import DataTrafficLight
from carla_data_classes.DataTrafficSignType import DataTrafficSignType
from carla_data_classes.DataWaypoint import DataWaypoint
from carla_noise_generator.normal_noise_generator import apply_normal_noise, NoiseTypeEnum
from scenario_rasterizer import ScenarioSegment, InfrastructureRasterizer

ACTOR_BLOCK_FILTERING_SWITCH = True

CONTINUATION_LANE_TYPES = {lane.PREDECESSOR,
                           lane.SUCCESSOR}

logger = logging.getLogger(__name__)


# noinspection PyMethodMayBeStatic,PyUnresolvedReferences,PyArgumentList
class MapHelper(object):
    """
    This class provides methods returning information using the OpenDrive map data
    """
    DEBUG_LIFESPAN = 5  # Time in seconds for the debug output to last

    def __init__(self, rasterizer: InfrastructureRasterizer):
        super(MapHelper, self).__init__()

        self._rasterizer = rasterizer

    def is_object_in_block_of_actor(self, obj: DataActor, block: ScenarioSegment) -> bool:
        """
        Returns whether the given object is inside the ScenarioPartitionBlock of the given Actor
        :param obj: Object to be checked to be in the ScenarioPartitionBlock
        :param block: The block ScenarioPartitionBlock for which the location should be checked
        :return: True if the obj is inside the Actors' ScenarioPartitionBlock, False otherwise
        """
        if self._rasterizer is None:
            return True  # without rasterization just get the world as one block
        # Get GeoLocation for the obj
        geo_location_for_obj = self.transform_data_location_to_geo_location(obj.location)
        # Check of the GeoLocation is inside of the block
        return block.is_in(geo_location_for_obj.to_geolocation())

    def get_best_block_for_actor(self, geo_location: GeoLocation, actor: Actor) -> ScenarioSegment:
        """
        Return ScenarioPartitionBlock for current actor
        :param geo_location: GeoLocation of the actor as this is calculated through the simulation
        :param actor: Actor for which the ScenarioPartitionBlock should be calculated
        :return: The most probable ScenarioPartitionBlock for the given Actor
        """
        # Get rotation of current Actor
        rotation = actor.get_transform().rotation
        # Return most probable block for current location and rotation
        return self._rasterizer.get_most_probable_block(geo_location, rotation)

    def get_blocks_for_location(self, geo_location: GeoLocation) -> List[ScenarioSegment]:
        """
        Returns all ScenarioPartitionBlocks the given actor is in
        :param actor: Actor for which the ScenarioPartitionBlock should be calculated
        :return: List of ScenarioPartitionBlock for the given actor
        """
        if self._rasterizer is None:
            return []
        return self._rasterizer.calculate_scenario_blocks(geo_location)

    def get_static_map_information(self, blocks) -> DataStaticMapInformation:
        """
        Returns the DataStaticMapInformation object containing all static information for the current map
        :return: Filled DataStaticMapInformation object
        """
        # Get all block by rasterizer
        all_blocks = blocks
        # Allow for trheading
        future_pool = []
        static_block_info = []
        # static_blocks = list(map(lambda block: self.get_data_static_block_for_block(block), all_blocks))
        with concurrent.futures.ThreadPoolExecutor(max_workers=10) as executor:
            logger.debug(f'Starting Workers for block data')
            # results = executor.map(self.get_static_map_information,all_blocks)
            for block in all_blocks:
                future = executor.submit(self.get_data_static_block_for_block, block)
                logger.debug(future)
                future_pool.append(future)

        fpool_size = len(future_pool)
        logger.info(f'{fpool_size} threads to process')
        completed_tasks = 0
        for future in concurrent.futures.as_completed(future_pool):
            result = future.result()
            logger.debug(result)
            completed_tasks += 1
            logger.info(f'Tasks completed: {completed_tasks}/{fpool_size}')
            static_block_info.append(result)
        # concurrent.futures.wait(future_pool, return_when=concurrent.futures.ALL_COMPLETED)

        # Get static block information for all blocks
        # static_blocks = list(map(lambda block: self.get_data_static_block_for_block(block), blocks))
        # return DataStaticMapInformation(static_blocks=static_blocks)
        return DataStaticMapInformation(static_blocks=[future.result() for future in future_pool])

    def get_data_static_block_for_block(self, block: ScenarioSegment) -> DataStaticBlock:
        """
        Returns the DataStaticBlock object containing all static information for the given block
        :param block: The block for which the static information should be calculated
        :return: Filled DataStaticBlock object
        """
        lane_ids = block.lanes
        # Map lane_ids (str) to DataLane objects
        data_lanes = list(
            map(lambda lane_id: self.get_data_lane_for_lane(self.get_lane_for_lane_id(int(lane_id))), lane_ids))
        # Store existing critical sections for later return and duplicate checks
        critical_sections: List[DataCriticalSection] = []
        for lane_1 in data_lanes:
            for lane_2 in data_lanes:
                # Do not compare the same lanes with each other
                if lane_1.ad_map_lane_id == lane_2.ad_map_lane_id:
                    continue
                # The critical section between lane_1 and lane_2 ist already calculated
                if len(list(filter(lambda critical_section: critical_section.is_for(lane_1, lane_2),
                                   critical_sections))) > 0:
                    continue
                # Calculate critical section for current lane_1 and lane_2
                data_critical_section = self.get_critical_section_for_lanes(lane_1, lane_2)
                # For some lane combination there might be no contact point -> no critical section (parallel lanes)
                if data_critical_section is not None:
                    critical_sections.append(data_critical_section)
        return DataStaticBlock(block_id=block.id, lanes=data_lanes, critical_sections=critical_sections)

    # TODO FILTERING BLOCK

    def get_actors_in_block(self, block: ScenarioSegment, actors: List[DataActor] = None,
                            include_sensors: bool = False) -> List[DataActor]:
        """
        Returns a list of all actors in the world
        :param include_sensors: If sensors should be included
        :param block: The block which filters the actors
        :param actors: (Optional) Use the given list of actors instead of getting them through self.get_actors()
        :type actors: List[DataActor]
        :return: List of all actors
        """
        filtered_actors = []
        if ACTOR_BLOCK_FILTERING_SWITCH:
            for actor in actors:
                block = self._rasterizer.get_block_for_lane(actor.ad_map_lane_id)
                if block is None:
                    block = self._rasterizer.get_most_probable_block(actor.geo_location.to_geolocation(),
                                                                     actor.rotation.to_rotation())
                if block is None:
                    raise ValueError(
                        f'Could not find block for lane {actor.ad_map_lane_id} ({actor.road_id} - {actor.lane_id})')
                if block.id == block.id:
                    filtered_actors.append(actor)
        if not include_sensors:
            for actor in filtered_actors:
                if type(actor) is ServerSideSensor:
                    filtered_actors.remove(actor)
        return filtered_actors

    def get_actors(self, include_sensors: bool = False, noise_position: bool = False, noise_rotation: bool = False) -> \
            List[DataActor]:
        """
        Returns a list of all actors in the world
        :return: List of all actors
        """
        all_actors = list(self._world.get_actors())
        filtered_actors = all_actors
        if not include_sensors:
            filtered_actors = list(filter(lambda actor: type(actor) is not ServerSideSensor, all_actors))
        # Transform filtered Actors to DataActors
        data_actors = list(map(lambda actor: self.get_data_actor_from_actor(actor), filtered_actors))
        # TODO NORMALLY WE SHOULD SET PARAMS TO BEGINNING
        apply_normal_noise(actors=data_actors, noise_type=[NoiseTypeEnum.ALL_NOISE], mean=0.0, deviation=0.1, shape=2)
        return data_actors

    def get_data_actor_from_actor(self, actor: Actor) -> DataActor:
        """
        Returns the filled DataActor from the carla Actor
        :param actor: The actor which should be transformed into the DataActor class
        :return: Filled DataActor object
        """
        data_actor = DataActor(actor)
        geo_location = self.transform_data_location_to_geo_location(data_actor.location)
        data_actor.geo_location = geo_location
        return data_actor

    def get_data_waypoint_for_data_actor(self, actor: DataActor, data_traffic_light: DataTrafficLight) -> DataWaypoint:
        """
        Returns the filled DataWaypoint using the given actor
        :param data_traffic_light: The effecting TrafficLight for the DataActor (if existing)
        :param actor: The actor for which the DataWaypoint should be calculated
        :return: Filled DataWaypoint object
        """
        distance_from_lane_start = self.get_distance_to_start_of_lane(actor.geo_location.to_geolocation(),
                                                                      actor.ad_map_lane_id)
        if distance_from_lane_start < 0:
            distance_from_lane_start = 0
        data_waypoint = DataWaypoint(ad_map_lane_id=actor.ad_map_lane_id, road_id=actor.road_id, lane_id=actor.lane_id,
                                     location=actor.location, distance_from_lane_start=distance_from_lane_start,
                                     traffic_light=data_traffic_light,
                                     actor=actor)
        return data_waypoint

    def get_vehicles(self) -> List[DataActor]:
        """
        Return the list of all Vehicles in the world
        :return: List of all Vehicles currently active in the world
        """
        # Its the first time asking for the traffic light
        # Get all actors of the world
        actors = self.get_actors()
        # Filter actors to get traffic lights only
        vehicles = list(filter(lambda actor: type(actor) is Vehicle, actors))
        return vehicles

    def get_traffic_lights(self) -> List[DataActor]:
        """
        Return the list of all TrafficLights in the world
        :return: List of all TrafficLights currently active in the world
        """
        # The traffic light are already loaded
        if self._traffic_lights is not None:
            return self._traffic_lights
        # Its the first time asking for the traffic light
        # Get all actors of the world
        actors = self.get_actors()
        # Filter actors to get traffic lights only
        traffic_lights = list(filter(lambda actor: type(actor) is TrafficLight, actors))
        return traffic_lights

    def get_lane_ids_for_block(self, block: ScenarioSegment) -> List[int]:
        """
        Returns the list of all lanes as ids for the given Block
        :param block: Block for which the lanes should be calculated
        :return: List of lane ids inside the block
        """
        return list(map(lambda current_lane: int(current_lane), block.lanes))

    def get_lanes_for_block(self, block: ScenarioSegment) -> List[lane.Lane]:
        """
        Returns the list of all lanes for the given Block
        :param block: Block for which the lanes should be calculated
        :return: List of Lanes inside the block
        """
        # Get lane ids from the block
        lane_ids = block.lanes
        # Create list of Lane objects based on the lane ids
        lanes: List[lane.Lane] = list(map(lambda lane_id: ad.map.lane.getLane(int(lane_id)), lane_ids))
        return lanes

    def get_lane_for_lane_id(self, lane_id: int) -> lane.Lane:
        """
        Returns the lane.Lane Object for the given lane_id
        :param lane_id: The lane_id for which the lane.Lane object should be calculated
        :return: The lane.Lane object
        """
        if type(lane_id) == lane.LaneId:
            lane_id = int(str(lane_id))
        if type(lane_id) == str:
            lane_id = int(lane_id)
        return ad.map.lane.getLane(lane_id)

    def get_road_id_for_lane(self, lane: lane.Lane) -> int:
        """
        Returns the road id for the given lane
        :param lane: The lane from which the road id should be calculated
        :return: The road id of the given lane
        """
        return self.get_road_id_for_ad_map_lane_id(lane.id)

    def get_road_id_for_ad_map_lane_id(self, lane_id: int) -> int:
        """
        Returns the road id for the given ad map lane_id
        :param lane_id: The lane_id from which the road id should be calculated
        :return: The road id of the given lane
        """
        # Apparently there is exists a road with id 0. As leading zeros are removed we have a special case
        if len(str(lane_id)) == 3:
            return 0
        # We think that the lane id '190145' is split into: road_id: 19 and lane_id: (150-145) to get the
        # OpenDrive format. The '0' is the separator
        split = int(str(lane_id)[:-4])
        return split

    def get_open_drive_lane_id_for_ad_map_lane_id(self, lane_id: int) -> int:
        """
        Returns the OpenDrive lane id for the given lane
        :param lane_id: The lane_id from which the OpenDrive lane id should be calculated
        :return: The OpenDrive lane id of the given lane
        """
        # Apparently there is exists a road with id 0. As leading zeros are removed we have a special case
        if len(str(lane_id)) == 3:
            return int(str(lane_id))
        # We think that the lane id '190145' is split into: road_id: 19 and lane_id: (150-145) to get the
        # OpenDrive format. The '0' is the separator
        split = int(str(lane_id)[-4:])
        return 150 - split

    def get_open_drive_lane_id_for_lane(self, lane: lane.Lane) -> int:
        """
        Returns the OpenDrive lane id for the given lane
        :param lane: The lane from which the OpenDrive lane id should be calculated
        :return: The OpenDrive lane id of the given lane
        """
        return self.get_open_drive_lane_id_for_ad_map_lane_id(lane.id)

    def get_data_lane_for_lane_id(self, lane_id: id) -> DataLane:
        """
        Maps the map lane_id to the DataLane dataclass
        :param lane_id: lane_id to be mapped to DataLane
        :return: DataLane instance based on lane.Lane
        """
        lane = self.get_lane_for_lane_id(lane_id)
        return self.get_data_lane_for_lane(lane)

    def get_data_lane_for_lane(self, lane: lane.Lane) -> DataLane:
        """
        Maps the map lane to the DataLane dataclass
        :param lane: Lane to be mapped to DataLane
        :return: DataLane instance based on lane.Lane
        """
        # Collect necessary information
        road_id = self.get_road_id_for_lane(lane)
        open_drive_lane_id = self.get_open_drive_lane_id_for_lane(lane)
        predecessor_lanes = self.get_predecessor_lanes(lane)
        successor_lanes = self.get_successor_lanes(lane)
        intersecting_lanes = self.get_overlap_lanes(lane)
        speed_limits = self.get_speed_limits_for_lane(lane)
        # Get the left an right lane markings/edges
        left_edge = ad.map.point.toENU(lane.edgeLeft.ecefEdge)
        right_edge = ad.map.point.toENU(lane.edgeRight.ecefEdge)
        mid_points = DataLane.calculate_midpoints(left_edge, right_edge)
        landmarks = self.get_data_landmarks_for_lane(lane)
        data_lane_type = DataLaneType(int(lane.type))
        # Build dataclasses
        data_lane = DataLane(ad_map_lane_id=int(str(lane.id)), lane_id=open_drive_lane_id, road_id=road_id,
                             lane_type=data_lane_type, lane_width=float(lane.width), lane_length=float(lane.length),
                             direction=lane.direction, speed_limits=speed_limits, landmarks=landmarks,
                             predecessor_lanes=predecessor_lanes, successor_lanes=successor_lanes,
                             intersecting_lanes=intersecting_lanes, lane_midpoints=mid_points)
        return data_lane

    def get_speed_limits_for_lane(self, lane: lane.Lane) -> List[DataSpeedLimit]:
        """
        Returns a list of SpeedLimit objects for the given lane
        :param lane: THe lane for which the speed limits should be calculated
        :return: List of filled SpeedLimit objects
        """
        speed_limits_lane = list(lane.speedLimits)
        return list(map(lambda limit: DataSpeedLimit(limit, lane.length), speed_limits_lane))

    def get_matched_position_confidence_list(self, geo_location: GeoLocation, rotation: Rotation = None,
                                             increasing_sort: bool = False) -> List[MapMatchedPosition]:
        """
        Returns a list of MapMatchedPositions for the given location and rotation. Each position maps a lane
        with a distance and probability to the given location and rotation.
        :param increasing_sort: Flag if the position should by adjusted increasingly
        :param geo_location: Location for which the match should be calculated
        :param rotation: Rotation for which the match should be calculated
        :return: List of MapMatchedPositions
        """
        # Initialize Map matcher
        map_matcher = ad.map.match.AdMapMatching()
        if rotation is not None:
            # Get heading direction based on the rotation
            enu_heading = ad.map.point.createENUHeading(ad.map.point.degree2radians(rotation.yaw))
            map_matcher.addHeadingHint(enu_heading, ad.map.access.getENUReferencePoint())
        # Create GeoPoint from GeoLocation
        geo_point = ad.map.point.createGeoPoint(geo_location.longitude, geo_location.latitude, geo_location.altitude)
        # Short version: but throws RuntimeError on junctions
        # lane_id = ad.map.lane.uniqueLaneId(geo_point)
        search_radius = 0
        confidence_threshold = 0.5
        # Match GeoPoint to the map and get list of matched positions (lanes)
        matched_position_confidence_list = []
        if increasing_sort:
            # Widen search radius until a lane is found (important for intersections)
            while len(matched_position_confidence_list) == 0:
                search_radius = search_radius + 1
                if search_radius > 1:
                    confidence_threshold = 0.3
                if search_radius > 10:
                    confidence_threshold = 0.0
                matched_position_confidence_list = list(
                    map_matcher.getMapMatchedPositions(geo_point, search_radius, confidence_threshold))
        else:
            matched_position_confidence_list = list(
                map_matcher.getMapMatchedPositions(geo_point, 5, 0))
        return matched_position_confidence_list

    def get_max_probability_lane_candidate_for_geo_location_and_rotation(self, geo_location: GeoLocation,
                                                                         rotation: Rotation) -> MapMatchedPosition:
        """
        Get the Matched Position with the highest probability for the given location and rotation
        :param geo_location: Location for which the match should be calculated
        :param rotation: Rotation for which the match should be calculated
        :return: MapMatchedPosition that has the highest probability
        """
        # Get the position confidence list for the given location and rotation
        matched_position_confidence_list = self.get_matched_position_confidence_list(geo_location, rotation)
        if len(matched_position_confidence_list) == 0:
            return None
        # Get most probable match, based on distance
        max_probability_candidate = max(matched_position_confidence_list,
                                        key=lambda matching: float(matching.probability))
        return max_probability_candidate

    def get_min_distance_lane_candidate_for_geo_location_and_rotation(self, geo_location: GeoLocation,
                                                                      rotation: Rotation) -> MapMatchedPosition:
        """
        Get the Matched Position with the smallest distance for the given location and rotation
        :param geo_location: Location for which the match should be calculated
        :param rotation: Rotation for which the match should be calculated
        :return: MapMatchedPosition that has the smallest distance
        """
        # Get the position confidence list for the given location and rotation
        matched_position_confidence_list = self.get_matched_position_confidence_list(geo_location, rotation)
        # Get most probable match, based on distance
        min_distance_candidate = min(matched_position_confidence_list,
                                     key=lambda matching: float(matching.matchedPointDistance))
        return min_distance_candidate

    def get_lane_id_for_geo_location_and_rotation(self, geo_location: GeoLocation, rotation: Rotation,
                                                  use_min_distance: bool = False) -> int:
        """
        Return the lane_id on which the given vehicle is currently driving
        :param geo_location: The GeoLocation for which the lane should be calculated
        :param rotation: The rotation of the Actor
        :return: lane_id string of the current lane
        """
        if use_min_distance:
            candidate = self.get_min_distance_lane_candidate_for_geo_location_and_rotation(geo_location,
                                                                                           rotation)
        else:
            # Get candidate with the highest probability matching
            candidate = self.get_max_probability_lane_candidate_for_geo_location_and_rotation(geo_location,
                                                                                              rotation)
        if candidate is None:
            return -1
        # Extract lane_id from most probable match
        lane_id = candidate.lanePoint.paraPoint.laneId
        return int(str(lane_id))

    def get_distance_to_start_of_lane(self, geo_location: GeoLocation, lane_id: int,
                                      rotation: Rotation = None) -> float:
        """
        Returns the distance of the given location and rotation to the start of the lane
        :param geo_location: The GeoLocation for which the distance should be calculated
        :param rotation: The Rotation for which the distance should be calculated
        :param lane_id: The lane_id for which the distance should be calculated
        :return: The distance to the start of the lane. If the lane is not found: -1
        """
        # Get candidate with the highest probability matching
        max_probability_candidate = self.get_max_probability_lane_candidate_for_geo_location_and_rotation(geo_location,
                                                                                                          rotation)
        if max_probability_candidate is None:
            return -1
        # If the lanes do not match: search in all other surrounding lane matches
        if max_probability_candidate.lanePoint.paraPoint.laneId != lane_id:
            matched_positions = self.get_matched_position_confidence_list(geo_location, rotation)
            max_probability_candidate = None
            for matched_position in matched_positions:
                # The current matched position matches the given lane
                if matched_position.lanePoint.paraPoint.laneId == lane_id:
                    # Save it for later
                    max_probability_candidate = matched_position
            # No match for the given lane_id was found
            if max_probability_candidate is None:
                return -1
        # Get Lane object based on the max_probability_candidate
        current_lane = self.get_lane_for_lane_id(max_probability_candidate.lanePoint.paraPoint.laneId)
        # The parametric offset in the range of [0;1] within the lane's geometry as defined
        # in the map.
        # 0.0 refers to the start of the lanes points.
        # 1.0 refers to the end of the lanes points.
        # 0.5 refers to in between at half length of the lane.
        # Be aware: Depending on the route direction on the lane either the parametric offset
        # 0.0 or 1.0 can define the start point of that route on that lane.
        # https://github.com/carla-simulator/map/blob/b1ec6f3047f05de1df801d4d3441bde3e34e8983/ad_map_access/generated/include/ad/map/point/ParaPoint.hpp#L136
        distance_from_start = max_probability_candidate.lanePoint.paraPoint.parametricOffset * float(
            current_lane.length)
        if current_lane.direction == lane.NEGATIVE:
            distance_from_start = float(current_lane.length) - float(distance_from_start)
        return float(distance_from_start)

    def get_road_id_for_actor(self, geo_location: GeoLocation, rotation: Rotation) -> int:
        """
        Return the road_id on which the given vehicle is currently driving
        :param geo_location: The GeoLocation for which the lane should be calculated
        :param rotation: The rotation of the Actor
        :return: lane_id string of the current lane
        """
        lane_id = self.get_lane_id_for_geo_location_and_rotation(geo_location, rotation)
        current_lane = self.get_lane_for_lane_id(lane_id)
        road_id = self.get_road_id_for_lane(current_lane)
        return road_id

    def get_critical_section_for_lanes(self, lane_1: DataLane, lane_2: DataLane) -> DataCriticalSection:
        """
        Returns the Critical section object for the given lanes
        :param lane_1: Lane 1 for which the section should be calculated
        :param lane_2: Lane 1 for which the section should be calculated
        :return: DataCriticalSection of the given lanes
        """
        contact_point_for_lanes = self.get_contact_point_for_lanes(lane_1, lane_2)
        if contact_point_for_lanes is None:
            return None
        geo_location_for_contact_point = self.transform_data_location_to_geo_location(contact_point_for_lanes)
        distance_lane_1 = self.get_distance_to_start_of_lane(geo_location_for_contact_point.to_geolocation(),
                                                             lane_1.ad_map_lane_id)
        distance_lane_2 = self.get_distance_to_start_of_lane(geo_location_for_contact_point.to_geolocation(),
                                                             lane_2.ad_map_lane_id)

        return DataCriticalSection(contact_point_for_lanes, lane_1, distance_lane_1, lane_2, distance_lane_2)

    def transform_data_location_to_geo_location(self, data_location: DataLocation) -> DataGeoLocation:
        geo_point = ad.map.point.toGeo(ad.map.point.createENUPoint(data_location.x, -data_location.y, data_location.z))
        return DataGeoLocation(geo_point=geo_point)

    def get_contact_point_for_lanes(self, lane_1: DataLane, lane_2: DataLane) -> DataLocation:
        """
        Returns the first matching contact point for the given lanes. The midpoints of the lanes have to be
        calculated before using this method. See @DataLane.set_midpoints
        :param lane_1: Lane 1 to be matched
        :param lane_2: Lane 2 to be matched
        :return: Location of the contact point of the given lanes
        """
        for lane_1_index in range(len(lane_1.lane_midpoints) - 1):
            lane_1_midpoint_1 = lane_1.lane_midpoints[lane_1_index]
            lane_1_midpoint_2 = lane_1.lane_midpoints[lane_1_index + 1]
            for lane_2_index in range(len(lane_2.lane_midpoints) - 1):
                lane_2_midpoint_1 = lane_2.lane_midpoints[lane_2_index]
                lane_2_midpoint_2 = lane_2.lane_midpoints[lane_2_index + 1]
                intersection = MapHelper.get_intersect(lane_1_midpoint_1, lane_1_midpoint_2, lane_2_midpoint_1,
                                                       lane_2_midpoint_2)
                if intersection is None:
                    return None
                if MapHelper.is_between(lane_1_midpoint_1, lane_1_midpoint_2, intersection) and MapHelper.is_between(
                        lane_2_midpoint_1, lane_2_midpoint_2, intersection):
                    return intersection
        return None

    def on_same_lane(self, geo_location_1: GeoLocation, rotation_1: Rotation, geo_location_2: GeoLocation,
                     rotation_2: Rotation) -> bool:
        """
        Checks whether the given locations belongs to the same lane
        :param geo_location_1: GeoLocation of actor 1
        :param rotation_1: Rotation of actor 1
        :param geo_location_2: GeoLocation of actor 2
        :param rotation_2: Rotation of actor 2
        :return: Whether the actors are on the same lane
        """
        lane_id_1 = self.get_lane_id_for_geo_location_and_rotation(geo_location_1, rotation_1)
        lane_id_2 = self.get_lane_id_for_geo_location_and_rotation(geo_location_2, rotation_2)
        return lane_id_1 == lane_id_2

    def get_successor_lanes(self, current_lane: lane.Lane) -> List[DataContactLaneInfo]:
        """
        Returns a list of lanes representing each successor lane for the given lane
        :param current_lane: Given lane from which should be looked ahead
        :return: List of successor lanes
        """
        contact_lane_infos: List[DataContactLaneInfo] = self.get_contact_lane_infos(current_lane)
        if current_lane.direction == lane.POSITIVE:
            return list(filter(lambda contact_lane: contact_lane.contact_location == DataContactLocation.SUCCESSOR,
                               contact_lane_infos))
        else:
            return list(filter(lambda contact_lane: contact_lane.contact_location == DataContactLocation.PREDECESSOR,
                               contact_lane_infos))

    def get_predecessor_lanes(self, current_lane: lane.Lane) -> List[DataContactLaneInfo]:
        """
        Returns a list of lanes representing each predecessor lane for the given lane
        :param current_lane: Given lane from which should be looked ahead
        :return: List of predecessor lanes
        """
        contact_lane_infos: List[DataContactLaneInfo] = self.get_contact_lane_infos(current_lane)
        if current_lane.direction == lane.POSITIVE:
            return list(filter(lambda contact_lane: contact_lane.contact_location == DataContactLocation.PREDECESSOR,
                               contact_lane_infos))
        else:
            return list(filter(lambda contact_lane: contact_lane.contact_location == DataContactLocation.SUCCESSOR,
                               contact_lane_infos))

    def get_overlap_lanes(self, current_lane: lane.Lane) -> List[DataContactLaneInfo]:
        """
        Returns list of all overlapping lanes for a junction
        :param current_lane: The lane for which the overlap lanes should be returned
        :return:
        """
        contact_lane_infos: List[DataContactLaneInfo] = self.get_contact_lane_infos(current_lane)
        return list(filter(lambda contact_lane: contact_lane.contact_location == DataContactLocation.OVERLAP,
                           contact_lane_infos))

    def get_contact_lane_infos(self, current_lane: lane.Lane) -> List[DataContactLaneInfo]:
        """
        Returns a list of DataContactLaneInfo objects from the given lane
        :param current_lane: The lane for which the contact lane infos should be collected
        :return: Filled list of DataContactLaneInfo objects
        """
        # Get the contact lanes for the given lane
        contact_lanes = current_lane.contactLanes
        contact_lane_infos = []
        for contact_lane in contact_lanes:
            # Collect all necessary data for the current contact lane
            ad_map_lane_id = int(str(contact_lane.toLane))
            lane_id = self.get_open_drive_lane_id_for_ad_map_lane_id(ad_map_lane_id)
            road_id = self.get_road_id_for_ad_map_lane_id(ad_map_lane_id)
            # Map to internal enums
            contact_type = DataContactType(int(contact_lane.types[0]))
            contact_location = DataContactLocation(int(contact_lane.location))
            contact_lane_info = DataContactLaneInfo(ad_map_lane_id=ad_map_lane_id, lane_id=lane_id, road_id=road_id,
                                                    contact_location=contact_location, contact_type=contact_type)
            # Only set TrafficLight id if present (default is 0)
            if contact_lane.trafficLightId != 0:
                contact_lane_info.traffic_light_id = int(str(contact_lane.trafficLightId))
            contact_lane_infos.append(contact_lane_info)
        return contact_lane_infos

    def get_contact_lanes(self, current_lane: lane.Lane, filter_pre_successor: bool = False) -> List[lane.Lane]:
        """
        Returns list of all contact lanes for a specific lane
        :param filter_pre_successor: Set if the result should be filtered (PREDECESSOR and SUCCESSOR are filtered out)
        :param current_lane: The lane for which the contact lanes should be returned
        :return: Return list of lanes that have contact to the given lane
        """
        contact_lanes = list(current_lane.contactLanes)
        if filter_pre_successor:
            contact_lanes = list(
                filter(lambda contact_lane: contact_lane.location not in CONTINUATION_LANE_TYPES, contact_lanes))
        contact_lane_ids: List[int] = list(map(lambda contact_lane: int(str(contact_lane.toLane)),
                                               contact_lanes))
        contact_lanes: List[lane.Lane] = list(map(lambda lane_id: self.get_lane_for_lane_id(lane_id),
                                                  contact_lane_ids))
        return contact_lanes

    def get_landmarks_for_lane(self, current_lane: lane.Lane) -> List[DataContactLaneInfo]:
        """
        Returns a list of all traffic lights for the current lane (can be empty)
        :param current_lane: The lane for which the traffic lights should be returned
        :return: Filled list of TrafficLights belonging to the given lane
        """
        # The traffic light information is stored in the junction ahead of the current lane.
        # Therefore we have to get the successor lanes first
        successor_lanes: List[DataContactLaneInfo] = self.get_successor_lanes(current_lane)
        # Only take the ones which are continuing the current lane
        lane_continuation_successor_lane_infos = list(filter(
            lambda suc_lane: suc_lane.contact_type == DataContactType.LANE_CONTINUATION, successor_lanes))
        # Map to actual lane objects
        lane_continuation_successor_lanes = list(
            map(lambda lane_info: self.get_lane_for_lane_id(lane_info.ad_map_lane_id),
                lane_continuation_successor_lane_infos))
        # Now iterate over all successor lanes, find the fitting predecessor (the current lane) and get the
        # traffic_light id from the respective contact point
        landmarks_for_current_lane = []
        for continuation_lane in lane_continuation_successor_lanes:
            predecessor_lanes = self.get_predecessor_lanes(continuation_lane)
            current_lane_predecessors = list(
                filter(lambda pre_lane: pre_lane.ad_map_lane_id == current_lane.id and
                                        pre_lane.contact_type != DataContactType.LANE_CONTINUATION,
                       predecessor_lanes))
            landmarks_for_current_lane.extend(current_lane_predecessors)
        return landmarks_for_current_lane

    def get_data_landmarks_for_lane(self, current_lane: lane.Lane) -> List[DataLandmark]:
        """
        Returns a list of all landmarks for the current lane with their own DataLandmarks object
        :param current_lane: The lane for which the landmarks should be calculated
        :return: Filled list of DataLandmark objects for the given lane
        """
        landmarks_for_lane = self.get_landmarks_for_lane(current_lane)
        signs_for_current_lane = []
        for landmark in landmarks_for_lane:
            # Check what type of landmark is currently inspected
            if landmark.contact_type == DataContactType.STOP:
                sign = DataLandmark(landmark_type=DataLandmarkType.TRAFFIC_SIGN,
                                    traffic_sign_type=DataTrafficSignType.STOP)
            elif landmark.contact_type == DataContactType.YIELD:
                sign = DataLandmark(landmark_type=DataLandmarkType.TRAFFIC_SIGN,
                                    traffic_sign_type=DataTrafficSignType.YIELD)
            else:
                if landmark.traffic_light_id != 0:
                    sign = DataLandmark(landmark_type=DataLandmarkType.TRAFFIC_LIGHT,
                                        traffic_sign_type=DataTrafficSignType.UNKNOWN,
                                        traffic_light_id=landmark.traffic_light_id)
                else:
                    sign = DataLandmark(landmark_type=DataLandmarkType.UNKNOWN,
                                        traffic_sign_type=DataTrafficSignType.UNKNOWN,
                                        traffic_light_id=None)
            signs_for_current_lane.append(sign)

        # Remove duplicates
        signs_unique: List[DataLandmark] = []
        for sign in signs_for_current_lane:
            already_exists = len([current_sign for current_sign in signs_unique if
                                  current_sign.landmark_type == sign.landmark_type
                                  and current_sign.traffic_sign_type == sign.traffic_sign_type
                                  and current_sign.traffic_light_id == sign.traffic_light_id]) != 0
            if not already_exists:
                signs_unique.append(sign)
        return signs_unique

    def get_traffic_light_for_lane(self, lane: lane.Lane) -> DataLandmark:
        """
        Returns a traffic light affecting the given lane (if existing)
        :param lane: The lane for which the traffic light should be returned
        :return: A DataLandmark object standing for the TrafficLight, if existing
        """
        lane_landmarks = self.get_data_landmarks_for_lane(lane)
        traffic_light_filter = list(
            filter(lambda landmark: landmark.landmark_type == DataLandmarkType.TRAFFIC_LIGHT, lane_landmarks))
        if len(traffic_light_filter) > 0:
            return traffic_light_filter[0]
        return None

    def get_all_lanes_for_intersection(self, current_lane: lane.Lane) -> List[lane.Lane]:
        """
        Returns a list of all lanes that belong to an intersection based on the given lane
        :param current_lane: The lane for which the intersection lanes should be returned
        :return: List of all lanes for an intersection
        """
        # Return empty list if the current lane is not part of an intersection
        if current_lane.type != lane.INTERSECTION:
            return []
        # Call the recursive method to get all lanes by checking the contact lanes
        all_lanes_for_junction = self.__get_all_lanes_for_intersection_rec(current_lane, set())
        return list(all_lanes_for_junction)

    def __get_all_lanes_for_intersection_rec(self, current_lane: lane.Lane, analyzed_lanes=None) -> \
            Set[lane.Lane]:
        """
        Recursive method to get all lanes for an intersection
        :param current_lane: The current lane of the recursive call
        :param analyzed_lanes: All already tracked lanes
        :return: Set of all recursive contact lane for the given lane
        """
        # Add current lane to the final set
        if analyzed_lanes is None:
            analyzed_lanes = set()
        analyzed_lanes.add(current_lane)
        # Get all contact lanes for the current lane
        contact_lanes = self.get_contact_lanes(current_lane, True)
        # Iterate the contact lanes
        for contact_lane in contact_lanes:
            # If the current contact lane is already part of the set: skip
            if contact_lane.id in [current_lane.id for current_lane in analyzed_lanes]:
                continue
            # The current contact lane is not part of the set: recursive call
            analyzed_lanes.update(self.__get_all_lanes_for_intersection_rec(contact_lane, analyzed_lanes))
        # Return all lanes collected at this stage
        return analyzed_lanes

    @staticmethod
    def get_intersect(lane_1_start: DataLocation, lane_1_end: DataLocation, lane_2_start: DataLocation,
                      lane_2_end: DataLocation) -> DataLocation:
        """
        Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
        a1: [x, y] a point on the first line
        a2: [x, y] another point on the first line
        b1: [x, y] a point on the second line
        b2: [x, y] another point on the second line
        """
        s = np.vstack([lane_1_start.to_tuple(), lane_1_end.to_tuple(), lane_2_start.to_tuple(),
                       lane_2_end.to_tuple()])  # s for stacked
        h = np.hstack((s, np.ones((4, 1))))  # h for homogeneous
        l1 = np.cross(h[0], h[1])  # get first line
        l2 = np.cross(h[2], h[3])  # get second line
        x, y, z = np.cross(l1, l2)  # point of intersection
        if z == 0:  # lines are parallel
            return None
        return DataLocation(x=x / z, y=y / z, z=0)

    @staticmethod
    def is_between(a, b, c):
        cross_product = (c.y - a.y) * (b.x - a.x) - (c.x - a.x) * (b.y - a.y)

        # compare versus epsilon for floating point values, or != 0 if using integers
        if abs(cross_product) > 0.1:
            return False

        dot_product = (c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y)
        if dot_product < 0:
            return False

        squared_length_ba = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y)
        if dot_product > squared_length_ba:
            return False

        return True
