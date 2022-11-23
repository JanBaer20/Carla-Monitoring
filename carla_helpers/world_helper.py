import math
from typing import List, Tuple, Optional, Dict

from carla import TrafficLight, LaneType, Junction, Waypoint, Actor, TrafficLightState, Vector3D, Location, \
    GeoLocation, Map, World, Vehicle, ServerSideSensor, Rotation, Walker, LandmarkType

from carla_data_classes.DataActor import DataActor
from scenario_rasterizer import ScenarioSegment, InfrastructureRasterizer

ACTOR_BLOCK_FILTERING_SWITCH = True

USEABLE_LANE_TYPES = {LaneType.Driving,
                      LaneType.Bidirectional,
                      LaneType.Parking,
                      LaneType.Entry,
                      LaneType.Exit,
                      LaneType.OffRamp,
                      LaneType.OnRamp,
                      LaneType.Restricted,
                      LaneType.Special1,
                      LaneType.Special2,
                      LaneType.Special3,
                      LaneType.Sidewalk,
                      LaneType.Biking}


class WorldHelper(object):
    """
    This class provides methods that return information using the waypoint API of carla
    """
    DEBUG_LIFESPAN = 5  # Time in seconds for the debug output to last

    def __init__(self, world, rasterizer: InfrastructureRasterizer = None):
        super(WorldHelper, self).__init__()

        self._world: World = world
        self._debug = world.debug
        self._map: Map = self._world.get_map()
        self._traffic_lights = None
        self._spectator_camera = None
        self._rasterizer = rasterizer

    @staticmethod
    def get_ad_map_lane_id(waypoint: Waypoint) -> int:
        return int(str(f"{waypoint.road_id}0{waypoint.lane_id + 150}"))

    def is_object_in_block_of_actor(self, obj: Actor, actor: Actor) -> bool:
        """
        Returns whether the given object is inside the ScenarioPartitionBlock of the given Actor
        :param obj: Object to be checked to be in the ScenarioPartitionBlock
        :param actor: Actor for which the ScenarioPartitionBlock should be calculated
        :return: True if the obj is inside the Actors' ScenarioPartitionBlock, False otherwise
        """
        if self._rasterizer is None:
            return True  # without rasterization just get the world as one block
        # Get the most probable block for the actor
        block_for_actor = self.get_best_block_for_actor(actor)  # Das ist zu ineffizient
        # Check if there is a block near the actor
        if block_for_actor is None:
            # There is no block. Therefore, the obj is not inside it
            return False
        # Get GeoLocation for the obj
        geo_location_for_obj = self.get_geolocation_for_location(obj.get_location())
        # Check of the GeoLocation is inside of the block
        return block_for_actor.is_in(geo_location_for_obj)

    def get_best_block_for_actor(self, actor: Actor) -> ScenarioSegment:
        """
        Return ScenarioPartitionBlock for current actor
        :param actor: Actor for which the ScenarioPartitionBlock should be calculated
        :return: The most probable ScenarioPartitionBlock for the given Actor
        """
        # Calculate GeoLocation for current Actor
        geo_location: GeoLocation = self.get_geolocation_for_actor(actor)
        # Get rotation of current Actor
        rotation = self.get_rotation_of_actor(actor)
        # Return most probable block for current location and rotation
        return self._rasterizer.get_most_probable_block(geo_location, rotation)

    def get_blocks_for_actor(self, actor: Actor) -> List[ScenarioSegment]:
        """
        Returns all ScenarioPartitionBlocks the given actor is in
        :param actor: Actor for which the ScenarioPartitionBlock should be calculated
        :return: List of ScenarioPartitionBlock for the given actor
        """
        geo_location: GeoLocation = self.get_geolocation_for_actor(actor)

        if self._rasterizer is None:
            return []
        return self._rasterizer.calculate_scenario_blocks(geo_location)

    def get_rotation_of_actor(self, actor: Actor) -> Rotation:
        """
        Returns the rotation of a given Actor
        :param actor: Actor for which the rotation should be returned
        :return: Rotation of the given Actor
        """
        return actor.get_transform().rotation

    def get_geolocation_for_actor(self, actor: Actor) -> GeoLocation:
        """
        Returns the GeoLocation for a given Actor
        :param actor: The actor
        :return: GeoLocation for given Actor
        """
        return self.get_geolocation_for_location(actor.get_location())

    def get_geolocation_for_location(self, location: Location) -> GeoLocation:
        """
        Transforms given carla.Location to carla.GeoLocation for the current map
        :param location: carla.Location to be transformed
        :return: Transformed carla.GeoLocation
        """
        return self._map.transform_to_geolocation(location)

    # TODO FILTERING BLOCK

    def get_actors_in_block(self, ego_vehicle: Vehicle, include_sensors: bool = False) -> List[Actor]:
        """
        Returns a list of all actors in the world
        :return: List of all actors
        """
        all_actors = list(self._world.get_actors())
        filtered_actors = all_actors
        if ACTOR_BLOCK_FILTERING_SWITCH:
            filtered_actors = [actor for actor in filtered_actors if
                               self.is_object_in_block_of_actor(actor, ego_vehicle)]
        if not include_sensors:
            filtered_actors = list(filter(lambda actor: type(actor) is not ServerSideSensor, all_actors))
        return filtered_actors

    def get_actors(self, include_sensors: bool = False, noise_position: bool = False, noise_rotation: bool = False) -> \
            List[Actor]:
        """
        Returns a list of all actors in the world
        :return: List of all actors
        """
        all_actors = list(self._world.get_actors())
        filtered_actors = all_actors
        if not include_sensors:
            filtered_actors = list(filter(lambda actor: type(actor) is not ServerSideSensor, all_actors))
        return filtered_actors

    def get_data_actors(self, include_sensors: bool = False, noise_position: bool = False,
                        noise_rotation: bool = False) -> List[DataActor]:
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
        return data_actors

    def get_spectator_camera(self) -> Optional[Actor]:
        """
        Returns the spectator_camera object
        :return: The spectator_camera object
        """
        # Get all actors of the world
        actors = self._world.get_actors()
        # Filter actors to get the spectator
        spectator = list(actors.filter("*spectator"))
        # There is at least one spectator
        if len(spectator) >= 1:
            return spectator[0]
        return None

    def get_vehicles(self) -> List[Actor]:
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

    def get_pedestrians(self) -> List[Actor]:
        """
        Return the list of all Vehicles in the world
        :return: List of all Vehicles currently active in the world
        """
        # Its the first time asking for the traffic light
        # Get all actors of the world
        actors = self.get_actors()
        # Filter actors to get traffic lights only
        vehicles = list(filter(lambda actor: type(actor) is Walker, actors))
        return vehicles

    def get_traffic_lights(self) -> List[TrafficLight]:
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
        # Save traffic lights for later use
        self._traffic_lights = traffic_lights
        return traffic_lights

    def get_traffic_light_state(traffic_light: TrafficLight) -> TrafficLightState:
        """
        Return the state (red, yellow, green) of the given traffic_light
        :return: The state of the given traffic_light as string
        """
        return traffic_light.state

    def advance_waypoint_until_junction(self, waypoint: Waypoint) -> Waypoint:
        """
        Advance the given waypoint until it reaches the next junction
        :param waypoint: The waypoint to advance
        :return: The waypoint closest to the junction
        """
        # While the current waypoint is not part of the intersection: advance
        while not waypoint.is_intersection:
            # Get next waypoint in 0.5 meters
            next_waypoint = waypoint.next(0.5)[0]
            if next_waypoint and not next_waypoint.is_intersection:
                waypoint = next_waypoint
            else:
                break
        return waypoint

    def get_traffic_light_stop_line(self, traffic_light: TrafficLight) -> [(Vector3D, Vector3D)]:
        """
        Returns a tuple of two points that build the stop line for the given traffic light
        :param traffic_light: The TrafficLight for which the stop line should be returned
        :return: Tuple of two Vector3Ds that represent the left and right side of the stop
        line for the given TrafficLight
        """
        results_lines = []
        stop_waypoints: List[Waypoint] = traffic_light.get_stop_waypoints()
        for waypoint in stop_waypoints:
            lane_width = waypoint.lane_width
            yaw_waypoint = waypoint.transform.rotation.yaw
            waypoint_location = waypoint.transform.location

            lft_lane_wp = self.rotate_point(Vector3D(0.4 * lane_width, 0.0, waypoint_location.z), yaw_waypoint + 90)
            lft_lane_wp = waypoint_location + Location(lft_lane_wp)
            rgt_lane_wp = self.rotate_point(Vector3D(0.4 * lane_width, 0.0, waypoint_location.z), yaw_waypoint - 90)
            rgt_lane_wp = waypoint_location + Location(rgt_lane_wp)
            results_lines.append((lft_lane_wp, rgt_lane_wp))
        return results_lines

    def get_lane_id_for_actor(self, actor: Actor) -> str:
        """
        Return the lane_id on which the given vehicle is currently driving
        :param actor: The vehicle from which the lane should be calculated
        :return: lane_id string of the current lane
        """
        nearest_waypoint = self.get_nearest_waypoint_for_actor(actor)
        return nearest_waypoint.lane_id

    def get_road_id_for_actor(self, actor: Actor) -> str:
        """
        Return the road_id of the road closest to the actor
        :param actor: The actor fro which the road should be calculated
        :return: road_id string of the current road
        """
        nearest_waypoint = self.get_nearest_waypoint_for_actor(actor)
        return nearest_waypoint.road_id

    def get_nearest_waypoint_for_data_actor(self, actor: DataActor) -> Waypoint:
        """
        Returns the nearest Waypoint for the given DataActor
        :param actor: The actor from which the lane should be calculated
        :return: Waypoint closest to the given DataActor
        """
        location = actor.location.to_location()
        return self.get_nearest_waypoint_for_location(location)

    def get_nearest_waypoint_for_actor(self, actor: Actor) -> Waypoint:
        """
        Returns the nearest Waypoint for the given actor
        :param actor: The actor from which the lane should be calculated
        :return: Waypoint closest to the given actor
        """
        location = actor.get_transform().location
        return self.get_nearest_waypoint_for_location(location)

    def get_nearest_waypoint_for_location(self, location: Location) -> Waypoint:
        """
        Returns the nearest Waypoint for the given location
        :param location: The location from which the lane should be calculated
        :return: Waypoint closest to the given location
        """
        waypoint = self._map.get_waypoint(location)
        return waypoint

    def get_traffic_lights_for_vehicle(self, vehicle: Actor) -> List[TrafficLight]:
        """
        Returns all traffic lights that belong to the lane the vehicle is currently driving on
        :param vehicle:
        :return:
        """
        # Get current lane for the new vehicle
        current_lane_waypoint = self.get_nearest_waypoint_for_actor(vehicle)
        if current_lane_waypoint is None:
            return []
        # Get traffic light for the current lane
        return self.get_traffic_lights_for_lane(current_lane_waypoint)

    def get_traffic_lights_for_lane(self, lane: Waypoint) -> List[TrafficLight]:
        """
        Returns all traffic lights that belong to the lane
        :param lane: The lane as one concrete waypoint
        :return: List of all traffic lights belonging to the lane
        """
        # Get all traffic lights
        traffic_lights = self.get_traffic_lights()
        affected_traffic_lights = []
        # Check each traffic light if it belongs to the current_lane
        for traffic_light in traffic_lights:
            # Get all waypoints for the traffic light
            waypoints = traffic_light.get_affected_lane_waypoints()
            # Check if there is a waypoint for the traffic light that belongs to lane
            waypoints = filter(lambda w: self.waypoints_are_one_same_lane(lane, w), waypoints)
            # There is a match: save traffic light
            if len(list(waypoints)) > 0:
                affected_traffic_lights.append(traffic_light)
        return affected_traffic_lights

    def waypoints_are_one_same_lane(self, waypoint_1: Waypoint, waypoint_2: Waypoint) -> bool:
        """
        Checks whether the two waypoints are located on the same road and lane
        :param waypoint_1: First waypoint
        :param waypoint_2: Second waypoint
        :return: True if the waypoints are on the same road and lane, False otherwise
        """
        return waypoint_1.road_id == waypoint_2.road_id and waypoint_1.lane_id == waypoint_2.lane_id

    def actor_is_on_same_lane(self, actor: Actor, waypoint: Waypoint) -> bool:
        """
        Checks whether the given actor belongs to the lane of the given waypoint
        :param actor: The actor to check
        :param waypoint: The waypoint to check
        :return: Whether the actor belongs to the same lane
        """

    def get_junction_for_vehicle(self, vehicle: Actor) -> Junction:
        """
        Returns the current junction on which the vehicle is driving, if there is any
        :param vehicle: The vehicle from which the junction should be calculated
        :return: Junction, if vehicle is currently driving on a junction, else None
        """
        lane = self.get_nearest_waypoint_for_actor(vehicle)
        junction = lane.get_junction()
        return junction

    def get_lanes_for_junction(self, junction: Junction) -> List[Waypoint]:
        """
        Returns list of lanes for a specific junction
        :param junction: The junction from which the lanes should be extracted
        :return: List of Waypoints. Each waypoint stands for each lane of the junction
        """
        if junction is None:
            return
        junction_waypoints = junction.get_waypoints(LaneType.Any)
        lanes = []
        for lane_start, lane_end in junction_waypoints:
            lanes.append(lane_start)
        return lanes

    def get_lanes_for_road(self, road: Waypoint) -> List[Waypoint]:
        """
        Returns list of lanes for a specific road
        :param road: The road for which the lanes should ba calculated
        :return: List of Waypoints. Each waypoint stands for an individual lane of the road
        """
        lanes = []
        # Get all lanes to the left
        left_lanes = self.get_left_lanes(road)
        if left_lanes is not None:
            # Reverse to ensure correct order
            left_lanes.reverse()
            lanes.extend(left_lanes)
        # Append current lane from where the left/right lanes are calculated
        lanes.append(road)
        # Get all lanes to the right
        right_lanes = self.get_right_lanes(road)
        if right_lanes is not None:
            lanes.extend(right_lanes)
        # Ensure that the lanes are marked from left to right
        lanes.reverse()
        # Remove duplicates
        lanes_unique = []
        for lane in lanes:
            if lane.lane_type in USEABLE_LANE_TYPES and \
                    len([current_lane for current_lane in lanes_unique if current_lane.id == lane.id]) == 0:
                lanes_unique.append(lane)
        return lanes_unique

    def get_left_lanes(self, lane: Waypoint) -> List[Waypoint]:
        """
        Returns list of all lanes to the left
        :param lane: The lane from which the left lanes should be calculated
        :return: List Waypoints. Each waypoint stands for an individual lane to the left of the given lane
        """
        lanes = []
        left_lane: Waypoint = lane.get_right_lane()
        while left_lane is not None and len(
                [current_lane for current_lane in lanes if current_lane.id == left_lane.id]) == 0:
            lanes.append(left_lane)
            left_lane = left_lane.get_right_lane()
        return lanes

    def get_right_lanes(self, lane: Waypoint) -> List[Waypoint]:
        """
        Returns list of all lanes to the right
        :param lane: The lane from which the right lanes should be calculated
        :return: List Waypoints. Each waypoint stands for an individual lane to the right of the given lane
        """
        lanes = []
        right_lane: Waypoint = lane.get_left_lane()
        while right_lane is not None and len(
                [current_lane for current_lane in lanes if current_lane.id == right_lane.id]) == 0:
            lanes.append(right_lane)
            right_lane = right_lane.get_left_lane()
        return lanes

    def get_all_waypoints_for_junction(self, junction: Junction) -> List[Tuple[str, List[Tuple[float, Waypoint]]]]:
        """
        Returns a list of tuples. Each tuple contains the (lane_id, [waypoints]) for each lane of the junction
        :param junction: The junction for which the waypoints are to be calculated
        :return: list of lane_id to waypoints tuples: List[Tuple(lane_id, List[Waypoint])]
        """
        # Get all lanes for the junction
        lanes = self.get_lanes_for_junction(junction)
        if lanes is None:
            return
        waypoints = []
        # Get all waypoints for each lane of the junction and create tuple for each lane
        for lane in lanes:
            waypoints.append((lane.id, self.get_all_waypoints_for_lane(lane)))
        return waypoints

    def get_all_waypoints_for_lane(self, lane: Waypoint, precision: float = 2.0) -> List[Tuple[float, Waypoint]]:
        """
        Returns a list of all waypoints with distance from the start for the current lane
        :param lane: The lane for which the waypoints should be calculated
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: List of {distance, Waypoint} for the lane
        """
        all_waypoints = []
        # Get all waypoints until the lane starts
        waypoints_until_start = self.get_all_waypoints_until_start_of_lane(lane, precision)
        # Reverse the list to start with the waypoint that is at the beginning of the lane
        waypoints_until_start.reverse()
        # Add waypoints until start of lane to waypoint list
        all_waypoints.extend(waypoints_until_start)
        # Add current waypoint as it is not included in the previous list
        all_waypoints.append(lane)
        # Get all waypoints until the lane end
        waypoints_until_end = self.get_all_waypoints_until_end_of_lane(lane, precision)
        # Add waypoints until end of lane to waypoint list
        all_waypoints.extend(waypoints_until_end)
        waypoint_distance_list = []
        waypoint_counter = 0
        # Calculate distance to start of lane for each waypoint
        for waypoint in all_waypoints:
            # The distance is based on the given precision (in m)
            distance = precision * waypoint_counter
            waypoint_distance_tuple: Tuple[float, Waypoint] = (distance, waypoint)
            waypoint_distance_list.append(waypoint_distance_tuple)
            waypoint_counter += 1
        return waypoint_distance_list

    def get_all_waypoints_until_end_of_lane(self, lane: Waypoint, precision: float = 2.0) -> List[Waypoint]:
        """
        Returns a list of all waypoints until the end of the current lane
        :param lane: The lane from which the waypoints should be returned
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: List of Waypoints until the lane ends
        """
        next_waypoints = lane.next_until_lane_end(float(precision))
        next_waypoints = list(filter(lambda waypoint: waypoint.road_id == lane.road_id, next_waypoints))
        return next_waypoints

    def get_all_waypoints_until_start_of_lane(self, lane: Waypoint, precision: float = 2.0) -> List[Waypoint]:
        """
        Returns a list of all waypoints until the start of the current lane
        :param lane: The lane from which the waypoints should be returned
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: List of Waypoints until the lane start
        """
        previous_waypoints = lane.previous_until_lane_start(float(precision))
        previous_waypoints = list(filter(lambda waypoint: waypoint.road_id == lane.road_id, previous_waypoints))
        return previous_waypoints

    def get_last_waypoint_of_lane(self, lane: Waypoint, precision: float = 2.0) -> Waypoint:
        """
        Returns the last waypoint of the given lane with the given precision
        :param lane: The lane of which the last waypoint should be returned
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: Last Waypoint of the given lane
        """
        waypoints_until_end_of_lane = self.get_all_waypoints_until_end_of_lane(lane, precision)
        last_waypoint = waypoints_until_end_of_lane[len(waypoints_until_end_of_lane) - 1]
        return last_waypoint

    def get_first_waypoint_of_lane(self, lane: Waypoint, precision: float = 2.0) -> Waypoint:
        """
        Returns the first waypoint of the given lane with the given precision
        :param lane: The lane of which the last waypoint should be returned
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: First Waypoint of the given lane
        """
        waypoints_until_start_of_lane = self.get_all_waypoints_until_start_of_lane(lane, precision)
        first_waypoint = waypoints_until_start_of_lane[len(waypoints_until_start_of_lane) - 1]
        return first_waypoint

    def get_follow_up_lanes(self, lane: Waypoint, precision: float = 2.0) -> List[Waypoint]:
        """
        Returns a list of waypoints representing each follow up lane for the given lane
        :param lane: Given lane from which should be looked ahead
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: List of Waypoints representing follow up lanes
        """
        last_waypoint = self.get_last_waypoint_of_lane(lane, precision)
        return last_waypoint.next(float(precision))

    def get_leading_lanes(self, lane: Waypoint, precision: float = 2.0) -> List[Waypoint]:
        """
        Returns a list of waypoints representing each leading lane for the given lane
        :param lane: Given lane from which should be looked at
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: List of Waypoints representing leading lanes
        """
        first_waypoint = self.get_first_waypoint_of_lane(lane, precision)
        return first_waypoint.previous(float(precision))

    def get_all_actors_for_lane(self, lane: Waypoint, ego_vehicle: Vehicle = None) -> List[Actor]:
        """
        Returns a list of all actors that belong to the given waypoint
        :param lane: The waypoint for which the actors should be calculated
        :return: List of actors belonging to the given waypoint
        """
        lane_actors = []
        # Get all actors in the world
        if ACTOR_BLOCK_FILTERING_SWITCH:
            all_actors = self.get_actors_in_block(ego_vehicle)
        else:
            all_actors = self.get_actors()
        for actor in all_actors:
            # Get the nearest waypoint for the current actor
            actor_waypoint = self.get_nearest_waypoint_for_actor(actor)
            # If not on the same lane as the given one: skip
            if self.waypoints_are_one_same_lane(lane, actor_waypoint):
                # The actor is on the same lane: append to final list
                lane_actors.append(actor)
        return lane_actors

    def get_nearest_waypoint_on_lane_for_actor(self, waypoints: List[Tuple[float, Waypoint]], actor: Actor) -> Tuple[
        float, Waypoint]:
        """
        Returns the nearest waypoint for the given actor amongst the given list of lanes
        :param waypoints: The lanes that should be searched
        :param actor: The actor that should be nearest to one of the waypoints
        :return: The nearest waypoint to the given actor amongst the given lanes
        """
        location_of_actor = actor.get_location()
        nearest_waypoint = None
        for waypoint_distance_tuple in waypoints:
            (distance, waypoint) = waypoint_distance_tuple
            # Calculated distance between the actor and the current waypoint
            distance = waypoint.transform.location.distance(location_of_actor)
            # Check if there is already a nearest waypoint saved
            if nearest_waypoint is None:
                nearest_waypoint = (distance, waypoint)
            (min_distance, min_distance_waypoint) = nearest_waypoint
            # Check if current waypoint is closer than the saved one
            if distance < min_distance:
                # Update nearest waypoint
                nearest_waypoint = (distance, waypoint)
        return nearest_waypoint

    def get_nearest_actors_for_waypoints(self, waypoints: List[Waypoint], actors: List[Actor]) -> Dict[
        Waypoint, List[Actor]]:
        """
        Returns a mapping: waypoint -> nearest actors
        :param waypoints: Waypoints for which the actors should be mapped to
        :param actors: The actors that should be mapped to the waypoints
        :return: Dictionary containing the mapping: waypoint -> nearest actors
        """
        waypoint_actor_mapping = {}
        # Initialize dict entry for each waypoint with an empty list
        for (_, waypoint) in waypoints:
            waypoint_actor_mapping[waypoint.id] = []
        for actor in actors:
            # Get nearest waypoint to the current actor
            (_, nearest_waypoint) = self.get_nearest_waypoint_on_lane_for_actor(waypoints, actor)
            if nearest_waypoint.id in waypoint_actor_mapping:
                # Append mapping: nearest_waypoint -> actor
                waypoint_actor_mapping[nearest_waypoint.id].append(actor)
        return waypoint_actor_mapping

    def get_all_actors_for_lane(self, lane: Waypoint) -> List[Actor]:
        """
        Returns a list of actors that are located on the given lane
        :param lane: The lane for which the actors are searched
        :return: List of Actors located on the given lane
        """
        # Get all actors in the scene
        actors = self.get_actors()
        actors_on_lane = []
        for actor in actors:
            # Get nearest waypoint for current actor
            nearest_waypoint = self.get_nearest_waypoint_for_actor(actor)
            # Check if the actors' waypoint belongs to the same lane as the given waypoint
            if self.waypoints_are_one_same_lane(nearest_waypoint, lane):
                actors_on_lane.append(actor)
        return actors_on_lane

    def get_all_vehicles_for_lane_id(self, lane_id: str) -> List[Actor]:
        """
        Returns a list of actors that are located on the given lane id
        :param lane_id: The lane_id for which the actors are searched
        :return: List of Actors located on the given lane id
        """
        # Get all actors in the scene
        actors = self.get_vehicles()
        actors_on_lane = []
        for actor in actors:
            # Get nearest waypoint for current actor
            nearest_waypoint = self.get_nearest_waypoint_for_actor(actor)
            print(actor.attributes)
            print(self.get_ad_map_lane_id(nearest_waypoint))
            # Check if the actors' waypoint belongs to the same lane as the given waypoint
            if self.get_ad_map_lane_id(nearest_waypoint) == lane_id:
                actors_on_lane.append(actor)
        return actors_on_lane

    def get_all_stop_signs_for_lane_id(self, lane_id: str) -> List[Actor]:
        """
        Returns a list of Stop and Yield Signs that are located on the given lane id
        :param lane_id: The lane_id for which the Stop and Yield Signs are searched
        :return: List of Stop and Yield Signs located on the given lane id
        """
        # Get all landmarks of type Stop or Yield Sign in the scene
        actors = []
        actors.extend(self._map.get_all_landmarks_of_type(LandmarkType.StopSign))
        actors.extend(self._map.get_all_landmarks_of_type(LandmarkType.YieldSign))
        actors_on_lane = []
        for actor in actors:
            # Get nearest waypoint for current actor
            if actor is not None:
                nearest_waypoint = actor.waypoint
                # Check if the actors' waypoint belongs to the same lane as the given waypoint
                if nearest_waypoint is not None and nearest_waypoint.lane_id == lane_id:
                    actors_on_lane.append(actor)
        return actors_on_lane

    def rotate_point(self, point, angle):
        """
        rotate a given point by a given angle
        """
        x_ = math.cos(math.radians(angle)) * point.x - math.sin(math.radians(angle)) * point.y
        y_ = math.sin(math.radians(angle)) * point.x + math.cos(math.radians(angle)) * point.y
        return Vector3D(x_, y_, point.z)
