import json
import random
import time
from typing import List

import carla as carla
from carla import Transform, AttachmentType, Location, Rotation, Vehicle, World, Color, Walker

from carla_data_classes.DataActor import DataActor
from carla_data_classes.DataBlock import DataBlock
from carla_data_classes.DataGeoLocation import DataGeoLocation
from carla_data_classes.DataLane import DataLane
from carla_data_classes.DataTick import DataTick
from carla_data_classes.DataTrafficLight import DataTrafficLight
from carla_data_classes.EnhancedJSONEncoder import EnhancedJSONEncoder
from carla_helpers.debug_helper import DebugHelper
from carla_helpers.map_helper import MapHelper
from carla_helpers.world_helper import WorldHelper
from scenario_rasterizer import InfrastructureRasterizer, ScenarioSegmentVisualizer


def spawn_random_vehicle(world, bicycle: bool = False) -> Vehicle:
    """
    Spawns a random vehicle at an available spawn point on the map
    :param world: The carla world in which the vehicle should be spawned
    :param bicycle: Sets whether the vehicle should be a bicycle or not (default: false)
    :return: Returns the spawned vehicle object
    """
    # Get current map
    current_map = world.get_map()
    # Get list of possible spawn points for vehicles
    spawn_points = current_map.get_spawn_points()
    print("Available spawn locations: ", spawn_points)
    # Get blueprint library to create new vehicles/actors based on templates
    blueprint_library = world.get_blueprint_library()
    if bicycle:
        vehicle_blueprint = random.choice((blueprint_library.filter('vehicle.bh.crossbike')))
    else:
        # Choose a vehicle blueprint at random.
        vehicle_blueprint = random.choice(blueprint_library.filter('vehicle.*.*'))

    # Dummy vehicle object
    new_vehicle = None

    for spawn_point in spawn_points:
        print("Location for new vehicle: ", spawn_point)
        # Spawn vehicle into world
        new_vehicle = world.try_spawn_actor(vehicle_blueprint, spawn_point)
        if new_vehicle is not None:
            # Find a specific blueprint.
            collision_sensor_blueprint = blueprint_library.find('sensor.other.collision')
            # Create relative position of sensor
            relative_transform = Transform(Location(x=0, y=0, z=0), Rotation(yaw=0))
            # Add collision sensor to newly spawned vehicle
            collision_sensor = world.spawn_actor(collision_sensor_blueprint, relative_transform, new_vehicle,
                                                 AttachmentType.Rigid)
            break
    if new_vehicle is not None:
        new_vehicle.set_autopilot(False)
    # Return newly created vehicle
    return new_vehicle


def track_manual_vehicle():
    """
    Track and output debug information of the vehicle that is controlled by manual controls
    :return: -
    """
    vehicles = world_helper.get_vehicles()
    ego_vehicle = vehicles[0]

    # Get spectator of the world
    spectator = world_helper.get_spectator_camera()
    if spectator is not None:
        # Set set position for spectator
        spectator.set_transform(
            Transform(Location(x=ego_vehicle.get_location().x - 10, y=ego_vehicle.get_location().y, z=23.705683),
                      Rotation(pitch=-44.795460, yaw=0.404636, roll=0.000001)))

    # Get the current lane for the newly spawned vehicle as it changes position
    current_lane = world_helper.get_nearest_waypoint_for_actor(ego_vehicle)
    current_lane_wayoints = world_helper.get_all_waypoints_for_lane(current_lane, precision=10)
    for (_, waypoint) in current_lane_wayoints:
        debug.draw_string(waypoint.transform.location, f"{waypoint.road_id}/{waypoint.lane_id}",
                          life_time=50.0, color=Color(255, 0, 0))
    # Get follow up lanes of current lane
    follow_up_lanes = world_helper.get_follow_up_lanes(current_lane)
    # Get leading lanes of current land
    leading_lanes = world_helper.get_leading_lanes(current_lane)
    adjacent_lanes = follow_up_lanes
    adjacent_lanes.extend(leading_lanes)

    # Display all adjacent lanes
    for adjacent_lane in adjacent_lanes:
        waypoints = world_helper.get_all_waypoints_for_lane(adjacent_lane)
        for (_, waypoint) in waypoints:
            debug.draw_string(waypoint.transform.location, f"{waypoint.road_id}/{waypoint.lane_id}",
                              life_time=50.0, color=Color(0, 0, 255))

    # Get the best block for the ego_vehicle
    best_block = world_helper.get_best_block_for_actor(ego_vehicle)
    # Display the block in the simulation
    _block_visualizer.visualize_segments_in_carla([best_block], draw_block_ids=False, debug_lifespan=50)


def track_map_for_manual_vehicle():
    """
    This method collects and prints most of the important information of the simulation
    It uses the newly written methods of map_helper/world_helper
    :return: -
    """
    vehicles = world_helper.get_vehicles()
    ego_vehicle = vehicles[0]

    # Unfortunately we have to get the current lane from carla:
    current_waypoint = world_helper.get_nearest_waypoint_for_actor(ego_vehicle)
    current_lane_id = world_helper.get_ad_map_lane_id(current_waypoint)

    # Get spectator of the world
    spectator = world_helper.get_spectator_camera()
    if spectator is not None:
        # Set set position for spectator
        spectator.set_transform(
            Transform(Location(x=ego_vehicle.get_location().x - 10, y=ego_vehicle.get_location().y, z=23.705683),
                      Rotation(pitch=-44.795460, yaw=0.404636, roll=0.000001)))

    # Get AdMapAccess lane:
    current_lane = _map_helper.get_lane_for_lane_id(current_lane_id)
    current_data_lane = _map_helper.get_data_lane_for_lane(current_lane)
    _block_visualizer.draw_lane(current_data_lane, color=carla.Color(0, 0, 255), life_time=30)

    # Get GeoLocation and rotation for Ego
    ego_geo_location = world_helper.get_geolocation_for_actor(ego_vehicle)

    tic = time.perf_counter()
    static_map_information = _map_helper.get_static_map_information()
    print(json.dumps(static_map_information, cls=EnhancedJSONEncoder))
    toc = time.perf_counter()
    diff = f"It lasted {toc - tic:0.4f} seconds"
    print(diff)

    # Get the ego block
    ego_block = _rasterizer.get_block_for_lane(current_lane_id)
    block_lanes = list(map(lambda lane_id: _map_helper.get_data_lane_for_lane_id(lane_id), ego_block.lanes))
    _block_visualizer.visualize_segments_in_carla([ego_block], debug_lifespan=30)
    _block_visualizer.draw_lanes(block_lanes, life_time=30)

    print("Calculate static map information")
    static_block_information = _map_helper.get_static_map_information(blocks)
    block_json = json.dumps(static_block_information, cls=EnhancedJSONEncoder)
    print("Static map information has been calculated")

    landmarks_for_current_lane = _map_helper.get_data_landmarks_for_lane(current_lane)
    traffic_light_landmark = _map_helper.get_traffic_light_for_lane(current_lane)
    if traffic_light_landmark is not None:
        traffic_light = world.get_traffic_light_from_opendrive_id(str(traffic_light_landmark.traffic_light_id))
        print(traffic_light)

    test = DataGeoLocation(latitude=0.001667, longitude=0.001815, altitude=0.055369)
    block = _map_helper.get_best_block_for_actor(test, ego_vehicle)

    lanes = list(map(lambda lane_id: _map_helper.get_lane_for_lane_id(lane_id), ego_block.lanes))
    # Get all DataLanes from the lane_id list of the ego_block
    data_lanes: List[DataLane] = list(
        map(lambda lane_id: _map_helper.get_data_lane_for_lane_id(lane_id), ego_block.lanes))
    # Get all carla Actors as DataActors from carla
    all_actors = world_helper.get_actors()
    data_actors = []
    # Create DataActors from carla.Actors
    for current_actor in all_actors:
        data_actor = _map_helper.get_data_actor_from_actor(current_actor)
        # Get lane for current actor
        actor_waypoint = world_helper.get_nearest_waypoint_for_data_actor(data_actor)
        ad_map_lane_id = world_helper.get_ad_map_lane_id(actor_waypoint)
        open_drive_lane_id = _map_helper.get_open_drive_lane_id_for_ad_map_lane_id(ad_map_lane_id)
        road_id = _map_helper.get_road_id_for_ad_map_lane_id(ad_map_lane_id)
        # Add Lane and Road information to DataActors
        data_actor.update_lane_information(ad_map_lane_id=ad_map_lane_id, lane_id=open_drive_lane_id, road_id=road_id)
        data_actors.append(data_actor)

    # Filter all actors be the ego_block
    actors_in_block: List[DataActor] = _map_helper.get_actors_in_block(ego_block, data_actors)
    for block_actor in actors_in_block:
        debug_helper.draw_debug_output_for_data_actor(block_actor)
    waypoints = []
    for actor in actors_in_block:
        if actor.id == ego_vehicle.id:
            actor.ego_vehicle = True
        else:
            actor.ego_vehicle = False
        # Get TrafficLight (if existing)
        current_lane = _map_helper.get_lane_for_lane_id(actor.ad_map_lane_id)
        traffic_light_landmark = _map_helper.get_traffic_light_for_lane(current_lane)
        if traffic_light_landmark is not None:
            traffic_light = world.get_traffic_light_from_opendrive_id(str(traffic_light_landmark.traffic_light_id))
            data_traffic_light = DataTrafficLight(traffic_light)
        else:
            data_traffic_light = None
        data_waypoint = _map_helper.get_data_waypoint_for_data_actor(actor, data_traffic_light)
        waypoints.append(data_waypoint)
    data_block = DataBlock(block_id=ego_block.id, waypoints=waypoints)
    data_tick = DataTick(tick=0.0, data=data_block)
    ticks = [data_tick]
    print(json.dumps(ticks, cls=EnhancedJSONEncoder))

    walker: Walker = world_helper.get_pedestrians()[0]
    waypoint_walker = world_helper.get_nearest_waypoint_for_actor(walker)
    walker_geo_location = world_helper.get_geolocation_for_location(walker.get_location())
    # Unfortunately we have to get the current lane from carla:
    current_walker_lane_id = _map_helper.get_lane_id_for_geo_location_and_rotation(
        walker_geo_location, walker.get_transform().rotation, True)

    # Get AdMapAccess lane:
    current_lane_walker = _map_helper.get_lane_for_lane_id(current_walker_lane_id)
    current_data_lane_walker = _map_helper.get_data_lane_for_lane(current_lane_walker)
    _block_visualizer.draw_lane(current_data_lane_walker, color=carla.Color(0, 0, 255), life_time=30)


# This file connects to the carla simulator and lets you test newly written code with the debugger,
# instead of calling the methods in the scenario runner
# The test environment connects to the simulator and spawns a vehicle. From there on you can change the code
if __name__ == '__main__':
    print("Connect to carla simulator")
    # Find carla simulator at localhost on port 2000
    client = carla.Client('localhost', 2000)
    # Try to connect for 10 seconds. Fail if not successful
    client.set_timeout(10.0)
    print("Connected to carla simulator")
    world: World = client.get_world()
    debug = world.debug
    _rasterizer = InfrastructureRasterizer()
    open_drive = world.get_map().to_opendrive()
    blocks = _rasterizer.analyze_map_from_xodr_content(open_drive, junction_ext=0.0)
    _map_helper = MapHelper(_rasterizer)
    _block_visualizer = ScenarioSegmentVisualizer()

    world_helper = WorldHelper(world, rasterizer=_rasterizer)
    debug_helper = DebugHelper(world, debug_lifespan=20.0, text_only=False)

    track_map_for_manual_vehicle()
