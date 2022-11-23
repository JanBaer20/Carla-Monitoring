from time import sleep
import py_trees
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_criteria import Criterion
from srunner.scenariomanager.timer import GameTime

from carla import World, Vehicle, Map

from scenario_rasterizer import InfrastructureRasterizer, ScenarioSegmentVisualizer
from carla_helpers.world_helper import WorldHelper
from carla_helpers.map_helper import MapHelper

import carla
import argparse
import socket

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 65000  # The port used by the server

class VehicleRightBeforeLeft(Criterion):
    """
    This class contains an atomic test for stopped vehicles.
    """

    MAX_IDLE_TIME = 1  # Amount of time in seconds until a vehicle is flagged as "stopped"
    MIN_VELOCITY = 0.5  # Amount of speed necessary to start the test

    def __init__(self, actor, world: World, debug_mode, name="VehicleDataTest", terminate_on_failure=False):
        super(VehicleRightBeforeLeft, self).__init__(name, actor, 0,
                                                 terminate_on_failure=terminate_on_failure)

        self.logger.debug("%s.__init__()" % self.__class__.__name__)

        self._actor = self.actor
        self._rasterizer = InfrastructureRasterizer()
        open_drive = world.get_map().to_opendrive()
        print("save world to disk")
        f = open("/home/janbaer/Desktop/demofile2.txt", "a")
        f.write(open_drive)
        f.close()
        # Analyze blocks for current map
        print("Analyze blocks for current map.")
        blocks = self._rasterizer.analyze_map_from_xodr_content(open_drive)
        print("All blocks have been calculated")
        self._world = world
        self._world_helper = WorldHelper(world, self._rasterizer)
        self._map_helper = MapHelper(self._rasterizer)
        #static_map_information = self._map_helper.get_static_map_information(blocks=blocks)
        self._debug_mode = debug_mode
        # Save starting point as "stopped"
        self._last_stopped_time = GameTime.get_time()
        # Save if the vehicle has moved once
        self._has_initially_moved = False
        # Initialize list to store already tracked stop times
        self._stopped_times = []
        self._current_road = -1
        self._current_lane = -1
        self._straight = []
        self._left_turns = []
        self._right_turns = []
        self._straight_wp = []
        self._right_turn_wp = []
        self._intersection = None


    # Gets called each tick and checks the implemented test
    def update(self):
        """
        Check velocity == 0
        :return: Current status of the test
        """
        if self._debug_mode:
            print(f"Current time: {GameTime.get_time()}")

        # Saves the status of the current update cycle of the VehicleStoppedTest
        new_test_status = py_trees.common.Status.RUNNING

        # Get all carla Actors as DataActors from carla
        all_actors = self._world_helper.get_actors()
        # Create DataActors from carla.Actors
        data_actors = [self._map_helper.get_data_actor_from_actor(current_actor) for current_actor in all_actors]
        #logger.debug(f'data actors: {data_actors}')

        # Get ego vehicle Data Actor by ID
        data_ego_vehicles = list(filter(lambda veh: veh.id == self.actor.id, data_actors))
        if len(data_ego_vehicles) != 1:
            raise ValueError(f'Got wrong amount of ego vehicles: len(data_ego_vehicles) - {data_ego_vehicles}')
        data_ego_vehicle = data_ego_vehicles[0]
        #logger.debug(f'ego actors: {data_ego_vehicle}')

        print("isJunction")
        actor_waypoint = self._world_helper.get_nearest_waypoint_for_data_actor(data_ego_vehicle)
        lane_id = self._world_helper.get_ad_map_lane_id(actor_waypoint)
        lane = self._map_helper.get_lane_for_lane_id(lane_id)
        print(actor_waypoint.is_junction)
        

        if lane_id in self._left_turns:
            print("Vehicle turned left")
            for turn_lane_id in self._right_turns:
                road = self._map_helper.get_road_id_for_ad_map_lane_id(turn_lane_id)
                turn_lane = self._map_helper.get_lane_for_lane_id(turn_lane_id)
                for oncoming_lane_id in self._map_helper.get_lanes_for_road_id(road):
                    oncoming_lane = self._map_helper.get_lane_for_lane_id(oncoming_lane_id)
                    if(turn_lane.direction != oncoming_lane.direction):
                        if len(self._world_helper.get_all_stop_signs_for_lane_id(oncoming_lane_id)) == 0:
                            if(self._intersection is not None and len(self._world.get_traffic_lights_in_junction(self._intersection.id)) == 0):
                                if len(self._world_helper.get_all_vehicles_for_lane_id(oncoming_lane_id)) > 0:
                                    self.send_violation(b"Right vehicle ignored")
            for turn_lane_id in self._straight:
                print("test")
                road = self._map_helper.get_road_id_for_ad_map_lane_id(turn_lane_id)
                turn_lane = self._map_helper.get_lane_for_lane_id(turn_lane_id)
                for oncoming_lane_id in self._map_helper.get_lanes_for_road_id(road):
                    print("test2")
                    oncoming_lane = self._map_helper.get_lane_for_lane_id(oncoming_lane_id)
                    if(turn_lane.direction != oncoming_lane.direction):
                        print("test3")
                        if len(self._world_helper.get_all_stop_signs_for_lane_id(oncoming_lane_id)) == 0:
                            print("test4")
                            if(self._intersection is not None and len(self._world.get_traffic_lights_in_junction(self._intersection.id)) == 0):
                                print("test5")
                                print(oncoming_lane_id)
                                print(lane_id)
                                if len(self._world_helper.get_all_vehicles_for_lane_id(oncoming_lane_id)) > 0:
                                    self.send_violation(b"Oncoming vehicle ignored")
        # if self._map_helper.get_road_id_for_lane(self._map_helper.get_lane_for_lane_id(lane_id)) in self._right_turns:
        #     print("Vehicle turned right")
        #     for road in self._right_turn_wp:
        #         for lane in self._world_helper.get_lanes_for_road(road):
        #             if lane.lane_type != carla.LaneType.Shoulder and lane.lane_type != carla.LaneType.Sidewalk:
        #                 if(abs(lane.transform.rotation.yaw - road.transform.rotation.yaw) > 170 and abs(lane.transform.rotation.yaw - road.transform.rotation.yaw) < 190):
        #                     if len(lane.get_landmarks_of_type(5, carla.LandmarkType.StopSign)) == 0 and len(lane.get_landmarks_of_type(5, carla.LandmarkType.YieldSign)) == 0 and len(self._world_helper.get_all_actors_for_lane(lane)) > 0:
        #                         if(self._intersection is not None and len(self._world.get_traffic_lights_in_junction(self._intersection)) == 0):
        #                             self.send_violation("Right vehicle ignored")
        if lane_id in self._straight:
            print("Vehicle went straight")
            for turn_lane_id in self._right_turns:
                road = self._map_helper.get_road_id_for_ad_map_lane_id(turn_lane_id)
                turn_lane = self._map_helper.get_lane_for_lane_id(turn_lane_id)
                for oncoming_lane_id in self._map_helper.get_lanes_for_road_id(road):
                    oncoming_lane = self._map_helper.get_lane_for_lane_id(oncoming_lane_id)
                    if(turn_lane.direction != oncoming_lane.direction):
                        if len(self._world_helper.get_all_stop_signs_for_lane_id(oncoming_lane_id)) == 0:
                            if(self._intersection is not None and len(self._world.get_traffic_lights_in_junction(self._intersection.id)) == 0):
                                if len(self._world_helper.get_all_vehicles_for_lane_id(oncoming_lane_id)) > 0:
                                    self.send_violation(b"Right vehicle ignored")

        if not actor_waypoint.is_junction and (self._map_helper.get_lane_for_lane_id(lane_id) != self._current_road or self._current_lane != lane_id):
            self._straight.clear()
            self._left_turns.clear()
            self._right_turns.clear()
            self._right_turn_wp.clear()
            self._straight_wp.clear()
            self._intersection = None

            intersection_waypoint = self._world_helper.advance_waypoint_until_junction(actor_waypoint)

            for ip in intersection_waypoint.next(2.0):
                if ip.get_junction() is not None:
                    self._intersection = ip.get_junction()
                    for wp in self._world_helper.get_lanes_for_junction(ip.get_junction()):
                        # print(self._world_helper.get_ad_map_lane_id(wp))
                        # print(self._map_helper.get_successor_lanes(self._map_helper.get_lane_for_lane_id(lane_id)))
                        if self._world_helper.get_ad_map_lane_id(wp) in [x.ad_map_lane_id for x in self._map_helper.get_successor_lanes(self._map_helper.get_lane_for_lane_id(lane_id))]:
                            # print(self._world_helper.get_ad_map_lane_id(wp))
                            # print(wp.next_until_lane_end(0.5)[-1].transform)
                            # print(intersection_waypoint.transform)
                            angle = (wp.next_until_lane_end(0.5)[-1].transform.rotation.yaw - intersection_waypoint.transform.rotation.yaw) % 360
                            next_road = wp.next_until_lane_end(0.5)[-1].next(0.5)[0]
                            # print(wp.transform.rotation.yaw - intersection_waypoint.transform.rotation.yaw)
                            # print(self._map_helper.get_road_id_for_lane(self._map_helper.get_lane_for_lane_id(self._world_helper.get_ad_map_lane_id(wp))))
                            if angle < 30 or angle > 330:
                                oncoming_lane_id = self._world_helper.get_ad_map_lane_id(next_road)
                                road_id = self._map_helper.get_road_id_for_lane(self._map_helper.get_lane_for_lane_id(oncoming_lane_id))
                                if oncoming_lane_id not in self._straight:
                                    self._straight.append(oncoming_lane_id)
                                    self._straight_wp.append(next_road)
                                next_lane_id = self._world_helper.get_ad_map_lane_id(next_road.next(5)[0])
                                next_road_id = self._map_helper.get_road_id_for_lane(self._map_helper.get_lane_for_lane_id(next_lane_id))
                                if next_lane_id not in self._straight:
                                    self._straight.append(next_lane_id)
                                    self._straight_wp.append(next_road.next(5)[0])
                            elif angle < 330 and angle > 210:
                                oncoming_lane_id = self._world_helper.get_ad_map_lane_id(next_road)
                                road_id = self._map_helper.get_road_id_for_lane(self._map_helper.get_lane_for_lane_id(oncoming_lane_id))
                                if oncoming_lane_id not in self._left_turns:
                                    self._left_turns.append(oncoming_lane_id)
                                next_lane_id = self._world_helper.get_ad_map_lane_id(next_road.next(5)[0])
                                next_road_id = self._map_helper.get_road_id_for_lane(self._map_helper.get_lane_for_lane_id(next_lane_id))
                                if next_lane_id not in self._left_turns:
                                    self._left_turns.append(next_lane_id)
                            elif angle > 30 and angle < 150:
                                oncoming_lane_id = self._world_helper.get_ad_map_lane_id(next_road)
                                road_id = self._map_helper.get_road_id_for_lane(self._map_helper.get_lane_for_lane_id(oncoming_lane_id))
                                if oncoming_lane_id not in self._right_turns:
                                    self._right_turns.append(oncoming_lane_id)
                                    self._right_turn_wp.append(next_road)
                                next_lane_id = self._world_helper.get_ad_map_lane_id(next_road.next(5)[0])
                                next_road_id = self._map_helper.get_road_id_for_lane(self._map_helper.get_lane_for_lane_id(next_lane_id))
                                if next_lane_id not in self._right_turns:
                                    self._right_turns.append(next_lane_id)
                                    self._right_turn_wp.append(next_road.next(5)[0])

        self._current_road = self._map_helper.get_road_id_for_lane(self._map_helper.get_lane_for_lane_id(lane_id))
        # print(lane_id)
        # print(self._map_helper.get_lane_for_lane_id(lane_id))
        print("left")
        print(self._left_turns)
        print("right")
        print(self._right_turns)
        print("straight")
        print(self._straight)

        if lane_id > 0:
            # print("Successor lanes")
            # print(self._map_helper.get_successor_lanes(self._map_helper.get_lane_for_lane_id(lane_id)))
            # print("Predecessor lanes")
            # print(self._map_helper.get_predecessor_lanes(self._map_helper.get_lane_for_lane_id(lane_id)))
            #print(self._map_helper.get_data_lane_for_lane_id(lane_id).intersecting_lanes)
            # print("Predecessor lanes")
            # for lane in self._map_helper.get_contact_lane_infos(self._map_helper.get_lane_for_lane_id(lane_id)):
            #     print(lane)
            #print("intersection lanes")
            # for lane in self._map_helper.get_all_lanes_for_intersection(self._map_helper.get_lane_for_lane_id(lane_id)):
            #     print(lane)
            print("Road")
            print(self._map_helper.get_road_id_for_lane(self._map_helper.get_lane_for_lane_id(lane_id)))

        # The ego vehicle stopped long enough. Set test status
        self.test_status = py_trees.common.Status.FAILURE


        # Record event by increasing actual result by 1
        self.actual_value += 1

        # Terminate test if the "terminate on failure" flag is set and the current status if "FAILURE"
        if self._terminate_on_failure and (self.test_status == py_trees.common.Status.FAILURE):
            new_test_status = py_trees.common.Status.FAILURE

        # Log update cycle information
        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_test_status))

        return new_test_status

    def send_violation(self, msg):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            self._socket = s
            s.sendall(msg)

def monitor(args):
    world = None
    print(args.actor)

    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)

    world = client.get_world()

    GameTime.restart()
    possible_vehicles = world.get_actors().filter(args.filter)
    for vehicle in possible_vehicles:
        if vehicle.attributes['role_name'] == 'hero' and args.actor == 'ego':
            actor = vehicle
        elif vehicle.attributes['role_name'] == args.actor:
            actor = vehicle
    
    CarlaDataProvider.set_client(client)
    CarlaDataProvider.register_actor(actor)

    test = VehicleRightBeforeLeft(actor, world=world, debug_mode=True,
                                                terminate_on_failure=args.sof)

    while True:
        snapshot = world.get_snapshot()
        if snapshot:
            timestamp = snapshot.timestamp
            GameTime.on_carla_tick(timestamp)
        
        CarlaDataProvider.on_carla_tick()
        test.update()
        sleep(1/int(args.Hz))

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-a', '--actor',
        action='store',
        default='ego',
        dest='actor',
        help='actor name (default ego vehicle)')
    argparser.add_argument(
        '--sof',
        action='store_true',
        dest='sof',
        help='Stop on failure')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '-Hz',
        action='store',
        default=1,
        dest='Hz',
        help='Tickrate of the execution')
    args = argparser.parse_args()

    print("Test")
    try:
        print("Test")
        monitor(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__ == '__main__':

    main()
